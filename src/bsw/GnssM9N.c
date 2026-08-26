/**********************************************************************************************************************
 * \file GnssM9N.c
 * \brief
 *********************************************************************************************************************/
#include "GnssM9N.h"
#include "Uart.h"
#include "IfxAsclin_Asc.h"
#include "IfxAsclin.h"
#include "ConfigurationIsr.h"
#include "SysTime.h"

/* local used defines */
#define GNSSM9N_BUFFER_SIZE    100u
#define GNSS_POLL_PERIOD_MS    100u      /* must match the task calling GnssM9N_poll */
#define GNSS_TIMEOUT_MS        2000u    /* > the ~850 ms gap between 1 Hz bursts */
#define GNSS_NOT_PRESENT_TICKS (GNSS_TIMEOUT_MS / GNSS_POLL_PERIOD_MS)
#define RING_BUFFER_SIZE       512u

/* Largest UBX payload the shared frame buffer can hold: it also carries the
 * 4-byte header and the 2 checksum bytes. */
#define GNSS_UBX_MAX_RX_PAYLOAD (GNSSM9N_BUFFER_SIZE - 6u)
#define GNSS_TX_DEADLINE_TICKS 1000000u

/* UBX */
#define GNSS_UBX_SYNC1         0xB5u
#define GNSS_UBX_SYNC2         0x62u
#define GNSS_UBX_MAX_PAYLOAD   32u
#define CFG_UART1OUTPROT_NMEA         0x10740002u
#define CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007u
#define CFG_RATE_MEAS                 0x30210001u
#define CFG_RATE_NAV                  0x30210002u
#define CFG_NAVSPG_DYNMODEL           0x20110021u
#define UBX_CFG_LAYER_RAM          0x01u
#define UBX_CLASS_CFG              0x06u
#define UBX_ID_CFG_VALSET          0x8Au
#define UBX_VALSET_U1_LEN             9u
#define UBX_VALSET_U2_LEN            10u
#define UBX_MEAS_RATE               100u /* 100ms --> 10Hz */

/* Dynamic platform model (CFG-NAVSPG-DYNMODEL, table 21 of the interface
 * description). The receiver runs an internal Kalman filter; this tells it
 * what motion to expect, which sets how much it trusts a new measurement
 * against its own prediction.
 *
 * The default is PORT (0) -- gentle handheld motion. On a multirotor that
 * model rejects real acceleration as implausible, so the reported position
 * lags the true one, and ground-oriented models additionally assume altitude
 * changes slowly.
 *
 * AIR1 (<1 g) rather than AIR2: the first flights are vertical only, so the
 * horizontal dynamics stay low and the extra headroom of AIR2 would only add
 * noise. Revisit when the quadcopter starts manoeuvring. */
#define UBX_DYNMODEL_AIR1             6u

#define UBX_NAV_PVT_YEAR           4u
#define UBX_NAV_PVT_MONTH          6u
#define UBX_NAV_PVT_DAY            7u
#define UBX_NAV_PVT_HOUR           8u
#define UBX_NAV_PVT_MIN            9u
#define UBX_NAV_PVT_SEC           10u
#define UBX_NAV_PVT_TIME_VALIDITY 11u

#define UBX_NAV_PVT_FIXTYPE       20u
#define UBX_NAV_PVT_FLAGS         21u
#define UBX_NAV_PVT_NUMSV         23u
#define UBX_NAV_PVT_LON           24u
#define UBX_NAV_PVT_LAT           28u
#define UBX_NAV_PVT_HMSL          36u
#define UBX_NAV_PVT_HACC          40u
#define UBX_NAV_PVT_VACC          44u
#define UBX_NAV_PVT_GSPEED        60u
#define UBX_NAV_PVT_HEADMOT       64u
#define UBX_NAV_PVT_PDOP          76u
#define UBX_NAV_PVT_FLAGS3        78u
#define UBX_NAV_PVT_ITOW           0u
#define UBX_NAV_PVT_PAYLOAD_LEN   92u

/* Frame layout of the RX buffer: class, id and the two length bytes come
 * first, so a payload offset from the interface description needs +4. */
#define UBX_PAYLOAD_OFFSET         4u

#define UBX_CLASS_ACK             0x05u
#define UBX_ID_ACK_ACK            0x01u
#define UBX_ID_ACK_NAK            0x00u
#define UBX_CLASS_NAV             0x01u
#define UBX_ID_NAV_PVT            0x07u

/* UBX-NAV-PVT flag bits (see docs/GNSS_UBX.md section 6) */
#define UBX_PVT_FLAGS_GNSSFIXOK   0x01u
#define UBX_PVT_FLAGS3_INVALIDLLH 0x01u

/* UBX-NAV-PVT `valid` bitfield, payload offset 11. The interface description
 * (section 3.3.4) is explicit: a value field is only meaningful when its
 * validity flag is set. */
#define UBX_PVT_VALID_DATE        0x01u
#define UBX_PVT_VALID_TIME        0x02u
#define UBX_PVT_VALID_FULLYRESOLV 0x04u

/* fixType values worth acting on (docs/GNSS_UBX.md section 6). 2D has no
 * trustworthy altitude, so it is not enough for a 3-D state estimate. */
#define UBX_FIXTYPE_3D            3u
#define UBX_FIXTYPE_GNSS_DR       4u

/* Horizontal accuracy above which the fix is not worth fusing [mm].
 *
 * Deliberately loose: this is a coarse "the receiver is lost" gate, not a
 * flight-quality decision. The ASW gets hAccM in metres and should apply its
 * own, tighter threshold for whatever it is doing -- 10 m is fine for a
 * position hold reference, far too loose for a precision landing. */
#define GNSS_HACC_USABLE_MM       10000u


/* local used variables */

static uint8                   g_buffer[GNSSM9N_BUFFER_SIZE];
static uint8                   g_len         = 0u;
static uint16                  g_errors      = 0u;
static volatile uint32         g_bytes       = 0u;
static uint32                  g_sentences   = 0u;
static uint16                  g_poll_counter = GNSS_NOT_PRESENT_TICKS;
static uint8                   g_ring_buf_overflow_counter = 0u;
static GnssM9N_Nav             g_nav;

static uint8                   g_tx_discards = 0u;

/* isr variables */
static volatile uint8          g_ring_buffer[RING_BUFFER_SIZE];
static volatile uint16         g_ring_head = 0u;
static volatile uint16         g_ring_tail = 0u;

static uint16                  g_ubx_payload_len = 0u;

/* ubx variables */
static boolean gsv_cfg_sent = FALSE;
static uint8 g_detect_ack = 0u;

static gnss_parse_state_t parse_state = GNSS_IDLE;
static uint8 last_byte;
static uint32 g_ubx_nak    = 0u;
static uint32 g_ubx_navpvt = 0u;
static uint8 g_ubx_ack = 0u;

/* How many CFG-VALSET acknowledgements we are still waiting for. Counted
 * rather than compared against a literal so adding a key cannot silently
 * invalidate the check. */
static uint8 g_cfg_expected_acks = 0u;


/* local functions */
void           asclin4IsrReceive   (void);
static boolean GnssM9N_timeout     (void);
static boolean gnss_txByte         (uint8 value);
static boolean gnss_sendUbx        (uint8 msgClass, uint8 msgId, const uint8 *payload, uint8 payload_len);
static boolean gnss_ubx_decode     (const uint8 *buffer, uint8 buffer_len, uint8 payload_len, GnssM9N_Nav *Nav);
static void    gnss_checksumUbx    (const uint8 *message, uint8 payload_len, uint8 *ck_a, uint8 *ck_b);
static boolean gnss_cfgValsetU1    (uint8 value, uint32 key);
static boolean gnss_cfgValsetU2    (uint16 value, uint32 key);


/* cppcheck-suppress misra-c2012-17.3 ; deviation: IFX_INTERRUPT is a vendor
 * macro that emits the vector-table entry; cppcheck does not expand it and
 * reports the handler as an implicit declaration. */
IFX_INTERRUPT(asclin4IsrReceive, 0, ISR_PRIORITY_ASCLIN4_RX);

/* cppcheck-suppress misra-c2012-8.7 ; deviation: referenced by the interrupt
 * vector table, not by C code; static would break the vector entry. */
void asclin4IsrReceive(void)
{
    uint8 fill_level = IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN4);
    for (uint8 i=0u; i<fill_level; i++)
    {
        uint32 rx_word   = IfxAsclin_readRxData(&MODULE_ASCLIN4);
        uint8  fifo_byte = (uint8)(rx_word & 0xFFu);
        g_bytes++;

        uint16 ring_next = g_ring_head +1u;
        if (ring_next >= RING_BUFFER_SIZE)
        {
            ring_next = 0u;
        }

        if ( (ring_next) != g_ring_tail)
        {
            g_ring_buffer[g_ring_head] = fifo_byte;
            g_ring_head = ring_next;
        }
        else
        {
            g_ring_buf_overflow_counter++;
        }
    }
}

static boolean gnss_cfgValsetU1(uint8 value, uint32 key)
{
    uint8 payload[UBX_VALSET_U1_LEN];
    boolean status = FALSE;

    payload[0] = 0u;
    payload[1] = UBX_CFG_LAYER_RAM;
    payload[2] = 0u;
    payload[3] = 0u;
    payload[4] = (uint8) ( key      & 0xFFu);
    payload[5] = (uint8) ((key >>8) & 0xFFu);
    payload[6] = (uint8) ((key>>16) & 0xFFu);
    payload[7] = (uint8) ((key>>24) & 0xFFu);
    payload[8] = value;

    status = gnss_sendUbx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, UBX_VALSET_U1_LEN);

    return status;
}

static boolean gnss_cfgValsetU2(uint16 value, uint32 key)
{
    uint8 payload[UBX_VALSET_U2_LEN];
    boolean status = FALSE;

    payload[0] = 0u;
    payload[1] = UBX_CFG_LAYER_RAM;
    payload[2] = 0u;
    payload[3] = 0u;
    payload[4] = (uint8) ( key      & 0xFFu);
    payload[5] = (uint8) ((key >>8) & 0xFFu);
    payload[6] = (uint8) ((key>>16) & 0xFFu);
    payload[7] = (uint8) ((key>>24) & 0xFFu);
    payload[8] = (uint8) ( value    & 0xFFu);
    payload[9] = (uint8) ((value>>8)& 0xFFu);

    status = gnss_sendUbx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, UBX_VALSET_U2_LEN);

    return status;
}


static boolean gnss_txByte(uint8 value)
{
    uint32 start = SysTime_getTicks();
    boolean discard = FALSE;

    while ((IfxAsclin_getTxFifoFillLevel(&MODULE_ASCLIN4) >= 16u) &&
            (discard != TRUE))
    {
        if ((SysTime_getTicks() - start) > GNSS_TX_DEADLINE_TICKS)
        {
            g_tx_discards++;
            discard = TRUE;
        }
    }
    if (discard != TRUE)
    {
        IfxAsclin_writeTxData(&MODULE_ASCLIN4, (uint32)value);
    }

    return discard != TRUE;
}

static void gnss_checksumUbx(const uint8 *message, uint8 payload_len, uint8 *ck_a, uint8 *ck_b){

    /* build checksum */
    for(uint8 i= 0u; i < payload_len; i++)
    {
        *ck_a += message[i];
        *ck_b += *ck_a;
    }
}

static boolean gnss_sendUbx (uint8 msgClass, uint8 msgId, const uint8 *payload, uint8 payload_len)
{
    uint8   ubx_message[GNSS_UBX_MAX_PAYLOAD + 8u];
    uint8   ck_a         = 0u;
    uint8   ck_b         = 0u;
    boolean send_success = TRUE;

    /* Single point of exit (MISRA 15.5): the body is skipped rather than
     * returned from early. */
    if ( (payload_len > GNSS_UBX_MAX_PAYLOAD) ||
         (payload_len == 0u)            )
    {
        send_success = FALSE;
    }
    else
    {
    ubx_message[0u] = GNSS_UBX_SYNC1;
    ubx_message[1u] = GNSS_UBX_SYNC2;
    ubx_message[2u] = msgClass;
    ubx_message[3u] = msgId;
    ubx_message[4u] = payload_len;
    ubx_message[5u] = 0u;

    /* add payload into message */
    for (uint8 i = 0u; i < payload_len; i++)
    {
        ubx_message[6u+ i] = payload[i];
    }

    /* build checksum */
    (void)gnss_checksumUbx(&ubx_message[2], 4u+payload_len, &ck_a, &ck_b);

    ubx_message[6u+payload_len] = ck_a;
    ubx_message[6u+payload_len+1u] = ck_b;

    /* transmit array */
    for(uint8 i=0u; i<(6u + payload_len + 2u); i++)
    {
        if (gnss_txByte(ubx_message[i]) != TRUE)
        {
            send_success = FALSE;
        }
    }

    if ( (send_success == TRUE)      &&
         (msgClass == UBX_CLASS_CFG) )
    {
        g_cfg_expected_acks++;
    }
    }

    return send_success;
}

/* Assemble a little-endian 32-bit value from four payload bytes.
 *
 * Every byte is widened to uint32 BEFORE it is shifted. Without that the
 * bytes promote to signed int and anything above 127 sign-extends, which
 * silently corrupts every latitude, velocity and accuracy figure. */
static uint32 ubx_rd_u32(const uint8 *p)
{
    return ((uint32)p[0])
         | ((uint32)p[1] <<  8)
         | ((uint32)p[2] << 16)
         | ((uint32)p[3] << 24);
}

static uint16 ubx_rd_u16(const uint8 *p)
{
    return (uint16)(((uint16)p[0]) | ((uint16)p[1] << 8));
}

/* Decode UBX-NAV-PVT into Nav. The caller has already verified the checksum
 * and the class/id, so this only does field extraction. */
static void ubx_decode_navPvt(const uint8 *payload, GnssM9N_Nav *Nav)
{
    uint8 flags  = payload[UBX_NAV_PVT_FLAGS];
    uint8 flags3 = payload[UBX_NAV_PVT_FLAGS3];
    uint8 valid  = payload[UBX_NAV_PVT_TIME_VALIDITY];

    /* Publish the raw validity bits as well as the derived flags: when the
     * time looks wrong, which of the three is clear says why. */
    Nav->validDate     = ((valid & UBX_PVT_VALID_DATE)        != 0u) ? 1u : 0u;
    Nav->validTime     = ((valid & UBX_PVT_VALID_TIME)        != 0u) ? 1u : 0u;
    Nav->fullyResolved = ((valid & UBX_PVT_VALID_FULLYRESOLV) != 0u) ? 1u : 0u;

    /* fullyResolved is included on purpose: date and time can both be flagged
     * valid while the receiver still carries seconds-level uncertainty, which
     * is useless for timestamping a measurement. */
    Nav->timeOk = ((Nav->validDate     != 0u) &&
                   (Nav->validTime     != 0u) &&
                   (Nav->fullyResolved != 0u)) ? 1u : 0u;

    /* iTOW is GPS time of week, not UTC -- it is meaningful whenever the
     * receiver has a fix, independent of whether UTC has been resolved, so
     * it is NOT gated on validDate. It is the right timestamp to fuse on. */
    Nav->iTOW = ubx_rd_u32(&payload[UBX_NAV_PVT_ITOW]);

    /* Zero the calendar fields rather than publish the receiver's internal
     * free-running clock: with no satellite it counts up from a firmware
     * epoch (seen as 2020-08-02 on 2026-08-22), which looks like real data. */
    if (Nav->validDate != 0u)
    {
        Nav->year       = ubx_rd_u16(&payload[UBX_NAV_PVT_YEAR]);
        Nav->month      = payload[UBX_NAV_PVT_MONTH];
        Nav->day        = payload[UBX_NAV_PVT_DAY];
    }
    else
    {
        Nav->year       = 0u;
        Nav->month      = 0u;
        Nav->day        = 0u;
    }

    if (Nav->validTime != 0u)
    {
        Nav->hour       = payload[UBX_NAV_PVT_HOUR];
        Nav->min        = payload[UBX_NAV_PVT_MIN];
        Nav->sec        = payload[UBX_NAV_PVT_SEC];
    }
    else
    {
        Nav->hour       = 0u;
        Nav->min        = 0u;
        Nav->sec        = 0u;
    }

    Nav->fixQuality = payload[UBX_NAV_PVT_FIXTYPE];
    Nav->numSats    = payload[UBX_NAV_PVT_NUMSV];

    Nav->lon        = (sint32)ubx_rd_u32(&payload[UBX_NAV_PVT_LON]);
    Nav->lat        = (sint32)ubx_rd_u32(&payload[UBX_NAV_PVT_LAT]);
    Nav->hMSL       = (sint32)ubx_rd_u32(&payload[UBX_NAV_PVT_HMSL]);
    Nav->gSpeed     = (sint32)ubx_rd_u32(&payload[UBX_NAV_PVT_GSPEED]);
    Nav->headMot    = (sint32)ubx_rd_u32(&payload[UBX_NAV_PVT_HEADMOT]);
    Nav->hAcc       = ubx_rd_u32(&payload[UBX_NAV_PVT_HACC]);
    Nav->vAcc       = ubx_rd_u32(&payload[UBX_NAV_PVT_VACC]);
    Nav->pDOP       = ubx_rd_u16(&payload[UBX_NAV_PVT_PDOP]);

    /* fixType alone is NOT enough -- the interface description requires the
     * validity flags too (docs/GNSS_UBX.md section 7). */
    if (((flags  & UBX_PVT_FLAGS_GNSSFIXOK)   != 0u) &&
        ((flags3 & UBX_PVT_FLAGS3_INVALIDLLH) == 0u))
    {
        Nav->fixOk = 1u;
    }
    else
    {
        Nav->fixOk = 0u;
    }

    /* navOk -- the single flag downstream code should gate on.
     *
     * Three independent things have to hold, and none of them implies the
     * others:
     *   fixOk     the receiver itself says the solution is valid
     *   3D fix    a 2D fix invents an altitude from an assumed plane
     *   hAcc      a fix can be "valid" and still be tens of metres out
     *             while the receiver reacquires
     *
     * Time validity is NOT part of this. UTC resolution and position quality
     * are separate: the receiver can know the time from one satellite while
     * having no position at all, and can hold a good fix through a leap
     * second it has not resolved. */
    if ((Nav->fixOk != 0u) &&
        ((Nav->fixQuality == UBX_FIXTYPE_3D) ||
         (Nav->fixQuality == UBX_FIXTYPE_GNSS_DR)) &&
        (Nav->hAcc <= GNSS_HACC_USABLE_MM))
    {
        Nav->navOk = 1u;
    }
    else
    {
        Nav->navOk = 0u;
    }
}

static boolean gnss_ubx_decode (const uint8 *buffer, uint8 buffer_len, uint8 payload_len, GnssM9N_Nav *Nav){
    uint8   ck_a  = 0u;
    uint8   ck_b  = 0u;
    boolean valid = TRUE;

    /* Named indices: "buffer_len - 1" promotes to int and would be compared
     * against a uint8 array element (MISRA 10.4). */
    uint8 idx_ck_b = (uint8)(buffer_len - 1u);
    uint8 idx_ck_a = (uint8)(buffer_len - 2u);

    gnss_checksumUbx(buffer, 4u+payload_len, &ck_a, &ck_b);

    if ((buffer[idx_ck_b] != ck_b) ||
        (buffer[idx_ck_a] != ck_a) )
    {
        valid = FALSE;
    }
    else
    {

    /* Dispatch on class/id. Anything else is ignored on purpose -- the frame
     * was well formed, we simply do not consume that message. */
    if (buffer[0] == UBX_CLASS_ACK)
    {
        if (buffer[1] == UBX_ID_ACK_ACK)
        {
            g_ubx_ack++;
        }
        else if (buffer[1] == UBX_ID_ACK_NAK)
        {
            g_ubx_nak++;
        }
        else
        {
            /* other ACK-class message */
        }
    }
    else if ((buffer[0] == UBX_CLASS_NAV) &&
             (buffer[1] == UBX_ID_NAV_PVT))
    {
        /* Length guard: a short frame would read past the payload. */
        if (payload_len >= UBX_NAV_PVT_PAYLOAD_LEN)
        {
            ubx_decode_navPvt(&buffer[UBX_PAYLOAD_OFFSET], Nav);
            g_ubx_navpvt++;
        }
    }
    else
    {
        /* not consumed */
    }
    }

    return valid;
}




/*
 * GNSS-NEO-M9N init function:
 * Step 1: create config for ASC Interface on ASCLIN4: IfxAsclin_Asc_initModuleConfig
 * Step 2: customize config for GNSS
 * Step 3: Init the ASC Module                       : IfxAsclin_Asc_initModule
 * Step 4: Clean the Buffer                          : IfxAsclin_flushRxFifo
 * Step 5: Clear all Flags                           : IfxAsclin_clearAllFlags
 * GNSS now writes it's bytes to Asclin hardware FIFO Buffer
 */
boolean GnssM9N_init(void)
{
    IfxAsclin_Status status = IfxAsclin_Status_configurationError;
    IfxAsclin_Asc_Config config;
    IfxAsclin_Asc_initModuleConfig(&config, &MODULE_ASCLIN4);

    /* set baudrate for GNSS*/
    config.baudrate.baudrate =  UART_SPEED_38400;

    static const IfxAsclin_Asc_Pins pins = {
        .cts       = NULL_PTR,                        /* no hardware flow control */
        .rx        = &IfxAsclin4_RXC_P22_6_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL_PTR,                        /* no hardware flow control */
        .tx        = &IfxAsclin4_TX_P22_5_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_ttlSpeed1
    };
    config.pins = &pins;

    /* interrupt config */
    config.interrupt.rxPriority    = ISR_PRIORITY_ASCLIN4_RX;
    config.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* No software FIFO buffers — transmission goes directly to the HW FIFO */
    config.txBuffer     = NULL_PTR;
    config.txBufferSize = 0;
    config.rxBuffer     = NULL_PTR;
    config.rxBufferSize = 0;

    /* Block scope: used only here, and an internal-linkage name must be
     * unique across every translation unit -- Uart.c owns a g_asclin too
     * (MISRA 5.9 / 8.9). */
    static IfxAsclin_Asc s_gnssAsclin;

    status = IfxAsclin_Asc_initModule(&s_gnssAsclin, &config);

    /* The reset used to sit behind "status == IfxAsclin_Status_noError",
     * which is NEVER true here: iLLD returns configurationError whenever
     * txBuffer or rxBuffer is NULL_PTR, and this driver passes NULL_PTR for
     * both on purpose (it owns the HW FIFO and its own ring). The whole block
     * was therefore dead code on the host AND on silicon -- it only went
     * unnoticed because the statics start at zero after a cold boot. It now
     * runs unconditionally, so a warm re-init genuinely clears the state. */
    (void)status;
    {
      /* hardware reset on silicon */
      IfxAsclin_flushRxFifo(&MODULE_ASCLIN4);
      IfxAsclin_clearAllFlags(&MODULE_ASCLIN4);

      g_ring_head = 0u;
      g_ring_tail = 0u;
      g_ring_buf_overflow_counter = 0u;

      /* software reset */
      g_len          = 0u;
      g_errors       = 0u;
      g_bytes        = 0u;
      g_sentences    = 0u;
      g_poll_counter = GNSS_NOT_PRESENT_TICKS;

      /* reset TX to GNSS */
      gsv_cfg_sent        = FALSE;
      g_cfg_expected_acks = 0u;
      g_tx_discards       = 0u;

      /* UBX receive state */
      parse_state       = GNSS_IDLE;
      last_byte         = 0u;
      g_ubx_payload_len = 0u;
      g_ubx_ack         = 0u;
      g_ubx_nak         = 0u;
      g_ubx_navpvt      = 0u;
      g_detect_ack      = 0u;

      /* navigation solution */
      /* All-zero template, block scope: clears g_nav without memset (which
       * would pull in string.h) and without listing every field. */
      static const GnssM9N_Nav navZero = {0};
      g_nav = navZero;

      g_buffer[0] = 0u;
    }

    /* cppcheck-suppress misra-c2012-10.4 ; deviation: both operands are the
     * vendor enum IfxAsclin_ClockSource; cppcheck cannot resolve the return
     * type of the iLLD accessor and reports an essential-type mismatch. */
    return (IfxAsclin_getClockSource(&MODULE_ASCLIN4) == IfxAsclin_ClockSource_ascFastClock);
}

static boolean GnssM9N_timeout(void)
{
    boolean expired = TRUE;

    if (g_poll_counter < GNSS_NOT_PRESENT_TICKS)
    {
        g_poll_counter++;
        expired = FALSE;
    }

    return expired;
}

/*
 * Function that reads the information provided by GNSS M9N every 100ms
 */
boolean GnssM9N_read(GnssM9N_Sample *sample){
    boolean status    = FALSE;

    if (GnssM9N_timeout() != TRUE)
    {
        status = TRUE;
    }

    /* One-shot receiver configuration, once the link has proven itself.
     *
     * Sent only after presence is TRUE: the M9N is still booting while
     * GnssM9N_init runs, and a frame sent into a receiver that is not
     * listening yet is lost without any trace.
     *
     * RAM layer only (see gnss_cfgValsetU1), so a power cycle undoes anything
     * wrong here -- including a mistake that would otherwise silence the
     * receiver permanently. */
    if ( (status == TRUE)           &&
         (gsv_cfg_sent == FALSE) )
    {
        (void)gnss_cfgValsetU1(0u           , CFG_UART1OUTPROT_NMEA);        /* 0 = off */
        (void)gnss_cfgValsetU1(1u           , CFG_MSGOUT_UBX_NAV_PVT_UART1); /* 1/epoch */
        (void)gnss_cfgValsetU2(UBX_MEAS_RATE, CFG_RATE_MEAS);
        (void)gnss_cfgValsetU2(1u           , CFG_RATE_NAV);
        (void)gnss_cfgValsetU1(UBX_DYNMODEL_AIR1, CFG_NAVSPG_DYNMODEL);
        gsv_cfg_sent = TRUE;
    }

    /* read from ring buffer */
    uint16 head = g_ring_head;
    while (g_ring_tail != head)
    {
        uint8 byte = g_ring_buffer[g_ring_tail];


       /*  decode statemachine for NMEA & UBX */
        switch (parse_state)
        {
            case GNSS_IDLE:
                g_len = 0;
                if ( (last_byte == 0xB5u) &&
                     (byte      == 0x62u) )
                {
                    parse_state = GNSS_UBX;
                }

                break;
            case GNSS_UBX:
                g_detect_ack++;
                g_buffer[g_len] = byte;
                g_len++;

                if (g_len > 3u){
                    /* ubx_rd_u16 widens each byte before shifting; the
                     * hand-rolled version shifted a uint8 by 8, past its
                     * own width (MISRA 12.2). */
                    g_ubx_payload_len = ubx_rd_u16(&g_buffer[2]);
                    if (g_ubx_payload_len > GNSS_UBX_MAX_RX_PAYLOAD){
                        parse_state = GNSS_IDLE;
                    }

                    if (g_len >= (g_ubx_payload_len + 6u))
                    {
                        if (gnss_ubx_decode(g_buffer, g_len, g_ubx_payload_len, &g_nav) != TRUE)
                        {
                            g_errors++;
                        }
                        parse_state = GNSS_IDLE;
                    }
                }

                break;
            default:
                break;
        }
        last_byte = byte;

        g_ring_tail++;
        if (g_ring_tail >= RING_BUFFER_SIZE)
        {
            g_ring_tail = 0u;
        }
        g_poll_counter = 0u;


    }
    sample->rxBytes   = g_bytes;
    sample->sentences = g_sentences;
    sample->errors    = g_errors;
    sample->fixType   = g_nav.fixQuality;
    sample->numSats   = g_nav.numSats;

    /* Scale once, here, so every consumer sees the same units. Copied on
     * EVERY call, not only when a frame arrived -- NAV-PVT lands at 1 Hz
     * while this runs at 100 ms, so g_nav is the last-known value. */
    /* cfgOk: every configuration command we sent came back acknowledged.
     * FALSE before the send too -- an unconfigured receiver is not "OK". */
    sample->cfgOk      = ((gsv_cfg_sent != FALSE) &&
                          (g_ubx_ack >= g_cfg_expected_acks)) ? 1u : 0u;
    sample->fixOk      = g_nav.fixOk;
    sample->timeOk     = g_nav.timeOk;
    sample->navOk      = g_nav.navOk;
    sample->latDeg     = (float32)g_nav.lat     * 1.0e-7f;
    sample->lonDeg     = (float32)g_nav.lon     * 1.0e-7f;
    sample->altM       = (float32)g_nav.hMSL    * 1.0e-3f;
    sample->speedMps   = (float32)g_nav.gSpeed  * 1.0e-3f;
    sample->headingDeg = (float32)g_nav.headMot * 1.0e-5f;
    sample->hAccM      = (float32)g_nav.hAcc    * 1.0e-3f;
    sample->vAccM      = (float32)g_nav.vAcc    * 1.0e-3f;
    sample->iTOW       = g_nav.iTOW;
    sample->latRaw     = g_nav.lat;
    sample->lonRaw     = g_nav.lon;
    sample->year      = g_nav.year;
    sample->month     = g_nav.month;
    sample->day       = g_nav.day;
    sample->hour      = g_nav.hour;
    sample->min       = g_nav.min;
    sample->sec       = g_nav.sec;

    return status;
}
