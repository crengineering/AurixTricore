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
#define GNSS_TX_DEADLINE_TICKS 1000000

/* UBX */
#define GNSS_UBX_SYNC1         0xB5u
#define GNSS_UBX_SYNC2         0x62u
#define GNSS_UBX_MAX_PAYLOAD   32u
#define CFG_UART1OUTPROT_NMEA         0x10740002u
#define CFG_MSGOUT_UBX_NAV_PVT_UART1  0x20910007u
#define UBX_CFG_LAYER_RAM          0x01u
#define UBX_CLASS_CFG              0x06u
#define UBX_ID_CFG_VALSET          0x8Au
#define UBX_PAYLOAD_LEN            9u

#define UBX_NAV_PVT_YEAR           4u
#define UBX_NAV_PVT_MONTH          6u
#define UBX_NAV_PVT_DAY            7u
#define UBX_NAV_PVT_HOUR           8u
#define UBX_NAV_PVT_MIN            9u
#define UBX_NAV_PVT_SEC           10u

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


/* local used variables */
static IfxAsclin_Asc           g_asclin;

static uint8                   g_buffer[GNSSM9N_BUFFER_SIZE];
static uint8                   g_len         = 0u;
static uint16                  g_errors      = 0u;
static volatile uint32         g_bytes       = 0u;
static uint32                  g_sentences   = 0u;
static boolean                 g_timeout     = TRUE;
static uint16                  g_poll_counter = GNSS_NOT_PRESENT_TICKS;
static uint8                   g_ring_buf_overflow_counter = 0u;
static GnssM9N_Nav             g_nav;
static uint8                   g_tx_discards = 0u;

/* isr variables */
static volatile char           g_ring_buffer[RING_BUFFER_SIZE];
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


/* local functions */
void           asclin4IsrReceive   (void);
static boolean GnssM9N_timeout     (void);
static boolean gnss_txByte         (uint8 value);
static boolean gnss_sendUbx        (uint8 msgClass, uint8 msgId, const uint8 *payload, uint8 payload_len);
static boolean gnss_ubx_decode     (const uint8 *buffer, uint8 buffer_len, uint8 payload_len, GnssM9N_Nav *Nav);
static void    gnss_checksumUbx    (const uint8 *message, uint8 payload_len, uint8 *ck_a, uint8 *ck_b);

IFX_INTERRUPT(asclin4IsrReceive, 0, ISR_PRIORITY_ASCLIN4_RX);

/* cppcheck-suppress misra-c2012-8.7 ; deviation: referenced by the interrupt
 * vector table, not by C code; static would break the vector entry. */
void asclin4IsrReceive(void)
{
    uint8 fill_level = IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN4);
    for (uint8 i=0u; i<fill_level; i++)
    {
        char fifo_byte = (char)(IfxAsclin_readRxData(&MODULE_ASCLIN4) & 0xFFu);
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
    uint8 payload[UBX_PAYLOAD_LEN];
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

    status = gnss_sendUbx(UBX_CLASS_CFG, UBX_ID_CFG_VALSET, payload, UBX_PAYLOAD_LEN);

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

    if ( (payload_len > GNSS_UBX_MAX_PAYLOAD) ||
         (payload_len == 0u)            )
    {
        return FALSE;
    }

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

    Nav->iTOW       = ubx_rd_u32(&payload[UBX_NAV_PVT_ITOW]);
    Nav->year       = ubx_rd_u16(&payload[UBX_NAV_PVT_YEAR]);
    Nav->month      = payload[UBX_NAV_PVT_MONTH];
    Nav->day        = payload[UBX_NAV_PVT_DAY];
    Nav->hour       = payload[UBX_NAV_PVT_HOUR];
    Nav->min        = payload[UBX_NAV_PVT_MIN];
    Nav->sec        = payload[UBX_NAV_PVT_SEC];

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
}

static boolean gnss_ubx_decode (const uint8 *buffer, uint8 buffer_len, uint8 payload_len, GnssM9N_Nav *Nav){
    uint8 ck_a = 0u;
    uint8 ck_b = 0u;
    gnss_checksumUbx(buffer, 4u+payload_len, &ck_a, &ck_b);

    if ((buffer[buffer_len-1] != ck_b) ||
        (buffer[buffer_len-2] != ck_a) )
    {
        return FALSE;
    }

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

    return TRUE;
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

    status = IfxAsclin_Asc_initModule(&g_asclin, &config);

    if (status == IfxAsclin_Status_noError)
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
      g_timeout      = TRUE;
      g_poll_counter = GNSS_NOT_PRESENT_TICKS;

      /* reset TX to GNSS */
      gsv_cfg_sent = FALSE;
    }

    return (IfxAsclin_getClockSource(&MODULE_ASCLIN4) == IfxAsclin_ClockSource_ascFastClock);
}

static boolean GnssM9N_timeout(void)
{
    if (g_poll_counter >= GNSS_NOT_PRESENT_TICKS)
    {
        return TRUE;
    }
    else
    {
        g_poll_counter ++;
        return FALSE;
    }
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

    /* deactive unused sentences gsv_deactivated gsv_deactivate_increment */
    if ( (status == TRUE)           &&
         (gsv_cfg_sent == FALSE) )
    {
        (void)gnss_cfgValsetU1(1u, CFG_UART1OUTPROT_NMEA);
        (void)gnss_cfgValsetU1(1u, CFG_MSGOUT_UBX_NAV_PVT_UART1);
        gsv_cfg_sent = TRUE;
    }

    /* read from ring buffer */
    uint16 head = g_ring_head;
    while (g_ring_tail != head)
    {
        char byte = g_ring_buffer[g_ring_tail];


       /*  decode statemachine for NMEA & UBX */
        switch (parse_state)
        {
            case GNSS_IDLE:
                g_len = 0;
                if ( (last_byte   == 0xB5u) &&
                     ((uint8)byte == 0x62u) )
                {
                    parse_state = GNSS_UBX;
                }

                break;
            case GNSS_UBX:
                g_detect_ack++;
                g_buffer[g_len] = byte;
                g_len++;

                if (g_len > 3u){
                    g_ubx_payload_len = ((uint8)g_buffer[3]<<8) | ((uint8)(g_buffer[2]));
                    if (g_ubx_payload_len > 94){
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
    sample->fixOk      = g_nav.fixOk;
    sample->latDeg     = (float32)g_nav.lat     * 1.0e-7f;
    sample->lonDeg     = (float32)g_nav.lon     * 1.0e-7f;
    sample->altM       = (float32)g_nav.hMSL    * 1.0e-3f;
    sample->speedMps   = (float32)g_nav.gSpeed  * 1.0e-3f;
    sample->headingDeg = (float32)g_nav.headMot * 1.0e-5f;
    sample->hAccM      = (float32)g_nav.hAcc    * 1.0e-3f;
    sample->year      = g_nav.year;
    sample->month     = g_nav.month;
    sample->day       = g_nav.day;
    sample->hour      = g_nav.hour;
    sample->min       = g_nav.min;
    sample->sec       = g_nav.sec;

    return status;
}
