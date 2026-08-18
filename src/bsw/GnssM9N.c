/**********************************************************************************************************************
 * \file GnssM9N.c
 * \brief
 *********************************************************************************************************************/
#include "GnssM9N.h"
#include "Uart.h"
#include "IfxAsclin_Asc.h"
#include "IfxAsclin.h"
#include "ConfigurationIsr.h"

/* local used defines */
#define GNSSM9N_BUFFER_SIZE    96u
#define GNSS_POLL_PERIOD_MS    1u       /* must match the task calling GnssM9N_poll */
#define GNSS_TIMEOUT_MS        2000u    /* > the ~850 ms gap between 1 Hz bursts */
#define GNSS_NOT_PRESENT_TICKS (GNSS_TIMEOUT_MS / GNSS_POLL_PERIOD_MS)
#define RING_BUFFER_SIZE 512u
/* local used variables */
static IfxAsclin_Asc  g_asclin;

static char           g_buffer[GNSSM9N_BUFFER_SIZE];
static uint8          g_len         = 0u;
static uint16         g_errors      = 0u;
static volatile uint32         g_bytes       = 0u;
static uint32         g_sentences   = 0u;
static boolean        g_timeout     = TRUE;
static uint16         g_poll_counter = GNSS_NOT_PRESENT_TICKS;
static uint8          g_ring_buf_overflow_counter = 0u;
static GnssM9N_Nav           g_nav;

/* isr variables */
static volatile char   g_ring_buffer[RING_BUFFER_SIZE];
static volatile uint16 g_ring_head = 0u;
static volatile uint16 g_ring_tail = 0u;

/* local functions */
static boolean GnssM9N_decode(const char *buffer, uint8 buffer_len, GnssM9N_Nav *Nav);
void asclin4IsrReceive(void);

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
    }

    return (IfxAsclin_getClockSource(&MODULE_ASCLIN4) == IfxAsclin_ClockSource_ascFastClock);
}

/*
 * Function that reads the information provided by GNSS M9N every 100ms
 */
boolean GnssM9N_read(GnssM9N_Sample *sample){
    boolean status    = FALSE;

    if (g_timeout != TRUE)
    {
        status = TRUE;
    }
    status = TRUE;

    /* read from ring buffer */
    uint16 head = g_ring_head;
    while (g_ring_tail != head)
    {
        char byte = g_ring_buffer[g_ring_tail];
        if (byte == ('\r') || byte == ('\n'))
        {
            if (g_len > 0u)
            {
                g_buffer[g_len] = '\0';
                GnssM9N_decode(g_buffer, g_len, &g_nav);
                g_len = 0u;
                g_sentences++;

            }
        }
        else if (g_len < GNSSM9N_BUFFER_SIZE - 1u)
        {
            g_buffer[g_len] = byte;
            g_len++;

        }
        else
        {
            g_len = 0u;
        }

        g_ring_tail++;
        if (g_ring_tail >= RING_BUFFER_SIZE)
        {
            g_ring_tail = 0u;
        }

    }

    sample->rxBytes   = g_bytes;
    sample->sentences = g_sentences;
    sample->errors    = g_errors;
    sample->fixType   = 0u;
    sample->numSats   = g_nav.numSats;

    return status;
}

static boolean GnssM9N_decode(const char *buffer, uint8 buffer_len, GnssM9N_Nav *Nav){
    boolean decoding_status = FALSE;
    gnss_type_t message_type = NONE;
    uint8 count_comma = 0u;
    uint8 numSats = 0u;
    boolean digit_read = FALSE;

    if (buffer_len < 6u)
    {
        return FALSE;
    }

    if ( (buffer[1] == 'G') &&
         (buffer[2] == 'N') &&
         (buffer[3] == 'G') &&
         (buffer[4] == 'G') &&
         (buffer[5] == 'A') )
    {
        message_type = GNGGA;
    }

    switch (message_type)
    {
        case GNGGA:
        {
            // detect number of satellites
            for (uint8 i=6u; i < buffer_len; i++)
            {
                if ((count_comma ==  7u) &&
                    (buffer[i] >= '0') &&
                    (buffer[i] <= '9'))
                {
                    numSats = numSats*10 + buffer[i] - '0';
                    digit_read = TRUE;
                }
                if (buffer[i] == ',')
                {
                    count_comma++;
                }

            }

            if (numSats >= 32)
            {
                decoding_status = FALSE;

            }
            else if (digit_read == TRUE)
            {
                Nav->numSats = numSats;
                decoding_status = TRUE;
            }
            break;
        }
        default:
        {
            decoding_status = FALSE;
            break;
        }
    }



    return decoding_status;
}
