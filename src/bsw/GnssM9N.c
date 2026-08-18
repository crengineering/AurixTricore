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
/* local used variables */
static IfxAsclin_Asc  g_asclin;

static char           g_buffer[GNSSM9N_BUFFER_SIZE];
static uint8          g_len         = 0u;
static uint16         g_errors      = 0u;
static uint32         g_bytes       = 0u;
static uint32         g_sentences   = 0u;
static boolean        g_timeout     = TRUE;
static uint16         g_poll_counter = GNSS_NOT_PRESENT_TICKS;

static volatile uint32 g_isrCount;
static volatile uint32 g_isrBytes = 0u;
/* local functions */
void asclin4IsrReceive(void);

IFX_INTERRUPT(asclin4IsrReceive, 0, ISR_PRIORITY_ASCLIN4_RX);

/* cppcheck-suppress misra-c2012-8.7 ; deviation: referenced by the interrupt
 * vector table, not by C code; static would break the vector entry. */
void asclin4IsrReceive(void)
{
    uint8 fill_level = IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN4);
    g_isrCount++;
    for (uint8 i=0; i<fill_level; i++)
    {
        (void)(IfxAsclin_readRxData(&MODULE_ASCLIN4) & 0xFFu);
        g_isrBytes++;
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
 * GNSS-NEO-M9N poll function
 * Function to poll GNSS bytes from the Asclin hardware FIFO Buffer and build the messages
 * Information on the GNSS protocol:
 * - Burst mode 1 Hz -> 3840 Bytes/s = 4 Bytes/ms
 * - emitts GGA, GLL, GSA, GSV, RMC & VTG ~ 500-700 bytes every second
 * - bytes end with <CR> (\r), <LF> (\n)
 */
void GnssM9N_poll (void){
    /* check if a Fifo Overflow occured */
    if (IfxAsclin_getRxFifoOverflowFlagStatus(&MODULE_ASCLIN4) != FALSE)
    {
        IfxAsclin_clearRxFifoOverflowFlag(&MODULE_ASCLIN4);
        g_errors++;
    }

    /* check if a frame error is present */
    if (IfxAsclin_getFrameErrorFlagStatus(&MODULE_ASCLIN4) != FALSE)
    {
        IfxAsclin_clearFrameErrorFlag(&MODULE_ASCLIN4);
        g_errors++;
    }

    /* check if any bytes in the Fifo buffer and empty it if available */
    while (IfxAsclin_getRxFifoFillLevel(&MODULE_ASCLIN4) > 0u){
        /* pop fifo buffer byte */
        char fifo_byte = (char)(IfxAsclin_readRxData(&MODULE_ASCLIN4) & 0xFFu);
        g_bytes++;
        if (fifo_byte == ('\r') || fifo_byte == ('\n'))
        {
            if (g_len > 0u)
            {
                g_buffer[g_len] = '\0';
/*
                if ( (g_buffer[1] == 'G') &&
                     (g_buffer[2] == 'N') &&
                     (g_buffer[3] == 'G') &&
                     (g_buffer[4] == 'S') &&
                     (g_buffer[5] == 'A') )
                {
                    Uart_println(g_buffer);
                    //g_num_sats = (uint8)g_buffer[8];
                }
*/
                g_len = 0u;
                g_sentences++;
            }
        }
        else if (g_len < GNSSM9N_BUFFER_SIZE - 1u)
        {
            g_buffer[g_len] = fifo_byte;
            g_len++;

        }
        else
        {
            g_len = 0u;
        }
        g_poll_counter = 0u;
    }

    if (g_poll_counter >= GNSS_NOT_PRESENT_TICKS)
    {
        g_timeout = TRUE;
    }
    else
    {
        g_timeout = FALSE;
        g_poll_counter ++;
    }
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

    //sample->rxBytes   = g_bytes;
    //sample->sentences = g_sentences;
    sample->rxBytes   = g_isrCount;
    sample->sentences = g_isrBytes;
    sample->errors    = g_errors;
    sample->fixType   = 0u;
    sample->numSats   = 0u;

    return status;
}
