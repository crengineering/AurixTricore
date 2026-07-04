#include "Xcp.h"
#include "Version.h"
#include "Diagnostics.h"
#include "Nvm.h"
#include "Ifx_Types.h"
#include "lwip/udp.h"
#include <string.h>

#define XCP_PORT                    5555u
#define XCP_MAX_CTO                 64u     /* max command/response packet size  */
#define XCP_MAX_DTO                 64u

/* command codes (master -> slave) */
#define XCP_CMD_CONNECT             0xFFu
#define XCP_CMD_DISCONNECT          0xFEu
#define XCP_CMD_GET_STATUS          0xFDu
#define XCP_CMD_SYNCH               0xFCu
#define XCP_CMD_GET_COMM_MODE_INFO  0xFBu
#define XCP_CMD_GET_ID              0xFAu
#define XCP_CMD_SET_MTA             0xF6u
#define XCP_CMD_UPLOAD              0xF5u
#define XCP_CMD_SHORT_UPLOAD        0xF4u
#define XCP_CMD_DOWNLOAD            0xF0u
#define XCP_CMD_SHORT_DOWNLOAD      0xEDu

/* DAQ commands */
#define XCP_CMD_SET_DAQ_PTR         0xE2u
#define XCP_CMD_WRITE_DAQ           0xE1u
#define XCP_CMD_SET_DAQ_LIST_MODE   0xE0u
#define XCP_CMD_START_STOP_DAQ_LIST 0xDEu
#define XCP_CMD_START_STOP_SYNCH    0xDDu
#define XCP_CMD_GET_DAQ_PROC_INFO   0xDAu
#define XCP_CMD_GET_DAQ_RES_INFO    0xD9u
#define XCP_CMD_FREE_DAQ            0xD6u
#define XCP_CMD_ALLOC_DAQ           0xD5u
#define XCP_CMD_ALLOC_ODT           0xD4u
#define XCP_CMD_ALLOC_ODT_ENTRY     0xD3u

/* static DAQ resources: one list on event channel 0 (the 100 ms task) */
#define XCP_DAQ_MAX_ODTS            4u
#define XCP_DAQ_MAX_ENTRIES         8u
#define XCP_DAQ_MAX_ODT_DATA        63u     /* MAX_DTO - PID byte            */

/* packet identifiers (slave -> master) */
#define XCP_PID_RES                 0xFFu   /* positive response                 */
#define XCP_PID_ERR                 0xFEu   /* error packet                      */

/* error codes */
#define XCP_ERR_CMD_SYNCH           0x00u
#define XCP_ERR_CMD_UNKNOWN         0x20u
#define XCP_ERR_OUT_OF_RANGE        0x22u
#define XCP_ERR_WRITE_PROTECTED     0x25u

/* Writes are only allowed inside the calibration block or the persistent
 * parameter block (each skipping its magic word, which only the slave
 * itself may set) — protects the rest of RAM. */
static boolean xcpWriteAllowed(uint32 addr, uint32 len)
{
    boolean inCal = (boolean)((addr >= (XCP_CAL_ADDR + 4u))
                              && ((addr + len) <= (XCP_CAL_ADDR + XCP_CAL_SIZE)));
    boolean inNvm = (boolean)((addr >= (XCP_NVM_ADDR + 4u))
                              && ((addr + len) <= (XCP_NVM_ADDR + XCP_NVM_SIZE)));

    return (boolean)((inCal != FALSE) || (inNvm != FALSE));
}

/* identification string returned via GET_ID + UPLOAD */
static const char s_xcpIdent[] = "AurixTricore v" SW_VERSION_STRING;

static struct udp_pcb *s_xcpPcb;
static uint16          s_resCtr;        /* response counter (transport header)  */
static uint8          *s_mta;           /* memory transfer address              */
static boolean         s_connected;

/* DAQ state (single dynamic list, absolute ODT numbering) */
typedef struct
{
    uint8 *addr;
    uint8  size;
} Xcp_OdtEntry;

static Xcp_OdtEntry s_daqEntries[XCP_DAQ_MAX_ODTS][XCP_DAQ_MAX_ENTRIES];
static uint8        s_daqEntryCount[XCP_DAQ_MAX_ODTS];   /* allocated per ODT */
static uint8        s_daqOdtCount;
static boolean      s_daqAllocated;
static volatile boolean s_daqRunning;
static uint8        s_daqPtrOdt;
static uint8        s_daqPtrEntry;
static ip_addr_t    s_masterAddr;       /* DTO destination (last CONNECT src)   */
static u16_t        s_masterPort;

static void xcpDaqFree(void)
{
    uint32 i;

    s_daqRunning   = FALSE;
    s_daqAllocated = FALSE;
    s_daqOdtCount  = 0u;
    s_daqPtrOdt    = 0u;
    s_daqPtrEntry  = 0u;

    for (i = 0u; i < XCP_DAQ_MAX_ODTS; i++)
    {
        s_daqEntryCount[i] = 0u;
    }
}

/* Send one XCP packet with the 4-byte Ethernet transport header in front */
static void xcpSend(const ip_addr_t *addr, u16_t port, const uint8 *packet, uint16 len)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)(len + 4u), PBUF_RAM);

    if (p != NULL)
    {
        uint8 *d = (uint8 *)p->payload;
        d[0] = (uint8)(len & 0xFFu);
        d[1] = (uint8)(len >> 8);
        d[2] = (uint8)(s_resCtr & 0xFFu);
        d[3] = (uint8)(s_resCtr >> 8);
        s_resCtr++;
        memcpy(&d[4], packet, len);

        udp_sendto(s_xcpPcb, p, addr, port);
        pbuf_free(p);
    }
}

static void xcpSendError(const ip_addr_t *addr, u16_t port, uint8 errCode)
{
    uint8 resp[2] = { XCP_PID_ERR, errCode };
    xcpSend(addr, port, resp, 2u);
}

static void xcpRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                    const ip_addr_t *addr, u16_t port)
{
    uint8  frame[4u + XCP_MAX_CTO];
    uint8  resp[XCP_MAX_CTO];
    uint16 frameLen;
    uint16 pktLen;
    const uint8 *cmd;

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(pcb);

    if (p == NULL)
    {
        return;
    }

    frameLen = (uint16)LWIP_MIN(p->tot_len, sizeof(frame));
    pbuf_copy_partial(p, frame, frameLen, 0);
    pbuf_free(p);

    if (frameLen < 5u)                          /* header + at least the PID    */
    {
        return;
    }

    pktLen = (uint16)(frame[0] | ((uint16)frame[1] << 8));
    if ((pktLen == 0u) || ((uint16)(pktLen + 4u) > frameLen))
    {
        return;                                 /* malformed frame              */
    }
    cmd = &frame[4];

    switch (cmd[0])
    {
        case XCP_CMD_CONNECT:
            s_connected  = TRUE;
            s_masterAddr = *addr;               /* DTO destination for DAQ      */
            s_masterPort = port;
            xcpDaqFree();
            resp[0] = XCP_PID_RES;
            resp[1] = 0x05u;                    /* RESOURCE: CAL/PAG + DAQ      */
            resp[2] = 0x00u;                    /* COMM_MODE_BASIC: Intel, byte */
            resp[3] = XCP_MAX_CTO;
            resp[4] = (uint8)(XCP_MAX_DTO & 0xFFu);
            resp[5] = (uint8)(XCP_MAX_DTO >> 8);
            resp[6] = 0x01u;                    /* protocol layer version       */
            resp[7] = 0x01u;                    /* transport layer version      */
            xcpSend(addr, port, resp, 8u);
            break;

        case XCP_CMD_DISCONNECT:
            s_connected = FALSE;
            xcpDaqFree();
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_GET_STATUS:
            resp[0] = XCP_PID_RES;
            resp[1] = 0x00u;                    /* session status               */
            resp[2] = 0x00u;                    /* resource protection          */
            resp[3] = 0x00u;                    /* reserved                     */
            resp[4] = 0x00u;                    /* session config id (2 bytes)  */
            resp[5] = 0x00u;
            xcpSend(addr, port, resp, 6u);
            break;

        case XCP_CMD_SYNCH:
            xcpSendError(addr, port, XCP_ERR_CMD_SYNCH);
            break;

        case XCP_CMD_GET_COMM_MODE_INFO:
            resp[0] = XCP_PID_RES;
            resp[1] = 0x00u;                    /* reserved                     */
            resp[2] = 0x00u;                    /* COMM_MODE_OPTIONAL           */
            resp[3] = 0x00u;                    /* reserved                     */
            resp[4] = 0x00u;                    /* MAX_BS                       */
            resp[5] = 0x00u;                    /* MIN_ST                       */
            resp[6] = 0x00u;                    /* QUEUE_SIZE                   */
            resp[7] = 0x10u;                    /* XCP driver version 1.0       */
            xcpSend(addr, port, resp, 8u);
            break;

        case XCP_CMD_GET_ID:                    /* ident readable via UPLOAD    */
        {
            uint32 identLen = (uint32)(sizeof(s_xcpIdent) - 1u);
            s_mta   = (uint8 *)s_xcpIdent;      /* master UPLOADs from MTA      */
            resp[0] = XCP_PID_RES;
            resp[1] = 0x00u;                    /* mode: data via UPLOAD        */
            resp[2] = 0x00u;                    /* reserved                     */
            resp[3] = 0x00u;                    /* reserved                     */
            resp[4] = (uint8)(identLen & 0xFFu);
            resp[5] = (uint8)((identLen >> 8) & 0xFFu);
            resp[6] = (uint8)((identLen >> 16) & 0xFFu);
            resp[7] = (uint8)((identLen >> 24) & 0xFFu);
            xcpSend(addr, port, resp, 8u);
            break;
        }

        case XCP_CMD_SET_MTA:                   /* addr ext cmd[3], addr cmd[4..7] LE */
            if (pktLen < 8u)
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_mta = (uint8 *)(  (uint32)cmd[4]
                              | ((uint32)cmd[5] << 8)
                              | ((uint32)cmd[6] << 16)
                              | ((uint32)cmd[7] << 24));
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_UPLOAD:                    /* n = cmd[1], read from MTA    */
        {
            uint8 n = cmd[1];
            if ((n == 0u) || (n > (XCP_MAX_CTO - 1u)) || (s_mta == NULL_PTR))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            resp[0] = XCP_PID_RES;
            memcpy(&resp[1], s_mta, n);
            s_mta += n;
            xcpSend(addr, port, resp, (uint16)(n + 1u));
            break;
        }

        case XCP_CMD_SHORT_UPLOAD:              /* n cmd[1], ext cmd[3], addr cmd[4..7] */
        {
            uint8 n = cmd[1];
            if ((pktLen < 8u) || (n == 0u) || (n > (XCP_MAX_CTO - 1u)))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_mta = (uint8 *)(  (uint32)cmd[4]
                              | ((uint32)cmd[5] << 8)
                              | ((uint32)cmd[6] << 16)
                              | ((uint32)cmd[7] << 24));
            resp[0] = XCP_PID_RES;
            memcpy(&resp[1], s_mta, n);
            s_mta += n;
            xcpSend(addr, port, resp, (uint16)(n + 1u));
            break;
        }

        case XCP_CMD_DOWNLOAD:                  /* n = cmd[1], data from cmd[2], write at MTA */
        {
            uint8 n = cmd[1];
            if ((n == 0u) || ((uint16)(n + 2u) > pktLen))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            if (!xcpWriteAllowed((uint32)s_mta, n))
            {
                xcpSendError(addr, port, XCP_ERR_WRITE_PROTECTED);
                break;
            }
            memcpy(s_mta, &cmd[2], n);
            s_mta  += n;
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;
        }

        case XCP_CMD_SHORT_DOWNLOAD:            /* n cmd[1], ext cmd[3], addr cmd[4..7], data cmd[8..] */
        {
            uint8  n = cmd[1];
            uint32 wrAddr;
            if ((pktLen < 9u) || (n == 0u) || ((uint16)(n + 8u) > pktLen))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            wrAddr = (uint32)cmd[4]
                     | ((uint32)cmd[5] << 8)
                     | ((uint32)cmd[6] << 16)
                     | ((uint32)cmd[7] << 24);
            if (!xcpWriteAllowed(wrAddr, n))
            {
                xcpSendError(addr, port, XCP_ERR_WRITE_PROTECTED);
                break;
            }
            memcpy((uint8 *)wrAddr, &cmd[8], n);
            s_mta   = (uint8 *)(wrAddr + n);
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;
        }

        case XCP_CMD_FREE_DAQ:
            xcpDaqFree();
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_ALLOC_DAQ:                 /* word cmd[2..3] = daq count   */
        {
            uint16 daqCount = (uint16)(cmd[2] | ((uint16)cmd[3] << 8));
            if (daqCount != 1u)                 /* exactly one list supported   */
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqAllocated = TRUE;
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;
        }

        case XCP_CMD_ALLOC_ODT:                 /* word daq, byte cmd[4] = odts */
            if (!s_daqAllocated || (cmd[4] == 0u) || (cmd[4] > XCP_DAQ_MAX_ODTS))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqOdtCount = cmd[4];
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_ALLOC_ODT_ENTRY:           /* byte cmd[4]=odt, cmd[5]=n    */
            if (!s_daqAllocated || (cmd[4] >= s_daqOdtCount)
                || (cmd[5] == 0u) || (cmd[5] > XCP_DAQ_MAX_ENTRIES))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqEntryCount[cmd[4]] = cmd[5];
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_SET_DAQ_PTR:               /* word daq, byte odt, byte entry */
            if (!s_daqAllocated || (cmd[4] >= s_daqOdtCount)
                || (cmd[5] >= s_daqEntryCount[cmd[4]]))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqPtrOdt   = cmd[4];
            s_daqPtrEntry = cmd[5];
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_WRITE_DAQ:                 /* size cmd[2], addr cmd[4..7]  */
            if (!s_daqAllocated || (cmd[2] == 0u) || (cmd[2] > XCP_DAQ_MAX_ODT_DATA))
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqEntries[s_daqPtrOdt][s_daqPtrEntry].size = cmd[2];
            s_daqEntries[s_daqPtrOdt][s_daqPtrEntry].addr =
                (uint8 *)(  (uint32)cmd[4]
                          | ((uint32)cmd[5] << 8)
                          | ((uint32)cmd[6] << 16)
                          | ((uint32)cmd[7] << 24));
            if (s_daqPtrEntry < (uint8)(XCP_DAQ_MAX_ENTRIES - 1u))
            {
                s_daqPtrEntry++;                /* auto-increment per spec      */
            }
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_SET_DAQ_LIST_MODE:         /* only event channel 0 exists  */
        {
            uint16 eventChannel = (uint16)(cmd[4] | ((uint16)cmd[5] << 8));
            if (eventChannel != 0u)
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;
        }

        case XCP_CMD_START_STOP_DAQ_LIST:       /* mode cmd[1]: 0 stop, 1 start */
            if (!s_daqAllocated)
            {
                xcpSendError(addr, port, XCP_ERR_OUT_OF_RANGE);
                break;
            }
            s_daqRunning = (boolean)(cmd[1] == 1u);
            resp[0] = XCP_PID_RES;
            resp[1] = 0x00u;                    /* FIRST_PID (absolute ODT 0)   */
            xcpSend(addr, port, resp, 2u);
            break;

        case XCP_CMD_START_STOP_SYNCH:          /* mode cmd[1]: 0 = stop all    */
            if (cmd[1] == 0u)
            {
                s_daqRunning = FALSE;
            }
            else if (s_daqAllocated)
            {
                s_daqRunning = TRUE;
            }
            resp[0] = XCP_PID_RES;
            xcpSend(addr, port, resp, 1u);
            break;

        case XCP_CMD_GET_DAQ_PROC_INFO:
            resp[0] = XCP_PID_RES;
            resp[1] = 0x01u;                    /* DAQ_CONFIG_TYPE: dynamic     */
            resp[2] = 0x01u;                    /* MAX_DAQ (2 bytes)            */
            resp[3] = 0x00u;
            resp[4] = 0x01u;                    /* MAX_EVENT_CHANNEL (2 bytes)  */
            resp[5] = 0x00u;
            resp[6] = 0x00u;                    /* MIN_DAQ                      */
            resp[7] = 0x00u;                    /* DAQ_KEY_BYTE: absolute ODT   */
            xcpSend(addr, port, resp, 8u);
            break;

        case XCP_CMD_GET_DAQ_RES_INFO:
            resp[0] = XCP_PID_RES;
            resp[1] = 1u;                       /* GRANULARITY_ODT_ENTRY_SIZE   */
            resp[2] = XCP_DAQ_MAX_ODT_DATA;     /* MAX_ODT_ENTRY_SIZE_DAQ       */
            resp[3] = 1u;                       /* GRANULARITY (STIM)           */
            resp[4] = 0u;                       /* MAX_ODT_ENTRY_SIZE_STIM      */
            resp[5] = 0x00u;                    /* TIMESTAMP_MODE: none         */
            resp[6] = 0x00u;                    /* TIMESTAMP_TICKS (2 bytes)    */
            resp[7] = 0x00u;
            xcpSend(addr, port, resp, 8u);
            break;

        default:
            xcpSendError(addr, port, XCP_ERR_CMD_UNKNOWN);
            break;
    }
}

/* Transmit all configured ODTs of the (single) DAQ list. Called from the
 * 100 ms task on CPU0 — the same context that runs the lwIP poll loop, so
 * calling into lwIP/udp_sendto here is safe (NO_SYS=1, single core). */
void xcpDaqCycle(void)
{
    uint8 odt;

    if (!s_daqRunning || (s_xcpPcb == NULL))
    {
        return;
    }

    for (odt = 0u; odt < s_daqOdtCount; odt++)
    {
        uint8  frame[1u + XCP_DAQ_MAX_ODT_DATA];
        uint16 len = 1u;
        uint8  e;

        frame[0] = odt;                          /* PID: absolute ODT number    */

        for (e = 0u; e < s_daqEntryCount[odt]; e++)
        {
            const Xcp_OdtEntry *entry = &s_daqEntries[odt][e];

            if ((entry->addr == NULL_PTR)
                || ((uint16)(len + entry->size) > sizeof(frame)))
            {
                continue;
            }
            memcpy(&frame[len], entry->addr, entry->size);
            len = (uint16)(len + entry->size);
        }

        if (len > 1u)
        {
            xcpSend(&s_masterAddr, s_masterPort, frame, len);
        }
    }
}

void xcpInit(void)
{
    s_xcpPcb = udp_new();

    if (s_xcpPcb != NULL)
    {
        if (udp_bind(s_xcpPcb, IP_ADDR_ANY, XCP_PORT) == ERR_OK)
        {
            udp_recv(s_xcpPcb, xcpRecv, NULL);
        }
        else
        {
            udp_remove(s_xcpPcb);
            s_xcpPcb = NULL;
        }
    }
}
