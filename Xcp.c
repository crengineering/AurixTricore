#include "Xcp.h"
#include "Version.h"
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

/* packet identifiers (slave -> master) */
#define XCP_PID_RES                 0xFFu   /* positive response                 */
#define XCP_PID_ERR                 0xFEu   /* error packet                      */

/* error codes */
#define XCP_ERR_CMD_SYNCH           0x00u
#define XCP_ERR_CMD_UNKNOWN         0x20u
#define XCP_ERR_OUT_OF_RANGE        0x22u

/* identification string returned via GET_ID + UPLOAD */
static const char s_xcpIdent[] = "AurixTricore v" SW_VERSION_STRING;

static struct udp_pcb *s_xcpPcb;
static uint16          s_resCtr;        /* response counter (transport header)  */
static uint8          *s_mta;           /* memory transfer address              */
static boolean         s_connected;

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
            s_connected = TRUE;
            resp[0] = XCP_PID_RES;
            resp[1] = 0x00u;                    /* RESOURCE: no CAL/DAQ/PGM     */
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

        default:
            xcpSendError(addr, port, XCP_ERR_CMD_UNKNOWN);
            break;
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
