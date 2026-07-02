#include "UdpEcho.h"
#include "lwip/udp.h"

#define UDP_ECHO_PORT 7u

static void udpEchoRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port)
{
    LWIP_UNUSED_ARG(arg);

    if (p != NULL)
    {
        /* udp_sendto does not take ownership of the pbuf */
        udp_sendto(pcb, p, addr, port);
        pbuf_free(p);
    }
}

void udpEchoInit(void)
{
    struct udp_pcb *pcb = udp_new();

    if (pcb != NULL)
    {
        if (udp_bind(pcb, IP_ADDR_ANY, UDP_ECHO_PORT) == ERR_OK)
        {
            udp_recv(pcb, udpEchoRecv, NULL);
        }
        else
        {
            udp_remove(pcb);
        }
    }
}
