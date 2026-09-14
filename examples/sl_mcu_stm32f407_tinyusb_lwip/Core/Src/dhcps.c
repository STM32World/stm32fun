#include "dhcps.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
#include <string.h>

#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68

#define DHCP_DISCOVER 1
#define DHCP_OFFER 2
#define DHCP_REQUEST 3
#define DHCP_ACK 5

#define DHCP_OPTION_SUBNET_MASK 1
#define DHCP_OPTION_ROUTER 3
#define DHCP_OPTION_LEASE_TIME 51
#define DHCP_OPTION_MSG_TYPE 53
#define DHCP_OPTION_SERVER_ID 54
#define DHCP_OPTION_END 255

struct dhcp_msg {
    uint8_t op, htype, hlen, hops;
    uint32_t xid;
    uint16_t secs, flags;
    ip4_addr_t ciaddr, yiaddr, siaddr, giaddr;
    uint8_t chaddr[16];
    uint8_t sname[64];
    uint8_t file[128];
    uint32_t magic_cookie;
    uint8_t options[308];
} __attribute__((packed));

static struct udp_pcb *dhcps_pcb;
static ip4_addr_t server_ip;
static ip4_addr_t netmask_ip;
static ip4_addr_t offered_ip;

static void dhcps_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (!p || p->len < sizeof(struct dhcp_msg) - 308) {
        if (p)
            pbuf_free(p);
        return;
    }

    struct dhcp_msg *req = (struct dhcp_msg *)p->payload;

    // Check DHCP Magic Cookie (0x63825363 in network byte order)
    if (req->magic_cookie != PP_HTONL(0x63825363)) {
        pbuf_free(p);
        return;
    }

    // Locate DHCP Message Type
    uint8_t msg_type = 0;
    uint8_t *opt = req->options;
    uint8_t *opt_end = (uint8_t *)p->payload + p->len;

    while (opt < opt_end && *opt != DHCP_OPTION_END) {
        if (*opt == DHCP_OPTION_MSG_TYPE) {
            msg_type = opt[2];
            break;
        }
        if (*opt == 0) {
            opt++;
            continue;
        }
        opt += opt[1] + 2;
    }

    if (msg_type == DHCP_DISCOVER || msg_type == DHCP_REQUEST) {
        struct pbuf *reply_pbuf = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct dhcp_msg), PBUF_RAM);
        if (reply_pbuf) {
            struct dhcp_msg *rep = (struct dhcp_msg *)reply_pbuf->payload;
            memset(rep, 0, sizeof(struct dhcp_msg));

            rep->op = 2; // Reply
            rep->htype = req->htype;
            rep->hlen = req->hlen;
            rep->xid = req->xid;
            rep->flags = req->flags;

            // Assign 192.168.7.2 to the client
            IP4_ADDR(&offered_ip, ip4_addr1(&server_ip), ip4_addr2(&server_ip), ip4_addr3(&server_ip), 2);
            rep->yiaddr = offered_ip;
            rep->siaddr = server_ip;
            memcpy(rep->chaddr, req->chaddr, 16);
            rep->magic_cookie = PP_HTONL(0x63825363);

            // Construct DHCP Options
            uint8_t *o = rep->options;

            *o++ = DHCP_OPTION_MSG_TYPE;
            *o++ = 1;
            *o++ = (msg_type == DHCP_DISCOVER) ? DHCP_OFFER : DHCP_ACK;

            *o++ = DHCP_OPTION_SERVER_ID;
            *o++ = 4;
            memcpy(o, &server_ip.addr, 4);
            o += 4;

            *o++ = DHCP_OPTION_LEASE_TIME;
            *o++ = 4;
            uint32_t lease = PP_HTONL(86400); // 1 day lease
            memcpy(o, &lease, 4);
            o += 4;

            *o++ = DHCP_OPTION_SUBNET_MASK;
            *o++ = 4;
            memcpy(o, &netmask_ip.addr, 4);
            o += 4;

            *o++ = DHCP_OPTION_ROUTER;
            *o++ = 4;
            memcpy(o, &server_ip.addr, 4);
            o += 4;

            *o++ = DHCP_OPTION_END;

            // Broadcast response to client port 68
            udp_sendto(pcb, reply_pbuf, IP_ADDR_BROADCAST, DHCP_CLIENT_PORT);
            pbuf_free(reply_pbuf);
        }
    }

    pbuf_free(p);
}

void dhcps_init(ip4_addr_t *ipaddr, ip4_addr_t *netmask) {
    server_ip = *ipaddr;
    netmask_ip = *netmask;

    dhcps_pcb = udp_new();
    if (dhcps_pcb) {
        udp_bind(dhcps_pcb, IP_ADDR_ANY, DHCP_SERVER_PORT);
        udp_recv(dhcps_pcb, dhcps_recv, NULL);
    }
}
