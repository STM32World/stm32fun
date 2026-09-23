#ifndef _DHCPS_H_
#define _DHCPS_H_

#include "lwip/ip_addr.h"

void dhcps_init(ip4_addr_t *ipaddr, ip4_addr_t *netmask);

#endif
