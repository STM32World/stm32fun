//*****************************************************************************
//
//! \file dhcpv6.c
//! \brief DHCPv6 APIs implement file.
//! \details Processig DHCPv6 protocol as SOLICIT, ADVERTISE.
//! \version 0.0.1
//! \date 2016/06/08
//! \par  Revision history
//!       <2016/07/18> 1st Release
//! \author JustinKim
//! \copyright
//!
//! Copyright (c)  2016, WIZnet Co., LTD.
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "dhcpv6.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#if (_WIZCHIP_ == W6100)

/* If you want to display debug & procssing message, Define _DHCP6_DEBUG_ in dhcpv6.h */
#define _DHCP6_DEBUG_
#ifdef _DHCP6_DEBUG_
#include <stdio.h>
#endif

/* DHCP6 state machine. */
#define STATE_DHCP6_INIT 0      ///< Initialize
#define STATE_DHCP6_SOLICIT 1   ///< send DISCOVER and wait OFFER
#define STATE_DHCP6_REQUEST 2   ///< send REQEUST and wait ACK or NACK
#define STATE_DHCP6_LEASED 3    ///< ReceiveD ACK and IP leased
#define STATE_DHCP6_REREQUEST 4 ///< send REQUEST for maintaining leased IP
#define STATE_DHCP6_RELEASE 5   ///< No use
#define STATE_DHCP6_STOP 6      ///< Stop procssing DHCPv6

/* DHCP6 message type */
#define DHCP6_SOLICIT 1       ///< DISCOVER message in OPT of @ref RIP_MSG
#define DHCP6_ADVERTISE 2     ///< OFFER message in OPT of @ref RIP_MSG
#define DHCP6_REQUEST 3       ///< REQUEST message in OPT of @ref RIP_MSG
#define DHCP6_CONFIRM 4       ///< DECLINE message in OPT of @ref RIP_MSG
#define DHCP6_RENEW 5         ///< ACK message in OPT of @ref RIP_MSG
#define DHCP6_REBIND 6        ///< NACK message in OPT of @ref RIP_MSG
#define DHCP6_REPLY 7         ///< RELEASE message in OPT of @ref RIP_MSG. No use
#define DHCP6_RELEASE 8       ///< INFORM message in OPT of @ref RIP_MSG. No use
#define DHCP6_DECLINE 9       ///< INFORM message in OPT of @ref RIP_MSG. No use
#define DHCP6_RECONFIGURE 10  ///< INFORM message in OPT of @ref RIP_MSG. No use
#define DHCP6_INFO_REQUEST 11 ///< INFORM message in OPT of @ref RIP_MSG. No use

uint8_t DNS6_Address[16] = {
    0,
};
//todo

/*
    @brief DHCPv6 option (cf. RFC3315)
*/
enum {
    OPT_CLIENTID = 1,
    OPT_SERVERID = 2,
    OPT_IANA = 3,
    OPT_IATA = 4,
    OPT_IAADDR = 5,
    OPT_REQUEST = 6,
    OPT_PREFERENCE = 7,
    OPT_ELAPSED_TIME = 8,
    OPT_RELAY_MSG = 9,
    OPT_AUTH = 11,
    OPT_UNICAST = 12,
    OPT_STATUS_CODE = 13,
    OPT_RAPID_COMMIT = 14,
    OPT_USER_CLASS = 15,
    OPT_VENDOR_CLASS = 16,
    OPT_VENDOR_OPTS = 17,
    OPT_INTERFACE_ID = 18,
    OPT_RECONF_MSG = 19,
    OPT_RECONF_ACCEPT = 20,
    SIP_Server_DNL = 21,
    SIP_Server_V6ADDR = 22,
    DNS_RecursiveNameServer = 23,
    Domain_Search_List = 24,
    OPT_IAPD = 25,
    OPT_IAPREFIX = 26,
    OPT_NIS_SERVERS = 27,
    OPT_NISP_SERVERS = 28,
    OPT_NIS_DOMAIN_NAME = 29,
    OPT_NISP_DOMAIN_NAME = 30,
    OPT_LIFETIME = 32,
    FQ_DOMAIN_NAME = 39
};

/*
    @brief DHCPv6 message format
*/
typedef struct {
    uint8_t *OPT; ///< Option
} __attribute__((packed)) RIP_MSG;

uint8_t DHCP6_SOCKET; // Socket number for DHCPv6

// Network information from DHCPv6 Server
uint8_t DHCP6_allocated_ip[16] = {
    0,
};                                    // IP address from DHCPv6
int8_t dhcp6_state = STATE_DHCP6_INIT; // DHCPv6 state
int8_t dhcp6_retry_count = 0;

volatile uint32_t dhcp6_tick_1s = 0; // unit 1 second
uint32_t dhcp6_tick_next = DHCP6_WAIT_TIME;

uint32_t DHCP6_XID; // Any number

RIP_MSG pDHCP6MSG; // Buffer pointer for DHCPv6 processing

uint8_t HOST_NAME6[] = DCHP6_HOST_NAME;

uint8_t DHCP6_CHADDR[6]; // DHCPv6 Client MAC address.


uint16_t DUID_type_s;
uint16_t Hardware_type_s;
uint8_t Time_s[4];
uint32_t Enterprise_num_s;
uint8_t Server_MAC[6];
uint8_t recv_IP[16];
uint32_t PreLifeTime;
uint32_t ValidLifeTime;
uint16_t code;
uint8_t IAID[4];
uint8_t T1[4];
uint8_t T2[4];
uint16_t iana_len;
uint16_t iaaddr_len;
uint16_t statuscode_len;
uint16_t Lstatuscode_len;
uint16_t serverid_len;
uint16_t clientid_len;
uint8_t status_msg[] = "";

unsigned size;
unsigned num;
unsigned num2 = 0;
unsigned growby;

/**
    @brief

    @param asize
    @param agrowby
*/
void InitDhcp6Option(unsigned asize, unsigned agrowby) {
    size = asize;
    growby = agrowby;
    num = 0;
}

/**
    @brief

    @param value
*/
void AppendDhcp6Option(uint8_t value) {
    uint32_t need;

    need = num + 1;
    if (need > size) {
        size++;
    }
    pDHCP6MSG.OPT[num] = value;
    num++;
}

/**
    @brief

    @param sMark
*/
void DumpDhcp6Option(char *sMark) {
    unsigned i;
    printf("%20s => size=%02d,num=%02d : ", sMark, size, num);
    for (i = num2; i < num; i++) {
        printf("%.2x ", pDHCP6MSG.OPT[i]);
    }
    printf("\r\n");
    num2 = num;
}

/**
    @brief

    @param option
*/
void DHCP6_Option_Select(uint8_t option) {
    switch (option)
    case OPT_CLIENTID:
case OPT_SERVERID:
case OPT_IANA:
case OPT_IATA:
case OPT_IAADDR:
case OPT_REQUEST:
case OPT_PREFERENCE:
case OPT_ELAPSED_TIME:
case OPT_RELAY_MSG:
case OPT_AUTH:
case OPT_UNICAST:
case OPT_STATUS_CODE:
case OPT_RAPID_COMMIT:
case OPT_USER_CLASS:
case OPT_VENDOR_CLASS:
case OPT_VENDOR_OPTS:
case OPT_INTERFACE_ID:
case OPT_RECONF_MSG:
case OPT_RECONF_ACCEPT:
case SIP_Server_DNL:
case SIP_Server_V6ADDR:
case DNS_RecursiveNameServer:
case Domain_Search_List:
case OPT_IAPD:
case OPT_IAPREFIX:
case OPT_NIS_SERVERS:
case OPT_NISP_SERVERS:
case OPT_NIS_DOMAIN_NAME:
case OPT_NISP_DOMAIN_NAME:
case FQ_DOMAIN_NAME:
    break;
}

/**
    @brief

*/
void send_DHCP6_SOLICIT(void) {
    uint16_t j;
    uint8_t ip[16];
    uint8_t rip_msg_size;

    size = 0;
    num = 0;
    growby = 0;

    InitDhcp6Option(34, 1);
    DumpDhcp6Option("option init");

    AppendDhcp6Option(DHCP6_SOLICIT);
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x00FF0000) >> 16));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x00FF0000) >> 8));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x00FF0000) >> 0));
    DumpDhcp6Option("Type&XID");

    // Elapsed time
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_ELAPSED_TIME);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x02);
    //    AppendDhcp6Option(0x0c);AppendDhcp6Option(0x1c);DumpDhcp6Option("Option Elapsed Time");

    // Client Identifier
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_CLIENTID);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x0a); //length
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x03); //DUID_Type
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x01); //Hard_Type

    AppendDhcp6Option(DHCP6_CHADDR[0]);
    AppendDhcp6Option(DHCP6_CHADDR[1]); // MAC Addr
    AppendDhcp6Option(DHCP6_CHADDR[2]);
    AppendDhcp6Option(DHCP6_CHADDR[3]);
    AppendDhcp6Option(DHCP6_CHADDR[4]);
    AppendDhcp6Option(DHCP6_CHADDR[5]);
    DumpDhcp6Option("Option Client ID");

    // Identity Association for Non-temporary Address
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_IANA);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x0c); // length
    AppendDhcp6Option(0x03);
    AppendDhcp6Option(0x00); // IAID
    AppendDhcp6Option(0x08);
    AppendDhcp6Option(0xdc);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x00); // T1
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x00); // T2
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x00);
    DumpDhcp6Option("Option IANA");

    // Fully Qualified Domain Name
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(39);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x06); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x04);
    //    AppendDhcp6Option(0x44);AppendDhcp6Option(0x45);
    //    AppendDhcp6Option(0x44);AppendDhcp6Option(0x59);DumpDhcp6Option("Option FQ Domain Name");

    // Vendor Class
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(16);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x0e); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x00);
    //    AppendDhcp6Option(0x01);AppendDhcp6Option(0x37);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x08);
    //    AppendDhcp6Option(0x4d);AppendDhcp6Option(0x53);
    //    AppendDhcp6Option(0x46);AppendDhcp6Option(0x54);
    //    AppendDhcp6Option(0x20);AppendDhcp6Option(0x35);
    //    AppendDhcp6Option(0x2e);AppendDhcp6Option(0x30);DumpDhcp6Option("Option Vendor Class");

    // Option Request
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_REQUEST);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x08); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_VENDOR_OPTS);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(DNS_RecursiveNameServer);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(Domain_Search_List);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(FQ_DOMAIN_NAME);DumpDhcp6Option("Option Request");

    // send broadcasting packet
    ip[0] = 0xff;
    ip[1] = 0x02;

    for (j = 2; j < 13; j++) {
        ip[j] = 0x00;
    }

    ip[13] = 0x01;
    ip[14] = 0x00;
    ip[15] = 0x02;

    rip_msg_size = size;

#ifdef _DHCP_DEBUG_
    printf("> Send DHCP_DISCOVER\r\n");
#endif

    sendto(DHCP6_SOCKET, (uint8_t *)pDHCP6MSG.OPT, rip_msg_size, ip, DHCP6_SERVER_PORT, 16);

#ifdef _DHCP_DEBUG_
    printf("> %d, %d\r\n", ret, rip_msg_size);
#endif

    //return ret
}

/**
    @brief

    @return uint8_t
*/
uint8_t send_DHCP6_REQUEST(void) {
    //uint16_t i;
    uint16_t j;
    uint8_t ip[16];
    uint8_t rip_msg_size;
    uint8_t ret = 1;

    size = 0;
    num = 0;
    growby = 0;

    if (iana_len == 0) {
        return 9;
    }
    printf("req : %x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x \r\n", recv_IP[0], recv_IP[1], recv_IP[2], recv_IP[3], recv_IP[4], recv_IP[5], recv_IP[6], recv_IP[7], recv_IP[8], recv_IP[9], recv_IP[10], recv_IP[11], recv_IP[12], recv_IP[13], recv_IP[14], recv_IP[15]);

    InitDhcp6Option(60, 1);
    DumpDhcp6Option("option init");

    AppendDhcp6Option(DHCP6_REQUEST);
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x00FF0000) >> 16));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x0000FF00) >> 8));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x000000FF) >> 0));
    DumpDhcp6Option("Type&XID");

    // Elapsed time
    //AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_ELAPSED_TIME);
    //AppendDhcp6Option(0x00);AppendDhcp6Option(0x02);
    //AppendDhcp6Option(0x0c);AppendDhcp6Option(0x1c);DumpDhcp6Option("Option Elapsed Time");

    // Identity Association for Non-temporary Address
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_IANA);
    AppendDhcp6Option((uint8_t)(iana_len >> 8));
    AppendDhcp6Option((uint8_t)iana_len); // length
    AppendDhcp6Option(IAID[0]);
    AppendDhcp6Option(IAID[1]); // IAID
    AppendDhcp6Option(IAID[2]);
    AppendDhcp6Option(IAID[3]);
    AppendDhcp6Option(T1[0]);
    AppendDhcp6Option(T1[1]); // T1
    AppendDhcp6Option(T1[2]);
    AppendDhcp6Option(T1[3]);
    AppendDhcp6Option(T2[0]);
    AppendDhcp6Option(T2[1]); // T2
    AppendDhcp6Option(T2[2]);
    AppendDhcp6Option(T2[3]);
    DumpDhcp6Option("Option IANA");

    // IA Address
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_IAADDR);
    AppendDhcp6Option((uint8_t)(iaaddr_len >> 8));
    AppendDhcp6Option((uint8_t)iaaddr_len); // length
    AppendDhcp6Option(recv_IP[0]);
    AppendDhcp6Option(recv_IP[1]); // IP
    AppendDhcp6Option(recv_IP[2]);
    AppendDhcp6Option(recv_IP[3]);
    AppendDhcp6Option(recv_IP[4]);
    AppendDhcp6Option(recv_IP[5]);
    AppendDhcp6Option(recv_IP[6]);
    AppendDhcp6Option(recv_IP[7]);
    AppendDhcp6Option(recv_IP[8]);
    AppendDhcp6Option(recv_IP[9]);
    AppendDhcp6Option(recv_IP[10]);
    AppendDhcp6Option(recv_IP[11]);
    AppendDhcp6Option(recv_IP[12]);
    AppendDhcp6Option(recv_IP[13]);
    AppendDhcp6Option(recv_IP[14]);
    AppendDhcp6Option(recv_IP[15]);
    AppendDhcp6Option((uint8_t)(PreLifeTime >> 24));
    AppendDhcp6Option((uint8_t)(PreLifeTime >> 16));
    AppendDhcp6Option((uint8_t)(PreLifeTime >> 8));
    AppendDhcp6Option((uint8_t)PreLifeTime);
    AppendDhcp6Option((uint8_t)(ValidLifeTime >> 24));
    AppendDhcp6Option((uint8_t)(ValidLifeTime >> 16));
    AppendDhcp6Option((uint8_t)(ValidLifeTime >> 8));
    AppendDhcp6Option((uint8_t)ValidLifeTime);
    DumpDhcp6Option("Option IA_addr");

    // Status code
#if 0
    // 20190318
    AppendDhcp6Option(0x00); AppendDhcp6Option(OPT_STATUS_CODE); DumpDhcp6Option("Option status_code type");
    AppendDhcp6Option((uint8_t)(Lstatuscode_len >> 8)); AppendDhcp6Option((uint8_t)Lstatuscode_len); DumpDhcp6Option("Option status_code length"); // length
    AppendDhcp6Option((uint8_t)(code >> 8)); AppendDhcp6Option((uint8_t)code); DumpDhcp6Option("Option status_code code"); // code
#endif
    //    for(i=0; i<(statuscode_len-2); i++)
    //        AppendDhcp6Option(status_msg[i]);
    //    DumpDhcp6Option("Option status_code msg");

#if 0
    // 20190318
    AppendDhcp6Option(0x41); AppendDhcp6Option(0x73);
    AppendDhcp6Option(0x73); AppendDhcp6Option(0x69);
    AppendDhcp6Option(0x67); AppendDhcp6Option(0x6e);
    AppendDhcp6Option(0x65); AppendDhcp6Option(0x64);
    AppendDhcp6Option(0x20); AppendDhcp6Option(0x61);
    AppendDhcp6Option(0x6e); AppendDhcp6Option(0x20);
    AppendDhcp6Option(0x61); AppendDhcp6Option(0x64);
    AppendDhcp6Option(0x64); AppendDhcp6Option(0x72);
    AppendDhcp6Option(0x65); AppendDhcp6Option(0x73);
    AppendDhcp6Option(0x73); AppendDhcp6Option(0x2e);
#endif

    // Client Identifier
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_CLIENTID);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x0a); //length
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x03); //DUID_Type
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x01); //Hard_Type

    AppendDhcp6Option(DHCP6_CHADDR[0]);
    AppendDhcp6Option(DHCP6_CHADDR[1]); // MAC Addr
    AppendDhcp6Option(DHCP6_CHADDR[2]);
    AppendDhcp6Option(DHCP6_CHADDR[3]);
    AppendDhcp6Option(DHCP6_CHADDR[4]);
    AppendDhcp6Option(DHCP6_CHADDR[5]);
    DumpDhcp6Option("Option Client ID");
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_CLIENTID);
    //    AppendDhcp6Option((uint8_t)(clientid_len>>8));AppendDhcp6Option((uint8_t)clientid_len); //length
    //    AppendDhcp6Option((uint8_t)(DUID_type>>8));AppendDhcp6Option((uint8_t)DUID_type); //DUID_Type
    //    AppendDhcp6Option((uint8_t)(Hardware_type>>8));AppendDhcp6Option((uint8_t)Hardware_type); //Hard_Type
    //    AppendDhcp6Option(DHCP_CHADDR[0]);AppendDhcp6Option(DHCP_CHADDR[1]); // MAC Addr
    //    AppendDhcp6Option(DHCP_CHADDR[2]);AppendDhcp6Option(DHCP_CHADDR[3]);
    //    AppendDhcp6Option(DHCP_CHADDR[4]);AppendDhcp6Option(DHCP_CHADDR[5]);DumpDhcp6Option("Option Client ID");

    //Server Identifier
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_SERVERID);
    AppendDhcp6Option((uint8_t)(serverid_len >> 8));
    AppendDhcp6Option((uint8_t)serverid_len); //length
    AppendDhcp6Option((uint8_t)(DUID_type_s >> 8));
    AppendDhcp6Option((uint8_t)DUID_type_s); //DUID_Type
    AppendDhcp6Option((uint8_t)(Hardware_type_s >> 8));
    AppendDhcp6Option((uint8_t)Hardware_type_s); //Hard_Type
#if 0
    // 20190318
    AppendDhcp6Option(Time_s[0]); AppendDhcp6Option(Time_s[1]); // Time
    AppendDhcp6Option(Time_s[2]); AppendDhcp6Option(Time_s[3]);
#endif
    AppendDhcp6Option(Server_MAC[0]);
    AppendDhcp6Option(Server_MAC[1]); // MAC Addr
    AppendDhcp6Option(Server_MAC[2]);
    AppendDhcp6Option(Server_MAC[3]);
    AppendDhcp6Option(Server_MAC[4]);
    AppendDhcp6Option(Server_MAC[5]);
    DumpDhcp6Option("Option Server ID");

    // Fully Qualified Domain Name
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(39);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x06); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x04);
    //    AppendDhcp6Option(0x44);AppendDhcp6Option(0x45);
    //    AppendDhcp6Option(0x44);AppendDhcp6Option(0x59);DumpDhcp6Option("Option FQ Domain Name");

    // Vendor Class
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(16);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x0e); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x00);
    //    AppendDhcp6Option(0x01);AppendDhcp6Option(0x37);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x08);
    //    AppendDhcp6Option(0x4d);AppendDhcp6Option(0x53);
    //    AppendDhcp6Option(0x46);AppendDhcp6Option(0x54);
    //    AppendDhcp6Option(0x20);AppendDhcp6Option(0x35);
    //    AppendDhcp6Option(0x2e);AppendDhcp6Option(0x30);DumpDhcp6Option("Option Vendor Class");

    // Option Request
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_REQUEST);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x08); // length
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_VENDOR_OPTS);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(DNS_RecursiveNameServer);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(Domain_Search_List);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(FQ_DOMAIN_NAME);DumpDhcp6Option("Option Request");

    // send broadcasting packet
    ip[0] = 0xff;
    ip[1] = 0x02;

    for (j = 2; j < 13; j++) {
        ip[j] = 0x00;
    }

    ip[13] = 0x01;
    ip[14] = 0x00;
    ip[15] = 0x02;

    rip_msg_size = size;

#ifdef _DHCP_DEBUG_
    printf("> Send DHCP_REQUEST\r\n");
#endif

    sendto(DHCP6_SOCKET, (uint8_t *)pDHCP6MSG.OPT, rip_msg_size, ip, DHCP6_SERVER_PORT, 16);
#ifdef _DHCP_DEBUG_
    printf("> %d, %d\r\n", ret, rip_msg_size);
#endif

    return ret;
}

/**
    @brief

    @return uint8_t
*/
uint8_t send_DHCP_INFOREQ(void) {
    //uint16_t i;
    uint16_t j;
    uint8_t ip[16];
    uint8_t rip_msg_size;
    uint8_t ret = 0;

    size = 0;
    num = 0;
    growby = 0;

    //printf("req : %x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x \r\n",recv_IP[0],recv_IP[1],recv_IP[2],recv_IP[3],recv_IP[4],recv_IP[5],recv_IP[6],recv_IP[7],recv_IP[8],recv_IP[9],recv_IP[10],recv_IP[11],recv_IP[12],recv_IP[13],recv_IP[14],recv_IP[15]);
    InitDhcp6Option(30, 1);
    DumpDhcp6Option("option init");

    AppendDhcp6Option(DHCP6_INFO_REQUEST);
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x00FF0000) >> 16));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x0000FF00) >> 8));
    AppendDhcp6Option((uint8_t)((DHCP6_XID & 0x000000FF) >> 0));
    DumpDhcp6Option("Type&XID");

    //    // Elapsed time
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_ELAPSED_TIME);
    //    AppendDhcp6Option(0x00);AppendDhcp6Option(0x02);
    //    AppendDhcp6Option(0x0c);AppendDhcp6Option(0x1c);DumpDhcp6Option("Option Elapsed Time");

    // Client Identifier
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_CLIENTID);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x0a); //length
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x03); //DUID_Type
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x01); //Hard_Type

    AppendDhcp6Option(DHCP6_CHADDR[0]);
    AppendDhcp6Option(DHCP6_CHADDR[1]); // MAC Addr
    AppendDhcp6Option(DHCP6_CHADDR[2]);
    AppendDhcp6Option(DHCP6_CHADDR[3]);
    AppendDhcp6Option(DHCP6_CHADDR[4]);
    AppendDhcp6Option(DHCP6_CHADDR[5]);
    DumpDhcp6Option("Option Client ID");

    // Option Request
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_REQUEST);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(0x06); // length
    //AppendDhcp6Option(0x00);AppendDhcp6Option(OPT_VENDOR_OPTS);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(DNS_RecursiveNameServer);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(Domain_Search_List);
    AppendDhcp6Option(0x00);
    AppendDhcp6Option(OPT_LIFETIME);
    DumpDhcp6Option("Option Request");

    // send broadcasting packet
    ip[0] = 0xff;
    ip[1] = 0x02;

    for (j = 2; j < 13; j++) {
        ip[j] = 0x00;
    }

    ip[13] = 0x01;
    ip[14] = 0x00;
    ip[15] = 0x02;

    rip_msg_size = size;

#ifdef _DHCP_DEBUG_
    printf("> Send DHCP_REQUEST\r\n");
#endif

    sendto(DHCP6_SOCKET, (uint8_t *)pDHCP6MSG.OPT, rip_msg_size, ip, DHCP6_SERVER_PORT, 16);

#ifdef _DHCP_DEBUG_
    printf("> %d, %d\r\n", ret, rip_msg_size);
#endif

    return ret;
}

/**
    @brief

    @return int8_t
*/
int8_t parseDHCP6MSG(void) {
    uint8_t svr_addr[16];
    uint16_t svr_port;
    uint8_t addlen;
    uint16_t len;
    uint8_t *p;
    uint8_t *e;
    uint8_t type, i;
    uint16_t opt_len;
    uint32_t end_point;

    if ((len = getSn_RX_RSR(DHCP6_SOCKET)) > 0) {
        len = recvfrom(DHCP6_SOCKET, (uint8_t *)pDHCP6MSG.OPT, len, svr_addr, (uint16_t *)&svr_port, &addlen);
#ifdef _DHCP6_DEBUG_
        printf("DHCP message : %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x(%d) %d received. \r\n", svr_addr[0], svr_addr[1], svr_addr[2], svr_addr[3], svr_addr[4], svr_addr[5], svr_addr[6], svr_addr[7], svr_addr[8], svr_addr[9], svr_addr[10], svr_addr[11], svr_addr[12], svr_addr[13], svr_addr[14], svr_addr[15], svr_port, len);
#endif
    } else {
        return 0;
    }

    type = 0;
    p = (uint8_t *)(pDHCP6MSG.OPT);
    e = p + len;
#ifdef _DHCP6_DEBUG_
    printf("in server port %x\r\n", *p);
#endif
    i = 0;
    /*
        while(p<e){
        	printf("%.2x ",*p);
        	i8f(i%16 == 1){
        		printf("\r\n");
        	}
        	p++;i++;
        }
        i=0;*/

    switch (*p) {
    case DHCP6_ADVERTISE:
    case DHCP6_REPLY: {
#ifdef _DHCP6_DEBUG_
        printf("in ADVER or REPLY(7) type : %x \r\n", *p);
#endif
        type = *p++; // type
        p++;         // xid[0]
        p++;         // xid[1]
        p++;         // xid[2]
        while (p < e) {
            p++;
            switch (*p) {
            case OPT_CLIENTID: {
#ifdef _DHCP6_DEBUG_
                printf("in clientid \r\n");
#endif
                p++;

                opt_len = (*p++ << 8);
                clientid_len = opt_len + (*p++);
#ifdef _DHCP6_DEBUG_
                printf("opt_len : %.4x\r\n", clientid_len);
#endif
                end_point = (uint32_t)p + clientid_len;

                while ((uint32_t)p != end_point) {
                    p++;
                }
                break;
            }

            case OPT_IANA: {
#ifdef _DHCP6_DEBUG_
                printf("in iana \r\n");
#endif
                p++;
                opt_len = (*p++ << 8);
                iana_len = opt_len + (*p++);
                end_point = (uint32_t)p + iana_len;

                //IAID
                IAID[0] = *p++;
                IAID[1] = *p++;
                IAID[2] = *p++;
                IAID[3] = *p++;
                //T1
                T1[0] = *p++;
                T1[1] = *p++;
                T1[2] = *p++;
                T1[3] = *p++;
                //T2
                T2[0] = *p++;
                T2[1] = *p++;
                T2[2] = *p++;
                T2[3] = *p++;
                //IA_NA-options
                while ((uint32_t)p < end_point) {
                    p++;
                    switch (*p) {
                    case OPT_IAADDR: {
#ifdef _DHCP6_DEBUG_
                        printf("in IA addr \r\n");
#endif
                        p++;
                        opt_len = (*p++ << 8);
                        iaaddr_len = opt_len + (*p++);
                        recv_IP[0] = *p++;
                        recv_IP[1] = *p++;
                        recv_IP[2] = *p++;
                        recv_IP[3] = *p++;
                        recv_IP[4] = *p++;
                        recv_IP[5] = *p++;
                        recv_IP[6] = *p++;
                        recv_IP[7] = *p++;
                        recv_IP[8] = *p++;
                        recv_IP[9] = *p++;
                        recv_IP[10] = *p++;
                        recv_IP[11] = *p++;
                        recv_IP[12] = *p++;
                        recv_IP[13] = *p++;
                        recv_IP[14] = *p++;
                        recv_IP[15] = *p++;

                        PreLifeTime = (*p++ << 24);
                        PreLifeTime += (*p++ << 16);
                        PreLifeTime += (*p++ << 8);
                        PreLifeTime += (*p++);
                        ValidLifeTime = (*p++ << 24);
                        ValidLifeTime += (*p++ << 16);
                        ValidLifeTime += (*p++ << 8);
                        ValidLifeTime += (*p++);

                        printf("IANA : %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", recv_IP[0], recv_IP[1], recv_IP[2], recv_IP[3], recv_IP[4], recv_IP[5], recv_IP[6], recv_IP[7], recv_IP[8], recv_IP[9], recv_IP[10], recv_IP[11], recv_IP[12], recv_IP[13], recv_IP[14], recv_IP[15]);
                        break;
                    }

                    case OPT_STATUS_CODE: {
#ifdef _DHCP6_DEBUG_
                        printf("in status code \r\n");
#endif
                        p++;
                        opt_len = (*p++ << 8);
                        statuscode_len = opt_len + (*p++);
                        Lstatuscode_len = statuscode_len;
#ifdef _DHCP6_DEBUG_
                        printf("status_len : %x\r\n", statuscode_len);
                        printf("Lstatus_len : %x\r\n", Lstatuscode_len);
#endif
                        code = (*p++ << 8);
                        code = code + (*p++);
                        //printf("status_code : %x\r\n", code);
                        //for(i=0; i<(statuscode_len-2); i++)
                        //    status_msg[i] = *p++;
                        //printf("status_msg : %s \r\n", status_msg);

                        p += statuscode_len - 2;
                        break;
                    }

                    default: {
#ifdef _DHCP6_DEBUG_
                        printf("in default \r\n");
#endif
                        p++;
                        opt_len = (*p++ << 8);
                        opt_len = opt_len + (*p++);
                        p += opt_len;
                        break;
                    }
                    }
                }
                break;
            }

            case OPT_IATA: {
#ifdef _DHCP6_DEBUG_
                printf("in iata \r\n");
#endif
                p++;
                opt_len = (*p++ << 8);
                opt_len = opt_len + (*p++);
                //IA_TA-options
                p += opt_len;
                break;
            }

            case OPT_SERVERID: {
#ifdef _DHCP6_DEBUG_
                printf("in serverid \r\n");
#endif

                p++;
                opt_len = (*p++ << 8);
                serverid_len = opt_len + (*p++);
#ifdef _DHCP6_DEBUG_
                printf("opt_len : %.4x\r\n", serverid_len);
#endif
                end_point = (uint32_t)p + serverid_len;

                DUID_type_s = (*p++ << 8);
                DUID_type_s = DUID_type_s + (*p++);
#ifdef _DHCP6_DEBUG_
                printf("DUID_type : %.4x\r\n", DUID_type_s);
#endif
                if (DUID_type_s == 0x02) {
                    Enterprise_num_s = (*p++ << 24);
                    Enterprise_num_s = Enterprise_num_s + (*p++ << 16);
                    Enterprise_num_s = Enterprise_num_s + (*p++ << 8);
                    Enterprise_num_s = Enterprise_num_s + (*p++);
                } else {
                    Hardware_type_s = (*p++ << 8);
                    Hardware_type_s = Hardware_type_s + (*p++);
#ifdef _DHCP6_DEBUG_
                    printf("Hardware_type : %.4x\r\n", Hardware_type_s);
#endif
                }

                if (DUID_type_s == 0x01) {
                    Time_s[0] = *p++;
                    Time_s[1] = *p++;
                    Time_s[2] = *p++;
                    Time_s[3] = *p++;
#ifdef _DHCP6_DEBUG_
                    printf("Time : ");
                    for (i = 0; i < 4; i++) {
                        printf("%.2x", Time_s[i]);
                    }
                    printf("\r\n");
#endif
                }

                Server_MAC[0] = *p++;
                Server_MAC[1] = *p++;
                Server_MAC[2] = *p++;
                Server_MAC[3] = *p++;
                Server_MAC[4] = *p++;
                Server_MAC[5] = *p++;

#ifdef _DHCP6_DEBUG_
                printf("Server_MAC : ");
                for (i = 0; i < 6; i++) {
                    printf("%.2x", Server_MAC[i]);
                }
                printf("\r\n");
#endif
                while ((uint32_t)p != end_point) {
                    p++;
                }
                break;
            }

            case DNS_RecursiveNameServer: {
#ifdef _DHCP6_DEBUG_
                printf("in DNS Recursive Name Server \r\n");
#endif
                p++;
                opt_len = (*p++ << 8);
                opt_len = opt_len + (*p++);
                end_point = (uint32_t)p + opt_len;

                DNS6_Address[0] = *p++;
                DNS6_Address[1] = *p++;
                DNS6_Address[2] = *p++;
                DNS6_Address[3] = *p++;
                DNS6_Address[4] = *p++;
                DNS6_Address[5] = *p++;
                DNS6_Address[6] = *p++;
                DNS6_Address[7] = *p++;
                DNS6_Address[8] = *p++;
                DNS6_Address[9] = *p++;
                DNS6_Address[10] = *p++;
                DNS6_Address[11] = *p++;
                DNS6_Address[12] = *p++;
                DNS6_Address[13] = *p++;
                DNS6_Address[14] = *p++;
                DNS6_Address[15] = *p++;

                while ((uint32_t)p < end_point) {
                    p++;
                }

                break;
            }

            case Domain_Search_List: {
#ifdef _DHCP6_DEBUG_
                printf("in Domain Search List \r\n");
#endif
                p++;
                opt_len = (*p++ << 8);
                opt_len = opt_len + (*p++);
                end_point = (uint32_t)p + opt_len;

                while ((uint32_t)p < end_point) {
                    p++;
                }

                break;
            }

            default: {
#ifdef _DHCP6_DEBUG_
                printf("in default \r\n");
#endif
                p++;
                opt_len = (*p++ << 8);
                opt_len = opt_len + (*p++);
                p += opt_len;
                break;
            }
            }
        }
    }
    }
    return type;
}

/**
    @brief

    @return uint8_t
*/
uint8_t DHCP6_run2(void) {
    uint8_t type;
    uint8_t ret;

    if (dhcp6_state == STATE_DHCP6_STOP) {
        return DHCP6_STOPPED; // Check DHCPv6 STOP State
    }

    if (getSn_SR(DHCP6_SOCKET) != SOCK_UDP) {
        // Check DHCPv6 SOCKET == UDP
        WIZCHIP_WRITE(_Sn_TTLR_(DHCP6_SOCKET), 0x01); // hop limit 1
        socket(DHCP6_SOCKET, (Sn_MR_UDP6), DHCP6_CLIENT_PORT, 0x00);
    }
    ret = DHCP6_RUNNING;
    type = parseDHCP6MSG();
    switch (dhcp6_state) {
    case STATE_DHCP6_INIT:
        send_DHCP_INFOREQ();
        dhcp6_state = STATE_DHCP6_RELEASE;
        break;
    case STATE_DHCP6_RELEASE:
        return DHCP6_IP_LEASED;
    default:
        break;
    }

    return ret;
}

/**
    @brief

    @param netinfo
    @return uint8_t
*/
uint8_t DHCP6_run(wiz_NetInfo *netinfo) {
    uint8_t type;
    uint8_t ret;
    uint8_t i;

    if (dhcp6_state == STATE_DHCP6_STOP) {
        return DHCP6_STOPPED;    // Check DHCPv6 STOP State
    }

    if (getSn_SR(DHCP6_SOCKET) != SOCK_UDP) {
        // Check DHCPv6 SOCKET == UDP
        WIZCHIP_WRITE(_Sn_TTLR_(DHCP6_SOCKET), 0x01); // hop limit 1
        socket(DHCP6_SOCKET, (Sn_MR_UDP6), DHCP6_CLIENT_PORT, 0x00);
    }

    ret = DHCP6_RUNNING;
    type = parseDHCPMSG();
    printf("type:%d, dhcp_state :%d\r\n", type, dhcp6_state);
    switch (dhcp6_state) {
    case STATE_DHCP6_INIT:
        DHCP6_allocated_ip[0] = 0;
        DHCP6_allocated_ip[1] = 0;
        DHCP6_allocated_ip[2] = 0;
        DHCP6_allocated_ip[3] = 0;
        DHCP6_allocated_ip[4] = 0;
        DHCP6_allocated_ip[5] = 0;
        DHCP6_allocated_ip[6] = 0;
        DHCP6_allocated_ip[7] = 0;
        DHCP6_allocated_ip[8] = 0;
        DHCP6_allocated_ip[9] = 0;
        DHCP6_allocated_ip[10] = 0;
        DHCP6_allocated_ip[11] = 0;
        DHCP6_allocated_ip[12] = 0;
        DHCP6_allocated_ip[13] = 0;
        DHCP6_allocated_ip[14] = 0;
        DHCP6_allocated_ip[15] = 0;

        send_DHCP6_SOLICIT();
        dhcp6_state = STATE_DHCP6_SOLICIT;
        break;
    case STATE_DHCP6_SOLICIT:
        if (type == DHCP6_ADVERTISE) {
#ifdef _DHCP6_DEBUG_
            printf("> Receive DHCP_ADVERTISE\r\n");
#endif

            DHCP6_allocated_ip[0] = recv_IP[0];
            DHCP6_allocated_ip[1] = recv_IP[1];
            DHCP6_allocated_ip[2] = recv_IP[2];
            DHCP6_allocated_ip[3] = recv_IP[3];
            DHCP6_allocated_ip[4] = recv_IP[4];
            DHCP6_allocated_ip[5] = recv_IP[5];
            DHCP6_allocated_ip[6] = recv_IP[6];
            DHCP6_allocated_ip[7] = recv_IP[7];
            DHCP6_allocated_ip[8] = recv_IP[8];
            DHCP6_allocated_ip[9] = recv_IP[9];
            DHCP6_allocated_ip[10] = recv_IP[10];
            DHCP6_allocated_ip[11] = recv_IP[11];
            DHCP6_allocated_ip[12] = recv_IP[12];
            DHCP6_allocated_ip[13] = recv_IP[13];
            DHCP6_allocated_ip[14] = recv_IP[14];
            DHCP6_allocated_ip[15] = recv_IP[15];

            ret = send_DHCP_REQUEST();
            if (ret == 9) {
                return 0;
            }
            dhcp6_state = STATE_DHCP6_REQUEST;
        } //else ret = check_DHCP_timeout();
        break;
    case STATE_DHCP6_REQUEST:
        // 20190318
        //NETUNLOCK();

        memcpy(netinfo->gua, recv_IP, 16);

        //NETLOCK();
        return DHCP6_IP_LEASED;
    default:
        break;
    }
    return ret;
}

/**
    @brief

*/
void DHCP6_stop(void) {
    close(DHCP6_SOCKET);
    dhcp6_state = STATE_DHCP6_STOP;
}

/**
    @brief

    @return uint8_t
*/
uint8_t check_DHCP6_timeout(void) {
    uint8_t ret = DHCP6_RUNNING;

    if (dhcp6_retry_count < MAX_DHCP6_RETRY) {
        if (dhcp6_tick_next < dhcp6_tick_1s) {
            switch (dhcp6_state) {
            case STATE_DHCP6_SOLICIT: {
#ifdef _DHCP6_DEBUG_
                printf("<<timeout>> state : STATE_DHCP_DISCOVER\r\n");
#endif
                send_DHCP6_SOLICIT();
                break;
            }
            default: {
                break;
            }
            }

            dhcp6_tick_1s = 0;
            dhcp6_tick_next = dhcp6_tick_1s + DHCP6_WAIT_TIME;
            dhcp6_retry_count++;
        }
    } else {
        // timeout occurred

        switch (dhcp6_state) {
        case STATE_DHCP6_SOLICIT:
            dhcp6_state = STATE_DHCP6_INIT;
            ret = DHCP6_FAILED;
            break;
        default:
            break;
        }
        reset_DHCP6_timeout();
    }
    return ret;
}

/**
    @brief

    @param s
    @param buf
*/
void DHCP6_init(uint8_t s, uint8_t *buf) {
    uint8_t zeroip[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    getSHAR(DHCP6_CHADDR);
    if ((DHCP6_CHADDR[0] | DHCP6_CHADDR[1] | DHCP6_CHADDR[2] | DHCP6_CHADDR[3] | DHCP6_CHADDR[4] | DHCP6_CHADDR[5]) == 0x00) {
#ifdef _DHCP6_DEBUG_
        printf("DHCP_init-set MAC\r\n");
#endif
        // assing temporary mac address, you should be set SHAR before call this function.
        DHCP6_CHADDR[0] = 0x00;
        DHCP6_CHADDR[1] = 0x08;
        DHCP6_CHADDR[2] = 0xdc;
        DHCP6_CHADDR[3] = 0x00;
        DHCP6_CHADDR[4] = 0x00;
        DHCP6_CHADDR[5] = 0x00;
        setSHAR(DHCP6_CHADDR);
    }

    DHCP6_SOCKET = s; // SOCK_DHCP

    memset(buf, 0, sizeof(buf));
    pDHCP6MSG.OPT = buf;
    DHCP6_XID = 0x515789;

    reset_DHCP_timeout();
    dhcp6_state = STATE_DHCP6_INIT;
}

/**
    @brief

*/
void reset_DHCP6_timeout(void) {
    dhcp6_tick_1s = 0;
    dhcp6_tick_next = DHCP6_WAIT_TIME;
    dhcp6_retry_count = 0;
}

/**
    @brief

*/
void DHCP6_time_handler(void) {
    dhcp6_tick_1s++;
}

#endif
