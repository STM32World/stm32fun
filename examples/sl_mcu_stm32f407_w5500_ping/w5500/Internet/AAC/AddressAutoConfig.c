//* ****************************************************************************
//! \file AddressAutoConfig.c
//! \brief IPv6 AddressAutoConfig Source File.
//! \version 1.0.0
//! \date 2019/01/01
//! \par  Revision history
//!             <2019/01/01> 1st Release
//! \author WIZnet
//! \copyright
//!
//! Copyright (c)  2019, WIZnet Co., LTD.
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights
//! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//! copies of the Software, and to permit persons to whom the Software is
//! furnished to do so, subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in
//! all copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//! SOFTWARE.
//!
//*****************************************************************************

#include "AddressAutoConfig.h"

#if (_WIZCHIP_ == W6100)

#define MY_MAX_DHCP6_RETRY 3
uint8_t AAC_SOCKET;                      // Socket number for AddressAutoConfi

/**
    @brief

*/
extern uint8_t DNS6_Address[16];

/**
    @brief

    @param netinfo
    @param test_buf
    @return uint8_t
*/
uint8_t AddressAutoConfig_Init(uint8_t s, wiz_NetInfo *netinfo) {
    uint8_t MO_flag;
    uint8_t result_aac = 0;
    uint8_t data_buf[2048] = {
        0,
    };

    AAC_SOCKET = s; // SOCK_AAC

    // DAD LLA
    printf("Duplicate_Address_Detection\r\n");
    Duplicate_Address_Detection(netinfo);
    ctlnetwork(CN_SET_NETINFO, netinfo);
    print_network_information();

    // RSRA
    printf("Address_Auto_Configuration Start\r\n");
    MO_flag = Address_Auto_Config_RA(AAC_SOCKET, data_buf, sizeof(data_buf), netinfo);
    ctlnetwork(CN_SET_NETINFO, netinfo);
    print_network_information();

    if (MO_flag == SLAAC_RDNSS) {
        // Completed

        printf("Address_Auto_Configuration Succeed\r\n");
        result_aac = 1;
    } else if (MO_flag == SLAAC_DHCP6) {
        // Need Stateless DHCP
        // Get Other Information

        printf("Address_Auto_Configuration Failed\r\n");
        printf("Stateless DHCP Start\r\n");

        memset(data_buf, 0, sizeof(data_buf));
        result_aac = Address_Auto_Config_SLDHCP(AAC_SOCKET, data_buf);
        if (result_aac == 1) {
            printf(" Stateless DHCP Succeed\r\n");
        } else {
            printf(" Stateless DHCP Failed\r\n");
        }
    } else if (MO_flag == SFAAC_DHCP6) {
        // Need Stateful DHCP
        // Get Managed Information

        printf("Address_Auto_Configuration Failed\r\n");
        printf("Stateful DHCP Start\r\n");

        memset(data_buf, 0, sizeof(data_buf));
        result_aac = Address_Auto_Config_SFDHCP(AAC_SOCKET, data_buf, netinfo);
        if (result_aac == 1) {
            printf("Stateful DHCP Succeed\r\n");
        } else {
            printf("Stateful DHCP Failed\r\n");
        }

        ctlnetwork(CN_SET_NETINFO, netinfo);
        print_network_information();
    } else {
        printf("Address_Auto_Configuration Failed MO_Flag : 0x%x\r\n", MO_flag);
        result_aac = 0;
    }

    while (SOCK_OK != close(AAC_SOCKET));

    return result_aac;
}

/**
    @brief

    @param sn
    @param test_buf
    @return uint8_t
*/
uint8_t Address_Auto_Config_SLDHCP(uint8_t sn, uint8_t *test_buf) {
    uint8_t result = 0;
    uint8_t tmp[16];
    uint32_t toggle = 1;
    uint32_t my_dhcp_retry = 0;

    DHCP6_init(sn, test_buf);

    while (1) {
        switch (DHCP6_run2()) {
        case DHCP6_IP_ASSIGN:
        case DHCP6_IP_CHANGED:
            /* If this block empty, act with default_ip_assign & default_ip_update  */
            //
            // This example calls the registered 'my_ip_assign' in the two case.
            //
            // Add to ...
            //
            //
            toggle = 1;
            if (toggle) {
                getGAR(tmp);
                printf("> DHCP GW : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                getSUBR(tmp);
                printf("> DHCP SN : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                getSIPR(tmp);
                printf("> DHCP IP : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                toggle = 0;
                close(sn); /*
								If renewal IP address was defferent previous IP address,
								socket becomes to disconnect or close for new connection.
*/
            }
            break;
        case DHCP6_IP_LEASED:
            //
            if (toggle) {
                getSHAR(tmp);
                printf("Mac address : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
                getLLAR(tmp);
                printf("your Link Local IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);
                getGUAR(tmp);
                printf("your Global IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);
                getGA6R(tmp);
                printf("your Gateway IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);

                printf("your DNSv6 is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", DNS6_Address[0], DNS6_Address[1], DNS6_Address[2], DNS6_Address[3],
                       DNS6_Address[4], DNS6_Address[5], DNS6_Address[6], DNS6_Address[7],
                       DNS6_Address[8], DNS6_Address[9], DNS6_Address[10], DNS6_Address[11],
                       DNS6_Address[12], DNS6_Address[13], DNS6_Address[14], DNS6_Address[15]);
                toggle = 0;
            }
            return 1;

        case DHCP6_FAILED:
            /* ===== Example pseudo code =====  */
            // The below code can be replaced your code or omitted.
            // if omitted, retry to process DHCP
            my_dhcp_retry++;
            if (my_dhcp_retry > MY_MAX_DHCP6_RETRY) {
#if DEBUG_MODE != DEBUG_NO
                printf(">> DHCP %d Failed\r\n", my_dhcp_retry);
#endif
                my_dhcp_retry = 0;
                DHCP6_stop(); // if restart, recall DHCP6_init()
            }
            break;
        default:
            break;
        }
    }

    return result;
}

/**
    @brief

    @param sn
    @param test_buf
    @return uint8_t
*/
uint8_t Address_Auto_Config_SFDHCP(uint8_t sn, uint8_t *test_buf, wiz_NetInfo *netinfo) {
    uint8_t result;
    uint8_t tmp[16];
    uint32_t toggle = 1;
    uint32_t my_dhcp_retry = 0;

    DHCP6_init(sn, test_buf);

    while (1) {
        switch (DHCP6_run(netinfo)) {
        case DHCP6_IP_ASSIGN:
        case DHCP6_IP_CHANGED:
            /* If this block empty, act with default_ip_assign & default_ip_update  */
            //
            // This example calls the registered 'my_ip_assign' in the two case.
            //
            // Add to ...
            //
            //
            toggle = 1;
            if (toggle) {
                //                    getGAR(tmp);  printf("> DHCP GW : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                //                    getSUBR(tmp); printf("> DHCP SN : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                //                    getSIPR(tmp); printf("> DHCP IP : %d.%d.%d.%d\r\n", tmp[0], tmp[1], tmp[2], tmp[3]);
                toggle = 0;
                close(sn); /*
								If renewal IP address was defferent previous IP address,
								socket becomes to disconnect or close for new connection.
*/
            }
            break;
        case DHCP6_IP_LEASED:
            //
            if (toggle) {
                getSHAR(tmp);
                printf("Mac address : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
                getLLAR(tmp);
                printf("your Link Local IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);
                getGUAR(tmp);
                printf("your Global IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);
                getGA6R(tmp);
                printf("your Gateway IP is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", tmp[0], tmp[1], tmp[2], tmp[3],
                       tmp[4], tmp[5], tmp[6], tmp[7],
                       tmp[8], tmp[9], tmp[10], tmp[11],
                       tmp[12], tmp[13], tmp[14], tmp[15]);

                printf("your DNSv6 is %.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x:%.2x%.2x \r\n", DNS6_Address[0], DNS6_Address[1], DNS6_Address[2], DNS6_Address[3],
                       DNS6_Address[4], DNS6_Address[5], DNS6_Address[6], DNS6_Address[7],
                       DNS6_Address[8], DNS6_Address[9], DNS6_Address[10], DNS6_Address[11],
                       DNS6_Address[12], DNS6_Address[13], DNS6_Address[14], DNS6_Address[15]);
                toggle = 0;
            }
            return 1;

        case DHCP6_FAILED:
            /* ===== Example pseudo code =====  */
            // The below code can be replaced your code or omitted.
            // if omitted, retry to process DHCP
            my_dhcp_retry++;
            if (my_dhcp_retry > MY_MAX_DHCP6_RETRY) {
#if DEBUG_MODE != DEBUG_NO
                printf(">> DHCP %d Failed\r\n", my_dhcp_retry);
#endif
                my_dhcp_retry = 0;
                DHCP6_stop(); // if restart, recall DHCP6_init()
                return 0;
            }
            break;
        default:
            break;
        }
    }

    return result;
}

/**
    @brief

    @param sn
    @param icmpbuf
    @param buf_size
    @param netinfo
    @return uint8_t
*/
uint8_t Address_Auto_Config_RA(uint8_t sn, uint8_t *icmpbuf, uint16_t buf_size, wiz_NetInfo *netinfo) {
    uint8_t result;

    volatile uint16_t size;
    uint8_t destip[16];
    uint16_t destport;
    uint8_t addr_len, o_len;
    uint8_t flags;

    uint8_t *p;
    uint8_t *e;
    int i;
    uint8_t o_type, type, code, RA_flag, MO_flag; //, O_flag;
    uint16_t Router_lifetime;
    uint32_t Reachable_time, Retrans_time;
    uint32_t end_point;
    uint8_t prefix_len, pi_flag;
    uint32_t validtime, prefertime, dnstime;
    uint8_t prefix[16];

    if (getSLCR() != 0x00) { //check clear CMD
#if (AutoConfig_debug == debug_on)
        printf("ERROR : RQCMD is not clear\r\n");
#endif
        result = ERROR_SLCMD;
        return result;
    }

    setICMP6BLKR(ICMP6BLKR_RA);
    printf("getICMP6BLKR() = 0x%x\r\n", getICMP6BLKR()); // 0X4

    setSn_PNR(sn, PROTOCOL_NUM_ICMPv6);						  //ICMPv6 : 58
    printf("getSn_PROTOR(%d) = 0x%x\r\n", sn, getSn_PNR(sn)); // 0X3a

    socket(sn, Sn_MR_IPRAW6, 0, 0);

#if 1
    printf("Sn_SR : %x \r\n", getSn_SR(sn)); // 0X21 33
#else
#if (AutoConfig_debug == debug_on)
    printf("Sn_SR : %x \r\n", getSn_SR(sn));
#endif
#endif

    setSLRTR(4000);
    printf("getSLRTR() = 0x%x\r\n", getSLRTR()); // 0xfa0 4000

    setSLRCR(0);
    printf("getSLRCR() = 0x%x\r\n", getSLRCR()); // 0

    setSLCR(SLCR_RS);
    printf("getSLCR() = 0x%x\r\n", getSLCR()); // 0
    printf("getSLIR() = 0x%x\r\n", getSLIR()); // 0
#if (AutoConfig_debug == debug_on)
    printf("Wait SLIR.....\r\n");
#endif

    do {
        if (getSn_RX_RSR(sn) > 0) {
            printf("getSLIR() = 0x%x\r\n", getSLIR()); // 0

            printf("getSn_RX_RSR(%d) = 0x%x\r\n", sn, getSn_RX_RSR(sn)); // 0x4a 74
            printf("size : %d \r\n", sizeof(icmpbuf));
            size = recvfrom(sn, icmpbuf, buf_size, destip, &destport, &addr_len);
            printf("recvfrom IP : %x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x:%x%x \r\n", destip[0], destip[1], destip[2], destip[3], destip[4], destip[5], destip[6], destip[7], destip[8], destip[9], destip[10], destip[11], destip[12], destip[13], destip[14], destip[15]);
            // =================================================
            // 20190402
            // Taylor
            //
            // Set Gateway from source
            memcpy(netinfo->gw6, destip, 16);

            // =================================================
        }

        p = icmpbuf;
    } while (*p != ROUTER_ADVERTISEMENT);

    e = p + size;
    switch (*p) {
    case ROUTER_ADVERTISEMENT:
        printf("RA\r\n");
        type = *p++;
        printf("type : %x \r\n", type);
        code = *p++;
        printf("code : %x \r\n", code);
        p++;
        p++; //checksum
        p++; //Cur hop limit
        RA_flag = *p++;
        printf("RA_flag : %x \r\n", RA_flag);
        Router_lifetime = *p++ << 8;
        Router_lifetime = Router_lifetime + (*p++);
        printf("Router_lifetime : %d s \r\n", Router_lifetime);
        Reachable_time = *p++ << 24;
        Reachable_time = Reachable_time + (*p++ << 16);
        Reachable_time = Reachable_time + (*p++ << 8);
        Reachable_time = Reachable_time + (*p++);
        printf("Reachable_time : %d ms \r\n", Reachable_time);
        Retrans_time = *p++ << 24;
        Retrans_time = Retrans_time + (*p++ << 16);
        Retrans_time = Retrans_time + (*p++ << 8);
        Retrans_time = Retrans_time + (*p++);
        printf("Retrans_time : %d ms \r\n", Retrans_time);

        while (p < e) {
            switch (*p) {
            case RAO_SLLA:
                o_type = *p++;
                printf("Option Type : %d (Source LLA) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                p += (o_len - 2);
                break;
            case RAO_TLLA:
                o_type = *p++;
                printf("Option Type : %d (Target LLA) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                p += (o_len - 2);
                break;
            case RAO_PI:
                o_type = *p++;
                printf("Option Type : %d (Prefix information) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                end_point = (uint32_t)p - 2 + o_len;
                prefix_len = *p++;
                printf("Prefix Length : %d \r\n", prefix_len);
                pi_flag = *p++;
                printf("Prefix Information Flag : %.2x \r\n", pi_flag);
                validtime = (*p++ << 24);
                validtime += (*p++ << 16);
                validtime += (*p++ << 8);
                validtime += (*p++);
                printf("valid lifetime : %d \r\n", validtime);
                prefertime = (*p++ << 24);
                prefertime += (*p++ << 16);
                prefertime += (*p++ << 8);
                prefertime += (*p++);
                printf("preferred lifetime : %d \r\n", prefertime);
                p++;
                p++;
                p++;
                p++; //reserved
                getLLAR(prefix);
                for (int i = 0; i < prefix_len / 8; i++) {
                    prefix[i] = *p++;

                    // =================================================
                    // 20190416
                    // Taylor
                    //
                    // Set Subnet Mask from prefix

                    netinfo->sn6[i] = 0xFF;

                    // =================================================
                }

                memcpy(netinfo->gua, prefix, 16);

                while ((uint32_t)p != end_point) {
                    p++;
                }
                printf("prefix : ");
                for (i = 0; i < prefix_len / 8; i++) {
                    printf("%.2x", prefix[i]);
                    if (1 == (i % 2)) {
                        printf(":");
                    }
                }
                printf(":\r\n");
                break;
            case RAO_RH:
                o_type = *p++;
                printf("Option Type : %d (Redirected Header) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                p += (o_len - 2);
                break;
            case RAO_MTU:
                o_type = *p++;
                printf("Option Type : %d (MTU) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                p += (o_len - 2);
                break;
            case RAO_RDNS:
                o_type = *p++;
                printf("Option Type : %d (Recursive DNS Server) \r\n", o_type);
                o_len = (*p++) * 8;
                printf("Option length : %d \r\n", o_len);
                end_point = (uint32_t)p - 2 + o_len;
                p++;
                p++; //reserved
                dnstime = (*p++ << 24);
                dnstime += (*p++ << 16);
                dnstime += (*p++ << 8);
                dnstime += (*p++);
                printf("DNS lifetime : %d \r\n", dnstime);
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
                while ((uint32_t)p != end_point) {
                    p++;
                }
                printf("DNS IP : ");
                for (i = 0; i < 15; i++) {
                    printf("%.2x", DNS6_Address[i]);
                    if (1 == (i % 2)) {
                        printf(":");
                    }
                }
                printf("%x\r\n", DNS6_Address[15]);
                break;
            default:
                printf("default\r\n");
                o_type = *p++;
                printf("o_type : %d \r\n", o_type);
                o_len = (*p++) * 8;
                printf("o_len : %d \r\n", o_len);
#if 1
                // 20231106 taylor
                if (o_len > 0) {
                    p += (o_len - 2);
                } else {
                    p ++;
                }
#else
                p += (o_len - 2);
#endif
                break;
            } // ICMP option
        }	 //while
    }

    close(sn);

    printf("RA : %x \r\n", RA_flag);

    //M_flag = RA_flag >> 7;
    MO_flag = RA_flag >> 6;

    printf("MO : %x \r\n", MO_flag);

    result = MO_flag;

    return result;
}

/**
    @brief

    @param netinfo
    @return uint8_t
*/
uint8_t Duplicate_Address_Detection(wiz_NetInfo *netinfo) {
    uint8_t result;

    uint8_t WIZ_LLA[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    uint8_t flags;
    uint8_t tmp_array[16];
    uint16_t i;

    setSLRTR(2000);
    setSLRCR(5);

    Generate_EUI64(netinfo->mac, WIZ_LLA);

    setSLDIP6R(WIZ_LLA); //target address setting

    //setSLIMR(SLIR_TIOUT|SLIR_NS); //only external interrupt???

    if (getSLCR() != 0x00) { //check clear CMD
#if (AutoConfig_debug == debug_on)
        printf("ERROR : RQCMD is not clear %x \r\n", getSLCR());
#endif
        while (1)
            ;
    }

    setSLCR(SLCR_NS);

#if (AutoConfig_debug == debug_on)
    printf("Wait RQIR.....\r\n");
#endif

    do {
        flags = getSLIR();

        if (flags & SLIR_TOUT) {
#if (AutoConfig_debug == debug_on)
            printf("\r\nTimeout !!! DAD SUCCESSED\r\n");
#endif
#if 0
            // 20190401
            NETCFG_UNLOCK();
#endif
            //-- Set Link Local  Address
            memcpy(netinfo->lla, WIZ_LLA, 16);

#if 0
            // 20190404
            // Display memcpy result

            printf("WIZ_LLA:\r\n");
            for (i = 0; i < 16; i += 2) {
                printf("%.2x%.2x:", WIZ_LLA[i], WIZ_LLA[i + 1]);
            }
            printf("\r\n");

            printf("netinfo->mac:\r\n");
            for (i = 0; i < 16; i += 2) {
                printf("%.2x%.2x:", netinfo->lla[i], netinfo->lla[i + 1]);
            }
            printf("\r\n");
#endif

#if 0
            // 20190401
            NETCFG_LOCK();
#endif
            result = SUCCESS;
        } else if (flags & SLIR_NS) {
#if (AutoConfig_debug == debug_on)
            printf("\r\nReceived NA !!! DAD FAILED\r\n");
#endif
            result = ERROR_DAD_FAIL;
        }
    } while (!((flags & SLIR_TOUT) || (flags & SLIR_NS)));
    printf("\r\nflags = 0x%x\r\n", flags);
    setSLIRCLR(flags);

    return result;
}

/**
    @brief

    @param mac_addr
    @param Link_Local_Addr
*/
void Generate_EUI64(uint8_t *mac_addr, uint8_t *Link_Local_Addr) {
    *Link_Local_Addr = 0xfe;
    *(Link_Local_Addr + 1) = 0x80;
    //00:00:00:00:00:00
    *(Link_Local_Addr + 8) = *(mac_addr); //flip the 7th bit of 1st byte
    *(Link_Local_Addr + 8) ^= 1 << 1;
    *(Link_Local_Addr + 9) = *(mac_addr + 1);
    *(Link_Local_Addr + 10) = *(mac_addr + 2);
    *(Link_Local_Addr + 11) = 0xFF;
    *(Link_Local_Addr + 12) = 0xFE;
    *(Link_Local_Addr + 13) = *(mac_addr + 3);
    *(Link_Local_Addr + 14) = *(mac_addr + 4);
    *(Link_Local_Addr + 15) = *(mac_addr + 5);
}

#endif
