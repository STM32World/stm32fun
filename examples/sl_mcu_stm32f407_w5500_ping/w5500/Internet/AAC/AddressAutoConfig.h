//* ****************************************************************************
//! \file AddressAutoConfig.h
//! \brief IPv6 AddressAutoConfig Header File.
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

#ifndef _ADDRESSAUTOCONFIG_H_
#define _ADDRESSAUTOCONFIG_H_

#include "stdio.h"
#include "stdint.h"
#if 1
// 20231020 taylor
#if 0
#endif
#else
#include "w6100.h"
#endif
#include "socket.h"
#if 1
// 20231020 taylor
#include "../DHCP6/dhcpv6.h"
#else
#include "dhcpv6.h"
#endif

#if (_WIZCHIP_ == W6100)

#define AutoConfig_debug debug_on
#define debug_on 1
#define debug_off 0

#define PROTOCOL_NUM_ICMPv6 58

#define ROUTER_ADVERTISEMENT 134

#define SUCCESS 0
#define ERROR_DAD_FAIL -1
#define ERROR_SLCMD    -2
#define ERROR_TIMEOUT  -3

#define SLAAC_RDNSS  0
#define SLAAC_DHCP6  1
#define SFAAC_DHCP6  3

#define RAO_SLLA	 1
#define RAO_TLLA	 2
#define RAO_PI		 3
#define RAO_RH		 4
#define RAO_MTU		 5
#define RAO_RDNS	25

/**
    @brief

    @param netinfo
    @return uint8_t
*/
uint8_t AddressAutoConfig_Init(uint8_t s, wiz_NetInfo* netinfo);

/**
    @brief

    @param mac_addr
    @param Link_Local_Addr
*/
void Generate_EUI64(uint8_t *mac_addr, uint8_t *Link_Local_Addr);

/**
    @brief

    @param netinfo
    @return uint8_t
*/
uint8_t Duplicate_Address_Detection(wiz_NetInfo* netinfo);

/**
    @brief

    @param sn
    @param icmpbuf
    @param buf_size
    @param netinfo
    @return uint8_t
*/
uint8_t Address_Auto_Config_RA(uint8_t sn, uint8_t *icmpbuf, uint16_t buf_size, wiz_NetInfo* netinfo);

/**
    @brief

    @param sn
    @param test_buf
    @return uint8_t
*/
uint8_t Address_Auto_Config_SLDHCP(uint8_t sn, uint8_t* test_buf);

/**
    @brief

    @param sn
    @param test_buf
    @return uint8_t
*/
uint8_t Address_Auto_Config_SFDHCP(uint8_t sn, uint8_t* test_buf, wiz_NetInfo* netinfo);

#endif
#endif // _ADDRESSAUTOCONFIG_H_
