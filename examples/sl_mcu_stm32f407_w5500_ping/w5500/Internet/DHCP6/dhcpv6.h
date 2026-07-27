//*****************************************************************************
//
//! \file dhcpv6.h
//! \brief DHCPv6 APIs Header file.
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
#ifndef _DHCP6_H_
#define _DHCP6_H_

#include <stdint.h>
#if 1
// 20231020 taylor
#if 0
#include "W6100.h"
#endif
#else
#include "W6100.h"
#endif
#include "socket.h"

#if (_WIZCHIP_ == W6100)

/*
    @brief
    @details If you want to display debug & procssing message, Define _DHCP_DEBUG_
    @note    If defined, it dependens on <stdio.h>
*/
#if 0
#define _DHCP6_DEBUG_
#endif

/* Retry to processing DHCPv6 */
#define MAX_DHCP6_RETRY 2  ///< Maxium retry count
#define DHCP6_WAIT_TIME 10 ///< Wait Time 10s

/* UDP port numbers for DHCPv6 */
#define DHCP6_SERVER_PORT 547 ///< DHCP6 server port number
#define DHCP6_CLIENT_PORT 546 ///< DHCP6 client port number

#define DCHP6_HOST_NAME "WIZnet\0"

/*
    @brief return value of @ref DHCP6_run()
*/
enum {
    DHCP6_FAILED = 0, ///< Procssing Fail
    DHCP6_RUNNING,    ///< Procssing DHCPv6 proctocol
    DHCP6_IP_ASSIGN,  ///< First Occupy IP from DHCPv6 server      (if cbfunc == null, act as default default_ip_assign)
    DHCP6_IP_CHANGED, ///< Change IP address by new ip from DHCPv6 (if cbfunc == null, act as default default_ip_update)
    DHCP6_IP_LEASED,  ///< Stand by
    DHCP6_STOPPED     ///< Stop procssing DHCPv6 protocol
};

/**
    @brief

    @param asize
    @param agrowby
*/
void InitDhcp6Option(unsigned asize, unsigned agrowby);
/**
    @brief

    @param value
*/
void AppendDhcp6Option(uint8_t value);
/**
    @brief

    @param sMark
*/
void DumpDhcp6Option(char *sMark);
/**
    @brief

    @param Option
*/
void DHCP6_Option_Select(uint8_t Option);
/*
    @brief DHCPv6 client initialization (outside of the main loop)
    @param s   - socket number
    @param buf - buffer for procssing DHCPv6 message
*/
void DHCP6_init(uint8_t s, uint8_t *buf);

/*
    @brief DHCPv6 1s Tick Timer handler
    @note SHOULD BE register to your system 1s Tick timer handler
*/
void DHCP6_time_handler(void);

/**
    @brief

    @param netinfo
    @return uint8_t
*/
uint8_t DHCP6_run(wiz_NetInfo *netinfo);

/**
    @brief

    @return uint8_t
*/
uint8_t DHCP6_run2(void);

/*
    @brief Stop DHCPv6 procssing
    @note If you want to restart. call DHCP6_init() and DHCP_run()
*/
void DHCP6_stop(void);

/* send DISCOVER message to DHCPv6 server */
void send_DHCP6_SOLICIT(void);
uint8_t send_DHCP6_REQUEST(void);

/* check the timeout in DHCPv6 process */
uint8_t check_DHCP6_timeout(void);

/* Intialize to timeout process.  */
void reset_DHCP6_timeout(void);

int8_t parseDHCP6MSG(void);
uint8_t DHCP6_run2(void);

#endif
#endif /* _DHCP6_H_ */
