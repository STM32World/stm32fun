#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/**
 * ---------------------------------------------------------------------
 * Operating System and Platform Architecture
 * ---------------------------------------------------------------------
 */
#define NO_SYS                  1       // Bare-metal mode (No RTOS)
#define SYS_LIGHTWEIGHT_PROT    0       // Disable OS-level protection macros
#define MEM_ALIGNMENT           4       // STM32 Cortex-M4 32-bit alignment

/**
 * ---------------------------------------------------------------------
 * Memory Management
 * ---------------------------------------------------------------------
 * F407 has 192KB RAM, so we can afford comfortable heaps and pools.
 */
#define MEM_LIBC_MALLOC         0       // Use lwIP's internal memory manager
#define MEM_SIZE                (16 * 1024) // 16 KB heap size for lwIP

/* Packet Buffers (PBUFs) */
#define PBUF_POOL_SIZE          16      // Number of buffers in raw packet pool
#define PBUF_POOL_BUFSIZE       1536    // Ethernet frame size (1514 + padding)

/**
 * ---------------------------------------------------------------------
 * Protocol Support
 * ---------------------------------------------------------------------
 */
#define LWIP_ARP                1       // Address Resolution Protocol (Required)
#define LWIP_ETHERNET           1       // Ethernet support (Required for USB CDC-NCM/ECM)
#define LWIP_IPV4               1       // IPv4 Support
#define LWIP_ICMP               1       // Ping responder
#define LWIP_IGMP               1       // Multicast support
#define LWIP_DHCP               1       // DHCP Server/Client support
#define LWIP_UDP                1       // UDP Protocol
#define LWIP_TCP                1       // TCP Protocol
#define LWIP_DNS                0       // Domain Name Resolution

/**
 * ---------------------------------------------------------------------
 * TCP Sockets and Window Tuning
 * ---------------------------------------------------------------------
 */
#define TCP_MSS                 1460    // Maximum Segment Size (Standard Ethernet)
#define TCP_WND                 (4 * TCP_MSS) // Receive window scale
#define TCP_SND_BUF             (4 * TCP_MSS) // Send buffer scale
#define TCP_SND_QUEUELEN        (4 * TCP_SND_BUF/TCP_MSS)

/**
 * ---------------------------------------------------------------------
 * Checksum Offloading Options
 * ---------------------------------------------------------------------
 * Calculated via STM32 software to avoid endianness issue mismatch
 * between PC and TinyUSB packet formats.
 */
#define CHECKSUM_GEN_IP         1
#define CHECKSUM_GEN_UDP        1
#define CHECKSUM_GEN_TCP        1
#define CHECKSUM_GEN_ICMP       1
#define CHECKSUM_CHECK_IP       1
#define CHECKSUM_CHECK_UDP      1
#define CHECKSUM_CHECK_TCP      1
#define CHECKSUM_CHECK_ICMP     1

/**
 * ---------------------------------------------------------------------
 * Network Interface & Debugging Configuration
 * ---------------------------------------------------------------------
 */
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETCONN                0   // Requires OS (Disable)
#define LWIP_SOCKET                 0   // Requires OS (Disable)

#endif /* __LWIPOPTS_H__ */
