#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/**
 * =====================================================================
 * 1. System Architecture (Bare-Metal STM32F4)
 * =====================================================================
 */
#define NO_SYS                      1       // Bare-metal execution (No RTOS)
#define SYS_LIGHTWEIGHT_PROT        0       // No inter-thread locking needed
#define MEM_ALIGNMENT               4       // 32-bit alignment for Cortex-M4

/**
 * =====================================================================
 * 2. Memory Pools & Buffers
 * =====================================================================
 */
#define MEM_LIBC_MALLOC             0       // Use internal lwIP memory heap
#define MEM_SIZE                    (16 * 1024) // 16 KB heap size

#define PBUF_POOL_SIZE              16      // Number of pre-allocated RX/TX buffers
#define PBUF_POOL_BUFSIZE           1536    // Standard Ethernet frame size + padding

/**
 * =====================================================================
 * 3. Layer 2 / Layer 3 Protocol Support
 * =====================================================================
 */
#define LWIP_ARP                    1       // Address Resolution Protocol
#define LWIP_ETHERNET               1       // Ethernet support (Required for USB CDC-NCM/ECM)
#define LWIP_IPV4                   1       // IPv4 Stack
#define LWIP_ICMP                   1       // ICMP / Ping support
#define LWIP_UDP                    1       // UDP protocol support
#define LWIP_TCP                    1       // TCP protocol support
#define LWIP_DNS                    0       // Disabled to save flash/RAM

/* Crucial for receiving DHCP offers on 0.0.0.0 */
#define IP_SOF_BROADCAST            1       // Filter and handle broadcast options
#define IP_SOF_BROADCAST_RECV       1       // Permit pre-bind packet reception in ip4_input


/**
 * =====================================================================
 * 4. Network Interface & Callbacks
 * =====================================================================
 */
#define LWIP_NETIF_STATUS_CALLBACK  1       // IP address change callback
#define LWIP_NETIF_LINK_CALLBACK    1       // Network link state change callback
#define LWIP_NETCONN                0       // Requires OS (Disabled)
#define LWIP_SOCKET                 0       // Requires OS (Disabled)

/**
 * =====================================================================
 * 5. DHCP Client Options
 * =====================================================================
 */
#define LWIP_DHCP                   1       // Enable DHCP Client
#define DHCP_DOES_ARP_CHECK         0       // Disable ARP checking after offer (prevents drops)
#define ETHARP_TRUST_IP_MAC         1       // Dynamically learn ARP entries from incoming traffic

/**
 * =====================================================================
 * 6. Dynamic Web Server (HTTPD)
 * =====================================================================
 */
#define LWIP_HTTPD                  1       // Integrated HTTP Server
#define LWIP_HTTPD_CUSTOM_FILES     1       // Dynamic file handling (fs_open_custom)
#define LWIP_HTTPD_DYNAMIC_HEADERS  1       // Custom HTTP response headers

/**
 * =====================================================================
 * 7. TCP Connection Parameters
 * =====================================================================
 */
#define TCP_MSS                     1460    // Max Segment Size
#define TCP_WND                     (4 * TCP_MSS) // RX window size
#define TCP_SND_BUF                 (4 * TCP_MSS) // TX buffer size
#define TCP_SND_QUEUELEN            (4 * TCP_SND_BUF / TCP_MSS)

/**
 * =====================================================================
 * 8. Software Checksums
 * =====================================================================
 */
#define CHECKSUM_GEN_IP             1
#define CHECKSUM_GEN_UDP            1
#define CHECKSUM_GEN_TCP            1
#define CHECKSUM_GEN_ICMP           1

#define CHECKSUM_CHECK_IP           0
#define CHECKSUM_CHECK_UDP          0
#define CHECKSUM_CHECK_TCP          1
#define CHECKSUM_CHECK_ICMP         1

/**
 * =====================================================================
 * 9. Debug Traces
 * =====================================================================
 */
#define LWIP_DEBUG                  1
#define LWIP_PLATFORM_DIAG(x)       do { printf x; } while(0)
#define LWIP_DBG_TYPES_ON           LWIP_DBG_ON

#define DHCP_DEBUG                  LWIP_DBG_ON
#define IP_DEBUG                    LWIP_DBG_ON
#define UDP_DEBUG                   LWIP_DBG_ON

#endif /* __LWIPOPTS_H__ */
