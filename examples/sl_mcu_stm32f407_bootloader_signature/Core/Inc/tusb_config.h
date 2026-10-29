#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// RHPort 0 for STM32F407 FS
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT      0
#endif

#define CFG_TUSB_MCU          OPT_MCU_STM32F4
#define CFG_TUSB_OS           OPT_OS_NONE
#define CFG_TUSB_DEBUG        0

// Device Configuration
#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     OPT_MODE_DEFAULT_SPEED
#define CFG_TUD_ENDPOINT0_SIZE    64

 /* Port 0 Configuration: USB OTG FS (Full Speed) on STM32F407 */
 #define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))

// DFU Class Configuration
#define CFG_TUD_DFU               1
#define CFG_TUD_DFU_RUNTIME       1
#define CFG_TUD_DFU_XFER_BUFSIZE  1024 // Standard ST DFU block size is 1024
#define CFG_TUD_DFU_ATTR          0x07
#define CFG_TUD_DFU_DETACH_TIMEOUT 1000

#ifdef __cplusplus
 }
#endif

#endif /* TUSB_CONFIG_H_ */
