#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Board / MCU Target Selection
//--------------------------------------------------------------------+

// Specify target MCU for STM32H7 series
#define CFG_TUSB_MCU                OPT_MCU_STM32H7

// Operating System configuration (Bare-metal / HAL)
#define CFG_TUSB_OS                 OPT_OS_NONE

// Debug log level (0: Off, 1: Error, 2: Warning, 3: Info)
#define CFG_TUSB_DEBUG              0

/* Enable Device Stack */
#define CFG_TUD_ENABLED             1

//--------------------------------------------------------------------+
// USB Port Configuration (STM32H7 OTG_HS = RHPORT1)
//--------------------------------------------------------------------+

// RHPORT0 = USB_OTG_FS (Disabled)
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_NONE

// RHPORT1 = USB_OTG_HS (Enabled using Embedded Full-Speed PHY)
#define CFG_TUSB_RHPORT1_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// Active Root Hub Port used by TinyUSB device stack
#define BOARD_TUD_RHPORT            1

// Disable hardware VBUS sensing inside TinyUSB's dwc2 driver
#define DWC2_VBUS_SENSING           0

// Control Endpoint 0 Size
#define CFG_TUD_ENDPOINT0_SIZE      64

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

// Enable USB MIDI Class Driver
#define CFG_TUD_MIDI                1

// Disable unused USB Class Drivers to save FLASH/RAM
#define CFG_TUD_CDC                 0
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_VENDOR              0

//--------------------------------------------------------------------+
// MIDI Class Settings
//--------------------------------------------------------------------+

// RX (Receive) FIFO buffer size in bytes (Must be power of 2)
#define CFG_TUD_MIDI_RX_BUFSIZE     64

// TX (Transmit) FIFO buffer size in bytes
#define CFG_TUD_MIDI_TX_BUFSIZE     64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
