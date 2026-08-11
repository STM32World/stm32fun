/**
 ******************************************************************************
 * @file           : tusb_config.h
 * @brief          : TinyUSB config header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STM32World <lth@stm32world.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Board / MCU Target Selection
//--------------------------------------------------------------------+

// Specify target MCU for STM32F4 series
#define CFG_TUSB_MCU                OPT_MCU_STM32F4

// Operating System configuration
// Options: OPT_OS_NONE (Bare-metal / HAL), OPT_OS_FREERTOS, OPT_OS_CMSIS_RTOS2
#define CFG_TUSB_OS                 OPT_OS_NONE

// Debug log level (0: Off, 1: Error, 2: Warning, 3: Info)
#define CFG_TUSB_DEBUG              0

/* Enable Device Stack */
#define CFG_TUD_ENABLED             1

/* Port 0 Configuration: USB OTG FS (Full Speed) on STM32F407 */
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

//--------------------------------------------------------------------+
// USB Core & Endpoint Configuration
//--------------------------------------------------------------------+

// Control Endpoint 0 Size (Standard is 64 bytes for Full-Speed USB)
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

// RX (Receive) FIFO buffer size in bytes.
// Must be a power of 2. 64 bytes = 16 4-byte USB-MIDI packets.
#define CFG_TUD_MIDI_RX_BUFSIZE     64

// TX (Transmit) FIFO buffer size in bytes.
#define CFG_TUD_MIDI_TX_BUFSIZE     64

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
