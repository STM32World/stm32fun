/**
 ******************************************************************************
 * @file           : usb_descriptors.c
 * @brief          : TinyUSB USB descriptors
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

#include "tusb.h"
#include "main.h"

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,                  // USB 2.0 specification

    .bDeviceClass       = 0x00,                    // Class defined at Interface level
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,  // Endpoint 0 max packet size

    .idVendor           = 0xCAFE,                  // Vendor ID (Example / Testing)
    .idProduct          = 0x4000,                  // Product ID (Example)
    .bcdDevice          = 0x0100,                  // Device release number

    .iManufacturer      = 0x01,                    // Index of Manufacturer string
    .iProduct           = 0x02,                    // Index of Product string
    .iSerialNumber      = 0x03,                    // Index of Serial string

    .bNumConfigurations = 0x01                     // 1 Configuration
};

// Invoked when receiving GET DEVICE DESCRIPTOR request
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING, // TinyUSB MIDI uses 2 interfaces (Control + Streaming)
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

#define EPNUM_MIDI_OUT      0x01
#define EPNUM_MIDI_IN       0x81

uint8_t const desc_configuration[] =
{
    // Config Descriptor Header:
    // Config Index (1), Interface Count, String Index (0), Total Length, Attribute (0x00), Power in mA (100mA)
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // MIDI Interface Descriptor:
    // Interface Number, String Index, EP OUT Address, EP IN Address, EP Size (64)
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 0, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, 64)
};

// Invoked when receiving GET CONFIGURATION DESCRIPTOR request
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index; // Suppress unused parameter warning
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// Array of pointer to string descriptors
char const* string_desc_arr [] =
{
    (const char[]) { 0x09, 0x04 }, // 0: Supported Language (0x0409 = English US)
    "STM32World",                  // 1: Manufacturer
    "MIDI Buzzer",                 // 2: Product Name (Appears in DAW/OS)
    "",                            // 3: Serial Number (Generated dynamically)
};

static uint16_t _desc_str[32];

// Invoked when receiving GET STRING DESCRIPTOR request
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;
    uint8_t chr_count;

    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else if (index == 3)
    {
        // 12 bytes total for 96-bit UID
        const uint8_t *uid = (const uint8_t *)UID_BASE;
        static const char hex_lut[] = "0123456789ABCDEF";

        chr_count = 24; // 12 bytes * 2 hex chars per byte

        // Convert each byte directly into UTF-16 hex characters
        for (uint8_t i = 0; i < 12; i++)
        {
            _desc_str[1 + (i * 2)]     = hex_lut[(uid[i] >> 4) & 0x0F];
            _desc_str[1 + (i * 2) + 1] = hex_lut[uid[i] & 0x0F];
        }
    }
    else
    {
        if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;

        const char* str = string_desc_arr[index];

        // Cap string length at 31 characters
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 31) chr_count = 31;

        // Convert ASCII string into UTF-16
        for (uint8_t i = 0; i < chr_count; i++)
        {
            _desc_str[1 + i] = str[i];
        }
    }

    // First byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}
