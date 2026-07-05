#include "tusb.h"
#include <stdio.h>
#include <string.h>

#define STRID_LANGID       0
#define STRID_MANUFACTURER 1
#define STRID_PRODUCT      2
#define STRID_SERIAL       3

enum {
    ITF_NUM_MTP = 0,
    ITF_NUM_TOTAL
};

// Device Descriptor - Kept as Google/Nexus identity to guarantee libmtp handling
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = 64,             // Explicitly 64 bytes for Control Endpoint 0
    .idVendor           = 0x18D1,
    .idProduct          = 0x4EE1,
    .bcdDevice          = 0x0100,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

// Configuration Descriptor (39 bytes total) - All endpoints strictly set to 64 bytes
uint8_t const desc_configuration[] = {
    // Configuration Descriptor Header
    0x09,                           // bLength
    TUSB_DESC_CONFIGURATION,        // bDescriptorType (0x02)
    0x27, 0x00,                     // wTotalLength (39 bytes)
    0x01,                           // bNumInterfaces
    0x01,                           // bConfigurationValue
    0x00,                           // iConfiguration
    0xC0,                           // bmAttributes (Self-powered)
    0x32,                           // bMaxPower (100mA)

    // Interface Descriptor (Still Image Class configured for MTP)
    0x09,                           // bLength
    TUSB_DESC_INTERFACE,            // bDescriptorType (0x04)
    0x00,                           // bInterfaceNumber (0)
    0x00,                           // bAlternateSetting
    0x03,                           // bNumEndpoints
    0x06,                           // bInterfaceClass (Still Image Class = 6)
    0x01,                           // bInterfaceSubClass (1)
    0x01,                           // bInterfaceProtocol (1)
    0x00,                           // iInterface (0 = No string descriptor lookup loop)

    // Endpoint 1: Interrupt IN (Events)
    0x07,                           // bLength
    TUSB_DESC_ENDPOINT,             // bDescriptorType (0x05)
    0x81,                           // bEndpointAddress (IN 1)
    TUSB_XFER_INTERRUPT,            // bmAttributes (0x03)
    0x40, 0x00,                     // wMaxPacketSize (Forced to 64 bytes to align FIFO)
    0x0A,                           // bInterval (10ms)

    // Endpoint 2: Bulk OUT (Data From Host)
    0x07,                           // bLength
    TUSB_DESC_ENDPOINT,             // bDescriptorType (0x05)
    0x02,                           // bEndpointAddress (OUT 2)
    TUSB_XFER_BULK,                 // bmAttributes (0x02)
    0x40, 0x00,                     // wMaxPacketSize (Strictly 64 bytes)
    0x00,                           // bInterval

    // Endpoint 3: Bulk IN (Data To Host)
    0x07,                           // bLength
    TUSB_DESC_ENDPOINT,             // bDescriptorType (0x05)
    0x82,                           // bEndpointAddress (IN 2)
    TUSB_XFER_BULK,                 // bmAttributes (0x02)
    0x40, 0x00,                     // wMaxPacketSize (Strictly 64 bytes)
    0x00                            // bInterval
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

// String Descriptors Array
char const* string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: Language ID
    "Google Inc.",                 // 1: Manufacturer
    "Nexus MTP Device",            // 2: Product
    "1234567890AF",                // 3: Serial
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count = 0;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char* str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    uint8_t* header_ptr = (uint8_t*)_desc_str;
    header_ptr[0] = (2 * chr_count) + 2;
    header_ptr[1] = TUSB_DESC_STRING;

    return _desc_str;
}
