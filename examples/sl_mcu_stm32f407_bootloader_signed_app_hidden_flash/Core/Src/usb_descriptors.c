#include "tusb.h"
#include <string.h>

/* STM32 Standard DFU Identifiers */
#define USB_VID   0x0483
#define USB_PID   0xDF11

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static const tusb_desc_device_t desc_device = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = USB_VID,
        .idProduct = USB_PID,
        .bcdDevice = 0x011a,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
    ITF_NUM_DFU,
    ITF_NUM_TOTAL
};

// 1. Ensure your length calculation is correct.
// TUD_CONFIG_DESC_LEN is 9 bytes.
// TUD_DFU_DESC_LEN(1) is 9 bytes for the interface + 9 bytes for the functional descriptor = 18 bytes.
// Total should be 27 bytes.
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(1))

uint8_t const desc_configuration[] __attribute__((aligned(4))) = {
        // Configuration number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

        // Interface number, Alternate count, string index, attributes, detach timeout, transfer size
        TUD_DFU_DESCRIPTOR(ITF_NUM_DFU, 1, 4, 0x07, 1000, CFG_TUD_DFU_XFER_BUFSIZE)
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

static char const *string_desc_arr[] = {
        (const char[] ) { 0x09, 0x04 }, // 0: English (0x0409)
        "CurrentMakers",          // 1: Manufacturer
        "Streamline Firmware Update",    // 2: Product
        "123456",                      // 3: Serial

        /* * String Index 4: This is the Magic String for DFU Regions.
         * Format: "@InterfaceName /StartAddr/NumSectors*SectorSizeAttributes"
         * Example for 64KB RAM starting at 0x20000000:
         * a = Read/Write, e = Readable, g = Readable/Writable/Erasable
         */
        "@Internal WASM /0x00000000/320*001Kg"
};

static uint16_t _desc_str[64 + 1]; // Increased size for the long DFU string

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    size_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;

        const char *str = string_desc_arr[index];

        chr_count = strlen(str);
        if (chr_count > 63)
            chr_count = 63;

        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

uint8_t const* tud_descriptor_bos_cb(void) {
    return NULL;
}
