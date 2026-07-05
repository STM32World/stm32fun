#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

// MCU Definition & Port Routing Configuration
#define CFG_TUSB_MCU                                        OPT_MCU_STM32F4
#define CFG_TUD_ENABLED                                     1

// Map STM32F407 Native USB OTG FS (Port 0) directly to Device Stack Mode
#define CFG_TUSB_RHPORT0_MODE                               OPT_MODE_DEVICE

// MTP Class Main Buffer Settings
#define CFG_TUD_MTP                                         1
#define CFG_TUD_MTP_EP_BUFSIZE                              64
#define CFG_TUD_MTP_EP_CONTROL_BUFSIZE                      64

// MTP Device Info Metadata Configuration
#define CFG_TUD_MTP_DEVICEINFO_EXTENSIONS                   ""

#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_OPERATIONS \
    MTP_OP_GET_DEVICE_INFO, \
    MTP_OP_OPEN_SESSION, \
    MTP_OP_CLOSE_SESSION, \
    MTP_OP_GET_STORAGE_IDS, \
    MTP_OP_GET_STORAGE_INFO, \
    MTP_OP_GET_NUM_OBJECTS, \
    MTP_OP_GET_OBJECT_HANDLES, \
    MTP_OP_GET_OBJECT_INFO, \
    MTP_OP_GET_OBJECT, \
    MTP_OP_DELETE_OBJECT, \
    MTP_OP_SEND_OBJECT_INFO, \
    MTP_OP_SEND_OBJECT

#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_EVENTS
#define CFG_TUD_MTP_DEVICEINFO_SUPPORTED_DEVICE_PROPERTIES
#define CFG_TUD_MTP_DEVICEINFO_CAPTURE_FORMATS

// Tells the host that we handle generic files transparently
#define CFG_TUD_MTP_DEVICEINFO_PLAYBACK_FORMATS \
    MTP_OBJ_FORMAT_UNDEFINED

#ifdef __cplusplus
}
#endif

#endif // _TUSB_CONFIG_H_
