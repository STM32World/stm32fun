/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Lars Boegild Thomsen <lth@stm32world.com>
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bsp/board_api.h"
#include "tusb.h"
#include "lfs.h"

extern lfs_t littlefs;

//------------- device info -------------//
#define DEV_INFO_MANUFACTURER   "STM32World"
#define DEV_INFO_MODEL          "MTP LittleFS"
#define DEV_INFO_VERSION        "1.0"
#define DEV_PROP_FRIENDLY_NAME  "STM32World MTP"

//------------- storage info -------------//
#define STORAGE_DESCRIPTION { 'l', 'i', 't', 't', 'l', 'e', 'f', 's', 0 }
#define VOLUME_IDENTIFIER { 'v', 'o', 'l', 0 }

enum {
  STORAGE_DESC_LEN = TU_ARRAY_SIZE((uint16_t[]) STORAGE_DESCRIPTION),
  VOLUME_ID_LEN = TU_ARRAY_SIZE((uint16_t[])VOLUME_IDENTIFIER)
};

typedef MTP_STORAGE_INFO_STRUCT(STORAGE_DESC_LEN, VOLUME_ID_LEN) storage_info_t;

storage_info_t storage_info = {
  .storage_type = MTP_STORAGE_TYPE_FIXED_RAM,
  .filesystem_type = MTP_FILESYSTEM_TYPE_GENERIC_HIERARCHICAL,
  .access_capability = MTP_ACCESS_CAPABILITY_READ_WRITE,
  .max_capacity_in_bytes = 0,
  .free_space_in_bytes = 0,
  .free_space_in_objects = 0xFFFFFFFF,
  .storage_description = {
    .count = (TU_FIELD_SIZE(storage_info_t, storage_description)-1) / sizeof(uint16_t),
    .utf16 = STORAGE_DESCRIPTION
  },
  .volume_identifier = {
    .count = (TU_FIELD_SIZE(storage_info_t, volume_identifier)-1) / sizeof(uint16_t),
    .utf16 = VOLUME_IDENTIFIER
  }
};

#define FS_MAX_FILE_COUNT 64UL
#define FS_MAX_FILENAME_LEN 64
#define FS_FIXED_DATETIME "20260705T170000.0"

enum {
  SUPPORTED_STORAGE_ID = 0x00010001u
};

typedef struct {
  char path[256];
  uint32_t parent;
  uint16_t format;
  uint32_t size;
} mtp_handle_map_t;

static mtp_handle_map_t handle_table[FS_MAX_FILE_COUNT];
static uint32_t current_handle_count = 0;
static uint32_t send_obj_handle = 0;
static bool is_session_opened = false;

static lfs_file_t read_lfs_file;
static lfs_file_t write_lfs_file;
static bool read_file_is_open = false;
static bool write_file_is_open = false;

static int32_t fs_get_device_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_open_close_session(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_storage_ids(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_storage_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_device_properties(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object_handles(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_get_partial_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_delete_object(tud_mtp_cb_data_t* cb_data);
static int32_t fs_send_object_info(tud_mtp_cb_data_t* cb_data);
static int32_t fs_send_object(tud_mtp_cb_data_t* cb_data);

typedef int32_t (*fs_op_handler_t)(tud_mtp_cb_data_t* cb_data);
typedef struct {
  uint32_t op_code;
  fs_op_handler_t handler;
} fs_op_handler_dict_t;

fs_op_handler_dict_t fs_op_handler_dict[] = {
  { MTP_OP_GET_DEVICE_INFO,       fs_get_device_info    },
  { MTP_OP_OPEN_SESSION,          fs_open_close_session },
  { MTP_OP_CLOSE_SESSION,         fs_open_close_session },
  { MTP_OP_GET_STORAGE_IDS,       fs_get_storage_ids       },
  { MTP_OP_GET_STORAGE_INFO,      fs_get_storage_info      },
  { MTP_OP_GET_DEVICE_PROP_DESC,  fs_get_device_properties  },
  { MTP_OP_GET_DEVICE_PROP_VALUE, fs_get_device_properties },
  { MTP_OP_GET_OBJECT_HANDLES,    fs_get_object_handles    },
  { MTP_OP_GET_OBJECT_INFO,       fs_get_object_info       },
  { MTP_OP_GET_OBJECT,            fs_get_object            },
  { MTP_OP_GET_PARTIAL_OBJECT,    fs_get_partial_object    },
  { MTP_OP_DELETE_OBJECT,         fs_delete_object         },
  { MTP_OP_SEND_OBJECT_INFO,      fs_send_object_info      },
  { MTP_OP_SEND_OBJECT,           fs_send_object           },
};

static inline mtp_handle_map_t* fs_get_mapped_file(uint32_t handle) {
  if (handle == 0 || handle > current_handle_count) {
    return NULL;
  }
  return &handle_table[handle - 1];
}

static void utf16_to_ascii(const uint16_t* utf16, char* ascii, size_t max_len) {
  size_t i = 0;
  while (utf16[i] != 0 && i < max_len - 1) {
    ascii[i] = (char)(utf16[i] & 0x7F);
    i++;
  }
  ascii[i] = '\0';
}

static void ascii_to_utf16(const char* ascii, uint16_t* utf16) {
  size_t i = 0;
  while (ascii[i] != '\0' && i < (FS_MAX_FILENAME_LEN - 1)) {
    utf16[i] = (uint16_t)ascii[i];
    i++;
  }
  utf16[i] = 0;
}

//--------------------------------------------------------------------+
// Control Request callbacks
//--------------------------------------------------------------------+
bool tud_mtp_request_cancel_cb(tud_mtp_request_cb_data_t* cb_data) {
  (void) cb_data;
  return true;
}

bool tud_mtp_request_device_reset_cb(tud_mtp_request_cb_data_t* cb_data) {
  (void) cb_data;
  return true;
}

int32_t tud_mtp_request_get_extended_event_cb(tud_mtp_request_cb_data_t* cb_data) {
  (void) cb_data;
  return false;
}

int32_t tud_mtp_request_get_device_status_cb(tud_mtp_request_cb_data_t* cb_data) {
  uint16_t* buf16 = (uint16_t*)(uintptr_t) cb_data->buf;
  buf16[0] = 4;
  buf16[1] = MTP_RESP_OK;
  return 4;
}

//--------------------------------------------------------------------+
// Bulk Only Protocol Wrappers
//--------------------------------------------------------------------+
int32_t tud_mtp_command_received_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  fs_op_handler_t handler = NULL;
  for (size_t i = 0; i < TU_ARRAY_SIZE(fs_op_handler_dict); i++) {
    if (fs_op_handler_dict[i].op_code == command->header.code) {
      handler = fs_op_handler_dict[i].handler;
      break;
    }
  }

  int32_t resp_code = (handler == NULL) ? MTP_RESP_OPERATION_NOT_SUPPORTED : handler(cb_data);
  if (resp_code > MTP_RESP_UNDEFINED) {
    io_container->header->code = (uint16_t)resp_code;
    tud_mtp_response_send(io_container);
  }
  return resp_code;
}

int32_t tud_mtp_data_xfer_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;

  fs_op_handler_t handler = NULL;
  for (size_t i = 0; i < TU_ARRAY_SIZE(fs_op_handler_dict); i++) {
    if (fs_op_handler_dict[i].op_code == command->header.code) {
      handler = fs_op_handler_dict[i].handler;
      break;
    }
  }

  int32_t resp_code = (handler == NULL) ? MTP_RESP_OPERATION_NOT_SUPPORTED : handler(cb_data);
  if (resp_code > MTP_RESP_UNDEFINED) {
    io_container->header->code = (uint16_t)resp_code;
    tud_mtp_response_send(io_container);
  }
  return 0;
}

int32_t tud_mtp_data_complete_cb(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* resp = &cb_data->io_container;

  if (command->header.code == MTP_OP_GET_PARTIAL_OBJECT) {
    const uint32_t len = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
    (void) mtp_container_add_uint32(resp, len);
    resp->header->code = MTP_RESP_OK;
    tud_mtp_response_send(resp);
    return 0;
  }

  resp->header->type = MTP_CONTAINER_TYPE_RESPONSE_BLOCK;
  resp->header->code = (cb_data->xfer_result == XFER_RESULT_SUCCESS) ? MTP_RESP_OK : MTP_RESP_GENERAL_ERROR;
  resp->header->len = sizeof(mtp_container_header_t);
  tud_mtp_response_send(resp);
  return 0;
}

int32_t tud_mtp_response_complete_cb(tud_mtp_cb_data_t* cb_data) {
  (void) cb_data;
  return 0;
}

//--------------------------------------------------------------------+
// File System Operational Pipeline Core
//--------------------------------------------------------------------+
static int32_t fs_get_device_info(tud_mtp_cb_data_t* cb_data) {
  int32_t resp_code = 0;
  mtp_container_info_t* io_container = &cb_data->io_container;
  (void) mtp_container_add_cstring(io_container, DEV_INFO_MANUFACTURER);
  (void) mtp_container_add_cstring(io_container, DEV_INFO_MODEL);
  (void) mtp_container_add_cstring(io_container, DEV_INFO_VERSION);

  enum { MAX_SERIAL_NCHARS = 32 };
  uint16_t serial_utf16[MAX_SERIAL_NCHARS+1];
  size_t nchars = board_usb_get_serial(serial_utf16, MAX_SERIAL_NCHARS);
  serial_utf16[tu_min32(nchars, MAX_SERIAL_NCHARS)] = 0;
  (void) mtp_container_add_string(io_container, serial_utf16);

  if (!tud_mtp_data_send(io_container)) {
    resp_code = MTP_RESP_DEVICE_BUSY;
  }
  return resp_code;
}

static int32_t fs_open_close_session(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  if (command->header.code == MTP_OP_OPEN_SESSION) {
    if (is_session_opened) return MTP_RESP_SESSION_ALREADY_OPEN;
    is_session_opened = true;
  } else {
    if (!is_session_opened) return MTP_RESP_SESSION_NOT_OPEN;
    is_session_opened = false;
    if (read_file_is_open) { lfs_file_close(&littlefs, &read_lfs_file); read_file_is_open = false; }
    if (write_file_is_open) { lfs_file_close(&littlefs, &write_lfs_file); write_file_is_open = false; }
  }
  return MTP_RESP_OK;
}

static int32_t fs_get_storage_ids(tud_mtp_cb_data_t* cb_data) {
  mtp_container_info_t* io_container = &cb_data->io_container;
  uint32_t storage_ids [] = { SUPPORTED_STORAGE_ID };
  (void) mtp_container_add_auint32(io_container, 1, storage_ids);
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_storage_info(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t storage_id = command->params[0];
  TU_VERIFY(SUPPORTED_STORAGE_ID == storage_id, -1);

  lfs_ssize_t allocated_blocks = lfs_fs_size(&littlefs);
  if (allocated_blocks < 0) allocated_blocks = 0;

  storage_info.max_capacity_in_bytes = littlefs.cfg->block_count * littlefs.cfg->block_size;
  storage_info.free_space_in_bytes = storage_info.max_capacity_in_bytes - ((uint32_t)allocated_blocks * littlefs.cfg->block_size);

  (void) mtp_container_add_raw(io_container, &storage_info, sizeof(storage_info));
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_device_properties(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint16_t dev_prop_code = (uint16_t) command->params[0];

  if (command->header.code == MTP_OP_GET_DEVICE_PROP_DESC) {
    mtp_device_prop_desc_header_t device_prop_header;
    device_prop_header.device_property_code = dev_prop_code;
    switch (dev_prop_code) {
      case MTP_DEV_PROP_DEVICE_FRIENDLY_NAME:
        device_prop_header.datatype = MTP_DATA_TYPE_STR;
        device_prop_header.get_set = MTP_MODE_GET;
        (void) mtp_container_add_raw(io_container, &device_prop_header, sizeof(device_prop_header));
        (void) mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME);
        (void) mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME);
        (void) mtp_container_add_uint8(io_container, 0);
        tud_mtp_data_send(io_container);
        break;
      default:
        return MTP_RESP_PARAMETER_NOT_SUPPORTED;
    }
  } else {
    switch (dev_prop_code) {
      case MTP_DEV_PROP_DEVICE_FRIENDLY_NAME:
        (void) mtp_container_add_cstring(io_container, DEV_PROP_FRIENDLY_NAME);
        tud_mtp_data_send(io_container);
        break;
      default:
        return MTP_RESP_PARAMETER_NOT_SUPPORTED;
    }
  }
  return 0;
}

static int32_t fs_get_object_handles(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;

  const uint32_t storage_id = command->params[0];
  const uint32_t parent_handle = command->params[2];

  if (storage_id != 0xFFFFFFFFu && storage_id != SUPPORTED_STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }

  const char* lookup_dir_path = "/";
  if (parent_handle != 0 && parent_handle != 0xFFFFFFFFu) {
    mtp_handle_map_t* parent_obj = fs_get_mapped_file(parent_handle);
    if (parent_obj) lookup_dir_path = parent_obj->path;
  }

  uint32_t handles[FS_MAX_FILE_COUNT] = { 0 };
  uint32_t count = 0u;

  lfs_dir_t dynamic_dir;
  struct lfs_info entry_info;

  if (lfs_dir_open(&littlefs, &dynamic_dir, lookup_dir_path) == LFS_ERR_OK) {
    while (lfs_dir_read(&littlefs, &dynamic_dir, &entry_info) > 0) {
      if (strcmp(entry_info.name, ".") == 0 || strcmp(entry_info.name, "..") == 0) continue;

      char clean_path[512];
      if (strcmp(lookup_dir_path, "/") == 0) {
        snprintf(clean_path, sizeof(clean_path), "/%s", entry_info.name);
      } else {
        snprintf(clean_path, sizeof(clean_path), "%s/%s", lookup_dir_path, entry_info.name);
      }

      int index = -1;
      for (uint32_t i = 0; i < current_handle_count; i++) {
        if (strcmp(handle_table[i].path, clean_path) == 0) {
          index = (int)i;
          break;
        }
      }

      if (index == -1 && current_handle_count < FS_MAX_FILE_COUNT) {
        index = (int)current_handle_count++;
        strncpy(handle_table[index].path, clean_path, sizeof(handle_table[index].path));
        handle_table[index].parent = (parent_handle == 0xFFFFFFFFu) ? 0u : parent_handle;
        handle_table[index].format = (entry_info.type == LFS_TYPE_DIR) ? MTP_OBJ_FORMAT_ASSOCIATION : MTP_OBJ_FORMAT_TEXT;
        handle_table[index].size = entry_info.size;
      }

      if (index != -1 && count < FS_MAX_FILE_COUNT) {
        handles[count++] = (uint32_t)(index + 1);
      }
    }
    lfs_dir_close(&littlefs, &dynamic_dir);
  }

  (void) mtp_container_add_auint32(io_container, count, handles);
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_object_info(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t obj_handle = command->params[0];

  mtp_handle_map_t* f = fs_get_mapped_file(obj_handle);
  if (f == NULL) return MTP_RESP_INVALID_OBJECT_HANDLE;

  struct lfs_info entry_info;
  if (lfs_stat(&littlefs, f->path, &entry_info) == LFS_ERR_OK) {
    f->size = entry_info.size;
  }

  mtp_object_info_header_t obj_info_header = {
    .storage_id = SUPPORTED_STORAGE_ID,
    .object_format = f->format,
    .protection_status = 0,
    .object_compressed_size = f->size,
    .thumb_format = MTP_OBJ_FORMAT_UNDEFINED,
    .parent_object = f->parent,
    .association_type = (f->format == MTP_OBJ_FORMAT_ASSOCIATION) ? MTP_ASSOCIATION_GENERIC_FOLDER : MTP_ASSOCIATION_UNDEFINED,
  };

  const char* base_filename = strrchr(f->path, '/');
  base_filename = base_filename ? base_filename + 1 : f->path;

  uint16_t name_utf16[FS_MAX_FILENAME_LEN];
  ascii_to_utf16(base_filename, name_utf16);

  (void) mtp_container_add_raw(io_container, &obj_info_header, sizeof(obj_info_header));
  (void) mtp_container_add_string(io_container, name_utf16);
  (void) mtp_container_add_cstring(io_container, FS_FIXED_DATETIME);
  (void) mtp_container_add_cstring(io_container, FS_FIXED_DATETIME);
  (void) mtp_container_add_cstring(io_container, "");
  tud_mtp_data_send(io_container);
  return 0;
}

static int32_t fs_get_object(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t obj_handle = command->params[0];
  mtp_handle_map_t* f = fs_get_mapped_file(obj_handle);

  if (f == NULL || f->format == MTP_OBJ_FORMAT_ASSOCIATION) return MTP_RESP_INVALID_OBJECT_HANDLE;

  if (cb_data->phase == MTP_PHASE_COMMAND) {
    if (read_file_is_open) { lfs_file_close(&littlefs, &read_lfs_file); read_file_is_open = false; }

    if (lfs_file_open(&littlefs, &read_lfs_file, f->path, LFS_O_RDONLY) != LFS_ERR_OK) {
      return MTP_RESP_GENERAL_ERROR;
    }
    read_file_is_open = true;

    lfs_ssize_t read_bytes = lfs_file_read(&littlefs, &read_lfs_file, io_container->payload, io_container->payload_bytes);
    if (read_bytes < 0) read_bytes = 0;

    io_container->header->len = sizeof(mtp_container_header_t) + read_bytes;
    tud_mtp_data_send(io_container);

    if ((uint32_t)read_bytes >= f->size) {
      lfs_file_close(&littlefs, &read_lfs_file);
      read_file_is_open = false;
    }
  } else if (cb_data->phase == MTP_PHASE_DATA) {
    const uint32_t offset = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
    if (offset < f->size && read_file_is_open) {
      lfs_ssize_t read_bytes = lfs_file_read(&littlefs, &read_lfs_file, io_container->payload, io_container->payload_bytes);
      if (read_bytes < 0) read_bytes = 0;

      io_container->header->len = sizeof(mtp_container_header_t) + read_bytes;
      tud_mtp_data_send(io_container);
    }
    if (cb_data->total_xferred_bytes - sizeof(mtp_container_header_t) >= f->size) {
      if (read_file_is_open) {
        lfs_file_close(&littlefs, &read_lfs_file);
        read_file_is_open = false;
      }
    }
  }
  return 0;
}

static int32_t fs_get_partial_object(tud_mtp_cb_data_t* cb_data) {
  (void) cb_data;
  return 0;
}

static int32_t fs_send_object_info(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  mtp_container_info_t* io_container = &cb_data->io_container;
  const uint32_t storage_id = command->params[0];
  const uint32_t parent_handle = command->params[1];

  if (!is_session_opened) return MTP_RESP_SESSION_NOT_OPEN;
  if (storage_id != 0xFFFFFFFFu && storage_id != SUPPORTED_STORAGE_ID) return MTP_RESP_INVALID_STORAGE_ID;

  if (cb_data->phase == MTP_PHASE_COMMAND) {
    tud_mtp_data_receive(io_container);
  } else if (cb_data->phase == MTP_PHASE_DATA) {
    mtp_object_info_header_t* obj_info = (mtp_object_info_header_t*) io_container->payload;

    uint16_t name_utf16[FS_MAX_FILENAME_LEN];
    uint8_t* name_buffer_ptr = io_container->payload + sizeof(mtp_object_info_header_t);
    mtp_container_get_string(name_buffer_ptr, name_utf16);

    char ascii_filename[FS_MAX_FILENAME_LEN];
    utf16_to_ascii(name_utf16, ascii_filename, sizeof(ascii_filename));

    char combined_path[512];
    const char* base_dir = "/";
    if (parent_handle != 0 && parent_handle != 0xFFFFFFFFu) {
      mtp_handle_map_t* parent_obj = fs_get_mapped_file(parent_handle);
      if (parent_obj) base_dir = parent_obj->path;
    }

    if (strcmp(base_dir, "/") == 0) {
      snprintf(combined_path, sizeof(combined_path), "/%s", ascii_filename);
    } else {
      snprintf(combined_path, sizeof(combined_path), "%s/%s", base_dir, ascii_filename);
    }

    if (current_handle_count >= FS_MAX_FILE_COUNT) return MTP_RESP_STORE_FULL;

    uint32_t allocated_idx = current_handle_count++;
    mtp_handle_map_t* f = &handle_table[allocated_idx];

    strncpy(f->path, combined_path, sizeof(f->path));
    f->parent = (parent_handle == 0xFFFFFFFFu) ? 0u : parent_handle;
    f->format = obj_info->object_format;
    f->size = obj_info->object_compressed_size;

    send_obj_handle = allocated_idx + 1;

    if (f->format == MTP_OBJ_FORMAT_ASSOCIATION) {
      lfs_mkdir(&littlefs, f->path);
    } else {
      if (write_file_is_open) {
        lfs_file_close(&littlefs, &write_lfs_file);
        write_file_is_open = false;
      }

      if (parent_handle != 0 && parent_handle != 0xFFFFFFFFu) {
        lfs_mkdir(&littlefs, base_dir);
      }

      if (lfs_file_open(&littlefs, &write_lfs_file, f->path, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC) != LFS_ERR_OK) {
        current_handle_count--;
        return MTP_RESP_GENERAL_ERROR;
      }
      write_file_is_open = true;
    }

    (void) mtp_container_add_uint32(io_container, SUPPORTED_STORAGE_ID);
    (void) mtp_container_add_uint32(io_container, parent_handle);
    (void) mtp_container_add_uint32(io_container, send_obj_handle);
    return MTP_RESP_OK;
  }
  return 0;
}

static int32_t fs_send_object(tud_mtp_cb_data_t* cb_data) {
  mtp_container_info_t* io_container = &cb_data->io_container;
  mtp_handle_map_t* f = fs_get_mapped_file(send_obj_handle);
  if (f == NULL) return MTP_RESP_INVALID_OBJECT_HANDLE;

  if (cb_data->phase == MTP_PHASE_COMMAND) {
    tud_mtp_data_receive(io_container);
    return 0;
  } else if (cb_data->phase == MTP_PHASE_DATA) {
    if (f->format != MTP_OBJ_FORMAT_ASSOCIATION && io_container->payload_bytes > 0 && write_file_is_open) {
      lfs_file_write(&littlefs, &write_lfs_file, io_container->payload, io_container->payload_bytes);
    }

    uint32_t received_bytes = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
    if (received_bytes >= f->size) {
      if (f->format != MTP_OBJ_FORMAT_ASSOCIATION && write_file_is_open) {
        lfs_file_close(&littlefs, &write_lfs_file);
        write_file_is_open = false;
      }
      return MTP_RESP_OK; // <-- Returns the terminal code directly to push Dolphin over the finish line
    } else {
      tud_mtp_data_receive(io_container);
    }
  }
  return 0;
}

static int32_t fs_delete_object(tud_mtp_cb_data_t* cb_data) {
  const mtp_container_command_t* command = cb_data->command_container;
  const uint32_t obj_handle = command->params[0];

  if (!is_session_opened) return MTP_RESP_SESSION_NOT_OPEN;
  mtp_handle_map_t* f = fs_get_mapped_file(obj_handle);
  if (f == NULL) return MTP_RESP_INVALID_OBJECT_HANDLE;

  if (lfs_remove(&littlefs, f->path) == LFS_ERR_OK) {
    f->path[0] = 0;
    return MTP_RESP_OK;
  }
  return MTP_RESP_GENERAL_ERROR;
}
