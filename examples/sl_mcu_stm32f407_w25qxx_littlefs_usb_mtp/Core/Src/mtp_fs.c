#include "tusb.h"
#include <string.h>
#include <stdio.h>

#define STORAGE_ID   0x00010001

typedef struct {
    uint32_t handle;
    char filename[32];
    uint32_t size;
} mtp_file_entry_t;

static mtp_file_entry_t file_system_vols[] = {
    { 1, "readme.txt", 24 },
    { 2, "data.bin",   1024 }
};
#define FILE_COUNT (sizeof(file_system_vols)/sizeof(file_system_vols[0]))

static uint8_t pack_mtp_string(uint8_t* buf, const char* ascii_str) {
    uint8_t len = strlen(ascii_str);
    buf[0] = len + 1;
    for (uint8_t i = 0; i < len; i++) {
        buf[1 + (i * 2)] = (uint8_t)ascii_str[i];
        buf[2 + (i * 2)] = 0x00;
    }
    buf[1 + (len * 2)] = 0x00;
    buf[2 + (len * 2)] = 0x00;
    return 1 + (len * 2) + 2;
}

uint8_t tud_mtp_get_storage_ids_cb(uint32_t* id_list, uint8_t max_count) {
    printf("[MTP] GetStorageIDs: max_count=%d\n", max_count);
    if (max_count < 1) return 0;
    id_list[0] = STORAGE_ID;
    return 1;
}

void tud_mtp_get_storage_metadata_cb(uint32_t storage_id, mtp_storage_type_t* metadata) {
    printf("[MTP] GetStorageMetadata: id=0x%08lX\n", (unsigned long)storage_id);
    (void) storage_id;
    *metadata = MTP_STORAGE_TYPE_FIXED_RAM;
}

uint16_t tud_mtp_get_storage_name_cb(uint32_t storage_id, char* name_buf, uint16_t max_len) {
    printf("[MTP] GetStorageName: max_len=%d\n", max_len);
    (void) storage_id;
    strncpy(name_buf, "STM32 Flash Drive", max_len);
    return strlen(name_buf);
}

uint32_t tud_mtp_get_object_count_cb(uint32_t storage_id, uint32_t object_format, uint32_t parent_handle) {
    printf("[MTP] GetObjectCount: format=0x%04lX, parent=0x%08lX\n", (unsigned long)object_format, (unsigned long)parent_handle);
    (void) storage_id; (void) object_format; (void) parent_handle;
    return FILE_COUNT;
}

uint8_t tud_mtp_get_object_handles_cb(uint32_t storage_id, uint32_t object_format, uint32_t parent_handle, uint32_t* handle_list, uint8_t max_count) {
    printf("[MTP] GetObjectHandles: format=0x%04lX, parent=0x%08lX, max=%d\n", (unsigned long)object_format, (unsigned long)parent_handle, max_count);
    (void) storage_id; (void) object_format; (void) parent_handle;

    uint8_t count = 0;
    for (uint32_t i = 0; i < FILE_COUNT && count < max_count; i++) {
        handle_list[count++] = file_system_vols[i].handle;
    }
    return count;
}

uint8_t tud_mtp_get_object_info_cb(uint32_t storage_id, uint32_t handle, mtp_object_info_header_t* obj_info) {
    printf("[MTP] GetObjectInfo: handle=%lu\n", (unsigned long)handle);
    (void) storage_id;

    for (uint32_t i = 0; i < FILE_COUNT; i++) {
        if (file_system_vols[i].handle == handle) {
            memset(obj_info, 0, sizeof(mtp_object_info_header_t));
            obj_info->storage_id = STORAGE_ID;
            obj_info->object_format = MTP_OBJ_FORMAT_UNDEFINED;
            obj_info->parent_object = 0;
            obj_info->object_compressed_size = file_system_vols[i].size;
            return 1;
        }
    }
    return 0;
}

uint16_t tud_mtp_get_object_name_cb(uint32_t storage_id, uint32_t handle, char* name_buf, uint16_t max_len) {
    printf("[MTP] GetObjectName: handle=%lu\n", (unsigned long)handle);
    (void) storage_id;
    for (uint32_t i = 0; i < FILE_COUNT; i++) {
        if (file_system_vols[i].handle == handle) {
            strncpy(name_buf, file_system_vols[i].filename, max_len);
            return strlen(name_buf);
        }
    }
    return 0;
}

uint16_t tud_mtp_get_object_prop_value_cb(uint32_t storage_id, uint32_t handle, uint16_t prop_code, uint8_t* buffer, uint16_t max_len) {
    printf("[MTP] GetObjectPropValue: handle=%lu, prop=0x%04X\n", (unsigned long)handle, prop_code);
    (void) storage_id; (void) max_len;

    for (uint32_t i = 0; i < FILE_COUNT; i++) {
        if (file_system_vols[i].handle == handle) {
            switch (prop_code) {
                case MTP_OBJ_PROP_STORAGE_ID: {
                    uint32_t store_id = STORAGE_ID;
                    memcpy(buffer, &store_id, 4);
                    return 4;
                }
                case MTP_OBJ_PROP_OBJECT_FORMAT: {
                    uint16_t format = MTP_OBJ_FORMAT_UNDEFINED;
                    memcpy(buffer, &format, 2);
                    return 2;
                }
                case MTP_OBJ_PROP_OBJECT_SIZE: {
                    uint64_t size64 = file_system_vols[i].size;
                    memcpy(buffer, &size64, 8);
                    return 8;
                }
                case MTP_OBJ_PROP_PARENT_OBJECT: {
                    uint32_t parent = 0;
                    memcpy(buffer, &parent, 4);
                    return 4;
                }
                case MTP_OBJ_PROP_NAME:
                    return pack_mtp_string(buffer, file_system_vols[i].filename);
                default:
                    return 0;
            }
        }
    }
    return 0;
}

uint32_t tud_mtp_get_object_prop_list_cb(uint32_t storage_id, uint32_t handle, uint32_t object_format, uint32_t prop_code, uint32_t group_code, uint8_t* buffer, uint32_t max_len) {
    printf("[MTP] GetObjectPropList: handle=0x%08lX, prop=0x%08lX\n", (unsigned long)handle, (unsigned long)prop_code);
    (void) storage_id; (void) object_format; (void) group_code; (void) max_len;

    uint32_t write_offset = 0;
    uint32_t element_count = 0;
    uint8_t* count_ptr = buffer;
    write_offset += 4;

    for (uint32_t i = 0; i < FILE_COUNT; i++) {
        if (handle == 0xFFFFFFFF || handle == 0 || file_system_vols[i].handle == handle) {

            uint16_t props_to_respond[] = {
                MTP_OBJ_PROP_STORAGE_ID,
                MTP_OBJ_PROP_OBJECT_FORMAT,
                MTP_OBJ_PROP_OBJECT_SIZE,
                MTP_OBJ_PROP_PARENT_OBJECT,
                MTP_OBJ_PROP_NAME
            };
            uint8_t prop_num = sizeof(props_to_respond) / sizeof(props_to_respond[0]);

            for (uint8_t p = 0; p < prop_num; p++) {
                if (prop_code == 0 || prop_code == 0xFFFFFFFF || prop_code == props_to_respond[p]) {

                    memcpy(buffer + write_offset, &file_system_vols[i].handle, 4);   write_offset += 4;
                    memcpy(buffer + write_offset, &props_to_respond[p], 2);          write_offset += 2;

                    switch (props_to_respond[p]) {
                        case MTP_OBJ_PROP_STORAGE_ID: {
                            uint16_t type = MTP_DATA_TYPE_UINT32;
                            memcpy(buffer + write_offset, &type, 2);                 write_offset += 2;
                            uint32_t store_id = STORAGE_ID;
                            memcpy(buffer + write_offset, &store_id, 4);             write_offset += 4;
                            break;
                        }
                        case MTP_OBJ_PROP_OBJECT_FORMAT: {
                            uint16_t type = MTP_DATA_TYPE_UINT16;
                            memcpy(buffer + write_offset, &type, 2);                 write_offset += 2;
                            uint16_t format = MTP_OBJ_FORMAT_UNDEFINED;
                            memcpy(buffer + write_offset, &format, 2);               write_offset += 2;
                            break;
                        }
                        case MTP_OBJ_PROP_OBJECT_SIZE: {
                            uint16_t type = MTP_DATA_TYPE_UINT64;
                            memcpy(buffer + write_offset, &type, 2);                 write_offset += 2;
                            uint64_t size64 = file_system_vols[i].size;
                            memcpy(buffer + write_offset, &size64, 8);               write_offset += 8;
                            break;
                        }
                        case MTP_OBJ_PROP_PARENT_OBJECT: {
                            uint16_t type = MTP_DATA_TYPE_UINT32;
                            memcpy(buffer + write_offset, &type, 2);                 write_offset += 2;
                            uint32_t parent = 0;
                            memcpy(buffer + write_offset, &parent, 4);               write_offset += 4;
                            break;
                        }
                        case MTP_OBJ_PROP_NAME: {
                            uint16_t type = MTP_DATA_TYPE_STR;
                            memcpy(buffer + write_offset, &type, 2);                 write_offset += 2;
                            uint8_t packed_len = pack_mtp_string(buffer + write_offset, file_system_vols[i].filename);
                            write_offset += packed_len;
                            break;
                        }
                    }
                    element_count++;
                }
            }
        }
    }

    memcpy(count_ptr, &element_count, 4);
    return write_offset;
}

uint32_t tud_mtp_read_file_cb(uint32_t storage_id, uint32_t handle, uint64_t offset, uint32_t bytes_to_read, uint8_t* buffer) {
    (void) storage_id;
    for (uint32_t i = 0; i < FILE_COUNT; i++) {
        if (file_system_vols[i].handle == handle) {
            if (offset >= file_system_vols[i].size) return 0;
            if (offset + bytes_to_read > file_system_vols[i].size) {
                bytes_to_read = file_system_vols[i].size - offset;
            }
            memset(buffer, 0xAA, bytes_to_read);
            return bytes_to_read;
        }
    }
    return 0;
}

uint32_t tud_mtp_create_object_cb(uint32_t storage_id, uint32_t parent_handle, mtp_object_info_header_t const* obj_info) {
    (void) storage_id; (void) parent_handle; (void) obj_info;
    return 0;
}

uint32_t tud_mtp_write_file_cb(uint32_t storage_id, uint32_t handle, uint64_t offset, uint32_t bytes_to_write, uint8_t* buffer) {
    (void) storage_id; (void) handle; (void) offset; (void) buffer;
    return bytes_to_write;
}

uint8_t tud_mtp_delete_object_cb(uint32_t storage_id, uint32_t handle) {
    (void) storage_id; (void) handle;
    return 1;
}
