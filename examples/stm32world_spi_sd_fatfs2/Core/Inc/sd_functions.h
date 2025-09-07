/******************************************************************************
 *  File        : sd_functions.h
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/

#ifndef __SD_FUNCTIONS_H__
#define __SD_FUNCTIONS_H__

#include "fatfs.h"
#include <stdint.h>

extern char sd_path[];

// Mount and unmount
int sd_mount(void);
int sd_unmount(void);

// Basic file operations
int sd_write_file(const char *filename, const char *text);
int sd_append_file(const char *filename, const char *text);
int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read);
int sd_delete_file(const char *filename);
int sd_rename_file(const char *oldname, const char *newname);


// Directory handling
FRESULT sd_create_directory(const char *path);
void sd_list_directory_recursive(const char *path, int depth);
void sd_list_files(void);

// Space information
int sd_get_space_kb(void);

//csv File operations
// CSV Record structure
typedef struct CsvRecord {
    char field1[32];
    char field2[32];
    int value;
} CsvRecord;

// CSV reader (caller defines record array)
int sd_read_csv(const char *filename, CsvRecord *records, int max_records, int *record_count);

#endif // __SD_FUNCTIONS_H__
