/**
 ******************************************************************************
 * @file           : lwip_fs_fatfs.c
 * @brief          : Functions to read files from /www on the sd card
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

#include "lwip/apps/fs.h"
#include "lwip/mem.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

#define WEBROOT "/www"

int fs_open_custom(struct fs_file *file, const char *name) {
    FIL *f;
    FRESULT res;
    char full_path[256];

    if (strlen(name) + strlen(WEBROOT) >= sizeof(full_path)) {
        return 0;
    }
    snprintf(full_path, sizeof(full_path), "%s%s", WEBROOT, name);

    f = (FIL *)mem_malloc(sizeof(FIL));
    if (f == NULL) {
        return 0;
    }

    res = f_open(f, full_path, FA_READ);
    if (res != FR_OK) {
        mem_free(f);
        return 0;
    }

    file->pextension = f;
    file->len = f_size(f);
    file->index = 0;
    file->data = NULL;
    file->flags = 1;

    return 1;
}

void fs_close_custom(struct fs_file *file) {
    if (file && file->pextension) {
        FIL *f = (FIL *)file->pextension;
        f_close(f);
        mem_free(f);
        file->pextension = NULL;
    }
}

int fs_read_custom(struct fs_file *file, char *buffer, int count) {
    UINT br;
    FIL *f = (FIL *)file->pextension;

    if (f == NULL) {
        return -1;
    }

    if (file->index >= file->len) {
        return FS_READ_EOF;
    }

    if (f_read(f, buffer, count, &br) == FR_OK) {
        if (br == 0) {
            return FS_READ_EOF;
        }
        file->index += br;
        return br;
    }

    return -1;
}

