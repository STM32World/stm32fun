/******************************************************************************
 *  File        : sd_benchmark.c
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

#include "sd_benchmark.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "sd_functions.h"

/***************************************************************
 * 🔧 USER-MODIFIABLE SECTION
 * You are free to edit anything below this line
 ***************************************************************/

#define USE_DMA 1
#define TEST_SIZE 512000 // 500KB Test File

/***************************************************************
 * 🚫 DO NOT MODIFY BELOW THIS LINE
 * Auto-generated/system-managed code. Changes may be lost.
 ***************************************************************/

#if USE_DMA
#define MODE_STR "DMA"
#else
#define MODE_STR "Polling"
#endif

uint32_t sd_benchmark_write(const char *filename, uint32_t size_bytes) {
    FIL file;
    UINT written;
    uint8_t buffer[512];
    memset(buffer, 0xAA, sizeof(buffer));

    FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("f_open failed: %d\r\n", res);
        return 0;
    }

    uint32_t start = HAL_GetTick();
    uint32_t remaining = size_bytes;

    while (remaining > 0) {
        UINT to_write = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        res = f_write(&file, buffer, to_write, &written);
        if (res != FR_OK || written != to_write) {
            printf("f_write error\r\n");
            break;
        }
        remaining -= written;
    }

    f_close(&file);
    uint32_t elapsed = HAL_GetTick() - start;
    printf("[%s] Write %lu bytes in %lu ms\r\n", MODE_STR, size_bytes, elapsed);
    return elapsed;
}

uint32_t sd_benchmark_read(const char *filename, uint32_t size_bytes) {
    FIL file;
    UINT read;
    uint8_t buffer[512];

    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("f_open failed: %d\r\n", res);
        return 0;
    }

    uint32_t start = HAL_GetTick();
    uint32_t remaining = size_bytes;

    while (remaining > 0) {
        UINT to_read = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        res = f_read(&file, buffer, to_read, &read);
        if (res != FR_OK || read != to_read) {
            printf("f_read error\r\n");
            break;
        }
        remaining -= read;
    }

    f_close(&file);
    uint32_t elapsed = HAL_GetTick() - start;
    printf("[%s] Read %lu bytes in %lu ms\r\n", MODE_STR, size_bytes, elapsed);
    return elapsed;
}

void sd_benchmark(void) {
    if (sd_mount() == FR_OK) {
        printf("Starting Benchmark Test\r\n");
        uint32_t w = sd_benchmark_write("bench.bin", TEST_SIZE);
        uint32_t r = sd_benchmark_read("bench.bin", TEST_SIZE);

        if (w > 0) printf("Write speed: %lu KB/s\r\n", (TEST_SIZE / 1024 * 1000) / w);
        if (r > 0) printf("Read  speed: %lu KB/s\r\n", (TEST_SIZE / 1024 * 1000) / r);

        sd_unmount();
    }
}
