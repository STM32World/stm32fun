/**
 ******************************************************************************
 * @file           : flash_storage.c
 * @brief          : Flash storage source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STM32World <lth@stm32world.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "flash_storage.h"

uint32_t *flash_storage = (uint32_t*) FLASH_STORAGE_ADDRESS;

uint32_t sector_number(uint32_t *address) {
    uint32_t s = ((uint32_t) address - FLASH_STORAGE_ADDRESS) / FLASH_PAGE_SIZE;
    return s;
}

uint32_t sector_offset(uint32_t *address) {
    uint32_t sn = sector_number(address);
    uint32_t page_start_addr = FLASH_STORAGE_ADDRESS + (sn * FLASH_PAGE_SIZE);
    uint32_t o = address - (uint32_t*) page_start_addr;
    return o; // Offset in 4-byte words
}

int flash_storage_update(uint32_t *v, uint32_t *new, uint32_t len) {
    int ret = 0;

    uint32_t sn = sector_number(v);
    uint32_t word_offset = sector_offset(v);
    uint32_t byte_offset = word_offset * sizeof(uint32_t); // Convert to byte offset

    printf("sn=0x%08lx word_offset=0x%08lx byte_offset=0x%08lx\n", sn, word_offset, byte_offset);

    // Get the base address of the page being programmed
    uint32_t page_start_addr = FLASH_STORAGE_ADDRESS + (sn * FLASH_PAGE_SIZE);

    // Temporary buffer for one page
    uint8_t d[FLASH_PAGE_SIZE] = { 0 };

    // Read the entire old page into RAM buffer (CRITICAL FIX) ---
    memcpy(d, (uint8_t*) page_start_addr, FLASH_PAGE_SIZE);

    // Update the buffer with new data ---
    memcpy(&d[byte_offset], new, len);

    HAL_FLASH_Unlock();

    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef erase = { 0 };
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = FIRST_SECTOR + sn;
    erase.NbPages = 1;

    printf("Attempting to erase page: %lu\n", erase.Page);

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        printf("Erase failed! HAL Error: %lu\n", HAL_FLASH_GetError());
        HAL_FLASH_Lock();
        return -1;
    }

    // Loop iterates FLASH_PAGE_SIZE / 8 times (2048 / 8 = 256 times)
    for (int i = 0; i < (FLASH_PAGE_SIZE / 8); ++i) {
        // Calculate the physical address (advancing by i * 8 bytes)
        uint32_t program_address = page_start_addr + (i * 8);

        // Get the 8-byte (uint64_t) data from the RAM buffer 'd'
        uint64_t data_to_write = *((uint64_t*) &d[i * 8]); // Data must be 64-bit

        //printf("Update 0x%08lx\n", program_address);

        // Program the Double Word
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, program_address, data_to_write) != HAL_OK) {
            printf("Program error: Index %d Address 0x%08lx - HAL Error: %lu\n",
                    i, program_address, HAL_FLASH_GetError());
            ret = -1;
            break;
        }
    }

    HAL_FLASH_Lock();

    return ret;
}
