/*
 * flash_storage.c
 *
 *  Created on: Nov 6, 2025
 *      Author: lth
 */

#include <string.h>
#include "main.h"
#include "flash_storage.h"

//uint32_t flash_storage __attribute__((section(".flash_data")));
uint32_t *flash_storage = (uint32_t *)FLASH_STORAGE_ADDRESS;

uint32_t sector_number(uint32_t *address) {
    uint32_t s =  (address - (uint32_t *)FLASH_STORAGE_ADDRESS) / FLASH_PAGE_SIZE;
    return s;
}

uint32_t sector_offset(uint32_t *address) {
    uint32_t s = sector_number(address);
    uint32_t o = address - (uint32_t *)(s * FLASH_PAGE_SIZE);
    return o;
}

uint32_t *sector_address(uint32_t *address) {
    uint32_t s = ((address - (uint32_t *)FLASH_STORAGE_ADDRESS) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    return (uint32_t *)s;
}

int flash_storage_update(uint32_t *v, uint32_t *new, uint32_t len) {
    int ret = 0;

    // Temporary buffer for one page
    uint8_t d[FLASH_PAGE_SIZE] = {0};

    memcpy(&d, (uint32_t *)sector_address(v), FLASH_PAGE_SIZE);

    d[sector_offset(v)] = *new;


    return ret;
}
