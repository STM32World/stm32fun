/**
 ******************************************************************************
 * @file           : flash_storage.h
 * @brief          : Flash storage header
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

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#define FIRST_SECTOR ((FLASH_STORAGE_ADDRESS - 0x8000000)  / FLASH_PAGE_SIZE)
#define FLASH_PAGE_WORDS (FLASH_PAGE_SIZE / sizeof(uint32_t))

// This extern definition tells the compiler that 'flash_storage'
// is a pointer defined elsewhere, pointing to the flash address.
extern uint32_t *flash_storage;

// Function to update data in flash memory.
// v: The target address in flash to begin the update.
// new: Pointer to the new data to be written.
// len: Length of the new data in bytes.
int flash_storage_update(uint32_t *v, uint32_t *new, uint32_t len);

#endif /* INC_FLASH_STORAGE_H_ */
