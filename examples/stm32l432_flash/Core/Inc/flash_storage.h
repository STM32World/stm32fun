/*
 * flash_storage.h
 *
 *  Created on: Nov 6, 2025
 *      Author: lth
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

//#define PAGE_SIZE FLASH_PAGE_SIZE

extern uint32_t *flash_storage;
extern unsigned int _flash_storage_start;

int flash_storage_update(uint32_t *v, uint32_t *new, uint32_t len);

#endif /* INC_FLASH_STORAGE_H_ */
