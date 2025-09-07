/******************************************************************************
 *  File        : sd_spi.h (SDSC/SDHC support)
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

#ifndef __SD_SPI_H__
#define __SD_SPI_H__

#include "main.h"  // or your specific STM32 family header
#include <stdint.h>

#define CMD0  (0)
#define CMD8  (8)
#define CMD17 (17)
#define CMD24 (24)
#define CMD55 (55)
#define CMD58 (58)
#define ACMD41 (41)

typedef enum {
    SD_OK = 0,
    SD_ERROR
} SD_Status;

extern uint8_t card_initialized;

SD_Status SD_SPI_Init(void);
SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
uint8_t sd_is_sdhc(void);

#endif // __SD_SPI_H__
