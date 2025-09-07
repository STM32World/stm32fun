/******************************************************************************
 *  File        : sd_spi.c (SDSC/SDHC support)
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

#include "sd_spi.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/***************************************************************
 * 🔧 USER-MODIFIABLE SECTION
 * You are free to edit anything below this line
 ***************************************************************/

#define USE_DMA 1

extern SPI_HandleTypeDef hspi1;
#define SD_SPI_HANDLE hspi1

#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

/***************************************************************
 * 🚫 DO NOT MODIFY BELOW THIS LINE
 * Auto-generated/system-managed code. Changes may be lost.
 ***************************************************************/

#if USE_DMA
volatile int dma_tx_done = 0;
volatile int dma_rx_done = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &SD_SPI_HANDLE) dma_tx_done = 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) dma_rx_done = 1;
}
#endif

static void SD_TransmitByte(uint8_t data) {
    HAL_SPI_Transmit(&SD_SPI_HANDLE, &data, 1, HAL_MAX_DELAY);
}

static uint8_t SD_ReceiveByte(void) {
    uint8_t dummy = 0xFF, data = 0;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &dummy, &data, 1, HAL_MAX_DELAY);
    return data;
}

static void SD_TransmitBuffer(const uint8_t *buffer, uint16_t len) {
#if USE_DMA
    dma_tx_done = 0;
    HAL_SPI_Transmit_DMA(&SD_SPI_HANDLE, (uint8_t *)buffer, len);
    while (!dma_tx_done);
#else
    HAL_SPI_Transmit(&SD_SPI_HANDLE, (uint8_t *)buffer, len, HAL_MAX_DELAY);
#endif
}

static void SD_ReceiveBuffer(uint8_t *buffer, uint16_t len) {
#if USE_DMA
	static uint8_t tx_dummy[512];
    for (int i = 0; i < len; i++) tx_dummy[i] = 0xFF;  // Fill with 0xFF
    dma_rx_done = 0;
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_dummy, buffer, len);
    while (!dma_rx_done);
#else
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = SD_ReceiveByte();
    }
#endif
}

static SD_Status SD_WaitReady(void) {
    uint32_t timeout = HAL_GetTick() + 500;
    uint8_t resp;
    do {
        resp = SD_ReceiveByte();
        if (resp == 0xFF) return SD_OK;
    } while (HAL_GetTick() < timeout);
    return SD_ERROR;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t response, retry = 0xFF;

    SD_WaitReady();
    SD_TransmitByte(0x40 | cmd);
    SD_TransmitByte(arg >> 24);
    SD_TransmitByte(arg >> 16);
    SD_TransmitByte(arg >> 8);
    SD_TransmitByte(arg);
    SD_TransmitByte(crc);

    do {
        response = SD_ReceiveByte();
    } while ((response & 0x80) && --retry);

    return response;
}

static uint8_t sdhc = 0;
uint8_t sd_is_sdhc(void) {
    return sdhc;
}
uint8_t card_initialized = 0;

SD_Status SD_SPI_Init(void) {
    uint8_t i, response;
    uint8_t r7[4];
    uint32_t retry;

    SD_CS_HIGH();
    for (i = 0; i < 10; i++) SD_TransmitByte(0xFF);

    SD_CS_LOW();
    response = SD_SendCommand(CMD0, 0, 0x95);
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);
    if (response != 0x01) return SD_ERROR;

    SD_CS_LOW();
    response = SD_SendCommand(CMD8, 0x000001AA, 0x87);
    for (i = 0; i < 4; i++) r7[i] = SD_ReceiveByte();
    SD_CS_HIGH();
    SD_TransmitByte(0xFF);

    sdhc = 0;
    retry = HAL_GetTick() + 1000;
    if (response == 0x01 && r7[2] == 0x01 && r7[3] == 0xAA) {
        do {
            SD_CS_LOW();
            SD_SendCommand(CMD55, 0, 0xFF);
            response = SD_SendCommand(ACMD41, 0x40000000, 0xFF);
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
        } while (response != 0x00 && HAL_GetTick() < retry);

        if (response != 0x00) return SD_ERROR;

        SD_CS_LOW();
        response = SD_SendCommand(CMD58, 0, 0xFF);
        uint8_t ocr[4];
        for (i = 0; i < 4; i++) ocr[i] = SD_ReceiveByte();
        SD_CS_HIGH();
        if (ocr[0] & 0x40) sdhc = 1;
    } else {
        do {
            SD_CS_LOW();
            SD_SendCommand(CMD55, 0, 0xFF);
            response = SD_SendCommand(ACMD41, 0, 0xFF);
            SD_CS_HIGH();
            SD_TransmitByte(0xFF);
        } while (response != 0x00 && HAL_GetTick() < retry);
        if (response != 0x00) return SD_ERROR;
    }

    card_initialized = 1;
    return SD_OK;
}

SD_Status SD_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;
    if (!sdhc) sector *= 512;

    if (count == 1) {
        SD_CS_LOW();
        if (SD_SendCommand(CMD17, sector, 0xFF) != 0x00) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        uint8_t token;
        uint32_t timeout = HAL_GetTick() + 200;
        do {
            token = SD_ReceiveByte();
            if (token == 0xFE) break;
        } while (HAL_GetTick() < timeout);
        if (token != 0xFE) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        SD_ReceiveBuffer(buff, 512);
        SD_ReceiveByte();  // CRC
        SD_ReceiveByte();
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);
        return SD_OK;
    } else {
        return SD_ReadMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_ReadMultiBlocks(uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;
    if (!sdhc) sector *= 512;

    SD_CS_LOW();
    if (SD_SendCommand(18, sector, 0xFF) != 0x00) {
        SD_CS_HIGH();
        return SD_ERROR;
    }

    while (count--) {
        uint8_t token;
        uint32_t timeout = HAL_GetTick() + 200;

        do {
            token = SD_ReceiveByte();
            if (token == 0xFE) break;
        } while (HAL_GetTick() < timeout);

        if (token != 0xFE) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        SD_ReceiveBuffer(buff, 512);
        SD_ReceiveByte();  // discard CRC
        SD_ReceiveByte();

        buff += 512;
    }

    SD_SendCommand(12, 0, 0xFF);  // STOP_TRANSMISSION
    SD_CS_HIGH();
    SD_TransmitByte(0xFF); // Extra 8 clocks

    return SD_OK;
}

SD_Status SD_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;
    if (!sdhc) sector *= 512;

    if (count == 1) {
        SD_CS_LOW();
        if (SD_SendCommand(CMD24, sector, 0xFF) != 0x00) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        SD_TransmitByte(0xFE);
        SD_TransmitBuffer(buff, 512);
        SD_TransmitByte(0xFF);
        SD_TransmitByte(0xFF);

        uint8_t resp = SD_ReceiveByte();
        if ((resp & 0x1F) != 0x05) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        while (SD_ReceiveByte() == 0);
        SD_CS_HIGH();
        SD_TransmitByte(0xFF);

        return SD_OK;
    } else {
        return SD_WriteMultiBlocks(buff, sector, count);
    }
}

SD_Status SD_WriteMultiBlocks(const uint8_t *buff, uint32_t sector, uint32_t count) {
    if (!count) return SD_ERROR;
    if (!sdhc) sector *= 512;

    SD_CS_LOW();
    if (SD_SendCommand(25, sector, 0xFF) != 0x00) {
        SD_CS_HIGH();
        return SD_ERROR;
    }

    while (count--) {
        SD_TransmitByte(0xFC);  // Start multi-block write token

        SD_TransmitBuffer((uint8_t *)buff, 512);
        SD_TransmitByte(0xFF);  // dummy CRC
        SD_TransmitByte(0xFF);

        uint8_t resp = SD_ReceiveByte();
        if ((resp & 0x1F) != 0x05) {
            SD_CS_HIGH();
            return SD_ERROR;
        }

        while (SD_ReceiveByte() == 0);  // busy wait
        buff += 512;
    }

    SD_TransmitByte(0xFD);  // STOP_TRAN token
    while (SD_ReceiveByte() == 0);  // busy wait

    SD_CS_HIGH();
    SD_TransmitByte(0xFF);

    return SD_OK;
}
