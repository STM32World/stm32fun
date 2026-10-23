#ifndef INC_W5500_SPI_H_
#define INC_W5500_SPI_H_

#include "main.h"
#include "wizchip_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported Functions */

/**
 * @brief  Performs a hardware reset pulse on the W5500 chip.
 */
void W5500_HardwareReset(void);

/**
 * @brief  CS Pin control callbacks passed to ioLibrary_Driver.
 */
void W5500_Select(void);
void W5500_Unselect(void);

/**
 * @brief  Single byte SPI read/write callbacks passed to ioLibrary_Driver.
 */
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);

/**
 * @brief  Burst buffer SPI read/write callbacks passed to ioLibrary_Driver.
 */
void W5500_ReadBurst(uint8_t *pBuf, uint16_t len);
void W5500_WriteBurst(uint8_t *pBuf, uint16_t len);

/**
 * @brief  Initializes the W5500 HW, registers callbacks, and sets IP config.
 * @param  netInfo Pointer to populated network parameter struct.
 * @return 0 on success, -1 on driver initialization failure.
 */
int8_t W5500_Init(wiz_NetInfo *netInfo);

#ifdef __cplusplus
}
#endif

#endif /* INC_W5500_SPI_H_ */
