/*
 * cs43l22.h
 *
 *  Created on: Apr 25, 2025
 *      Author: lth
 */

#ifndef CS43L22_H_
#define CS43L22_H_

#ifdef DEBUG
#include <stdio.h>
#define CS_DBG(...) printf(__VA_ARGS__)
#else
#define CS_DBG(...)
#endif

#define CS43L22_DEFAULT_ADDR     0x4A

#define ITG3200_REG_WHO_AM_I     0x00
#define ITG3200_REG_SMPLRT_DIV   0x15
#define ITG3200_REG_DLPF_FS      0x16
#define ITG3200_REG_INT_CFG      0x17
#define ITG3200_REG_INT_STATUS   0x1a
#define ITG3200_REG_TEMP_DATA    0x1b
#define ITG3200_REG_GYRO_DATA    0x1d
#define ITG3200_REG_PWR_MGM      0x3e

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t i2c_addr;
} ICS43L22_HandleTypeDef;

typedef enum {
    ITG3200_OK,
    ITG3200_Err
} CS43L22_result_t;

#endif /* CS43L22_H_ */
