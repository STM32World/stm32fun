/**
  ******************************************************************************
  * @file           : itg3200.h
  * @brief          : ITG3200 Library Header
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Lars Boegild Thomsen <lth@stm32world.com>
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef ITG3200_H_
#define ITG3200_H_

#ifdef DEBUG
#include <stdio.h>
#define ITG_DBG(...) printf(__VA_ARGS__)
#else
#define ITG_DBG(...)
#endif

#define ITG3200_DEFAULT_ADDR     0x68

#define ITG3200_REG_WHO_AM_I     0x00
#define ITG3200_REG_SMPLRT_DIV   0x15
#define ITG3200_REG_DLPF_FS      0x16
#define ITG3200_REG_INT_CFG      0x17
#define ITG3200_REG_INT_STATUS   0x1a
#define ITG3200_REG_TEMP_DATA    0x1b
#define ITG3200_REG_GYRO_DATA    0x1d
#define ITG3200_REG_PWR_MGM      0x3e

typedef struct {
    uint8_t temp_hi;
    uint8_t temp_lo;
    uint8_t gyro_x_hi;
    uint8_t gyro_x_lo;
    uint8_t gyro_y_hi;
    uint8_t gyro_y_lo;
    uint8_t gyro_z_hi;
    uint8_t gyro_z_lo;
} ITG3200_Data_TypeDef;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t i2c_addr;
} ITG3200_HandleTypeDef;

typedef enum {
    ITG3200_OK,
    ITG3200_Err
} ITG3200_result_t;

ITG3200_result_t itg3200_init(ITG3200_HandleTypeDef *itg, I2C_HandleTypeDef *i2c, uint16_t i2c_addr);
ITG3200_result_t itg3200_get_temp(ITG3200_HandleTypeDef *itg, float *temperature);

#endif /* ITG3200_H_ */
