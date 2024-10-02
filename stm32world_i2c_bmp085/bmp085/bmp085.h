/**
 ******************************************************************************
 * @file           : bmp085.h
 * @brief          : BMP086 Header
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

#ifndef BMP085_H_
#define BMP085_H_

#ifdef DEBUG
#include <stdio.h>
#define BMP085_DBG(...) printf(__VA_ARGS__)
#else
#define BMP085_DBG(...)
#endif

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} BMP085_Calibration_TypeDef;

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t i2c_addr;
    BMP085_Calibration_TypeDef calibration_data;
} BMP085_HandleTypeDef;

typedef enum {
    BMP085_Ok,
    BMP085_Err
} BMP085_result_t;

BMP085_result_t BMP085_init(BMP085_HandleTypeDef *bmp085, I2C_HandleTypeDef *i2c, uint16_t i2c_addr);
BMP085_result_t BMP085_get_temp(BMP085_HandleTypeDef *bmp085, long *temp);

#endif /* BMP085_H_ */
