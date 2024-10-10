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

#define BMP085_DEFAULT_ADDR 0x77

// BMP085 Registers
#define BMP085_REG_CALIB_START 0xAA
#define BMP085_REG_CONTROL     0xF4
#define BMP085_REG_RESULT      0xF6

// Commands for temperature and pressure readings
#define BMP085_CMD_TEMP        0x2E
#define BMP085_CMD_PRESSURE    0x34

// Oversampling modes
#define BMP085_OSS_ULTRALOW    0
#define BMP085_OSS_STANDARD    1
#define BMP085_OSS_HIGH        2
#define BMP085_OSS_ULTRAHIGH   3

typedef struct {
    short ac1;
    short ac2;
    short ac3;
    unsigned short ac4;
    unsigned short ac5;
    unsigned short ac6;
    short b1;
    short b2;
    short mb;
    short mc;
    short md;
    long b5;
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

BMP085_result_t bmp085_init(BMP085_HandleTypeDef *bmp085, I2C_HandleTypeDef *i2c, uint16_t i2c_addr);
BMP085_result_t bmp085_get_temperature(BMP085_HandleTypeDef *bmp085, float *temperature);
BMP085_result_t bmp085_get_pressure(BMP085_HandleTypeDef *bmp085, float *pressure);

#endif /* BMP085_H_ */
