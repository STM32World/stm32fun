/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

#include "main.h"
#include "bmp085.h"

BMP085_result_t BMP085_write_registers(BMP085_HandleTypeDef *bmp085, uint8_t start, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Transmit(bmp085->i2c, (bmp085->i2c_addr << 1), &start, 1, 100) != HAL_OK) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t BMP085_read_registers(BMP085_HandleTypeDef *bmp085, uint8_t start, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Transmit(bmp085->i2c, (bmp085->i2c_addr << 1), &start, 1, 100) != HAL_OK) {
        return BMP085_Err;
    }

    if (HAL_I2C_Master_Receive(bmp085->i2c, (bmp085->i2c_addr << 1), data, len, 100) != HAL_OK) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t BMP085_init(BMP085_HandleTypeDef *bmp085, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {
    BMP085_DBG("BMP085_init\n");

    bmp085->i2c = i2c;
    bmp085->i2c_addr = i2c_addr;

    if (BMP085_read_registers(bmp085, 0xaa, (uint8_t *)&(bmp085->calibration_data), sizeof(bmp085->calibration_data)) != BMP085_Ok) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t BMP085_get_temp(BMP085_HandleTypeDef *bmp085, long *temp) {

    uint8_t reg = 0xf4;
    uint8_t value = 0x2e;

    uint8_t buf[3];

    if (BMP085_write_registers(bmp085, reg, value, 1) != BMP085_Ok) {
        return BMP085_Err;
    }

    if (BMP085_read_registers(bmp085, 0xaa, (uint8_t *)&(bmp085->calibration_data), sizeof(bmp085->calibration_data)) != BMP085_Ok) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}