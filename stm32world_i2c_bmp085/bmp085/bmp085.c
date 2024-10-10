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

#include <math.h>

#include "main.h"
#include "bmp085.h"

// Internal functions

BMP085_result_t bmp085_write_registers(BMP085_HandleTypeDef *bmp085, uint16_t start, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Mem_Write(bmp085->i2c, (bmp085->i2c_addr << 1), start, 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t bmp085_read_registers(BMP085_HandleTypeDef *bmp085, uint16_t start, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Mem_Read(bmp085->i2c, (bmp085->i2c_addr << 1), start, 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t bmp085_get_calibration_values(BMP085_HandleTypeDef *bmp085) {

    uint8_t tmp_data[22];

    if (bmp085_read_registers(bmp085, BMP085_REG_CALIB_START, (uint8_t*) &tmp_data, 22) != BMP085_Ok) {
        return BMP085_Err;
    }

    // If we got the data - let's flip the bytes and store the values
    // Notice the cast to preserve sign
    bmp085->calibration_data.ac1 = (int8_t) tmp_data[0] << 8 | tmp_data[1];
    bmp085->calibration_data.ac2 = (int8_t) tmp_data[2] << 8 | tmp_data[3];
    bmp085->calibration_data.ac3 = (int8_t) tmp_data[4] << 8 | tmp_data[5];
    bmp085->calibration_data.ac4 = tmp_data[6] << 8 | tmp_data[7];
    bmp085->calibration_data.ac5 = tmp_data[8] << 8 | tmp_data[9];
    bmp085->calibration_data.ac6 = tmp_data[10] << 8 | tmp_data[11];
    bmp085->calibration_data.b1 = (int8_t) tmp_data[12] << 8 | tmp_data[13];
    bmp085->calibration_data.b2 = (int8_t) tmp_data[14] << 8 | tmp_data[15];
    bmp085->calibration_data.mb = (int8_t) tmp_data[16] << 8 | tmp_data[17];
    bmp085->calibration_data.mc = (int8_t) tmp_data[18] << 8 | tmp_data[19];
    bmp085->calibration_data.md = (int8_t) tmp_data[20] << 8 | tmp_data[21];

    return BMP085_Ok;

}

// Public functions

BMP085_result_t bmp085_init(BMP085_HandleTypeDef *bmp085, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {
    BMP085_DBG("bmp085_init\n");

    bmp085->i2c = i2c;
    bmp085->i2c_addr = i2c_addr;

    if (bmp085_get_calibration_values(bmp085) != BMP085_Ok) {
        return BMP085_Err;
    }

    return BMP085_Ok;
}

BMP085_result_t bmp085_get_temperature(BMP085_HandleTypeDef *bmp085, float *temperature) {

    uint8_t buf[2] = { 0 };

    buf[0] = BMP085_CMD_TEMP; // Temperature

    if (bmp085_write_registers(bmp085, BMP085_REG_CONTROL, (uint8_t*) &buf, 1) != BMP085_Ok) {
        return BMP085_Err;
    }

    // I hate delays - try to check isReady instead
    HAL_Delay(5);

    if (bmp085_read_registers(bmp085, BMP085_REG_RESULT, (uint8_t*) &buf, 2) != BMP085_Ok) {
        return BMP085_Err;
    }

    //BMP085_DBG("buf[0] = %d buf[1] = %d\n", buf[0], buf[1]);

    // Got raw temperature
    long ut = (buf[0] << 8 | buf[1]);

    // Apply calibration data retrieved earlier
    long x1 = (ut - bmp085->calibration_data.ac6) * bmp085->calibration_data.ac5 / pow(2, 15);
    long x2 = bmp085->calibration_data.mc * pow(2, 11) / (x1 + bmp085->calibration_data.md);

    // Save the b5 for pressure calibration
    bmp085->calibration_data.b5 = x1 + x2;

    *temperature = (bmp085->calibration_data.b5 + 8) / pow(2, 4) / 10;

    return BMP085_Ok;
}

BMP085_result_t bmp085_get_pressure(BMP085_HandleTypeDef *bmp085, float *pressure) {

    uint8_t buf[3] = { 0 };

    buf[0] = BMP085_CMD_PRESSURE + (BMP085_OSS_HIGH << 6); // Temperature

    if (bmp085_write_registers(bmp085, BMP085_REG_CONTROL, (uint8_t*) &buf, 1) != BMP085_Ok) {
        return BMP085_Err;
    }

    // I hate delays - try to check isReady instead
    HAL_Delay(30);

    if (bmp085_read_registers(bmp085, BMP085_REG_RESULT, (uint8_t*) &buf, 3) != BMP085_Ok) {
        return BMP085_Err;
    }

    //BMP085_DBG("buf[0] = %d buf[1] = %d buf[2] = %d\n", buf[0], buf[1], buf[2]);

    // Got the raw pressure
    long up = (((long)buf[0] << 16) + ((long)buf[1] << 8) + buf[2]) >> (8 - BMP085_OSS_HIGH);

    long b6 = bmp085->calibration_data.b5 - 4000;
    long x1 = (bmp085->calibration_data.b2 * (b6 * b6 / pow(2, 12))) / pow(2, 11);
    long x2 = bmp085->calibration_data.ac2 * b6 / pow(2, 11);
    long x3 = x1 + x2;
    long b3 = ((bmp085->calibration_data.ac1 * 4 + x3) << BMP085_OSS_HIGH + 2) / 4;
    x1 = bmp085->calibration_data.ac2 * b6 / pow(2, 13);
    x2 = (bmp085->calibration_data.b1 * (b6 * b6 / pow(2, 12))) / pow(2, 16);
    x3 = ((x1 + x2) + 2) / pow(2, 2);
    unsigned long b4 = bmp085->calibration_data.ac4 * (unsigned long)(x3 + 32768) / pow(2, 15);
    unsigned long b7 = (unsigned long)(up - b3) * (50000 - BMP085_OSS_HIGH);
    long p;
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4;
    } else {
        p = (b7 / b4) * 2;
    }
    x1 = (p / pow(2, 8)) * (p / pow(2, 8));
    x1 = (x1 * 3038) / pow(2, 16);
    x2 = (-7357 * p) / pow(2, 16);
    p = p + (x1 + x2 + 3791) / pow(2, 4);

    *pressure = p;

    return BMP085_Ok;

}
