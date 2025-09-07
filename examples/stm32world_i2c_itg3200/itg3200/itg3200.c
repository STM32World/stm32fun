/**
 ******************************************************************************
 * @file           : itg3200.h
 * @brief          : ITG3200 Library Source
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
#include "itg3200.h"

// Private i2c read/write functions

ITG3200_result_t itg3200_write(ITG3200_HandleTypeDef *itg, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Transmit(itg->i2c, itg->i2c_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        ITG_DBG("I2C Transmit Error\n");
        return ITG3200_Err;
    }

    return ITG3200_OK;

}

ITG3200_result_t itg3200_read(ITG3200_HandleTypeDef *itg, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Receive(itg->i2c, itg->i2c_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        ITG_DBG("I2C Transmit Error\n");
        return ITG3200_Err;
    }

    return ITG3200_OK;
}

ITG3200_result_t itg3200_read_register(ITG3200_HandleTypeDef *itg, uint8_t reg, uint8_t *data, uint16_t len) {

    if (itg3200_write(itg, &reg, 1) != ITG3200_OK) {
        return ITG3200_Err;
    }

    if (itg3200_read(itg, data, len) != ITG3200_OK) {
        return ITG3200_Err;
    }

    return ITG3200_OK;
}

// Public functions

ITG3200_result_t itg3200_init(ITG3200_HandleTypeDef *itg, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {
    ITG_DBG("itg3200_init\n");

    itg->i2c = i2c;
    itg->i2c_addr = i2c_addr;

    HAL_Delay(50);

    uint8_t addr;
    uint8_t w_buf[2];

    if (itg3200_read_register(itg, ITG3200_REG_WHO_AM_I, &addr, 1) != ITG3200_OK) {
        return ITG3200_Err;
    }

    ITG_DBG("Received WHO_AM_I: %d addr = %d\n", addr, itg->i2c_addr);

    if (itg->i2c_addr != addr) {
        ITG_DBG("Who am i error!\n");
        return ITG3200_Err;
    }

    // Set sample rate
    w_buf[0] = ITG3200_REG_SMPLRT_DIV;
    w_buf[1] = 7;
    if (itg3200_write(itg, (uint8_t*) &w_buf, 2) != ITG3200_OK) {
        ITG_DBG("Write error\n");
    }

    // Set DLPF, Full Scale
    w_buf[0] = ITG3200_REG_DLPF_FS;
    w_buf[1] = 0b00011110;
    if (itg3200_write(itg, (uint8_t*) &w_buf, 2) != ITG3200_OK) {
        ITG_DBG("Write error\n");
    }

    // Set power
    w_buf[0] = ITG3200_REG_PWR_MGM;
    w_buf[1] = 0b00000001;
    if (itg3200_write(itg, (uint8_t*) &w_buf, 2) != ITG3200_OK) {
        ITG_DBG("Write error\n");
    }

    return ITG3200_OK;
}

ITG3200_result_t itg3200_get_temp(ITG3200_HandleTypeDef *itg, float *temperature) {
    //ITG_DBG("itg_get_temp\n");

    ITG3200_Temp_TypeDef raw;

    if (itg3200_read_register(itg, ITG3200_REG_TEMP_DATA, (uint8_t*)&raw, sizeof(raw)) != ITG3200_OK) {
        return ITG3200_Err;
    }

    *temperature = 35.0 + (((raw.temp_hi << 8) | raw.temp_lo) + 13200.0) / 280.0;

    //ITG_DBG("temp hi = %d, temp lo = %d temp = %f\n", temp_data[0], temp_data[1], t);

    return ITG3200_OK;
}

ITG3200_result_t itg3200_get_rot(ITG3200_HandleTypeDef *itg, float *x, float *y, float *z) {

    ITG3200_Gyro_TypeDef raw;

    if (itg3200_read_register(itg, ITG3200_REG_GYRO_DATA, (uint8_t*) &raw, sizeof(raw)) != ITG3200_OK) {
        return ITG3200_Err;
    }

    *x = (raw.gyro_x_hi << 8 | raw.gyro_x_lo) / 14.375;
    *y = (raw.gyro_y_hi << 8 | raw.gyro_y_lo) / 14.375;
    *z = (raw.gyro_z_hi << 8 | raw.gyro_z_lo) / 14.375;

    return ITG3200_OK;
}
