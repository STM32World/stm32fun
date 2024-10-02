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

static const float GYRO_TEMP_SENSITIVITY = 280.0;
static const int GYRO_TEMP_OFFSET = -13200;
static const float GYRO_TEMP_OFFSET_CELSIUS = 35.0;

// Private functions

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

    if (itg3200_read_register(itg, ITG3200_REG_WHO_AM_I, &addr, 1) != ITG3200_OK) {
        return ITG3200_Err;
    }

    ITG_DBG("Received WHO_AM_I: %d addr = %d\n", addr, itg->i2c_addr);

    if (itg->i2c_addr != addr) {
        ITG_DBG("Who am i error!\n");
        return ITG3200_Err;
    }

    return ITG3200_OK;
}

ITG3200_result_t itg3200_get_temp(ITG3200_HandleTypeDef *itg) {
    ITG_DBG("itg_get_temp\n");

    uint8_t temp_data[2];

    if (itg3200_read_register(itg, ITG3200_REG_TEMP_DATA, (uint8_t *)&temp_data, sizeof(temp_data)) != ITG3200_OK) {
        return ITG3200_Err;
    }

    float t = (((float)((temp_data[0] | temp_data[1]<<8) + GYRO_TEMP_OFFSET))/GYRO_TEMP_SENSITIVITY) + GYRO_TEMP_OFFSET_CELSIUS;

    ITG_DBG("temp hi = %d, temp lo = %d full = %f\n", temp_data[0], temp_data[1], t);

    return ITG3200_OK;
}
