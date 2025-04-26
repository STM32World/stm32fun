/*
 * cs43l22.c
 *
 *  Created on: Apr 25, 2025
 *      Author: lth
 */

#include "main.h"
#include "cs43l22.h"

// Private i2c read/write functions

CS43L22_result_t cs_write(CS43L22_HandleTypeDef *cs, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Transmit(cs->i2c, cs->i2c_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        CS_DBG("I2C Transmit Error\n");
        return Err;
    }

    return Ok;

}

CS43L22_result_t cs_read(CS43L22_HandleTypeDef *cs, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Master_Receive(cs->i2c, cs->i2c_addr << 1, data, len, HAL_MAX_DELAY) != HAL_OK) {
        CS_DBG("I2C Transmit Error\n");
        return Err;
    }

    return Ok;
}

CS43L22_result_t cs_read_register(CS43L22_HandleTypeDef *cs, uint8_t reg, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Mem_Read(cs->i2c, cs->i2c_addr << 1, reg, 1, data, len, HAL_TIMEOUT) != HAL_OK) {
        return Err;
    }

    return Ok;
}

CS43L22_result_t cs_write_register(CS43L22_HandleTypeDef *cs, uint8_t reg, uint8_t *data, uint16_t len) {

    if (HAL_I2C_Mem_Write(cs->i2c, cs->i2c_addr << 1, reg, 1, data, len, HAL_TIMEOUT) != HAL_OK) {
        return Err;
    }

    return Ok;
}


// Public functions

CS43L22_result_t cs_init(CS43L22_HandleTypeDef *cs, I2C_HandleTypeDef *i2c, uint16_t i2c_addr, GPIO_TypeDef *rst_port, uint16_t rst_pin) {
    CS_DBG("Running cs_init\n");

    cs->i2c = i2c;
    cs->i2c_addr = i2c_addr;
    cs->rst_port = rst_port;
    cs->rst_pin = rst_pin;

    cs_enable(cs); // Enable the CS43L22
    HAL_Delay(10); // Wait a bit for it to get ready

    uint8_t v;

    if (cs_read_register(cs, CS43L22_ID, &v, 1)) {
        CS_DBG("CS42L22 read_id_error\n");
    }

    cs->id = (v & 0b11111000) >> 3;
    cs->rev = (v & 0b00000111);

    if (cs->id != CS42L22_DEFAULT_ID) {
        CS_DBG("IC must be a tranny, it does not identify as a CS41L22\n");
        return Err;
    }

    CS_DBG("CS43L22 id = %d rev = %d\n", cs->id, cs->rev);

    // Let's power the thing on
    v = CS43L22_PWR_ON;
    if (cs_write_register(cs, CS43L22_PWR_CTL_1, &v, 1)) {
        CS_DBG("CS43L22 write error");
    }

    // Enable all
    v = CS43L22_ALL_ON;
    if (cs_write_register(cs, CS43L22_PWR_CTL_2, &v, 1)) {
        CS_DBG("CS43L22 write error");
    }

    // Default clocking
    v = CS43L22_SPEED_DEF;
    if (cs_write_register(cs, CS43L22_CLOCKING, &v, 1)) {
        CS_DBG("CS43L22 write error");
    }

    // Interface control
    v = CS43L22_INT_DEF;
    if (cs_write_register(cs, CS43L22_INT_CTL_1, &v, 1)) {
        CS_DBG("CS43L22 write error");
    }

    return Ok;
}

// Yank RST line high
void cs_enable(CS43L22_HandleTypeDef *cs) {
    HAL_GPIO_WritePin(cs->rst_port, cs->rst_pin, GPIO_PIN_SET);
}

// Yank RST line low
void cs_disable(CS43L22_HandleTypeDef *cs) {
    HAL_GPIO_WritePin(cs->rst_port, cs->rst_pin, GPIO_PIN_RESET);
}

