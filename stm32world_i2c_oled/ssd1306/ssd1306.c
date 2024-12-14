/**
 ******************************************************************************
 * @file           : ssd1606.c
 * @brief          : SSD1306 library source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * Parts of this Library was originally written by Olivier Van den Eede (4ilo)
 * in 2016.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"
#include "ssd1306.h"

// Internal library functions

// Send a byte to the command register
SSD1306_result_t ssd1306_write_command(SSD1306_handle_t *ssd1306, uint8_t command) {
    SSD1306_result_t res = SSD1306_Ok;
    if (HAL_I2C_Mem_Write(ssd1306->i2c, ssd1306->i2c_addr << 1, 0x00, 1, &command, 1, HAL_MAX_DELAY) != HAL_OK) {
        res = SSD1306_Err;
    }
    return res;
}

// Send data
SSD1306_result_t ssd1306_write_data(SSD1306_handle_t *ssd1306, uint8_t* buffer, size_t buff_size) {
    SSD1306_result_t res = SSD1306_Ok;
    if (HAL_I2C_Mem_Write(ssd1306->i2c, ssd1306->i2c_addr << 1, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY) != HAL_OK) {
        res = SSD1306_Err;
    }
    return res;
}

void ssd1306_SetDisplayOn(SSD1306_handle_t *ssd1306, const uint8_t on) {
    uint8_t value;
    if (on) {
        value = 0xAF;   // Display on
        SSD1306.DisplayOn = 1;
    } else {
        value = 0xAE;   // Display off
        SSD1306.DisplayOn = 0;
    }
    ssd1306_WriteCommand(value);
}

// Public functions

SSD1306_result_t ssd1306_init(SSD1306_handle_t *ssd1306, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {

    SSD1306_result_t res = SSD1306_Ok;

    SSD1306_DBG("ssd1306_init\n");

    ssd1306->i2c = i2c;
    ssd1306->i2c_addr = i2c_addr;

    // Init OLED
    ssd1306_display_on(ssd1306, 0); //display off

    return res;

}
