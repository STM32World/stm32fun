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

#include "../ssd1306_new/ssd1306.h"

#include "main.h"

const uint8_t init_cmd[] = {
        0x20, // Set Memory Addressing Mode
        0x00, // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
              // 10b,Page Addressing Mode (RESET); 11b,Invalid
        0xB0, // Set Page Start Address for Page Addressing Mode,0-7
};

// Internal library functions

// Send a byte to the command register
SSD1306_result_t ssd1306_write_cmd(SSD1306_handle_t *ssd1306, uint8_t cmd) {

    SSD1306_result_t res = SSD1306_Ok;

    if (HAL_I2C_Mem_Write(ssd1306->i2c, ssd1306->i2c_addr << 1, 0x00, 1, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        res = SSD1306_Err;
    }

    return res;

}

// Send data
SSD1306_result_t ssd1306_write_data(SSD1306_handle_t *ssd1306, uint8_t *buffer, size_t buffer_size) {

    SSD1306_result_t res = SSD1306_Ok;

    if (HAL_I2C_Mem_Write(ssd1306->i2c, ssd1306->i2c_addr << 1, 0x40, 1, buffer, buffer_size, HAL_MAX_DELAY) != HAL_OK) {
        res = SSD1306_Err;
    }

    return res;

}

SSD1306_result_t ssd1306_set_display(SSD1306_handle_t *ssd1306, uint8_t on) {
    SSD1306_result_t res = SSD1306_Ok;

    uint8_t value = on ? 0xAF : 0xAE;

    res = ssd1306_write_cmd(ssd1306, value);

    if (res == SSD1306_Ok) ssd1306->is_on = on;
    else ssd1306->is_on = 0;

    return res;
}

// Public functions

SSD1306_result_t ssd1306_init(SSD1306_handle_t *ssd1306, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {

    SSD1306_result_t res = SSD1306_Ok;

    SSD1306_DBG("ssd1306_init\n");

    ssd1306->i2c = i2c;
    ssd1306->i2c_addr = i2c_addr;

    // Set display off
    res = ssd1306_set_display(ssd1306, 0);

    if (res == SSD1306_Ok) {

        for (int i = 0; i < sizeof(init_cmd) / sizeof(init_cmd[0]); ++i) {
            res = ssd1306_write_cmd(ssd1306, init_cmd[i]);
            if (res != SSD1306_Ok) break;
        }

    }

    return res;

}
