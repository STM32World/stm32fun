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

// Public functions

SSD1306_result_t ssd1306_init(SSD1306_handle_t *ssd1306, I2C_HandleTypeDef *i2c, uint16_t i2c_addr) {

    SSD1306_result_t res = SSD1306_Ok;

    SSD1306_DBG("ssd1306_init\n");

    ssd1306->i2c = i2c;
    ssd1306->i2c_addr = i2c_addr;

    return res;

}
