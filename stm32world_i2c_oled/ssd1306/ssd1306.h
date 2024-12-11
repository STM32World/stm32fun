/**
 ******************************************************************************
 * @file           : ssd1606.h
 * @brief          : SSD1306 library header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * Parts of this Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#ifdef DEBUG
#include <stdio.h>
#define SSD1306_DBG(...) printf(__VA_ARGS__)
#else
#define SSD1306_DBG(...)
#endif

// The SSD1306 default i2c address
#ifndef SSD1306_DEFAULT_ADDR
#define SSD1306_DEFAULT_ADDR 0x3C
#endif

// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           128
#endif

#ifndef SSD1306_BUFFER_SIZE
#define SSD1306_BUFFER_SIZE   SSD1306_WIDTH * SSD1306_HEIGHT / 8
#endif

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t i2c_addr;
    uint8_t fb[SSD1306_BUFFER_SIZE]; // Frame buffer
} SSD1306_handle_t;

typedef enum {
    SSD1306_Ok,
    SSD1306_Err
} SSD1306_result_t;

SSD1306_result_t ssd1306_init(SSD1306_handle_t *ssd1306, I2C_HandleTypeDef *i2c, uint16_t i2c_addr);



#endif /* SSD1306_H_ */
