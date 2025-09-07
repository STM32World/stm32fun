/**
 ******************************************************************************
 * @file           : lcd.c
 * @brief          : Interface LCD
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

#include "lcd.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);

    uint8_t data_t[4];
    data_t[0] = data_u | 0x0C;  //en=1, rs=0 -> bxxxx1100
    data_t[1] = data_u | 0x08;  //en=0, rs=0 -> bxxxx1000
    data_t[2] = data_l | 0x0C;  //en=1, rs=0 -> bxxxx1100
    data_t[3] = data_l | 0x08;  //en=0, rs=0 -> bxxxx1000
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD<<1, (uint8_t*) data_t, 4, 100);
}

void lcd_send_data (char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data&0xf0);
    data_l = ((data<<4)&0xf0);
    data_t[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
    data_t[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
    data_t[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
    data_t[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001
    HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD<<1 ,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void) {
  // 4 bit initialisation
  HAL_Delay(50);  // wait for >40ms
  lcd_send_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
  lcd_send_cmd (0x30);
  HAL_Delay(1);  // wait for >100us
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20);  // 4bit mode
  HAL_Delay(10);

  // display initialisation
  lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  HAL_Delay(1);
  lcd_send_cmd (0x01);  // clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str) {
  while (*str) lcd_send_data (*str++);
}

void lcd_put_cur(int row, int col) {
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }
    lcd_send_cmd (col);
}
