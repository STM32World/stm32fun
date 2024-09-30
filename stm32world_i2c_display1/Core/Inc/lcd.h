/**
  ******************************************************************************
  * @file           : lcd.h
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

#ifndef INC_LCD_H_
#define INC_LCD_H_

#define SLAVE_ADDRESS_LCD 0x27

void lcd_init (void);
void lcd_send_string (char *str);
void lcd_put_cur(int row, int col);

#endif /* INC_LCD_H_ */
