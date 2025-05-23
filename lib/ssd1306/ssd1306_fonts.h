/**
 ******************************************************************************
 * @file           : ssd1306_fonts.h
 * @brief          : Fonts header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * Copyright (c) 2019 Andriy Honcharenko
 * Copyright (c) 2021 Roberto Benjami
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stdint.h"


#ifndef _ssd1306_fonts_h
#define _ssd1306_fonts_h

//
// Structure om font te definieren
//
typedef struct {
  const uint8_t FontWidth;    /*!< Font width in pixels */
  uint8_t FontHeight;         /*!< Font height in pixels */
  const uint16_t *data;       /*!< Pointer to data font data array */
} FontDef;


//
// The 3 fonts
//
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif // _ssd1306_fonts_h
