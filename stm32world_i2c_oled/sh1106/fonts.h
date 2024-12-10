/**
 * @file fonts.h
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Declares the fonts structure and links to the defined ones.
 * @version 0.1
 * @date 21-11-2022
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef FONTS_H_
#define FONTS_H_

#include <stdint.h>

/**
 * @brief  Font structure scan lines enumeration
 */
typedef enum {
    FONT_V = 0, /**< Vertical font scan lines. */
    FONT_H = 1, /**< Horizontal font scan lines. */
} font_scan_dir;

/**
 * @brief Describes a font.
 */
typedef struct font {
    const uint8_t width;        /**< Width of character. */
    const uint8_t height;       /**< Height of character. */
    const font_scan_dir scan;         /**< Font scan lines behavior. */
    const uint8_t min_char;     /**< Code of the first known symbol. */
    const uint8_t max_char;     /**< Code of the last known symbol. */
    const uint8_t characters[]; /**< Font data. */
} font_t;

extern const font_t fnt5x7;
extern const font_t fnt7x10;

#endif /* FONTS_H_ */
