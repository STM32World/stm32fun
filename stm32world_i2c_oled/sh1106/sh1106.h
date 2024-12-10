/**
 * @file sh1106.h
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Declares the display functions.
 * @version 0.1
 * @date 21-11-2022
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SH1106_H_
#define SH1106_H_

#include <memory.h>
#include <stdint.h>

#include "fonts.h"
//#include "i2c.h"

/**
 * @brief Width, in pixels, of the display.
 */
#define SCR_W ((uint8_t) 128)

/**
 * @brief Height, in pixels, of the display.
 */
#define SCR_H ((uint8_t) 64)

/**
 * @brief Endereço I²C do display.
 */
#define SH1106_ADDRESS 0x3C

/**
 * @brief Enumerate the display commands.
 */
typedef enum __attribute__((packed)) sh1106_commands {
    SH1106_CMD_COL_LOW = 0x02,        /**< Set the lower 4 bits of the column address,
                                       * modify lower 4 bits. */
    SH1106_CMD_COL_HIGH = 0x10,       /**< Set the higher 4 bits of the column address,
                                       * modify lower 4 bits. */
    SH1106_DC_DC_VOLTAGE  = 0x30,     /**< Sets DC-Dc output value, modify lower 2bits. */
    SH1106_CMD_START_LINE = 0x40,     /**< Set the display start line to
                                       * 0. */
    SH1106_CMD_CONTRAST = 0x81,       /**< Contrast control. */
    SH1106_CMD_SEG_NORM = 0xA0,       /**< Sets column 0 to SEG0. (Coordinate x
                                       * increasing.) */
    SH1106_CMD_SEG_INV = 0xA1,        /**< Sets column 127 to SEG0. (Coordinate x
                                       * decreasing.) */
    SH1106_CMD_DISPLAY_BY_RAM = 0xA4, /**< Configures the display to show the
                                       * RAM contents. */
    SH1106_CMD_DISPLAY_FULL_ON = 0xA5, /**< Configures the display to full on. */
    SH1106_CMD_INV_OFF         = 0xA6, /**< Configures the display not to invert the data
                                        * RAM data. */
    SH1106_CMD_INV_ON    = 0xA7,       /**< Sets the display to invert the RAM. */
    SH1106_CMD_SET_MUX   = 0xA8,       /**< Sets the multiplexer ratio. */
    SH1106_CMD_DC_DC_SET = 0xAD,       /**< Sets the DC-DC pump of the display. */
    SH1106_CMD_DISP_OFF  = 0xAE,       /**< Sets the display to off. */
    SH1106_CMD_DISP_ON   = 0xAF,       /**< Sets the display to on. */
    SH1106_CMD_PAGE_ADDR = 0xB0,       /**< Sets the page address to 0. */
    SH1106_CMD_COM_NORM  = 0xC0,       /**< Sets the internal scanning of COM0 to
                                        * COM[n-1]. (Increasing y coordinate.) */
    SH1106_CMD_COM_INV = 0xC8,    /**< Sets the internal scan from COM[n-1] * to COM0.
                                   * to COM0. (Decreasing y-coordinate.) */
    SH1106_CMD_SET_OFFSET = 0xD3, /**< Set the display COM offset. */
    SH1106_CMD_CLOCKDIV   = 0xD5, /**< Sets the display clock pre-scaler. */
    SH1106_CMD_SET_CHARGE = 0xD9, /**< Sets the display's discharge/precharge time.
                                   * display. */
    SH1106_CMD_COM_HW    = 0xDA,  /**< Sets the mode of the internal COM pins. */
    SH1106_CMD_VCOM_DSEL = 0xDB,  /**< Sets the voltage on the common pads at the
                                   * de-selection. */
} sh1106_commands_t;

/**
 * @brief Enumerates the display orientations.
 */
typedef enum __attribute__((packed)) sh1106_orientation {
    SH1106_ORIENTATION_NORMAL = 0, /**< No rotation */
    SH1106_ORIENTATION_CW     = 1, /**< Clockwise rotation */
    SH1106_ORIENTATION_CCW    = 2, /**< Counterclockwise rotation. */
    SH1106_ORIENTATION_180    = 3  /**< Rotation through 180 degrees. */
} sh1106_orientation_t;

/**
 * @brief Enumerates the drawing modes.
 */
typedef enum __attribute__((packed)) sh1106_draw_mode {
    SH1106_PSET = 0x00, /**< Pixel on. */
    SH1106_PRES = 0x01, /**< Pixel off. */
    SH1106_PINV = 0x02  /**< Pixel inverted. */
} sh1106_draw_mode_t;

/**
 * @brief Initializes the display with the default settings.
 */
void sh1106_init(void);

/**
 * @brief Clears the display's VRAM buffer.
 */
void sh1106_clear(void);

/**
 * @brief Sends the vRAM to the display.
 */
void sh1106_flush(void);

/**
 * @brief Draws a horizontal line on the screen.
 *
 * @param origin_x Horizontal position at the left end of the line.
 * @param end_x Horizontal position at the right end of the line.
 * @param y Vertical position of the line.
 */
void sh1106_draw_h_line(uint8_t origin_x, uint8_t end_x, uint8_t y);

/**
 * @brief Draws a vertical line on the screen.
 *
 * @param origin_y Vertical position of the upper end of the line.
 * @param end_y Vertical position of the lower end of the line.
 * @param x Horizontal position of the line.
 */
void sh1106_draw_v_line(uint8_t origin_y, uint8_t end_y, uint8_t x);

/**
 * @brief Draws a line.
 *
 * @param origin_x x from the origin point.
 * @param origin_y y from the origin point.
 * @param end_x x of the destination point.
 * @param end_y y of the destination point.
 */
void sh1106_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t end_x, uint8_t end_y);

/**
 * @brief Draws a rectangle.
 *
 * @param top y of the top edge.
 * @param left x of the left edge.
 * @param bottom y of the bottom edge.
 * @param right x of the right edge.
 * If true, the rectangle will be filled.
 */
void sh1106_draw_rect(uint8_t top, uint8_t left, uint8_t bottom, uint8_t right,
                      uint8_t filled);

/**
 * @brief Draws a circle.
 *
 * @param center_x x in the center of the circle
 * @param center_y y in the center of the circle
 * @param radius Radius of the circle.
 */
void sh1106_draw_circle(uint8_t center_x, uint8_t center_y, uint8_t radius);

/**
 * @brief Draws a bitmap.
 *
 * @param[in] bmp Reference to the bitmap.
 * @param top x left of the bitmap.
 * @param left y top of the bitmap.
 * @param width Horizontal size of the bitmap.
 * @param height Vertical size of the bitmap.
 */
void sh1106_draw_bitmap(const uint8_t* bmp, uint8_t top, uint8_t left, uint8_t width,
                        uint8_t height);

/**
 * @brief Prints a string.
 *
 * @param[in] str Reference to string.
 * @param top y top of string.
 * @param left x left of string.
 * @param[in] font Font to be used.
 * @return Position of the right corner of the string, in pixels.
 */
uint16_t sh1106_print(const uint8_t* str, uint8_t top, uint8_t left, const font_t* font);

#endif /* SH1106_H_ */
