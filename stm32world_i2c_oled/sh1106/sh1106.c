/**
 * @file sh1106.c
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Defines the display functions.
 * @version 0.1
 * @date 15-06-2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "main.h"

#include <string.h>

#include "sh1106.h"

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Atribui os valores presentes na máscara, bit a bit, no campo desejado.
 */
#define SET_MASK(target_field, mask) (target_field) |= (mask)

/**
 * @brief Limpa os valores presentes na máscara, bit a bit, do campo desejado.
 */
#define CLEAR_MASK(target_field, mask) (target_field) &= ~(mask)

/**
 * @brief Inverte os valores de um campo presentes na máscara, bit a bit.
 */
#define INVERT_MASK(target_field, mask) (target_field) ^= (mask)

/**
 * @brief Altura em bits/pixels de uma página.
 */
#define PAGE_HEIGHT 8

/**
 * @brief Orientação desejada da tela.
 */
#define VERTICAL_ORIENTATION 0

#if VERTICAL_ORIENTATION
/**
 * @brief Largura da tela, com base na orientação desejada.
 */
#define SCR_W ((uint8_t) 64)

/**
 * @brief Altura da tela, com base na orientação desejada.
 */
#define SCR_H ((uint8_t) 128)
#else
/**
 * @brief Largura da tela, com base na orientação desejada.
 */
#define SCR_W ((uint8_t) 128)

/**
 * @brief Altura da tela, com base na orientação desejada.
 */
#define SCR_H ((uint8_t) 64)
#endif

/**
 * @brief Estrutura os dados de controle do display.
 */
static struct sh1106_control {
    sh1106_draw_mode_t pixel_mode; /**< Modo atual de desenho.*/
    uint8_t __attribute__((aligned(4)))
    vRAM[(SCR_W * SCR_H) >> 3]; /**< Buffer de vídeo, 128x64 pixels, armazenados
                                 * em bytes.*/
} self = {
    .pixel_mode = SH1106_PSET,
    .vRAM       = {0},
};

/**
 * @brief Envia um comando para o display.
 *
 * @param cmd Byte a ser enviado.
 */
static void send_cmd(sh1106_commands_t cmd);

/**
 * @brief Envia um comando de dois bytes para o display.
 *
 * @param cmd Byte de comando.
 * @param follow_up Segundo byte.
 */
static void send_double_cmd(sh1106_commands_t cmd, uint8_t follow_up);

/**
 * @brief Envia uma sequencia de bytes como dados para RAM do display.
 *
 * @param[in] page_data Referência para a lista dos dados.
 */
static void send_page(const uint8_t* page_data);

/**
 * @brief Desenha um pixel na tela.
 *
 * @param x Posição horizontal do pixel.
 * @param y Posição vertical do pixel.
 */
static void draw_pixel(uint8_t x, uint8_t y);

/**
 * @brief Desenha uma linha horizontal no display, na orientação padrão.
 *
 * @param x Posição horizontal do início da linha.
 * @param y Posição vertical do início da linha.
 * @param w Tamanho da linha.
 */
static void draw_horizontal_line_base(uint8_t x, uint8_t y, uint8_t w);

/**
 * @brief Desenha uma linha vertical no display, na orientação padrão.
 *
 * @param x Posição horizontal do início da linha.
 * @param y Posição vertical do início da linha.
 * @param h Tamanho da linha.
 */
static void draw_vertical_line_base(uint8_t x, uint8_t y, uint8_t h);

/**
 * @brief Imprime um caractere na tela.
 *
 * @param x Posição horizontal esquerda do caractere.
 * @param y Posição vertical superior do caractere.
 * @param character Caractere a ser desenhando.
 * @param[in] fnt Fonte a ser utilizada.
 * @return uint8_t Posição horizontal direita do caractere após o desenho.
 */
static uint8_t print_char(uint8_t x, uint8_t y, char character, const font_t* fnt);

/**
 * @brief Desenha o padrão de um byte no display, verticalmente, sem considerar a
 * orientação do display.
 *
 * @param top Posição vertical onde o byte deve ser desenhado.
 * @param left Posição horizontal onde o byte deve ser desenhado.
 * @param byte Byte a ser desenhado.
 */
static inline void draw_byte_v(uint8_t top, uint8_t left, uint8_t byte);

/**
 * @brief Desenha o padrão de um byte no display, horizontalmente, sem considerar a
 * orientação do display.
 *
 * @param top Posição vertical onde o byte deve ser desenhado.
 * @param left Posição horizontal onde o byte deve ser desenhado.
 * @param byte Byte a ser desenhado.
 */
static inline void draw_byte_h(uint8_t top, uint8_t left, uint8_t byte);

void sh1106_init(void) {
    send_cmd(SH1106_CMD_DISP_OFF);

    /* Configura a coluna inicial de escrita. */
    send_double_cmd(SH1106_CMD_COL_LOW, SH1106_CMD_COL_HIGH);

    send_cmd(SH1106_CMD_START_LINE);

    send_cmd(SH1106_CMD_PAGE_ADDR);

    /* Contraste máximo. */
    send_double_cmd(SH1106_CMD_CONTRAST, 0xFF);

    /* Configura a rotação da tela para retrato. */
    send_cmd(SH1106_CMD_SEG_INV);
    send_cmd(SH1106_CMD_COM_INV);

    send_cmd(SH1106_CMD_DISPLAY_BY_RAM);

    send_cmd(SH1106_CMD_INV_OFF);

    /* 64 linhas. */
    send_double_cmd(SH1106_CMD_SET_MUX, 63);

    send_cmd(SH1106_DC_DC_VOLTAGE | 0x03);

    /* Liga o DC-DC interno para o display. */
    send_double_cmd(SH1106_CMD_DC_DC_SET, 0x8B);

    /* Offset 0. */
    send_double_cmd(SH1106_CMD_SET_OFFSET, 0x00);

    /* Divisão do clock: frequência padrão + 25%. */
    send_double_cmd(SH1106_CMD_CLOCKDIV, 0xA0);

    /* 2 ciclos de clock para descarga e 10 ciclos para pré-carga.*/
    send_double_cmd(SH1106_CMD_SET_CHARGE, 0x22);

    /* Configuração de hardware alternativa. */
    send_double_cmd(SH1106_CMD_COM_HW, 0x12);

    /* Configura VCOM igual a VREF para pré-carga mais rápida. */
    send_double_cmd(SH1106_CMD_VCOM_DSEL, 0x40);

    HAL_Delay(100);
    send_cmd(SH1106_CMD_DISP_ON);
    HAL_Delay(100);
}

void sh1106_clear(void) {
    memset(self.vRAM, 0, sizeof(self.vRAM));
}

void sh1106_flush(void) {
    for (uint8_t page = 0; page < 8; page++) {
        send_cmd(SH1106_CMD_PAGE_ADDR + page);
        send_double_cmd(SH1106_CMD_COL_LOW, SH1106_CMD_COL_HIGH);

        send_page(self.vRAM + (page << 7));
    }
}

void sh1106_draw_h_line(uint8_t origin_x, uint8_t end_x, uint8_t y) {
    uint8_t w = (end_x - origin_x) + 1;

#if VERTICAL_ORIENTATION
    draw_vertical_line_base(y, origin_x, w);
#else
    draw_horizontal_line_base(origin_x, y, w);
#endif
}

void sh1106_draw_v_line(uint8_t origin_y, uint8_t end_y, uint8_t x) {
    uint8_t h = (end_y - origin_y) + 1;

#if VERTICAL_ORIENTATION
    draw_horizontal_line_base(origin_y, x, h);
#else
    draw_vertical_line_base(x, origin_y, h);
#endif
}

void sh1106_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t end_x, uint8_t end_y) {
    int16_t dx = (int16_t) (end_x - origin_x);
    int16_t dy = (int16_t) (end_y - origin_y);
    int16_t dx2;
    int16_t dy2;
    int16_t di;
    int8_t dx_sym = (dx > 0) ? 1 : -1;
    int8_t dy_sym = (dy > 0) ? 1 : -1;

    if (dx == 0) {
        sh1106_draw_v_line(origin_y, end_y, origin_x);
        return;
    }
    if (dy == 0) {
        sh1106_draw_h_line(origin_x, end_x, origin_y);
        return;
    }


    /* Algoritmo de desenho de linha por diferencial. */
    dx  = (int16_t) (dx * dx_sym);
    dy  = (int16_t) (dy * dy_sym);
    dx2 = (int16_t) (dx << 1);
    dy2 = (int16_t) (dy << 1);

    if (dx >= dy) {
        di = (int16_t) (dy2 - dx);
        while (origin_x != end_x) {
            draw_pixel(origin_x, origin_y);
            origin_x += dx_sym;
            if (di < 0) {
                di = (int16_t) (di + dy2);
            } else {
                di = (int16_t) (di + (dy2 - dx2));
                origin_y += dy_sym;
            }
        }
    } else {
        di = (int16_t) (dx2 - dy);
        while (origin_y != end_y) {
            draw_pixel(origin_x, origin_y);
            origin_y += dy_sym;
            if (di < 0) {
                di = (int16_t) (di + dx2);
            } else {
                di = (int16_t) (di + (dx2 - dy2));
                origin_x += dx_sym;
            }
        }
    }

    draw_pixel(origin_x, origin_y);
}

void sh1106_draw_rect(uint8_t top, uint8_t left, uint8_t bottom, uint8_t right,
                      uint8_t filled) {
    sh1106_draw_h_line(left, right, top);
    sh1106_draw_h_line(left, right, bottom);
    sh1106_draw_v_line(top, bottom + 1, left - 1);
    sh1106_draw_v_line(top, bottom + 1, right - 1);

    if (filled) {
        for (; right > 0; right--) {
            sh1106_draw_v_line(top, bottom, right);
        }
    }
}

void sh1106_draw_circle(uint8_t center_x, uint8_t center_y, uint8_t radius) {
    int16_t err = (int16_t) (1 - radius);
    int16_t dx  = 0;
    int16_t dy  = (int16_t) (-2 * radius);
    int16_t x   = 0;
    int16_t y   = radius;

    int16_t const height_limit = (int16_t) (SCR_H - 1);
    int16_t const width_limit  = (int16_t) (SCR_W - 1);

    while (x < y) {
        if (err >= 0) {
            dy += 2;
            err = (int16_t) (err + dy);
            y--;
        }
        dx += 2;
        err = (int16_t) (err + dx + 1);
        x++;

        /* Desenho dos octantes. */
        if (center_x + x < width_limit) {
            if (center_y + y < height_limit)
                draw_pixel((uint8_t) (center_x + x), (uint8_t) (center_y + y));
            if (center_y - y > -1)
                draw_pixel((uint8_t) (center_x + x), (uint8_t) (center_y - y));
        }
        if (center_x - x > -1) {
            if (center_y + y < height_limit)
                draw_pixel((uint8_t) (center_x - x), (uint8_t) (center_y + y));
            if (center_y - y > -1)
                draw_pixel((uint8_t) (center_x - x), (uint8_t) (center_y - y));
        }
        if (center_x + y < width_limit) {
            if (center_y + x < height_limit)
                draw_pixel((uint8_t) (center_x + y), (uint8_t) (center_y + x));
            if (center_y - x > -1)
                draw_pixel((uint8_t) (center_x + y), (uint8_t) (center_y - x));
        }
        if (center_x - y > -1) {
            if (center_y + x < height_limit)
                draw_pixel((uint8_t) (center_x - y), (uint8_t) (center_y + x));
            if (center_y - x > -1)
                draw_pixel((uint8_t) (center_x - y), (uint8_t) (center_y - x));
        }
    }

    /* Pontos verticais e horizontais. */
    if (center_x + radius < width_limit)
        draw_pixel(center_x + radius, center_y);
    if (center_x - radius > -1)
        draw_pixel(center_x - radius, center_y);
    if (center_y + radius < height_limit)
        draw_pixel(center_x, center_y + radius);
    if (center_y - radius > -1)
        draw_pixel(center_x, center_y - radius);
}

void sh1106_draw_bitmap(const uint8_t* bmp, uint8_t top, uint8_t left, uint8_t width,
                        uint8_t height) {
    for (uint8_t y_pos = top; y_pos < top + height; y_pos += PAGE_HEIGHT) {
        for (uint8_t x_pos = left; x_pos < left + width; x_pos++) {
            if (*bmp != 0) {
                draw_byte_v(y_pos, x_pos, *bmp);
            }

            bmp++;
        }
    }
}

uint16_t sh1106_print(const uint8_t* str, uint8_t top, uint8_t left, const font_t* font) {
    if (str == NULL) {
        return 0;
    }

    uint8_t p_x         = left;
    uint8_t width_limit = (uint8_t) (SCR_W - font->width - 1);


    while (*str != '\0' && p_x < width_limit) {
        p_x += print_char(p_x, top, *str, font);
        str++;
    }

    return (p_x - left);
}

static void send_cmd(sh1106_commands_t cmd) {
    uint8_t cmd_[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, SH1106_ADDRESS << 1, cmd_, sizeof(cmd_), 10);
}

static void send_double_cmd(sh1106_commands_t cmd, uint8_t follow_up) {
    uint8_t cmd_[4] = {0x80, cmd, 0x00, follow_up};
    HAL_I2C_Master_Transmit(&hi2c1, SH1106_ADDRESS << 1, cmd_, sizeof(cmd_), 20);
}

static void send_page(const uint8_t* page_data) {
    uint8_t cmd_[128 * 2];

    memset(cmd_, 0xC0, sizeof(cmd_));
    for (uint16_t i = 1; i < 256; i += 2) {
        cmd_[i] = page_data[i >> 1];
    }

    HAL_I2C_Master_Transmit(&hi2c1, SH1106_ADDRESS << 1, cmd_, sizeof(cmd_), 20);
}

static void draw_pixel(uint8_t x, uint8_t y) {
    register uint32_t vram_offset;
    register uint32_t bit_pos;

#if VERTICAL_ORIENTATION
    vram_offset = ((x >> 3) << 7) + y;
    bit_pos     = x & 0x07;
#else
    vram_offset = ((y >> 3) << 7) + x;
    bit_pos     = y & 0x07;
#endif

    if (vram_offset >= ((SCR_W * SCR_H) >> 3)) {
        return;
    }

    switch (self.pixel_mode) {
    case SH1106_PSET:
        SET_MASK(self.vRAM[vram_offset], 1 << bit_pos);
        break;
    case SH1106_PRES:
        CLEAR_MASK(self.vRAM[vram_offset], 1 << bit_pos);
        break;
    case SH1106_PINV:
        INVERT_MASK(self.vRAM[vram_offset], 1 << bit_pos);
        break;
    }
}

static void draw_horizontal_line_base(uint8_t x, uint8_t y, uint8_t w) {
    uint8_t* ptr;
    uint8_t mask;

    ptr = &self.vRAM[(y >> 3) << 7] + x;

    mask = (uint8_t) (1 << (y & 0x07));

    switch (self.pixel_mode) {
    case SH1106_PSET:
        for (; w != 0; w--) {
            SET_MASK(*ptr, mask);
            ptr++;
        }
        break;
    case SH1106_PRES:
        for (; w != 0; w--) {
            CLEAR_MASK(*ptr, mask);
            ptr++;
        }
        break;
    case SH1106_PINV:
        for (; w != 0; w--) {
            INVERT_MASK(*ptr, mask);
            ptr++;
        }
        break;
    }
}

static void draw_vertical_line_base(uint8_t x, uint8_t y, uint8_t h) {
    uint8_t mask;
    uint8_t mod;
    uint8_t* ptr = &self.vRAM[(y >> 3) << 7] + x;

    /* Tabela de máscaras para o primeiro byte. */
    static const uint8_t first_byte_table[] = {
        0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE,
    };

    /* Tabela de máscaras para o último byte. */
    static const uint8_t last_byte_table[] = {
        0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F,
    };

    /* Verifica o primeiro byte. */
    if (y & 0x07) {
        mod  = 8 - (y & 0x07);
        mask = first_byte_table[mod];

        /* Verifica se a linha precisa da máscara completa. */
        if (mod > h) {
            mask &= (0xFF >> (mod - h));
        }

        switch (self.pixel_mode) {
        case SH1106_PSET:
            SET_MASK(*ptr, mask);
            break;
        case SH1106_PRES:
            CLEAR_MASK(*ptr, mask);
            break;
        case SH1106_PINV:
            INVERT_MASK(*ptr, mask);
            break;
        }

        if (mod > h)
            return;

        ptr += SCR_W;
        h -= mod;
    }

    /* Bytes intermediários. */
    switch (self.pixel_mode) {
    case SH1106_PSET:
        for (; h > 7; h -= 8) {
            *ptr = 0x00;
            ptr += SCR_W;
        }
        break;
    case SH1106_PRES:
        for (; h > 7; h -= 8) {
            *ptr = 0xFF;
            ptr += SCR_W;
        }
        break;
    case SH1106_PINV:
        for (; h > 7; h -= 8) {
            *ptr = ~(*ptr);
            ptr += SCR_W;
        }
        break;
    }


    /* Verifica o ultimo byte. */
    if (h) {
        mod  = (h & 0x07);
        mask = last_byte_table[mod];

        switch (self.pixel_mode) {
        case SH1106_PSET:
            SET_MASK(*ptr, mask);
            break;
        case SH1106_PRES:
            CLEAR_MASK(*ptr, mask);
            break;
        case SH1106_PINV:
            INVERT_MASK(*ptr, mask);
            break;
        }
    }
}

static uint8_t print_char(uint8_t x, uint8_t y, char character, const font_t* fnt) {
    const uint8_t* char_bmp;
    uint8_t x_limit;
    uint8_t y_limit;

    if (character < fnt->min_char || character > fnt->max_char) {
        character = fnt->max_char;
    }

    switch (fnt->scan) {
    case FONT_V:
        char_bmp = &fnt->characters[(character - fnt->min_char) * fnt->width];

        /* Fonte não é maior que a página. */
        if (fnt->height <= PAGE_HEIGHT) {
            for (x_limit = x + fnt->width; x < x_limit; x++) {
                draw_byte_v(y, x, *char_bmp);
                char_bmp++;
            }
            break;
        }

        /* Fonte é maior que a página. */
        for (x_limit = x + fnt->width; x < x_limit; x++) {
            for (y_limit = y + fnt->height; y < y_limit; y += PAGE_HEIGHT) {
                if (*char_bmp != 0) {
                    draw_byte_v(y, x, *char_bmp);
                }
                char_bmp++;
            }
        }
        break;
    case FONT_H:
        char_bmp = &fnt->characters[(character - fnt->min_char) * fnt->height];

        /* Fonte não é maior que a página. */
        if (fnt->width <= PAGE_HEIGHT) {
            for (y_limit = y + fnt->height; y < y_limit; y++) {
                draw_byte_h(y, x, *char_bmp);
                char_bmp++;
            }
            break;
        }

        /* Fonte é maior que a página. */
        for (y_limit = y + fnt->height; y < y_limit; y++) {
            for (x_limit = x + fnt->width; x < x_limit; x += PAGE_HEIGHT) {
                if (*char_bmp != 0) {
                    draw_byte_h(y, x, *char_bmp);
                }
                char_bmp++;
            }
        }
        break;
    default:
        break;
    }

    return fnt->width + 1;
}

static inline void draw_byte_v(uint8_t top, uint8_t left, uint8_t byte) {
    for (; byte != 0; byte >>= 1) {
        if (byte & 0x01) {
            draw_pixel(left, top);
        }

        top++;
    }
}

static inline void draw_byte_h(uint8_t top, uint8_t left, uint8_t byte) {
    for (; byte != 0; byte >>= 1) {
        if (byte & 0x01) {
            draw_pixel(left, top);
        }

        left++;
    }
}
