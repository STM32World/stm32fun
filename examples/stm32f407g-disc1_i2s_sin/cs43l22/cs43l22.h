/*
 * cs43l22.h
 *
 *  Created on: Apr 25, 2025
 *      Author: lth
 */

#ifndef CS43L22_H_
#define CS43L22_H_

#ifdef DEBUG
#include <stdio.h>
#define CS_DBG(...) printf(__VA_ARGS__)
#else
#define CS_DBG(...)
#endif

#define CS43L22_DEFAULT_ADDR      0x4A

#define CS43L22_DEFAULT_ID        0x1C
#define CS43L22_PWR_CTL_1_OFF     0x01
#define CS43L22_PWR_CTL_1_ON      0x9E
#define CS43L22_PWR_CTL_2_HPB_ON  0b10000000
#define CS43L22_PWR_CTL_2_HPB_OFF 0b11000000
#define CS43L22_PWR_CTL_2_HPA_ON  0b00100000
#define CS43L22_PWR_CTL_2_HPA_OFF 0b00110000
#define CS43L22_PWR_CTL_2_SPB_ON  0b00001000
#define CS43L22_PWR_CTL_2_SPB_OFF 0b00001100
#define CS43L22_PWR_CTL_2_SPA_ON  0b00000010
#define CS43L22_PWR_CTL_2_SPA_OFF 0b00000011
#define CS43L22_CLK_AUTO          0b10000000
#define CS43L22_CLK_SPD_DBL       0b00000000
#define CS43L22_CLK_SPD_SIN       0b00100000
#define CS43L22_CLK_SPD_HLF       0b01000000
#define CS43L22_CLK_SPD_QUA       0b01100000
#define CS43L22_CLK_32K           0b00010000
#define CS43L22_CLK_VID           0b00001000
#define CS43L22_CLK_RAT_0         0b00000000
#define CS43L22_CLK_RAT_1         0b00000010
#define CS43L22_CLK_RAT_2         0b00000100
#define CS43L22_CLK_RAT_3         0b00000110
#define CS43L22_CLK_MCLK_DIV      0b00000001
#define CS43L22_INT_CTL_1_MAS     0b10000000
#define CS43L22_INT_CTL_1_INV     0b01000000
#define CS43L22_INT_CTL_1_DSP     0b00010000
#define CS43L22_INT_CTL_1_DIF_LI2 0b00000100
#define CS43L22_INT_CTL_1_DIF_RJ  0b00001000
#define CS43L22_INT_CTL_1_AWL_32  0b00000000
#define CS43L22_INT_CTL_1_AWL_24  0b00000001
#define CS43L22_INT_CTL_1_AWL_20  0b00000010
#define CS43L22_INT_CTL_1_AWL_16  0b00000011

#define CS43L22_ID                0x01
#define CS43L22_PWR_CTL_1         0x02
#define CS43L22_PWR_CTL_2         0x04
#define CS43L22_CLK               0x05
#define CS43L22_INT_CTL_1         0x06
#define CS43L22_MASL_VOL          0x20
#define CS43L22_MASR_VOL          0x21
#define CS43L22_HPL_VOL           0x22
#define CS43L22_HPR_VOL           0x23

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t i2c_addr;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    uint8_t id;
    uint8_t rev;
} CS43L22_HandleTypeDef;

typedef enum {
    Ok,
    Err
} CS43L22_result_t;

CS43L22_result_t cs_init(CS43L22_HandleTypeDef *cs, I2C_HandleTypeDef *i2c, uint16_t i2c_addr, GPIO_TypeDef *rst_port, uint16_t rst_pin);
CS43L22_result_t cs_mas_vol(CS43L22_HandleTypeDef *cs, uint8_t vol);
CS43L22_result_t cs_hp_vol(CS43L22_HandleTypeDef *cs, uint8_t vol);


void cs_enable(CS43L22_HandleTypeDef *cs);
void cs_disable(CS43L22_HandleTypeDef *cs);

#endif /* CS43L22_H_ */
