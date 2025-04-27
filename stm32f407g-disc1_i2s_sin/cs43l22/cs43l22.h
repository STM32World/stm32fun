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
//#define CS43L22_ALL_ON            0xAF
#define CS43L22_SPEED_DEF         0xA0
#define CS43L22_INT_DEF           0x07

#define CS43L22_ID                0x01
#define CS43L22_PWR_CTL_1         0x02
#define CS43L22_PWR_CTL_2         0x04
#define CS43L22_CLOCKING          0x05
#define CS43L22_INT_CTL_1         0x06

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

void cs_enable(CS43L22_HandleTypeDef *cs);
void cs_disable(CS43L22_HandleTypeDef *cs);

#endif /* CS43L22_H_ */
