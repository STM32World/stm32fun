/*
 * trng.h
 *
 *  Created on: Sep 1, 2024
 *      Author: lth
 */

#ifndef TRNG_H_
#define TRNG_H_

#ifdef DEBUG
#include <stdio.h>
#define TRNG_DBG(...) printf(__VA_ARGS__);
#else
#define TRNG_DBG(...)
#endif

#define TRNG_ADDR 0x50060800

// Register offsets from reference manual
#define TRNG_CR_ADDR TRNG_ADDR
#define TRNG_SR_ADDR TRNG_ADDR + 1
#define TRNG_DR_ADDR TRNG_ADDR + 2

// Control register bits - from reference manual
#define TRNG_CR_IE (1 << 3)
#define TRNG_CR_RNGEN (1 << 2)

// Status register bits - from reference manual
#define TRNG_SR_SEIS (1 << 6)
#define TRNG_SR_CEIS (1 << 5)
#define TRNG_SR_SECS (1 << 2)
#define TRNG_SR_CECS (1 << 1)
#define TRNG_SR_DRDY (1 << 0)

void trng_init();
uint32_t trng_get();

#endif /* TRNG_H_ */
