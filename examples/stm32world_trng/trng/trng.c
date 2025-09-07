/*
 * trng.c
 *
 *  Created on: Sep 1, 2024
 *      Author: lth
 */

#include "main.h"

#include "trng.h"

uint32_t *trng_cr = (uint32_t *)TRNG_CR_ADDR;
uint32_t *trng_sr = (uint32_t *)TRNG_SR_ADDR;
uint32_t *trng_dr = (uint32_t *)TRNG_DR_ADDR;

uint32_t last_rng = 0;

uint32_t trng_get() {
    TRNG_DBG("TRNG get\n");
    TRNG_DBG("CR = 0x%08lx\n", *trng_cr);
    TRNG_DBG("SR = 0x%08lx\n", *trng_sr);

    //while ((*trng_sr & TRNG_SR_DRDY) == 0); // Wait for it.

    return *trng_dr;
}

void trng_init() {
    TRNG_DBG("TRNG Init\n");

    // Enable the RNG clock - without this the peripheral will not work.
    __HAL_RCC_RNG_CLK_ENABLE();

    TRNG_DBG("CR = 0x%08lx SR = 0x%08lx DR = 0x%08lx\n", (uint32_t)trng_cr, (uint32_t)trng_sr, (uint32_t)trng_dr);

    TRNG_DBG("TRNG Initial Values\n");
    TRNG_DBG("CR = 0x%08lx\n", *trng_cr);
    TRNG_DBG("SR = 0x%08lx\n", *trng_sr);

    // Fire up the RNG
    *trng_cr |= TRNG_CR_RNGEN;

    //trng_get();

}
