/*
 * printmutex.c
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#include "cmsis_os.h"
#include "printmutex.h"

osMutexId_t printMutexHandle;

const osMutexAttr_t printMutex_attributes = {
  .name = "printMutex"
};

