/*
 * ticktask.c
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */


#include <stdio.h>
#include "cmsis_os.h"

#include <printmutex.h>
#include <ticktask.h>

osThreadId_t tickTaskHandle;

const osThreadAttr_t tickTask_attributes = {
  .name = "tickTask",
  .stack_size = 192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Function implementing the statusTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartTickTask(void *argument) {

    /* Infinite loop */
    for (;;) {

        osDelay(1000);
        osMutexWait(printMutexHandle, osWaitForever);
        printf("Tick %lu\n", osKernelGetTickCount() / 1000);
        osMutexRelease(printMutexHandle);

    }

}
