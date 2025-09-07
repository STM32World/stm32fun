/*
 * stats.c
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#include <stdio.h>
#include "cmsis_os.h"

#include <printmutex.h>
#include <statustask.h>

osThreadId_t statusTaskHandle;

const osThreadAttr_t statusTask_attributes = {
  .name = "statusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/**
 * @brief Function implementing the statusTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartStatusTask(void *argument) {
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime;
    float runtime_percentage;

    /* Infinite loop */
    for (;;) {

        osDelay(10000);

        uxArraySize = uxTaskGetNumberOfTasks();
        pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); // a little bit scary!

        osMutexWait(printMutexHandle, osWaitForever);

        if (pxTaskStatusArray != NULL) {

            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize,
                    &ulTotalRunTime);

            printf("Task count = %lu\n", uxArraySize);
            printf("No       Name          P  S   Usage       Count       HW\n");

            for (x = 0; x < uxArraySize; x++) {

                runtime_percentage = (float) (100
                        * (float) pxTaskStatusArray[x].ulRunTimeCounter
                        / (float) ulTotalRunTime);

                printf("Task %2lu: %-12s %2lu %2d %8.4f (%12lu) %5i\n",
                        x,
                        pxTaskStatusArray[x].pcTaskName,
                        pxTaskStatusArray[x].uxCurrentPriority,
                        pxTaskStatusArray[x].eCurrentState,
                        runtime_percentage,
                        pxTaskStatusArray[x].ulRunTimeCounter,
                        pxTaskStatusArray[x].usStackHighWaterMark);

            }

            vPortFree(pxTaskStatusArray);

        } else {
            printf("Unable to allocate stack space\n");
        }

        osMutexRelease(printMutexHandle);

    }

}
