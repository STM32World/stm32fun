/*
 * ledtask.c
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#include "main.h"
#include "cmsis_os.h"
#include <ledtask.h>

osThreadId_t ledTaskHandle;

const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/**
 * @brief Function implementing the statusTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartLedTask(void *argument) {

    /* Infinite loop */
    for (;;) {

        osDelay(500);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    }

}
