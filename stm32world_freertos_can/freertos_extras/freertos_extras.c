/*
 * freertos_extras.c
 *
 *  Created on: Feb 11, 2025
 *      Author: lth
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"

//#include "queue.h"
#include "freertos_extras.h"

osStatus_t osMessageQueuePeek (osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout) {

    QueueHandle_t hQueue = (QueueHandle_t)mq_id;
    osStatus_t stat;
    BaseType_t yield;

    (void)msg_prio; /* Message priority is ignored */

    stat = osOK;

    if (IS_IRQ()) {
      if ((hQueue == NULL) || (msg_ptr == NULL) || (timeout != 0U)) {
        stat = osErrorParameter;
      }
      else {
        yield = pdFALSE;
        if (xQueuePeekFromISR(hQueue, msg_ptr) != pdPASS) {
        //if (xQueueReceiveFromISR (hQueue, msg_ptr, &yield) != pdPASS) {
          stat = osErrorResource;
        } else {
          portYIELD_FROM_ISR (yield);
        }
      }
    }
    else {
      if ((hQueue == NULL) || (msg_ptr == NULL)) {
        stat = osErrorParameter;
      }
      else {
        if (xQueuePeek(hQueue, msg_ptr, (TickType_t)timeout) != pdPASS) {
        //if (xQueueReceive (hQueue, msg_ptr, (TickType_t)xTicksToWait) != pdPASS) {
          if (timeout != 0U) {
            stat = osErrorTimeout;
          } else {
            stat = osErrorResource;
          }
        }
      }
    }

    return (stat);

}
