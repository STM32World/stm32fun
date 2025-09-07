/*
 * ledtask.h
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#ifndef LEDTASK_H_
#define LEDTASK_H_

extern osThreadId_t ledTaskHandle;

extern const osThreadAttr_t ledTask_attributes;

void StartLedTask(void *argument);

#endif /* LEDTASK_H_ */
