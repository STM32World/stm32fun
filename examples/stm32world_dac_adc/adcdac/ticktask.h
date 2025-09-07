/*
 * ticktask.h
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#ifndef TICKTASK_H_
#define TICKTASK_H_

extern osThreadId_t tickTaskHandle;

extern const osThreadAttr_t tickTask_attributes;

void StartTickTask(void *argument);

#endif /* TICKTASK_H_ */
