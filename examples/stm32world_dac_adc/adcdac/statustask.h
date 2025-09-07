/*
 * stats.h
 *
 *  Created on: Nov 11, 2024
 *      Author: lth
 */

#ifndef STATUSTASK_H_
#define STATUSTASK_H_

extern osThreadId_t statusTaskHandle;

extern const osThreadAttr_t statusTask_attributes;

void StartStatusTask(void *argument);

#endif /* STATUSTASK_H_ */
