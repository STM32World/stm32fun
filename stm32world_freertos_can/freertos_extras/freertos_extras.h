/*
 * freertos_extras.h
 *
 *  Created on: Feb 11, 2025
 *      Author: lth
 */

#ifndef FREERTOS_EXTRAS_H_
#define FREERTOS_EXTRAS_H_

osStatus_t osMessageQueuePeek (osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);

#endif /* FREERTOS_EXTRAS_H_ */
