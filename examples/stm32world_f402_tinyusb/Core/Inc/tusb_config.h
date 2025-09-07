/*
 * tusb_config.h
 *
 *  Created on: Sep 7, 2025
 *      Author: lth
 */

#ifndef INC_TUSB_CONFIG_H_
#define INC_TUSB_CONFIG_H_

// Debugging
#define CFG_TUSB_DEBUG               3

// Enable device stack
#define CFG_TUD_ENABLED              1

// Endpoint size for full-speed
#define CFG_TUD_ENDPOINT0_SIZE      64

// Enables 2 interfaces of class CDC
#define CFG_TUD_CDC                  2

#endif /* INC_TUSB_CONFIG_H_ */
