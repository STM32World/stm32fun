/*----------------------------------------------------------------------------
 | @file           : midi.h
 | @brief          : USB MIDI input handling
 |----------------------------------------------------------------------------
 | @attention
 |
 | Copyright (c) 2026 STM32World
 | All rights reserved.
 |
 | This software is licensed under terms that can be found in the LICENSE file
 | in the root directory of this software component.
 | If no LICENSE file comes with this software, it is provided AS-IS.
 |
 |----------------------------------------------------------------------------*/

#ifndef MIDI_H_
#define MIDI_H_

#include <stdint.h>

// Handle one MIDI channel message (status byte includes the channel)
void midi_handle_message(uint8_t status, uint8_t data1, uint8_t data2);

#endif /* MIDI_H_ */
