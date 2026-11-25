/*----------------------------------------------------------------------------
 | @file           : midi.c
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

#include <stdio.h>

#include "tusb.h"
#include "synth.h"
#include "params.h"
#include "midi.h"

// Log received MIDI messages on the UART (blocking printf, can cause audio dropouts)
#ifndef MIDI_LOG
#define MIDI_LOG 1
#endif

#if MIDI_LOG
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#endif

#define MIDI_NOTE_OFF       0x80
#define MIDI_NOTE_ON        0x90
#define MIDI_CONTROL_CHANGE 0xB0
#define MIDI_PITCH_BEND     0xE0

#define CC_MOD_WHEEL        1
#define CC_MOD_WHEEL_LSB    33
#define CC_SUSTAIN          64
#define CC_ALL_SOUND_OFF    120
#define CC_RESET_ALL        121
#define CC_ALL_NOTES_OFF    123

static uint8_t mod_wheel_msb[SYNTH_CHANNELS];

static void midi_control_change(uint8_t channel, uint8_t cc, uint8_t value) {
    switch (cc) {
    case CC_MOD_WHEEL:
        mod_wheel_msb[channel] = value;
        synth_mod_wheel(channel, (float) value / 127.0f);
        LOG("Ch %2d Mod wheel %d\n", channel + 1, value);
        return;
    case CC_MOD_WHEEL_LSB:
        synth_mod_wheel(channel, (float) ((mod_wheel_msb[channel] << 7) | value) / 16383.0f);
        return;
    case CC_SUSTAIN:
        synth_sustain(channel, value >= 64);
        LOG("Ch %2d Sustain %s\n", channel + 1, value >= 64 ? "on" : "off");
        return;
    case CC_ALL_SOUND_OFF:
        synth_all_sound_off(channel);
        LOG("Ch %2d All sound off\n", channel + 1);
        return;
    case CC_RESET_ALL:
        synth_pitch_bend(channel, 0);
        synth_mod_wheel(channel, 0.0f);
        synth_sustain(channel, 0);
        LOG("Ch %2d Reset controllers\n", channel + 1);
        return;
    case CC_ALL_NOTES_OFF:
        synth_all_notes_off(channel);
        LOG("Ch %2d All notes off\n", channel + 1);
        return;
    }

    const param_def_t *param = param_set_cc(channel, cc, value);
    if (param) {
        LOG("Ch %2d CC %3d = %3d -> %s\n", channel + 1, cc, value, param->name);
    } else {
        LOG("Ch %2d CC %3d = %3d (unmapped)\n", channel + 1, cc, value);
    }
}

void midi_handle_message(uint8_t status, uint8_t data1, uint8_t data2) {
    uint8_t channel = status & 0x0F;

    switch (status & 0xF0) {
    case MIDI_NOTE_ON:
        if (data2 > 0) {
            LOG("Ch %2d On : %d\n", channel + 1, data1);
            synth_note_on(channel, data1, data2);
            break;
        }
        // Note On with velocity 0 is Note Off
        /* no break */

    case MIDI_NOTE_OFF:
        LOG("Ch %2d Off: %d\n", channel + 1, data1);
        synth_note_off(channel, data1);
        break;

    case MIDI_CONTROL_CHANGE:
        midi_control_change(channel, data1, data2);
        break;

    case MIDI_PITCH_BEND:
        synth_pitch_bend(channel, (int16_t) (((data2 << 7) | data1) - 8192));
        break;

    default:
        break;
    }
}

// TinyUSB callback: MIDI data received
void tud_midi_rx_cb(uint8_t itf) {
    (void) itf;
    uint8_t packet[4];

    while (tud_midi_packet_read(packet)) {
        // packet[0]: cable number (high nibble) + code index (low nibble)
        midi_handle_message(packet[1], packet[2], packet[3]);
    }
}
