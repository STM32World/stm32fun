/*----------------------------------------------------------------------------
 | @file           : params.c
 | @brief          : Synth parameter table (MIDI CC to patch mapping)
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

#include <math.h>

#include "synth.h"
#include "params.h"

#define P(field) offsetof(synth_patch_t, field)

// The Minimoog panel, left to right. CC numbers follow General MIDI where
// there is a convention (volume, pan, portamento, resonance, cutoff, attack,
// decay), the rest use undefined controllers.
const param_def_t param_table[] = {
        // Controllers
        { "glide time",      5, PARAM_EXP,     0.001f, 5.0f,     P(glide_s),        0 },
        { "glide",          65, PARAM_ENUM,    0.0f,   1.0f,     P(glide_on),       0 },
        { "mod mix",        82, PARAM_LINEAR,  0.0f,   1.0f,     P(mod_mix),        0 },
        { "osc mod",        83, PARAM_ENUM,    0.0f,   1.0f,     P(osc_mod),        0 },
        { "filter mod",     84, PARAM_ENUM,    0.0f,   1.0f,     P(filter_mod),     0 },

        // Oscillator bank
        { "osc1 range",     14, PARAM_ENUM,    0.0f,   RANGE_COUNT - 1, P(osc_range[0]), 0 },
        { "osc1 wave",      15, PARAM_ENUM,    0.0f,   WAVE_COUNT - 1,  P(osc_wave[0]),  0 },
        { "osc2 range",     16, PARAM_ENUM,    0.0f,   RANGE_COUNT - 1, P(osc_range[1]), 0 },
        { "osc2 wave",      17, PARAM_ENUM,    0.0f,   WAVE_COUNT - 1,  P(osc_wave[1]),  0 },
        { "osc2 detune",    18, PARAM_BIPOLAR, -7.0f,  7.0f,     P(osc_detune[1]),  1 },
        { "osc3 range",     19, PARAM_ENUM,    0.0f,   RANGE_COUNT - 1, P(osc_range[2]), 0 },
        { "osc3 wave",      20, PARAM_ENUM,    0.0f,   WAVE_COUNT - 1,  P(osc_wave[2]),  0 },
        { "osc3 detune",    21, PARAM_BIPOLAR, -7.0f,  7.0f,     P(osc_detune[2]),  1 },
        { "osc3 kbd",       22, PARAM_ENUM,    0.0f,   1.0f,     P(osc3_kbd),       0 },

        // Mixer
        { "osc1 level",     23, PARAM_LINEAR,  0.0f,   1.0f,     P(mix_level[0]),   0 },
        { "osc2 level",     24, PARAM_LINEAR,  0.0f,   1.0f,     P(mix_level[1]),   0 },
        { "osc3 level",     25, PARAM_LINEAR,  0.0f,   1.0f,     P(mix_level[2]),   0 },
        { "noise level",    26, PARAM_LINEAR,  0.0f,   1.0f,     P(noise_level),    0 },
        { "osc1 on",        27, PARAM_ENUM,    0.0f,   1.0f,     P(mix_on[0]),      0 },
        { "osc2 on",        28, PARAM_ENUM,    0.0f,   1.0f,     P(mix_on[1]),      0 },
        { "osc3 on",        29, PARAM_ENUM,    0.0f,   1.0f,     P(mix_on[2]),      0 },
        { "noise on",       30, PARAM_ENUM,    0.0f,   1.0f,     P(noise_on),       0 },
        { "noise pink",     31, PARAM_ENUM,    0.0f,   1.0f,     P(noise_pink),     0 },
        { "drive",          81, PARAM_LINEAR,  0.0f,   1.0f,     P(drive),          0 },

        // Filter
        { "cutoff",         74, PARAM_EXP,     20.0f,  20000.0f, P(cutoff_hz),      0 },
        { "cutoff fine",    13, PARAM_EXP,     20.0f,  20000.0f, P(cutoff_hz),      1 },
        { "emphasis",       71, PARAM_LINEAR,  0.0f,   1.0f,     P(emphasis),       0 },
        { "contour amount", 77, PARAM_LINEAR,  0.0f,   1.0f,     P(contour_amount), 0 },
        { "kbd control",    78, PARAM_ENUM,    0.0f,   KBD_COUNT - 1, P(kbd_track), 0 },
        { "f attack",       85, PARAM_EXP,     0.001f, 10.0f,    P(f_attack_s),     0 },
        { "f decay",        86, PARAM_EXP,     0.004f, 20.0f,    P(f_decay_s),      0 },
        { "f sustain",      87, PARAM_LINEAR,  0.0f,   1.0f,     P(f_sustain),      0 },

        // Loudness contour
        { "l attack",       73, PARAM_EXP,     0.001f, 10.0f,    P(l_attack_s),     0 },
        { "l decay",        75, PARAM_EXP,     0.004f, 20.0f,    P(l_decay_s),      0 },
        { "l sustain",      79, PARAM_LINEAR,  0.0f,   1.0f,     P(l_sustain),      0 },
        { "decay switch",   80, PARAM_ENUM,    0.0f,   1.0f,     P(decay_sw),       0 },

        // Output
        { "volume",          7, PARAM_LINEAR,  0.0f,   1.0f,     P(volume),         0 },
        { "pan",            10, PARAM_BIPOLAR, -1.0f,  1.0f,     P(pan),            0 },
};

const size_t param_count = sizeof(param_table) / sizeof(param_table[0]);

// Last MSB received per channel for the 14-bit controllers 0-31
static uint8_t msb_cache[SYNTH_CHANNELS][32];

const param_def_t* param_find_cc(uint8_t cc) {
    for (size_t i = 0; i < param_count; i++) {
        if (param_table[i].cc == cc)
            return &param_table[i];
    }
    return NULL;
}

// u: unipolar position [0, 1], b: bipolar position [-1, 1]
static void param_apply(uint8_t channel, const param_def_t *param, float u, float b) {
    synth_patch_t *patch = synth_get_patch(channel);
    void *field = (uint8_t*) patch + param->offset;

    switch (param->curve) {
    case PARAM_LINEAR:
        *(float*) field = param->min + u * (param->max - param->min);
        break;
    case PARAM_EXP:
        *(float*) field = param->min * powf(param->max / param->min, u);
        break;
    case PARAM_BIPOLAR:
        *(float*) field = param->min + (b + 1.0f) * 0.5f * (param->max - param->min);
        break;
    case PARAM_ENUM: {
        int steps = (int) param->max + 1;
        int idx = (int) (u * steps);
        *(uint8_t*) field = (uint8_t) (idx >= steps ? steps - 1 : idx);
        break;
    }
    }

    synth_patch_update(channel);
}

static float clamp_bipolar(float b) {
    return b < -1.0f ? -1.0f : (b > 1.0f ? 1.0f : b);
}

const param_def_t* param_set_cc(uint8_t channel, uint8_t cc, uint8_t value) {
    if (channel >= SYNTH_CHANNELS || cc > 127)
        return NULL;
    value &= 0x7F;

    if (cc >= 32 && cc < 64) {
        // LSB of a 14-bit controller
        const param_def_t *param = param_find_cc(cc - 32);
        if (param == NULL || !param->fine)
            return NULL;
        uint16_t raw = ((uint16_t) msb_cache[channel][cc - 32] << 7) | value;
        param_apply(channel, param, (float) raw / 16383.0f, clamp_bipolar(((float) raw - 8192.0f) / 8191.0f));
        return param;
    }

    const param_def_t *param = param_find_cc(cc);
    if (param == NULL)
        return NULL;

    if (cc < 32)
        msb_cache[channel][cc] = value;

    param_apply(channel, param, (float) value / 127.0f, clamp_bipolar(((float) value - 64.0f) / 63.0f));
    return param;
}

uint8_t param_get_cc(uint8_t channel, const param_def_t *param) {
    synth_patch_t *patch = synth_get_patch(channel);
    if (param == NULL || patch == NULL)
        return 0;

    void *field = (uint8_t*) patch + param->offset;
    float x = 0.0f;

    switch (param->curve) {
    case PARAM_LINEAR:
        x = (*(float*) field - param->min) / (param->max - param->min);
        break;
    case PARAM_EXP:
        x = logf(*(float*) field / param->min) / logf(param->max / param->min);
        break;
    case PARAM_BIPOLAR: {
        float v = 64.0f + 63.0f * (2.0f * (*(float*) field - param->min) / (param->max - param->min) - 1.0f);
        return v < 0.0f ? 0 : (v > 127.0f ? 127 : (uint8_t) (v + 0.5f));
    }
    case PARAM_ENUM: {
        int steps = (int) param->max + 1;
        return (uint8_t) ((*(uint8_t*) field * 128 + 64) / steps);
    }
    }

    if (x < 0.0f)
        x = 0.0f;
    if (x > 1.0f)
        x = 1.0f;
    return (uint8_t) (x * 127.0f + 0.5f);
}
