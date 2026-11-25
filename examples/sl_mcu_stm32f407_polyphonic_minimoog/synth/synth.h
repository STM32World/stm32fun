/*----------------------------------------------------------------------------
 | @file           : synth.h
 | @brief          : Polyphonic, multitimbral Minimoog-style synthesizer engine
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
 |----------------------------------------------------------------------------
 |
 | Signal chain per voice (as on the Minimoog Model D):
 |
 |   OSC 1 ─┐
 |   OSC 2 ─┼─► MIXER ─► DRIVE ─► 24 dB LADDER FILTER ─► VCA ─► PAN ─► out
 |   OSC 3 ─┤                     ▲ cutoff                ▲
 |   NOISE ─┘                     │                       │
 |                   FILTER CONTOUR (ADS)       LOUDNESS CONTOUR (ADS)
 |                   + keyboard control
 |                   + modulation (OSC 3 / noise, mod wheel)
 |
 | Each MIDI channel has its own patch (set of panel settings).
 |----------------------------------------------------------------------------*/

#ifndef SYNTH_H_
#define SYNTH_H_

#include <stdint.h>

#define SAMPLE_FREQ 48000
// Voices shared by all channels. The F407 (168 MHz) manages 8 with the worst
// case patch at about 75% CPU, a faster MCU can define more.
#ifndef MAX_VOICES
#define MAX_VOICES 8
#endif

// One patch (set of panel settings) per MIDI channel
#define SYNTH_CHANNELS 16

// Number of stereo frames rendered per call to synth_process()
#define SYNTH_BLOCK_SIZE 64

// Pitch, cutoff and filter contour are updated every SYNTH_CONTROL_RATE samples
#define SYNTH_CONTROL_RATE 16

#define SYNTH_OSCILLATORS 3

// Oscillator range switch
typedef enum {
    RANGE_LO = 0,
    RANGE_32,
    RANGE_16,
    RANGE_8,
    RANGE_4,
    RANGE_2,
    RANGE_COUNT
} osc_range_t;

// Oscillator waveform switch. Oscillator 3 has a reverse saw instead of the triangle-saw.
typedef enum {
    WAVE_TRIANGLE = 0,
    WAVE_TRI_SAW,       // Reverse saw on oscillator 3
    WAVE_SAW,
    WAVE_SQUARE,
    WAVE_WIDE_PULSE,
    WAVE_NARROW_PULSE,
    WAVE_COUNT
} osc_wave_t;

// Filter keyboard control (the two switches on the Minimoog)
typedef enum {
    KBD_OFF = 0,
    KBD_THIRD,
    KBD_TWO_THIRDS,
    KBD_FULL,
    KBD_COUNT
} kbd_track_t;

// The panel settings for one channel. Edit the user fields, then call
// synth_patch_update() to recalculate the derived fields.
// Switches and selectors are uint8_t, everything else is float.
typedef struct {
    // Oscillator bank
    uint8_t osc_range[SYNTH_OSCILLATORS];   // osc_range_t
    uint8_t osc_wave[SYNTH_OSCILLATORS];    // osc_wave_t
    float osc_detune[SYNTH_OSCILLATORS];    // Semitones (oscillator 2 and 3)
    uint8_t osc3_kbd;                       // Oscillator 3 keyboard control on/off

    // Mixer
    float mix_level[SYNTH_OSCILLATORS];     // [0.0 - 1.0]
    uint8_t mix_on[SYNTH_OSCILLATORS];
    float noise_level;                      // [0.0 - 1.0]
    uint8_t noise_on;
    uint8_t noise_pink;                     // 0 = white, 1 = pink
    float drive;                            // [0.0 - 1.0] mixer overdrive (the "feedback" trick)

    // Filter
    float cutoff_hz;
    float emphasis;                         // [0.0 - 1.0] self-oscillates near 1.0
    float contour_amount;                   // [0.0 - 1.0] up to +5 octaves
    uint8_t kbd_track;                      // kbd_track_t

    // Contours
    float f_attack_s;
    float f_decay_s;
    float f_sustain;
    float l_attack_s;
    float l_decay_s;
    float l_sustain;
    uint8_t decay_sw;                       // Release uses decay time when on

    // Controllers
    float glide_s;
    uint8_t glide_on;
    float mod_mix;                          // 0.0 = oscillator 3, 1.0 = noise
    uint8_t osc_mod;                        // Modulation to oscillator pitch
    uint8_t filter_mod;                     // Modulation to filter cutoff

    // Output
    float volume;                           // [0.0 - 1.0]
    float pan;                              // [-1.0 (left) - 1.0 (right)]

    // Derived (calculated by synth_patch_update)
    float l_attack_coef;    // Per sample
    float l_decay_coef;
    float l_release_coef;
    float f_attack_coef;    // Per control step
    float f_decay_coef;
    float f_release_coef;
    float glide_coef;       // Per control step
    float cutoff_oct;       // log2(cutoff_hz)
    float feedback;         // Ladder feedback k
    float kbd_amount;       // Octaves per octave
    float mix_gain[SYNTH_OSCILLATORS + 1];   // Oscillators + noise, including on switches and drive
    float gain_l;
    float gain_r;
} synth_patch_t;

void synth_init(void);

// Render SYNTH_BLOCK_SIZE interleaved stereo 16-bit frames into out
void synth_process(int16_t *out);

// channel is 0-15 (MIDI channel 1-16)
void synth_note_on(uint8_t channel, uint8_t note, uint8_t velocity);
void synth_note_off(uint8_t channel, uint8_t note);
void synth_all_notes_off(uint8_t channel);
void synth_all_sound_off(uint8_t channel);

// Performance controls (not part of the patch)
void synth_pitch_bend(uint8_t channel, int16_t bend);      // -8192 to 8191, +/- 2 semitones
void synth_mod_wheel(uint8_t channel, float amount);       // [0.0 - 1.0]
void synth_sustain(uint8_t channel, uint8_t on);

int synth_active_voices(void);

synth_patch_t* synth_get_patch(uint8_t channel);
void synth_patch_update(uint8_t channel);
void synth_patch_reset(uint8_t channel);

uint8_t synth_next_wave(uint8_t channel);

// Print cycles per sample for each stage (cycles() returns a cycle counter)
void synth_benchmark(uint32_t (*cycles)(void));

void synth_set_master_volume(float vol);
float synth_get_master_volume(void);

#endif /* SYNTH_H_ */
