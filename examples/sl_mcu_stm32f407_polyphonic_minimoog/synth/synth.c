/*----------------------------------------------------------------------------
 | @file           : synth.c
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
 |----------------------------------------------------------------------------*/

// The audio path is always built optimised, even in the Debug configuration
#pragma GCC optimize ("O3", "fast-math")

#include <math.h>
#include <string.h>

#include "synth.h"

// Voice state lives in the F407's 64K core coupled RAM (zero wait states, not on the DMA bus)
#ifdef __arm__
#define FAST_RAM __attribute__((section(".ccmram")))
#else
#define FAST_RAM
#endif

#define PI 3.14159265358979323846f

#define PCM16_MAX 32767.0f

#define CONTROL_STEPS (SYNTH_BLOCK_SIZE / SYNTH_CONTROL_RATE)
#define CONTROL_FREQ ((float) SAMPLE_FREQ / SYNTH_CONTROL_RATE)

_Static_assert(SYNTH_BLOCK_SIZE % SYNTH_CONTROL_RATE == 0, "block size must be a multiple of the control rate");

#define MIX_GAIN 0.5f               // One oscillator at full level = 0.5 into the filter
#define NOISE_GAIN 0.7f
#define FILTER_SEED 0.0001f         // Initial ladder state so the filter can self-oscillate with the mixer off
#define OUTPUT_GAIN 0.7f
#define PITCH_BEND_SEMI 2.0f
#define MOD_PITCH_SEMI 7.0f         // Full mod wheel, oscillator modulation
#define MOD_FILTER_OCT 4.0f         // Full mod wheel, filter modulation
#define CONTOUR_OCTAVES 5.0f        // Full contour amount
#define FAST_RELEASE_S 0.005f       // Release time with the decay switch off
#define ENV_ATTACK_TARGET 1.3f      // Analog style attack aims past 1.0
#define ENV_END_LEVEL 0.0001f
#define OSC3_FREE_NOTE 60.0f        // Oscillator 3 base note with keyboard control off
#define OSC3_FREE_DETUNE 36.0f      // and its frequency knob range (+/- semitones)
#define MAX_OSC_FREQ (0.45f * SAMPLE_FREQ)
#define MIN_CUTOFF 20.0f
#define MAX_CUTOFF (0.45f * SAMPLE_FREQ)
#define MAX_FEEDBACK 4.15f          // Ladder self-oscillates at 4.0

// Octaves relative to 8' for LO, 32', 16', 8', 4', 2'
static const float range_octaves[RANGE_COUNT] = { -6.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f };

// Pulse widths for square, wide and narrow (approximations of the Model D)
#define PW_SQUARE 0.5f
#define PW_WIDE 0.3f
#define PW_NARROW 0.12f

typedef enum {
    ENV_IDLE = 0,
    ENV_ATTACK,
    ENV_DECAY,      // Decays towards the sustain level and stays there
    ENV_RELEASE
} env_stage_t;

typedef struct {
    env_stage_t stage;
    float level;
} envelope_t;

typedef struct {
    uint8_t active;
    uint8_t channel;
    uint8_t note;
    uint8_t held;           // Released while the sustain pedal is down
    uint32_t age;           // For voice stealing
    float velocity_gain;
    float pitch;            // Current note number (with glide)

    float phase[SYNTH_OSCILLATORS];
    float dt[SYNTH_OSCILLATORS];
    float inv_dt[SYNTH_OSCILLATORS];

    envelope_t loud;
    envelope_t filt;
    float loud_out;         // Loudness level interpolated per sample
    float d_loud;

    float s[4];             // Ladder filter state
    float G;                // Ladder one-pole coefficient g / (1 + g)
    float dG;               // Per sample change of G (smooths cutoff changes)
    float inv_den;          // 1 / (1 + k * G^4) at the control step
} voice_t;

typedef struct {
    float bend_semi;
    float mod;              // Mod wheel [0.0 - 1.0]
    uint8_t sustain;
    float last_note;        // Glide starting point for new voices
} channel_state_t;

static voice_t voices[MAX_VOICES] FAST_RAM;
static synth_patch_t patches[SYNTH_CHANNELS];
static channel_state_t channels[SYNTH_CHANNELS];

static uint32_t voice_counter = 0;

// Master Volume Scaler [0.0f to 1.0f]
static float master_volume = 0.5f;

// Noise generator state (shared by all voices)
static uint32_t noise_seed = 0x12345678;
static float pink_b0, pink_b1, pink_b2;

// Default patch: a fat two oscillator bass/lead
static const synth_patch_t default_patch = {
        .osc_range = { RANGE_8, RANGE_8, RANGE_16 },
        .osc_wave = { WAVE_SAW, WAVE_SAW, WAVE_SQUARE },
        .osc_detune = { 0.0f, 0.07f, 0.0f },
        .osc3_kbd = 1,

        .mix_level = { 0.8f, 0.8f, 0.5f },
        .mix_on = { 1, 1, 0 },
        .noise_level = 0.5f,
        .noise_on = 0,
        .noise_pink = 0,
        .drive = 0.0f,

        .cutoff_hz = 800.0f,
        .emphasis = 0.3f,
        .contour_amount = 0.5f,
        .kbd_track = KBD_THIRD,

        .f_attack_s = 0.005f,
        .f_decay_s = 0.4f,
        .f_sustain = 0.3f,
        .l_attack_s = 0.005f,
        .l_decay_s = 0.3f,
        .l_sustain = 0.8f,
        .decay_sw = 1,

        .glide_s = 0.1f,
        .glide_on = 0,
        .mod_mix = 0.0f,
        .osc_mod = 0,
        .filter_mod = 0,

        .volume = 0.8f,
        .pan = 0.0f
};

/*----------------------------------------------------------------------------
 | Helpers
 ----------------------------------------------------------------------------*/

// One-pole coefficient reaching the target in roughly 'seconds'
static float env_coef(float seconds, float rate, float tau_ratio) {
    if (seconds < 0.0005f)
        seconds = 0.0005f;
    float tau = seconds / tau_ratio;
    return 1.0f - expf(-1.0f / (tau * rate));
}

#define ATTACK_TAU_RATIO 1.466f     // ln(1.3 / 0.3): time to reach 1.0 aiming at 1.3
#define DECAY_TAU_RATIO 3.0f        // 95% of the way

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// Cubic soft clipper (tanh-like, no division): smooth, reaches +/- 1.0 at +/- 1.5
static inline float soft_clip(float x) {
    if (x > 1.5f)
        return 1.0f;
    if (x < -1.5f)
        return -1.0f;
    return x - (4.0f / 27.0f) * x * x * x;
}

// 2^x: split into integer and fraction in [-0.5, 0.5], polynomial for the fraction
// (relative error about 2e-6, 0.003 cents) and the integer goes into the exponent bits
static inline float fast_exp2f(float x) {
    float r = x + 0.5f;
    int32_t i = (int32_t) r;
    if ((float) i > r)
        i--;
    float f = x - (float) i;
    float p = 1.0f + f * (0.69314718f + f * (0.24022651f + f * (0.05550411f + f * (0.00961813f + f * 0.00133336f))));
    union {
        float f;
        int32_t i;
    } u = { p };
    u.i += i << 23;
    return u.f;
}

// tan(x) for 0 <= x <= 0.45 * pi (Pade approximation, under 1% error below 0.4 * pi)
static inline float fast_tanf(float x) {
    float x2 = x * x;
    return x * (15.0f - x2) / (15.0f - 6.0f * x2);
}

static inline float note_to_freq(float note) {
    return 440.0f * fast_exp2f((note - 69.0f) * (1.0f / 12.0f));
}

static inline uint32_t xorshift32(void) {
    uint32_t x = noise_seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    noise_seed = x;
    return x;
}

/*----------------------------------------------------------------------------
 | Oscillators (PolyBLEP band limited)
 ----------------------------------------------------------------------------*/

static inline float poly_blep(float t, float dt, float inv_dt) {
    if (t < dt) {
        t *= inv_dt;
        return t + t - t * t - 1.0f;
    }
    if (t > 1.0f - dt) {
        t = (t - 1.0f) * inv_dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

static inline float osc_saw(float t, float dt, float inv_dt) {
    return 2.0f * t - 1.0f - poly_blep(t, dt, inv_dt);
}

static inline float osc_pulse(float t, float dt, float inv_dt, float pw) {
    float v = (t < pw ? 1.0f : -1.0f) + poly_blep(t, dt, inv_dt);
    float t2 = t + (1.0f - pw);
    if (t2 >= 1.0f)
        t2 -= 1.0f;
    v -= poly_blep(t2, dt, inv_dt);
    return v - (2.0f * pw - 1.0f); // Remove DC
}

static inline float osc_triangle(float t) {
    return 1.0f - 4.0f * fabsf(t - 0.5f);
}

static inline float osc_sample(uint8_t wave, int osc, float t, float dt, float inv_dt) {
    switch (wave) {
    case WAVE_TRIANGLE:
        return osc_triangle(t);
    case WAVE_TRI_SAW:
        if (osc == 2)
            return -osc_saw(t, dt, inv_dt); // Reverse saw on oscillator 3
        return 0.5f * (osc_triangle(t) + osc_saw(t, dt, inv_dt));
    case WAVE_SAW:
        return osc_saw(t, dt, inv_dt);
    case WAVE_SQUARE:
        return osc_pulse(t, dt, inv_dt, PW_SQUARE);
    case WAVE_WIDE_PULSE:
        return osc_pulse(t, dt, inv_dt, PW_WIDE);
    case WAVE_NARROW_PULSE:
        return osc_pulse(t, dt, inv_dt, PW_NARROW);
    default:
        return 0.0f;
    }
}

/*----------------------------------------------------------------------------
 | Contours
 ----------------------------------------------------------------------------*/

static inline void env_step(envelope_t *e, float attack, float decay, float sustain, float release) {
    switch (e->stage) {
    case ENV_ATTACK:
        e->level += (ENV_ATTACK_TARGET - e->level) * attack;
        if (e->level >= 1.0f) {
            e->level = 1.0f;
            e->stage = ENV_DECAY;
        }
        break;
    case ENV_DECAY:
        e->level += (sustain - e->level) * decay;
        break;
    case ENV_RELEASE:
        e->level -= e->level * release;
        if (e->level < ENV_END_LEVEL) {
            e->level = 0.0f;
            e->stage = ENV_IDLE;
        }
        break;
    default:
        break;
    }
}

/*----------------------------------------------------------------------------
 | Patches
 ----------------------------------------------------------------------------*/

void synth_patch_update(uint8_t channel) {
    if (channel >= SYNTH_CHANNELS)
        return;
    synth_patch_t *p = &patches[channel];

    p->l_attack_coef = env_coef(p->l_attack_s, CONTROL_FREQ, ATTACK_TAU_RATIO);
    p->l_decay_coef = env_coef(p->l_decay_s, CONTROL_FREQ, DECAY_TAU_RATIO);
    p->l_release_coef = env_coef(p->decay_sw ? p->l_decay_s : FAST_RELEASE_S, CONTROL_FREQ, DECAY_TAU_RATIO);

    p->f_attack_coef = env_coef(p->f_attack_s, CONTROL_FREQ, ATTACK_TAU_RATIO);
    p->f_decay_coef = env_coef(p->f_decay_s, CONTROL_FREQ, DECAY_TAU_RATIO);
    p->f_release_coef = env_coef(p->decay_sw ? p->f_decay_s : FAST_RELEASE_S, CONTROL_FREQ, DECAY_TAU_RATIO);

    p->glide_coef = env_coef(p->glide_s, CONTROL_FREQ, DECAY_TAU_RATIO);

    p->cutoff_oct = log2f(clampf(p->cutoff_hz, MIN_CUTOFF, MAX_CUTOFF));
    p->feedback = MAX_FEEDBACK * clampf(p->emphasis, 0.0f, 1.0f);
    p->kbd_amount = (float) p->kbd_track / 3.0f;

    float drive = 1.0f + 3.0f * clampf(p->drive, 0.0f, 1.0f);
    for (int i = 0; i < SYNTH_OSCILLATORS; i++)
        p->mix_gain[i] = p->mix_on[i] ? p->mix_level[i] * MIX_GAIN * drive : 0.0f;
    p->mix_gain[SYNTH_OSCILLATORS] = p->noise_on ? p->noise_level * MIX_GAIN * NOISE_GAIN * drive : 0.0f;

    // Balance pan: centre = full level on both sides
    p->gain_l = p->volume * (p->pan > 0.0f ? 1.0f - p->pan : 1.0f);
    p->gain_r = p->volume * (p->pan < 0.0f ? 1.0f + p->pan : 1.0f);
}

void synth_patch_reset(uint8_t channel) {
    if (channel >= SYNTH_CHANNELS)
        return;
    patches[channel] = default_patch;
    synth_patch_update(channel);
}

void synth_init(void) {
    memset(voices, 0, sizeof(voices));
    memset(channels, 0, sizeof(channels));
    for (int ch = 0; ch < SYNTH_CHANNELS; ch++) {
        channels[ch].last_note = -1.0f;
        synth_patch_reset(ch);
    }
}

synth_patch_t* synth_get_patch(uint8_t channel) {
    return channel < SYNTH_CHANNELS ? &patches[channel] : NULL;
}

void synth_set_master_volume(float vol) {
    master_volume = clampf(vol, 0.0f, 1.0f);
}

float synth_get_master_volume(void) {
    return master_volume;
}

uint8_t synth_next_wave(uint8_t channel) {
    if (channel >= SYNTH_CHANNELS)
        return 0;
    patches[channel].osc_wave[0] = (patches[channel].osc_wave[0] + 1) % WAVE_COUNT;
    return patches[channel].osc_wave[0];
}

/*----------------------------------------------------------------------------
 | Rendering
 ----------------------------------------------------------------------------*/

// Control rate update: glide, pitch, modulation, filter contour and cutoff
static void voice_control(voice_t *v, const synth_patch_t *p, const channel_state_t *cs) {
    if (p->glide_on)
        v->pitch += ((float) v->note - v->pitch) * p->glide_coef;
    else
        v->pitch = v->note;

    float mod_depth = cs->mod * cs->mod; // Squared for finer control of small amounts
    float mod_src = 0.0f;
    if (mod_depth > 0.0f && (p->osc_mod || p->filter_mod)) {
        float noise = (float) (int32_t) xorshift32() * (1.0f / 2147483648.0f);
        float osc3 = osc_sample(p->osc_wave[2], 2, v->phase[2], v->dt[2], v->inv_dt[2]);
        mod_src = (1.0f - p->mod_mix) * osc3 + p->mod_mix * noise;
    }
    float pitch_mod = p->osc_mod ? mod_src * mod_depth * MOD_PITCH_SEMI : 0.0f;

    float note = v->pitch + cs->bend_semi;

    for (int i = 0; i < SYNTH_OSCILLATORS; i++) {
        float n;
        if (i == 2 && !p->osc3_kbd) {
            n = OSC3_FREE_NOTE + p->osc_detune[2] * (OSC3_FREE_DETUNE / 7.0f);
        } else {
            n = note + p->osc_detune[i];
            if (i < 2)
                n += pitch_mod; // Oscillator 3 is the modulation source
        }
        n += 12.0f * range_octaves[p->osc_range[i]];
        float f = note_to_freq(n);
        if (f > MAX_OSC_FREQ)
            f = MAX_OSC_FREQ;
        v->dt[i] = f * (1.0f / SAMPLE_FREQ);
        v->inv_dt[i] = SAMPLE_FREQ / f;
    }

    // Contours, gate is released by setting the stage to ENV_RELEASE. The loudness
    // contour is interpolated linearly between control steps.
    env_step(&v->filt, p->f_attack_coef, p->f_decay_coef, p->f_sustain, p->f_release_coef);
    env_step(&v->loud, p->l_attack_coef, p->l_decay_coef, p->l_sustain, p->l_release_coef);
    v->d_loud = (v->loud.level - v->loud_out) * (1.0f / SYNTH_CONTROL_RATE);

    float oct = p->cutoff_oct
            + p->contour_amount * CONTOUR_OCTAVES * v->filt.level
            + p->kbd_amount * (note - 60.0f) * (1.0f / 12.0f);
    if (p->filter_mod)
        oct += mod_src * mod_depth * MOD_FILTER_OCT;

    float fc = clampf(fast_exp2f(clampf(oct, 4.0f, 15.0f)), MIN_CUTOFF, MAX_CUTOFF);
    float g = fast_tanf(PI * fc * (1.0f / SAMPLE_FREQ));
    float G = g / (1.0f + g);
    float G2 = G * G;

    v->dG = (G - v->G) * (1.0f / SYNTH_CONTROL_RATE);
    v->inv_den = 1.0f / (1.0f + p->feedback * G2 * G2);
}

// Add one control step of a single oscillator into buf (waveform switch hoisted out of the loop)
static void osc_render(float *buf, float gain, uint8_t wave, int osc, float *phase, float dt, float inv_dt) {
    float t = *phase;

#define OSC_LOOP(expr) \
    for (int j = 0; j < SYNTH_CONTROL_RATE; j++) { \
        buf[j] += gain * (expr); \
        t += dt; \
        if (t >= 1.0f) \
            t -= 1.0f; \
    } \
    break

    switch (wave) {
    case WAVE_TRIANGLE:
        OSC_LOOP(osc_triangle(t));
    case WAVE_TRI_SAW:
        if (osc == 2) {
            OSC_LOOP(-osc_saw(t, dt, inv_dt));
        }
        OSC_LOOP(0.5f * (osc_triangle(t) + osc_saw(t, dt, inv_dt)));
    case WAVE_SAW:
        OSC_LOOP(osc_saw(t, dt, inv_dt));
    case WAVE_SQUARE:
        OSC_LOOP(osc_pulse(t, dt, inv_dt, PW_SQUARE));
    case WAVE_WIDE_PULSE:
        OSC_LOOP(osc_pulse(t, dt, inv_dt, PW_WIDE));
    case WAVE_NARROW_PULSE:
        OSC_LOOP(osc_pulse(t, dt, inv_dt, PW_NARROW));
    default:
        OSC_LOOP(0.0f);
    }
#undef OSC_LOOP

    *phase = t;
}

static inline void phase_skip(float *phase, float dt) {
    float t = *phase + dt * SYNTH_CONTROL_RATE;
    *phase = t - (float) (int) t;
}

static void voice_render(voice_t *v, const float *noise, float *restrict mix_l, float *restrict mix_r) {
    const synth_patch_t *p = &patches[v->channel];
    const channel_state_t *cs = &channels[v->channel];

    // Keep everything used per sample in locals (registers): the output
    // pointers could otherwise alias the voice and patch, forcing reloads
    const float k = p->feedback;
    const float gain_noise = p->mix_gain[SYNTH_OSCILLATORS];
    const float gain_l = p->gain_l * v->velocity_gain;
    const float gain_r = p->gain_r * v->velocity_gain;

    float m[SYNTH_CONTROL_RATE];

    for (int c = 0; c < CONTROL_STEPS; c++) {
        const int base = c * SYNTH_CONTROL_RATE;

        voice_control(v, p, cs);

        // Noise, oscillators and mixer
        if (gain_noise != 0.0f) {
            for (int j = 0; j < SYNTH_CONTROL_RATE; j++)
                m[j] = gain_noise * noise[base + j];
        } else {
            for (int j = 0; j < SYNTH_CONTROL_RATE; j++)
                m[j] = 0.0f;
        }

        for (int o = 0; o < SYNTH_OSCILLATORS; o++) {
            if (p->mix_gain[o] != 0.0f)
                osc_render(m, p->mix_gain[o], p->osc_wave[o], o, &v->phase[o], v->dt[o], v->inv_dt[o]);
            else
                phase_skip(&v->phase[o], v->dt[o]); // Keep free running
        }

        // 24 dB/octave ladder filter: four zero delay feedback one-poles with
        // global feedback, saturating input stage (Zavalishin TPT structure)
        float s0 = v->s[0], s1 = v->s[1], s2 = v->s[2], s3 = v->s[3];
        float G = v->G;
        const float dG = v->dG;
        const float inv_den = v->inv_den;
        float loud = v->loud_out;
        const float d_loud = v->d_loud;

        for (int j = 0; j < SYNTH_CONTROL_RATE; j++) {
            G += dG;
            const float H = 1.0f - G;
            const float S = G * (G * (G * H * s0 + H * s1) + H * s2) + H * s3;
            float x = soft_clip((m[j] - k * S) * inv_den);
            float vv, y;

            vv = (x - s0) * G; y = vv + s0; s0 = y + vv; x = y;
            vv = (x - s1) * G; y = vv + s1; s1 = y + vv; x = y;
            vv = (x - s2) * G; y = vv + s2; s2 = y + vv; x = y;
            vv = (x - s3) * G; y = vv + s3; s3 = y + vv; x = y;

            // Loudness contour and VCA
            loud += d_loud;
            const float out = x * loud;

            mix_l[base + j] += out * gain_l;
            mix_r[base + j] += out * gain_r;
        }

        v->s[0] = s0; v->s[1] = s1; v->s[2] = s2; v->s[3] = s3;
        v->G = G;
        v->loud_out = loud;
    }

    if (v->loud.stage == ENV_IDLE)
        v->active = 0;
}

void synth_process(int16_t *out_buffer) {
    float mix_l[SYNTH_BLOCK_SIZE] = { 0.0f };
    float mix_r[SYNTH_BLOCK_SIZE] = { 0.0f };
    float white[SYNTH_BLOCK_SIZE];
    float pink[SYNTH_BLOCK_SIZE];
    uint8_t need_white = 0, need_pink = 0;

    for (int v = 0; v < MAX_VOICES; v++) {
        const synth_patch_t *p = &patches[voices[v].channel];
        if (voices[v].active && p->mix_gain[SYNTH_OSCILLATORS] != 0.0f) {
            if (p->noise_pink)
                need_pink = 1;
            else
                need_white = 1;
        }
    }

    // Shared noise source (white and Paul Kellet's economy pink filter), only when used
    if (need_white || need_pink) {
        for (int i = 0; i < SYNTH_BLOCK_SIZE; i++) {
            float w = (float) (int32_t) xorshift32() * (1.0f / 2147483648.0f);
            white[i] = w;
            if (need_pink) {
                pink_b0 = 0.99765f * pink_b0 + w * 0.0990460f;
                pink_b1 = 0.96300f * pink_b1 + w * 0.2965164f;
                pink_b2 = 0.57000f * pink_b2 + w * 1.0526913f;
                pink[i] = (pink_b0 + pink_b1 + pink_b2 + w * 0.1848f) * 0.25f;
            }
        }
    }

    for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active)
            voice_render(&voices[v], patches[voices[v].channel].noise_pink ? pink : white, mix_l, mix_r);
    }

    const float gain = master_volume * OUTPUT_GAIN;

    for (int i = 0; i < SYNTH_BLOCK_SIZE; i++) {
        out_buffer[i * 2] = (int16_t) (soft_clip(mix_l[i] * gain) * PCM16_MAX);
        out_buffer[i * 2 + 1] = (int16_t) (soft_clip(mix_r[i] * gain) * PCM16_MAX);
    }
}

/*----------------------------------------------------------------------------
 | Notes and performance controls
 ----------------------------------------------------------------------------*/

static void voice_release(voice_t *v) {
    v->held = 0;
    v->loud.stage = ENV_RELEASE;
    v->filt.stage = ENV_RELEASE;
}

static int voice_allocate(uint8_t channel, uint8_t note) {
    int oldest = -1;
    int oldest_released = -1;

    for (int i = 0; i < MAX_VOICES; i++) {
        voice_t *v = &voices[i];
        // Retrigger the same note on the same channel
        if (v->active && v->channel == channel && v->note == note)
            return i;
    }

    for (int i = 0; i < MAX_VOICES; i++) {
        voice_t *v = &voices[i];
        if (!v->active)
            return i;
        if (v->loud.stage == ENV_RELEASE && (oldest_released < 0 || v->age < voices[oldest_released].age))
            oldest_released = i;
        if (oldest < 0 || v->age < voices[oldest].age)
            oldest = i;
    }

    // Steal the oldest released voice, otherwise the oldest playing voice
    return oldest_released >= 0 ? oldest_released : oldest;
}

void synth_note_on(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (channel >= SYNTH_CHANNELS || note > 127)
        return;

    if (velocity == 0) {
        synth_note_off(channel, note);
        return;
    }

    const synth_patch_t *p = &patches[channel];
    channel_state_t *cs = &channels[channel];
    voice_t *v = &voices[voice_allocate(channel, note)];

    if (!v->active || v->channel != channel) {
        // Fresh voice: start silent with a clean filter, glide from the channel's last note
        v->loud.level = 0.0f;
        v->loud_out = 0.0f;
        v->filt.level = 0.0f;
        memset(v->s, 0, sizeof(v->s));
        v->s[0] = FILTER_SEED;
        v->G = 0.0f;
        v->pitch = (p->glide_on && cs->last_note >= 0.0f) ? cs->last_note : (float) note;
    }
    // Retriggered voices continue from their current level (analog style)

    v->channel = channel;
    v->note = note;
    v->held = 0;
    v->age = ++voice_counter;
    v->velocity_gain = (float) velocity / 127.0f;
    v->loud.stage = ENV_ATTACK;
    v->filt.stage = ENV_ATTACK;
    v->active = 1;

    cs->last_note = note;
}

void synth_note_off(uint8_t channel, uint8_t note) {
    for (int i = 0; i < MAX_VOICES; i++) {
        voice_t *v = &voices[i];
        if (v->active && v->channel == channel && v->note == note && v->loud.stage != ENV_RELEASE) {
            if (channels[channel].sustain)
                v->held = 1;
            else
                voice_release(v);
        }
    }
}

void synth_all_notes_off(uint8_t channel) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active && voices[i].channel == channel)
            voice_release(&voices[i]);
    }
}

void synth_all_sound_off(uint8_t channel) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].channel == channel)
            voices[i].active = 0;
    }
}

void synth_pitch_bend(uint8_t channel, int16_t bend) {
    if (channel < SYNTH_CHANNELS)
        channels[channel].bend_semi = (float) bend * (PITCH_BEND_SEMI / 8192.0f);
}

void synth_mod_wheel(uint8_t channel, float amount) {
    if (channel < SYNTH_CHANNELS)
        channels[channel].mod = clampf(amount, 0.0f, 1.0f);
}

void synth_sustain(uint8_t channel, uint8_t on) {
    if (channel >= SYNTH_CHANNELS)
        return;
    channels[channel].sustain = on;
    if (!on) {
        for (int i = 0; i < MAX_VOICES; i++) {
            voice_t *v = &voices[i];
            if (v->active && v->channel == channel && v->held)
                voice_release(v);
        }
    }
}

int synth_active_voices(void) {
    int n = 0;
    for (int i = 0; i < MAX_VOICES; i++)
        n += voices[i].active;
    return n;
}

/*----------------------------------------------------------------------------
 | Benchmark: cycles per sample for each stage of one voice
 ----------------------------------------------------------------------------*/

#include <stdio.h>

void synth_benchmark(uint32_t (*cycles)(void)) {
    static float buf[SYNTH_CONTROL_RATE];
    static float noise[SYNTH_BLOCK_SIZE];
    static float mix_l[SYNTH_BLOCK_SIZE], mix_r[SYNTH_BLOCK_SIZE];
    static int16_t out[SYNTH_BLOCK_SIZE * 2];
    const int loops = 100;
    float phase = 0.0f;
    uint32_t t;

    synth_note_on(0, 60, 100);
    voice_t *v = &voices[voice_allocate(0, 60)];
    const synth_patch_t *p = &patches[0];
    (void) noise;

    static const char *wave_names[WAVE_COUNT] = { "triangle", "tri-saw", "saw", "square", "wide", "narrow" };
    for (int w = 0; w < WAVE_COUNT; w++) {
        t = cycles();
        for (int i = 0; i < loops; i++)
            osc_render(buf, 1.0f, w, 0, &phase, 0.01f, 100.0f);
        t = cycles() - t;
        printf("bench osc %-9s %4lu.%lu cycles/sample\n", wave_names[w], t / (loops * SYNTH_CONTROL_RATE),
                (t * 10 / (loops * SYNTH_CONTROL_RATE)) % 10);
    }

    t = cycles();
    for (int i = 0; i < loops; i++)
        voice_control(v, p, &channels[0]);
    t = cycles() - t;
    printf("bench voice_control    %4lu cycles/step (%lu/sample)\n", t / loops, t / (loops * SYNTH_CONTROL_RATE));

    t = cycles();
    for (int i = 0; i < loops; i++) {
        memset(mix_l, 0, sizeof(mix_l));
        memset(mix_r, 0, sizeof(mix_r));
        voice_render(v, noise, mix_l, mix_r);
        v->active = 1;
    }
    t = cycles() - t;
    printf("bench voice_render     %4lu cycles/sample (whole voice)\n", t / (loops * SYNTH_BLOCK_SIZE));

    synth_all_sound_off(0);
    t = cycles();
    for (int i = 0; i < loops; i++)
        synth_process(out);
    t = cycles() - t;
    printf("bench synth_process    %4lu cycles/sample (no voices)\n", t / (loops * SYNTH_BLOCK_SIZE));
}
