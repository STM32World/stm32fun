#!/usr/bin/env python3
"""Play demo sequences into the STM32World MiniMoog over raw MIDI (Linux).

Usage: demos.py <bass|pad|lead|multi> [length]

The synth's raw MIDI device is found automatically, or set MINIMOOG_MIDI
(for example /dev/snd/midiC4D0).  Patches are sent as CCs in real units
(seconds, Hz, levels) using the CC map in synth/params.c.
"""
import glob, math, os, sys, time

# Overall level of the demos, leaves headroom for the line input recording them
LEVEL = 0.35


def find_device():
    if 'MINIMOOG_MIDI' in os.environ:
        return os.environ['MINIMOOG_MIDI']
    for card in glob.glob('/proc/asound/card*'):
        devs = glob.glob('/dev/snd/midiC%sD*' % card[len('/proc/asound/card'):])
        if devs and os.path.exists(card + '/usbid') and open(card + '/usbid').read().strip() == 'cafe:4000':
            return devs[0]
    sys.exit('MiniMoog MIDI device not found, set MINIMOOG_MIDI')

# CC numbers (synth/params.c)
CC = dict(glide_time=5, volume=7, pan=10, cutoff_fine=13, osc1_range=14, osc1_wave=15, osc2_range=16, osc2_wave=17,
          osc2_detune=18, osc3_range=19, osc3_wave=20, osc3_detune=21, osc3_kbd=22, osc1_level=23, osc2_level=24,
          osc3_level=25, noise_level=26, osc1_on=27, osc2_on=28, osc3_on=29, noise_on=30, noise_pink=31,
          glide=65, emphasis=71, l_attack=73, cutoff=74, l_decay=75, contour=77, kbd=78, l_sustain=79, decay_sw=80,
          drive=81, mod_mix=82, osc_mod=83, filter_mod=84, f_attack=85, f_decay=86, f_sustain=87)
RANGE = {'LO': 0, "32'": 1, "16'": 2, "8'": 3, "4'": 4, "2'": 5}
WAVE = {'tri': 0, 'trisaw': 1, 'saw': 2, 'square': 3, 'wide': 4, 'narrow': 5}

def enum(i, steps): return min(127, round((i + 0.5) * 128 / steps))
def lin(x, lo=0.0, hi=1.0): return max(0, min(127, round((x - lo) / (hi - lo) * 127)))
def expo(x, lo, hi): return max(0, min(127, round(math.log(x / lo) / math.log(hi / lo) * 127)))
def exp14(x, lo, hi): return max(0, min(16383, round(math.log(x / lo) / math.log(hi / lo) * 16383)))
def bip14(x, lo, hi): return max(0, min(16383, round(8192 + (2 * (x - lo) / (hi - lo) - 1) * 8191)))

# The firmware's default patch (synth.c), in the units patch() takes
DEFAULTS = dict(osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='saw', osc2_detune=0.07,
                osc3_range="16'", osc3_wave='square', osc3_detune=0.0, osc3_kbd=1,
                osc1_level=0.8, osc2_level=0.8, osc3_level=0.5, noise_level=0.5,
                osc1_on=1, osc2_on=1, osc3_on=0, noise_on=0, noise_pink=0, drive=0.0,
                cutoff=800, emphasis=0.3, contour=0.5, kbd=1,
                f_attack=0.005, f_decay=0.4, f_sustain=0.3, l_attack=0.005, l_decay=0.3, l_sustain=0.8, decay_sw=1,
                glide_time=0.1, glide=0, mod_mix=0.0, osc_mod=0, filter_mod=0, volume=0.8, pan=0.0)


def patch(ch, **p):
    """Complete panel in real units (defaults + p) -> list of (status, cc, value) events."""
    p = {**DEFAULTS, **p}
    ev = []
    def cc(name, v): ev.append((0xB0 | ch, CC[name], v))
    for k, v in p.items():
        if k.endswith('_range'): cc(k, enum(RANGE[v], 6))
        elif k.endswith('_wave'): cc(k, enum(WAVE[v], 6))
        elif k.endswith('_detune'):
            r = bip14(v, -7, 7); cc(k, r >> 7); ev.append((0xB0 | ch, CC[k] + 32, r & 0x7F))
        elif k in ('osc3_kbd', 'osc1_on', 'osc2_on', 'osc3_on', 'noise_on', 'noise_pink', 'glide', 'decay_sw', 'osc_mod', 'filter_mod'):
            cc(k, 127 if v else 0)
        elif k == 'kbd': cc(k, enum(v, 4))
        elif k == 'cutoff':
            r = exp14(v, 20, 20000); cc('cutoff_fine', r >> 7); ev.append((0xB0 | ch, CC['cutoff_fine'] + 32, r & 0x7F))
        elif k in ('l_attack', 'f_attack'): cc(k, expo(v, 0.001, 10))
        elif k in ('l_decay', 'f_decay'): cc(k, expo(v, 0.004, 20))
        elif k == 'glide_time': cc(k, expo(v, 0.001, 5))
        elif k == 'pan': cc(k, max(0, min(127, round(64 + v * 63))))
        elif k == 'volume': cc(k, lin(v * LEVEL))
        else: cc(k, lin(v))
    return ev

def cutoff_ev(ch, hz):
    r = exp14(hz, 20, 20000)
    return [(0xB0 | ch, 13, r >> 7), (0xB0 | ch, 45, r & 0x7F)]

class Seq:
    def __init__(self): self.ev = []
    def at(self, t, msgs): self.ev += [(t, m) for m in msgs]
    def note(self, t, ch, n, dur, vel=100):
        self.ev.append((t, (0x90 | ch, n, vel))); self.ev.append((t + dur, (0x80 | ch, n, 0)))
    def sweep(self, t0, t1, ch, f0, f1, step=0.02):
        n = max(1, int((t1 - t0) / step))
        for i in range(n + 1):
            self.at(t0 + (t1 - t0) * i / n, cutoff_ev(ch, f0 * (f1 / f0) ** (i / n)))
    def play(self):
        fd = os.open(find_device(), os.O_WRONLY)
        start = time.perf_counter()
        for t, m in sorted(self.ev, key=lambda e: e[0]):
            d = start + t - time.perf_counter()
            if d > 0: time.sleep(d)
            os.write(fd, bytes(m))
        os.close(fd)

def bass():
    s = Seq()
    s.at(0, patch(0, osc1_range="16'", osc1_wave='saw', osc2_range="16'", osc2_wave='saw', osc2_detune=0.08,
                  osc3_range="32'", osc3_wave='square', osc3_level=0.45, osc3_on=1, osc1_level=0.8, osc2_level=0.8,
                  cutoff=260, emphasis=0.55, contour=0.55, kbd=1, f_attack=0.001, f_decay=0.22, f_sustain=0.08,
                  l_attack=0.002, l_decay=0.4, l_sustain=0.8, decay_sw=0, drive=0.35, volume=0.75))
    step = 60 / 116 / 4
    riff = [28, 40, 28, 38, 28, 43, 28, 40, 28, 40, 28, 38, 31, 43, 33, 45]
    for bar in range(3):
        for i, n in enumerate(riff):
            s.note(0.3 + (bar * 16 + i) * step, 0, n, step * 0.7)
    s.sweep(0.3 + 32 * step, 0.3 + 48 * step, 0, 260, 1600)
    return s, 0.3 + 48 * step + 0.6

def pad():
    s = Seq()
    s.at(0, patch(0, osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='saw', osc2_detune=0.12,
                  osc3_range="4'", osc3_wave='saw', osc3_detune=-0.07, osc3_level=0.35, osc3_on=1,
                  osc1_level=0.6, osc2_level=0.6, cutoff=180, emphasis=0.72, contour=0.15, kbd=1,
                  f_attack=0.5, f_decay=2.0, f_sustain=0.8, l_attack=0.5, l_decay=2.0, l_sustain=0.9, decay_sw=1,
                  volume=0.55))
    for n in (45, 57, 60, 64, 67):
        s.note(0.3, 0, n, 4.0, 90)
    s.sweep(0.4, 3.0, 0, 180, 5000)
    s.sweep(3.0, 4.3, 0, 5000, 600)
    for n in (41, 53, 57, 60, 64):
        s.note(4.6, 0, n, 3.2, 90)
    s.sweep(4.6, 7.6, 0, 600, 2500)
    return s, 4.6 + 3.2 + 2.2

def lead():
    s = Seq()
    s.at(0, patch(0, osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='saw', osc2_detune=0.1,
                  osc3_range='LO', osc3_wave='tri', osc3_kbd=0, osc3_on=0, osc1_level=0.8, osc2_level=0.7,
                  cutoff=900, emphasis=0.45, contour=0.45, kbd=2, f_attack=0.01, f_decay=0.7, f_sustain=0.5,
                  l_attack=0.01, l_decay=0.6, l_sustain=0.85, decay_sw=1, glide=1, glide_time=0.07,
                  mod_mix=0.0, osc_mod=1, volume=0.7))
    s.at(0, [(0xB0, 1, 0)])
    q = 60 / 100
    mel = [(69, .5), (72, .5), (74, .5), (76, 1.0), (79, .5), (76, .5), (74, .5), (72, .5), (74, .5), (69, 2.5)]
    t = 0.3
    for n, d in mel:
        s.note(t, 0, n, d * q + 0.03)   # small overlap = legato glide
        t += d * q
    # Vibrato comes in on the last note
    last = t - 2.5 * q
    for i in range(30):
        s.at(last + 0.4 + i * 0.03, [(0xB0, 1, min(127, i * 2))])
    return s, t + 1.0

def multi():
    s = Seq()
    s.at(0, patch(0, osc1_range="16'", osc1_wave='saw', osc2_range="16'", osc2_wave='square', osc2_detune=0.05,
                  osc2_level=0.6, cutoff=320, emphasis=0.4, contour=0.5, f_decay=0.25, f_sustain=0.15,
                  l_attack=0.002, l_decay=0.3, l_sustain=0.7, decay_sw=0, volume=0.7, pan=0.0))
    s.at(0, patch(1, osc1_range="8'", osc1_wave='trisaw', osc2_range="8'", osc2_wave='saw', osc2_detune=-0.15,
                  osc1_level=0.5, osc2_level=0.5, cutoff=1400, emphasis=0.2, contour=0.1, l_attack=0.4,
                  l_decay=1.5, l_sustain=0.9, decay_sw=1, volume=0.45, pan=-0.6))
    s.at(0, patch(2, osc1_range="4'", osc1_wave='narrow', osc2_on=0, cutoff=500, emphasis=0.8, contour=0.7,
                  kbd=3, f_attack=0.001, f_decay=0.18, f_sustain=0.0, l_attack=0.001, l_decay=0.35, l_sustain=0.0,
                  decay_sw=1, volume=0.5, pan=0.6))
    beat = 60 / 112
    chords = [(57, 60, 64), (53, 57, 60), (55, 59, 62), (52, 55, 59)]
    roots = [33, 29, 31, 28]
    arp = [0, 1, 2, 1]
    for bar in range(4):
        t0 = 0.3 + bar * 4 * beat
        for n in chords[bar]:
            s.note(t0, 1, n, 4 * beat - 0.05, 80)
        for b in range(8):
            s.note(t0 + b * beat / 2, 0, roots[bar] + (12 if b % 4 == 3 else 0), beat / 2 * 0.6)
        for b in range(16):
            s.note(t0 + b * beat / 4, 2, chords[bar][arp[b % 4]] + 12, beat / 4 * 0.5, 90)
    return s, 0.3 + 16 * beat + 1.5

DEMOS = dict(bass=bass, pad=pad, lead=lead, multi=multi)

if __name__ == '__main__':
    seq, length = DEMOS[sys.argv[1]]()
    if len(sys.argv) > 2 and sys.argv[2] == 'length':
        print('%.2f' % length)
    else:
        seq.play()
