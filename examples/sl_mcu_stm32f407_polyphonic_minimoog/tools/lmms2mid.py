#!/usr/bin/env python3
"""Convert an LMMS project (.mmpz or .mmp) into a MIDI file for the MiniMoog.

Usage: tools/lmms2mid.py <project.mmpz> [out.mid]

Every instrument track gets its own MIDI channel and a MiniMoog patch, sent as
CCs at the start of the file.  The patch is picked from the track's General
MIDI program (bass, organ, pad, lead...), or from SONGS below for tracks that
need something specific.  A drum track (sound font bank 128) is split over
several channels, one per kind of drum, because a channel only has one patch:
kick, snare, hats, cymbals and toms each get a drum patch of their own.

Volume automation on a track becomes CC 7.  Beat/Bassline tracks are not
converted.  Output goes to midi/<project>.mid by default.
"""
import os
import sys
import xml.etree.ElementTree as ET
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import demos           # noqa: E402
import make_midis      # noqa: E402  (sets demos.LEVEL for files)

LMMS_TICKS_PER_BEAT = 48             # 192 per 4/4 bar
LMMS_KEY_OFFSET = 12                 # LMMS key 57 is A4, MIDI note 69

# Patches in demos.patch() units, on top of the firmware defaults
PATCHES = {
    'bass': dict(osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='square', osc2_detune=0.05,
                 osc2_level=0.6, osc3_range="16'", osc3_wave='tri', osc3_level=0.4, osc3_on=1,
                 cutoff=320, emphasis=0.3, contour=0.5, kbd=1, f_attack=0.001, f_decay=0.25, f_sustain=0.2,
                 l_attack=0.002, l_decay=0.3, l_sustain=0.7, decay_sw=0, drive=0.2),
    'synthbass': dict(osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='saw', osc2_detune=0.1,
                      cutoff=450, emphasis=0.6, contour=0.6, kbd=1, f_attack=0.001, f_decay=0.2, f_sustain=0.1,
                      l_attack=0.002, l_decay=0.3, l_sustain=0.7, decay_sw=0, drive=0.3),
    'pluck': dict(osc1_range="8'", osc1_wave='narrow', osc2_on=0, cutoff=600, emphasis=0.7, contour=0.7, kbd=3,
                  f_attack=0.001, f_decay=0.18, f_sustain=0.0, l_attack=0.001, l_decay=0.4, l_sustain=0.0,
                  decay_sw=1),
    'organ': dict(osc1_range="8'", osc1_wave='square', osc2_range="4'", osc2_wave='square', osc2_detune=0.0,
                  osc2_level=0.5, osc3_range="16'", osc3_wave='tri', osc3_level=0.4, osc3_on=1,
                  cutoff=3000, emphasis=0.0, contour=0.0, kbd=2, l_attack=0.003, l_decay=0.08, l_sustain=1.0,
                  decay_sw=1),
    'pad': dict(osc1_range="8'", osc1_wave='trisaw', osc2_range="8'", osc2_wave='saw', osc2_detune=-0.12,
                osc1_level=0.6, osc2_level=0.5, cutoff=1000, emphasis=0.15, contour=0.2, kbd=2,
                f_attack=0.3, f_decay=1.0, f_sustain=0.7, l_attack=0.15, l_decay=0.8, l_sustain=0.9, decay_sw=1),
    'brass': dict(osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='saw', osc2_detune=0.08,
                  cutoff=500, emphasis=0.2, contour=0.6, kbd=2, f_attack=0.06, f_decay=0.4, f_sustain=0.5,
                  l_attack=0.02, l_decay=0.3, l_sustain=0.9, decay_sw=1),
    'reed': dict(osc1_range="8'", osc1_wave='wide', osc2_range="8'", osc2_wave='tri', osc2_level=0.6,
                 osc2_detune=0.04, cutoff=1500, emphasis=0.15, contour=0.2, kbd=2, f_decay=0.5, f_sustain=0.6,
                 l_attack=0.02, l_decay=0.3, l_sustain=0.8, decay_sw=1),
    'lead': dict(osc1_range="8'", osc1_wave='saw', osc2_range="8'", osc2_wave='square', osc2_detune=0.06,
                 osc2_level=0.6, cutoff=1800, emphasis=0.35, contour=0.3, kbd=2, f_decay=0.6, f_sustain=0.6,
                 l_attack=0.01, l_decay=0.5, l_sustain=0.9, decay_sw=1),
    # The original Popcorn was a Moog: short, bright and a bit hollow
    'popcorn': dict(osc1_range="8'", osc1_wave='square', osc2_range="8'", osc2_wave='saw', osc2_detune=0.05,
                    osc1_level=0.8, osc2_level=0.5, cutoff=1100, emphasis=0.35, contour=0.6, kbd=2,
                    f_attack=0.001, f_decay=0.15, f_sustain=0.1, l_attack=0.001, l_decay=0.25, l_sustain=0.3,
                    decay_sw=1),
}

# Drum kit: each kind of drum on its own channel, (patch, note or None to keep the note)
DRUMS = {
    'kick': (dict(osc1_range="8'", osc1_wave='tri', osc1_level=0.8, osc2_on=0, cutoff=60, emphasis=0.85,
                  contour=0.7, kbd=0, f_attack=0.001, f_decay=0.05, f_sustain=0.0, l_attack=0.001,
                  l_decay=0.3, l_sustain=0.0, decay_sw=1, drive=0.4), 33),
    'snare': (dict(osc1_range="8'", osc1_wave='tri', osc1_level=0.5, osc2_on=0, noise_on=1, noise_level=1.0,
                   cutoff=2500, emphasis=0.2, contour=0.6, kbd=0, f_attack=0.001, f_decay=0.08, f_sustain=0.0,
                   l_attack=0.001, l_decay=0.15, l_sustain=0.0, decay_sw=1), 54),
    'hats': (dict(osc1_on=0, osc2_on=0, noise_on=1, noise_level=1.0, cutoff=6000, emphasis=0.4, contour=0.3,
                  kbd=3, f_attack=0.001, f_decay=0.05, f_sustain=0.0, l_attack=0.001, l_decay=0.06,
                  l_sustain=0.0, decay_sw=1), None),
    'cymbal': (dict(osc1_on=0, osc2_on=0, noise_on=1, noise_level=1.0, cutoff=8000, emphasis=0.5, contour=0.2,
                    kbd=0, f_attack=0.001, f_decay=0.5, f_sustain=0.0, l_attack=0.001, l_decay=0.7,
                    l_sustain=0.0, decay_sw=1), None),
    'toms': (dict(osc1_range="8'", osc1_wave='tri', osc1_level=0.8, osc2_on=0, cutoff=150, emphasis=0.8,
                  contour=0.6, kbd=3, f_attack=0.001, f_decay=0.08, f_sustain=0.0, l_attack=0.001,
                  l_decay=0.35, l_sustain=0.0, decay_sw=1, drive=0.3), None),
}
DRUM_KIND = {35: 'kick', 36: 'kick', 37: 'snare', 38: 'snare', 39: 'snare', 40: 'snare',
             41: 'toms', 43: 'toms', 45: 'toms', 47: 'toms', 48: 'toms', 50: 'toms',
             49: 'cymbal', 51: 'cymbal', 52: 'cymbal', 53: 'cymbal', 55: 'cymbal', 57: 'cymbal', 59: 'cymbal'}
DRUM_LEVEL = dict(kick=0.9, snare=0.6, hats=0.35, cymbal=0.3, toms=0.7)

# Many channels add up: whole song level on top of each track's volume
SONG_LEVEL = 0.45
# Notes start this late, the patches are sent one channel at a time before that
LEAD_IN = 1.0
PATCH_SPACING = 0.05

# Per project overrides: track name -> patch name, or (patch name, extra settings).
# '_level' overrides SONG_LEVEL.
SONGS = {
    'popcorn2': {
        '_level': 0.4,
        'Popcorn': 'popcorn',
        'Bass 1': 'bass',
        'Synth Bass': 'synthbass',
        'Melody 2': ('pad', dict(l_attack=0.08)),
        'Arpeggio': 'pluck',
        'End Part': 'reed',
        'High Note': 'lead',
    },
}


def gm_patch(program):
    """Default patch name for a General MIDI program number."""
    family = program // 8
    return {0: 'pluck', 1: 'pluck', 2: 'organ', 3: 'pluck', 4: 'bass', 5: 'pad', 6: 'pad', 7: 'brass',
            8: 'reed', 9: 'reed', 10: 'lead', 11: 'pad'}.get(family, 'pluck')


def load(path):
    data = open(path, 'rb').read()
    if path.endswith('.mmpz'):
        data = zlib.decompress(data[4:])        # Qt qCompress: 4 byte length, then zlib
    return ET.fromstring(data)


def model_value(elem, name, default):
    """An LMMS model is either an attribute or a child element (when automated)."""
    if elem.get(name) is not None:
        return float(elem.get(name)), None
    child = elem.find(name)
    if child is not None:
        return float(child.get('value', default)), child.get('id')
    return default, None


def convert(path):
    root = load(path)
    song = root.find('song')
    stem = os.path.splitext(os.path.basename(path))[0].lower()
    overrides = SONGS.get(stem, {})
    head = next((h for h in root.iter('head') if h.get('bpm') or h.find('bpm') is not None), None)
    bpm, _ = model_value(head, 'bpm', 120) if head is not None else (120, None)
    tps = LMMS_TICKS_PER_BEAT * bpm / 60      # LMMS ticks per second

    # Automation: object id -> [(tick, value)]
    automation = {}
    for ap in song.iter('automationpattern'):
        pos = int(ap.get('pos', 0))
        for obj in ap.findall('object'):
            automation.setdefault(obj.get('id'), []).extend(
                (pos + int(t.get('pos')), float(t.get('value'))) for t in ap.findall('time'))

    seq = demos.Seq()
    song_level = overrides.get('_level', SONG_LEVEL)
    patch_time = [0.0]

    def send_patch(ch, p):
        seq.at(patch_time[0], demos.patch(ch, **p))
        patch_time[0] += PATCH_SPACING

    def sec(t): return LEAD_IN + t / tps
    # Drums on channels 10-14, tonal tracks on the rest
    drum_order = ['kick', 'snare', 'hats', 'cymbal', 'toms']
    free = [c for c in range(16) if not 9 <= c < 9 + len(drum_order)]
    drum_channels = {}
    report = []

    for track in song.find('trackcontainer').findall('track'):
        if track.get('type') != '0':
            continue
        it = track.find('instrumenttrack')
        notes = []
        for pat in track.findall('pattern'):
            pos = int(pat.get('pos'))
            for n in pat.findall('note'):
                length = int(n.get('len'))
                if length > 0:
                    notes.append((pos + int(n.get('pos')), length, int(n.get('key')) + LMMS_KEY_OFFSET,
                                  float(n.get('vol', 100))))
        if not notes:
            continue
        name = track.get('name')
        vol, vol_id = model_value(it, 'vol', 100)
        pan, _ = model_value(it, 'pan', 0)
        sf2 = it.find('.//sf2player')
        bank = int(sf2.get('bank', 0)) if sf2 is not None else 0
        program = int(sf2.get('patch', 0)) if sf2 is not None else 0

        def vel(v): return max(1, min(127, round(v * 64 / 100)))

        if bank == 128 or overrides.get(name) == 'drums':
            for t, length, key, v in notes:
                kind = DRUM_KIND.get(key, 'hats')
                if kind not in drum_channels:
                    ch = drum_channels[kind] = 9 + drum_order.index(kind)
                    p, _ = DRUMS[kind]
                    send_patch(ch, {**p, 'volume': song_level * DRUM_LEVEL[kind] * vol / 100})
                ch = drum_channels[kind]
                note = DRUMS[kind][1] or key
                seq.note(sec(t), ch, note, length / tps, vel(v))
            report.append(f'{name}: drums on channels ' +
                          ', '.join(f'{k} {c + 1}' for k, c in sorted(drum_channels.items(), key=lambda x: x[1])))
            continue

        if not free:
            print(f'{name}: out of MIDI channels, skipped', file=sys.stderr)
            continue
        ch = free.pop(0)
        spec = overrides.get(name, gm_patch(program))
        pname, extra = spec if isinstance(spec, tuple) else (spec, {})
        p = {**PATCHES[pname], **extra}
        level = p.pop('volume', 0.8)
        send_patch(ch, {**p, 'volume': song_level * level * vol / 100, 'pan': pan / 100})
        for t, length, key, v in notes:
            seq.note(sec(t), ch, key, length / tps, vel(v))
        # Volume automation -> CC 7
        for t, v in automation.get(vol_id, []) if vol_id else []:
            seq.at(sec(t), [(0xB0 | ch, 7, demos.lin(song_level * level * v / 100 * demos.LEVEL))])
        report.append(f'{name}: channel {ch + 1}, {pname} patch, {len(notes)} notes')

    end = max(t for t, _ in seq.ev)
    return seq, end, report, bpm


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    stem = os.path.splitext(os.path.basename(src))[0].lower()
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'midi', stem + '.mid')
    seq, length, report, bpm = convert(src)
    # Write the song's own tempo so bars line up in a DAW
    make_midis.TEMPO_US = round(60e6 / bpm)
    make_midis.TICKS_PER_S = make_midis.PPQ * 1e6 / make_midis.TEMPO_US
    with open(out, 'wb') as f:
        f.write(make_midis.smf(seq, stem))
    print('\n'.join(report))
    print(f'{os.path.relpath(out)}: {len(seq.ev)} events, {length:.1f} s')


if __name__ == '__main__':
    main()
