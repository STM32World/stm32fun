#!/usr/bin/env python3
"""Write the demo sequences from demos.py as Standard MIDI Files.

Usage: tools/make_midis.py            (writes midi/<demo>.mid)

Each file starts with the CCs that set up the patch for every channel it uses,
followed by the notes, so playing a file into the synth sets up the sound
before playing.  No third party modules needed.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import demos  # noqa: E402

PPQ = 480
TEMPO_US = 500000                    # 120 bpm: one tick = 1/960 s
TICKS_PER_S = PPQ * 1000000 / TEMPO_US

# Louder than the recordings, which leave headroom for the line input
demos.LEVEL = 0.6


def varlen(n):
    out = [n & 0x7f]
    n >>= 7
    while n:
        out.insert(0, 0x80 | (n & 0x7f))
        n >>= 7
    return bytes(out)


def smf(seq, name):
    """Format 0 file from a demos.Seq."""
    track = bytearray()
    # Track name and tempo
    title = f'STM32World MiniMoog - {name}'.encode()
    track += varlen(0) + b'\xff\x03' + varlen(len(title)) + title
    track += varlen(0) + b'\xff\x51\x03' + TEMPO_US.to_bytes(3, 'big')

    now = 0
    # sorted() is stable, so events at the same time keep their order (patch CCs before notes)
    for t, msg in sorted(seq.ev, key=lambda e: e[0]):
        tick = round(t * TICKS_PER_S)
        track += varlen(tick - now) + bytes(msg)
        now = tick
    track += varlen(PPQ) + b'\xff\x2f\x00'   # End of track a beat later

    header = b'MThd' + struct.pack('>IHHH', 6, 0, 1, PPQ)
    return header + b'MTrk' + struct.pack('>I', len(track)) + bytes(track)


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'midi')
    os.makedirs(out, exist_ok=True)
    for name, make in demos.DEMOS.items():
        seq, length = make()
        path = os.path.join(out, f'{name}.mid')
        with open(path, 'wb') as f:
            f.write(smf(seq, name))
        print(f'{os.path.relpath(path)}: {len(seq.ev)} events, {length:.1f} s')


if __name__ == '__main__':
    main()
