# Demo MIDI files

Standard MIDI Files for the STM32World MiniMoog. Each file starts by sending
the complete panel settings (as CCs) for every channel it uses, so it sets up
its own sound no matter what the synth was doing before.

| File | Channels | What it plays |
|------|----------|---------------|
| `bass.mid` | 1 | 16th note bass riff, the filter opens up in the last bar |
| `pad.mid` | 1 | Five note chords with high emphasis and a filter sweep |
| `lead.mid` | 1 | Lead with glide, vibrato from oscillator 3 on the mod wheel |
| `multi.mid` | 1, 2, 3 | Bass, pad (left) and resonant pluck arpeggio (right) at once |

## Playing them

**Linux** (aplaymidi is in the alsa-utils package):

    aplaymidi -l                                         # find the synth's port
    aplaymidi -p "STM32World MiniMoog" midi/bass.mid

**Windows, macOS or Linux with a DAW**: import the file into a MIDI track
(Reaper, Cakewalk, Ableton Live, Logic, ...) and set the track's MIDI output to
"STM32World MiniMoog". Make sure the CCs at the start of the file are sent too,
some DAWs strip or chase controllers differently.

Recordings of the same demos are in [../samples](../samples).

## Regenerating

The files are made from the sequences in `tools/demos.py`:

    tools/make_midis.py

## Converting LMMS projects

`tools/lmms2mid.py` turns an LMMS project (`.mmpz` or `.mmp`) into a MIDI
file for the synth:

    tools/lmms2mid.py ~/lmms/projects/Popcorn2.mmpz      # writes midi/popcorn2.mid

- Every instrument track gets its own MIDI channel (1-9, 15, 16) and a
  MiniMoog patch chosen from the track's General MIDI program (bass, organ,
  pad, brass, reed, lead, pluck). Track volume and pan go into the patch.
- A drum track (sound font bank 128) is split by drum into kick, snare, hats,
  cymbals and toms on channels 10-14, each with a drum patch made from the
  noise source and the self-oscillating filter.
- Volume automation becomes CC 7. The patches go out one channel at a time in
  the first second, then the song starts.
- Tracks that need a specific sound can be given one in `SONGS` in the
  script, by project name and track name.

The synth has 8 voices, so dense songs steal voices. Popcorn2 peaks at 8
voices and 60% CPU.
