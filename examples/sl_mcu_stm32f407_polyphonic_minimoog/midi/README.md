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
