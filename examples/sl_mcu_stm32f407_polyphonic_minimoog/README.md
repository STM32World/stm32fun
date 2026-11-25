# STM32 Polyphonic MIDI MiniMoog

Polyphonic, multitimbral USB MIDI synthesizer on the STM32F407 replicating
the Minimoog Model D audio chain, with audio out through the PCM5102 audio
DAC Streamline blade.

Wiki: https://stm32world.com/wiki/STM32_Polyphonic_MIDI_MiniMoog

- `synth/` - the synth engine, MIDI CC map and MIDI handling
- `samples/` - short demo recordings from the line out
- `tools/demos.py` - plays the demo sequences (patches as CCs) into the synth over raw MIDI
- `tools/record_demos.sh` - resets the board, plays each demo while recording
  the line input and writes `samples/<demo>.mp3`

Recording the demos (Linux, needs python3, parecord, sox and lame):

    tools/record_demos.sh            # all demos
    tools/record_demos.sh lead       # just one
