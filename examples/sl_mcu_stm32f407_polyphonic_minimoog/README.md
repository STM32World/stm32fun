# STM32 Polyphonic MIDI MiniMoog

Polyphonic, multitimbral USB MIDI synthesizer on the STM32F407 replicating
the Minimoog Model D audio chain, with audio out through the PCM5102 audio
DAC Streamline blade.

Wiki: https://stm32world.com/wiki/STM32_Polyphonic_MIDI_MiniMoog

- `synth/` - the synth engine, MIDI CC map and MIDI handling
- `midi/` - demo MIDI files that set up their own patches, see [midi/README.md](midi/README.md)
- `samples/` - short demo recordings from the line out
- `web/` - HTML5 panel and keyboard that talks to the synth over Web MIDI
- `tools/demos.py` - plays the demo sequences (patches as CCs) into the synth over raw MIDI
- `tools/record_demos.sh` - resets the board, plays each demo while recording
  the line input and writes `samples/<demo>.mp3`
- `tools/make_midis.py` - writes the demos as MIDI files to `midi/`
- `tools/lmms2mid.py` - converts an LMMS project into a MIDI file with a
  MiniMoog patch per track, see [midi/README.md](midi/README.md#converting-lmms-projects)

## Demo MIDI files

The `midi` folder has four demos as Standard MIDI Files: `bass.mid`,
`pad.mid`, `lead.mid` and `multi.mid` (three channels at once). Each file
first sends the complete patch for every channel it uses, so it sounds the
same whatever the synth was set to.

On Linux play them with aplaymidi (alsa-utils):

    aplaymidi -l                                         # find the synth's port
    aplaymidi -p "STM32World MiniMoog" midi/bass.mid

Elsewhere import them into a DAW and set the MIDI output to "STM32World
MiniMoog". See [midi/README.md](midi/README.md) for details.

## Recording the demos

Recording the demos (Linux, needs python3, parecord, sox and lame):

    tools/record_demos.sh            # all demos
    tools/record_demos.sh lead       # just one

## Web panel

![Web panel](web/screenshot.png)

`web/index.html` is a Minimoog style panel with a keyboard. It talks to the
synth directly with Web MIDI, so the only connection needed is the synth's USB
MIDI port.

### Running the web panel

Web MIDI only works on a secure origin (`https://` or `http://localhost`), so
opening `index.html` straight from disk (`file://`) does not work. Serve the
`web` folder with any local web server, for example the one built into Python:

1. Connect the synth over USB.
2. Start the server in the `web` folder:

       cd examples/sl_mcu_stm32f407_polyphonic_minimoog/web
       python3 -m http.server 8000

   On Windows use `py -m http.server 8000`. Any other static server works too,
   for example `npx http-server -p 8000`.
3. Open http://localhost:8000 in Chrome, Edge or Opera. Firefox works as well
   but first asks you to install a small site permission add-on for MIDI.
4. Allow MIDI access when the browser asks. The **Synth** dropdown selects
   "STM32World MiniMoog" automatically and the dot next to it turns green.
5. Stop the server with Ctrl+C when done.

If the Synth dropdown stays on "No MIDI output":

- Check that MIDI access is allowed for the page (site settings icon in the
  address bar) and reload.
- Make sure the synth is connected and shows up as a MIDI device.
- On Windows, close other programs (DAWs) that have the synth's MIDI port
  open, Windows only lets one program use a MIDI port at a time.

### Using the web panel

- Each of the 16 channel buttons shows that channel's patch; the page keeps all
  16 in the browser's local storage and sends the patch when you switch channel
  or press Send.
- Knobs: drag up/down (shift for fine), mouse wheel or arrow keys, double-click
  for the default.
- Keyboard: mouse/touch or the computer keyboard (Z-M and Q-I rows), octave
  buttons, sustain, pitch bend and mod wheel.
- "Keyboard in" forwards a hardware MIDI keyboard to the selected channel.
- Save/Load store a channel's patch as JSON.
