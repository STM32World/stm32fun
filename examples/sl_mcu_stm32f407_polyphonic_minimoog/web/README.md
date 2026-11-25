# STM32World MiniMoog web panel

A Minimoog style panel and keyboard for the synth, talking to it with Web MIDI.

Web MIDI needs `https://` or `http://localhost`, so serve this folder instead
of opening `index.html` from disk:

    python3 -m http.server 8000

then open http://localhost:8000 in Chrome, Edge, Opera or Firefox and allow
MIDI access. On Windows use `py -m http.server 8000`.

See the project [README](../README.md#running-the-web-panel) for details and
troubleshooting.
