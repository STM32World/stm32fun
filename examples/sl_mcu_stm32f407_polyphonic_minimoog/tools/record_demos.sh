#!/bin/bash
# Record the demo sequences from the synth's line out (Linux, PipeWire/PulseAudio).
#
# Usage: tools/record_demos.sh [demo ...]     (default: bass pad lead multi)
#
# For every demo: reset the board over SWD (back to the default patches), play
# the sequence with demos.py while recording the line input, check for clipping,
# trim silence, normalise to -1 dBFS and write samples/<demo>.mp3.
#
# Environment:
#   SOURCE      PulseAudio/PipeWire source to record (default: first analog input)
#   PROGRAMMER  STM32_Programmer_CLI used for the reset (set to "" to skip the reset)
#   KEEP_WAV=1  also keep samples/<demo>.wav

set -e
TOOLS=$(cd "$(dirname "$0")" && pwd)
OUT=$TOOLS/../samples
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$OUT"

SOURCE=${SOURCE:-$(pactl list short sources | awk '$2 ~ /^alsa_input.*analog/ {print $2; exit}')}
PROGRAMMER=${PROGRAMMER-$HOME/st/stm32cubeclt_1.18.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI}
DEMOS=${*:-bass pad lead multi}

for demo in $DEMOS; do
    if [ -n "$PROGRAMMER" ]; then
        "$PROGRAMMER" -c port=SWD mode=UR -rst >/dev/null 2>&1
        sleep 3.5   # USB re-enumeration
    fi

    parecord -d "$SOURCE" --rate=48000 --channels=2 --format=s16le --file-format=wav "$TMP/$demo.wav" &
    rec=$!
    sleep 1.5
    python3 "$TOOLS/demos.py" "$demo"
    sleep 3     # Release tails
    kill -INT $rec
    wait $rec 2>/dev/null || true

    peak=$(sox "$TMP/$demo.wav" -n stat 2>&1 | awk '/Maximum amplitude/ {print $3}')
    if awk "BEGIN {exit !($peak > 0.98)}"; then
        echo "$demo: clipped at the line input (peak $peak), lower LEVEL in demos.py" >&2
        exit 1
    fi

    # Trim leading/trailing silence, short fades, normalise to -1 dBFS
    sox "$TMP/$demo.wav" "$TMP/$demo.cut.wav" silence 1 0.01 -50d reverse silence 1 0.01 -50d reverse
    sox "$TMP/$demo.cut.wav" "$TMP/$demo.trim.wav" fade 0.005 -0 0.3 norm -1
    lame --quiet -V2 --tt "STM32World MiniMoog - $demo" --ta "STM32World" "$TMP/$demo.trim.wav" "$OUT/$demo.mp3"
    [ -n "$KEEP_WAV" ] && cp "$TMP/$demo.trim.wav" "$OUT/$demo.wav"
    echo "$demo: peak $peak, $(soxi -d "$TMP/$demo.trim.wav") -> samples/$demo.mp3"
done
