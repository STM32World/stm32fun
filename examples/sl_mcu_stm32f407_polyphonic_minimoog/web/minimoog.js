/*
 * STM32World MiniMoog - Web MIDI panel and keyboard
 *
 * Every control maps to a MIDI CC (see synth/params.c in the firmware). Each
 * of the 16 MIDI channels has its own patch in the synth, and this page keeps
 * a copy of all 16 (stored in localStorage) so switching channel shows that
 * channel's panel. Incoming CCs from the synth update the panel, ready for
 * when the firmware echoes parameter changes.
 */
'use strict';

/* --------------------------------------------------------------------------
 * Parameter table (must match synth/params.c)
 * ------------------------------------------------------------------------ */

const RANGES = ['LO', "32'", "16'", "8'", "4'", "2'"];
const WAVES = ['Tri', 'Tri-saw', 'Saw', 'Square', 'Wide', 'Narrow'];
const WAVES3 = ['Tri', 'Rev saw', 'Saw', 'Square', 'Wide', 'Narrow'];

// type: lin | exp | bip | enum | switch.  fine: 14-bit (MSB on cc, LSB on cc + 32)
const PARAMS = [
    // Controllers
    { id: 'glide_time', name: 'Glide', sec: 'controllers', cc: 5, type: 'exp', min: 0.001, max: 5, def: 0.1, unit: 's' },
    { id: 'mod_mix', name: 'Mod mix', sec: 'controllers', cc: 82, type: 'lin', min: 0, max: 1, def: 0, unit: 'mix' },
    { id: 'osc_mod', name: 'Osc mod', sec: 'controllers', cc: 83, type: 'switch', def: 0 },
    { id: 'filter_mod', name: 'Filter mod', sec: 'controllers', cc: 84, type: 'switch', def: 0 },

    // Oscillator bank (laid out three per row: range, detune/blank, waveform)
    { id: 'osc1_range', name: 'Osc 1 range', sec: 'oscillators', cc: 14, type: 'enum', options: RANGES, def: 3 },
    { id: '_blank1', sec: 'oscillators', blank: true },
    { id: 'osc1_wave', name: 'Osc 1 wave', sec: 'oscillators', cc: 15, type: 'enum', options: WAVES, def: 2 },
    { id: 'osc2_range', name: 'Osc 2 range', sec: 'oscillators', cc: 16, type: 'enum', options: RANGES, def: 3 },
    { id: 'osc2_detune', name: 'Osc 2 detune', sec: 'oscillators', cc: 18, type: 'bip', min: -7, max: 7, def: 0.07, fine: true, unit: 'st' },
    { id: 'osc2_wave', name: 'Osc 2 wave', sec: 'oscillators', cc: 17, type: 'enum', options: WAVES, def: 2 },
    { id: 'osc3_range', name: 'Osc 3 range', sec: 'oscillators', cc: 19, type: 'enum', options: RANGES, def: 2 },
    { id: 'osc3_detune', name: 'Osc 3 detune', sec: 'oscillators', cc: 21, type: 'bip', min: -7, max: 7, def: 0, fine: true, unit: 'st' },
    { id: 'osc3_wave', name: 'Osc 3 wave', sec: 'oscillators', cc: 20, type: 'enum', options: WAVES3, def: 3 },
    { id: 'osc3_kbd', name: 'Osc 3 kbd ctrl', sec: 'oscillators', cc: 22, type: 'switch', def: 1 },

    // Mixer (level knob + on switch per row)
    { id: 'osc1_level', name: 'Osc 1', sec: 'mixer', cc: 23, type: 'lin', min: 0, max: 1, def: 0.8, unit: 'level' },
    { id: 'osc1_on', name: 'On', sec: 'mixer', cc: 27, type: 'switch', def: 1 },
    { id: 'osc2_level', name: 'Osc 2', sec: 'mixer', cc: 24, type: 'lin', min: 0, max: 1, def: 0.8, unit: 'level' },
    { id: 'osc2_on', name: 'On', sec: 'mixer', cc: 28, type: 'switch', def: 1 },
    { id: 'osc3_level', name: 'Osc 3', sec: 'mixer', cc: 25, type: 'lin', min: 0, max: 1, def: 0.5, unit: 'level' },
    { id: 'osc3_on', name: 'On', sec: 'mixer', cc: 29, type: 'switch', def: 0 },
    { id: 'noise_level', name: 'Noise', sec: 'mixer', cc: 26, type: 'lin', min: 0, max: 1, def: 0.5, unit: 'level' },
    { id: 'noise_on', name: 'On', sec: 'mixer', cc: 30, type: 'switch', def: 0 },
    { id: 'drive', name: 'Drive', sec: 'mixer', cc: 81, type: 'lin', min: 0, max: 1, def: 0, unit: 'level' },
    { id: 'noise_pink', name: 'Pink', sec: 'mixer', cc: 31, type: 'switch', def: 0 },

    // Modifiers
    { label: 'Filter', sec: 'modifiers' },
    { id: 'cutoff', name: 'Cutoff', sec: 'modifiers', cc: 13, type: 'exp', min: 20, max: 20000, def: 800, fine: true, unit: 'Hz' },
    { id: 'emphasis', name: 'Emphasis', sec: 'modifiers', cc: 71, type: 'lin', min: 0, max: 1, def: 0.3, unit: 'level' },
    { id: 'contour_amount', name: 'Contour amt', sec: 'modifiers', cc: 77, type: 'lin', min: 0, max: 1, def: 0.5, unit: 'level' },
    { id: 'f_attack', name: 'Attack', sec: 'modifiers', cc: 85, type: 'exp', min: 0.001, max: 10, def: 0.005, unit: 's' },
    { id: 'f_decay', name: 'Decay', sec: 'modifiers', cc: 86, type: 'exp', min: 0.004, max: 20, def: 0.4, unit: 's' },
    { id: 'f_sustain', name: 'Sustain', sec: 'modifiers', cc: 87, type: 'lin', min: 0, max: 1, def: 0.3, unit: 'level' },
    { id: 'kbd_track', name: 'Kbd ctrl', sec: 'modifiers', cc: 78, type: 'enum', options: ['Off', '1/3', '2/3', 'Full'], def: 1 },
    { label: 'Loudness contour', sec: 'modifiers' },
    { id: 'l_attack', name: 'Attack', sec: 'modifiers', cc: 73, type: 'exp', min: 0.001, max: 10, def: 0.005, unit: 's' },
    { id: 'l_decay', name: 'Decay', sec: 'modifiers', cc: 75, type: 'exp', min: 0.004, max: 20, def: 0.3, unit: 's' },
    { id: 'l_sustain', name: 'Sustain', sec: 'modifiers', cc: 79, type: 'lin', min: 0, max: 1, def: 0.8, unit: 'level' },

    // Output
    { id: 'volume', name: 'Volume', sec: 'output', cc: 7, type: 'lin', min: 0, max: 1, def: 0.8, unit: 'level' },
    { id: 'pan', name: 'Pan', sec: 'output', cc: 10, type: 'bip', min: -1, max: 1, def: 0, unit: 'pan' },

    // Left of the keyboard, as on the Model D
    { id: 'glide', name: 'Glide', sec: 'perf', cc: 65, type: 'switch', def: 0 },
    { id: 'decay_sw', name: 'Decay', sec: 'perf', cc: 80, type: 'switch', def: 1 },
];

const PARAM_BY_ID = Object.fromEntries(PARAMS.filter(p => p.cc !== undefined).map(p => [p.id, p]));
const CONTROLS = PARAMS.filter(p => p.cc !== undefined);

// Knob position [0, 1] <-> value
function posToValue(p, pos) {
    pos = Math.min(1, Math.max(0, pos));
    switch (p.type) {
    case 'lin':
    case 'bip':
        return p.min + pos * (p.max - p.min);
    case 'exp':
        return p.min * Math.pow(p.max / p.min, pos);
    case 'enum':
        return Math.min(p.options.length - 1, Math.round(pos * (p.options.length - 1)));
    case 'switch':
        return pos >= 0.5 ? 1 : 0;
    }
}

function valueToPos(p, v) {
    switch (p.type) {
    case 'lin':
    case 'bip':
        return (v - p.min) / (p.max - p.min);
    case 'exp':
        return Math.log(v / p.min) / Math.log(p.max / p.min);
    case 'enum':
        return v / (p.options.length - 1);
    case 'switch':
        return v ? 1 : 0;
    }
}

// Value -> MIDI: returns { msb, lsb } (lsb only for fine parameters)
function valueToMidi(p, v) {
    if (p.type === 'switch')
        return { msb: v ? 127 : 0 };
    if (p.type === 'enum') // Centre of the value's slice, as params.c splits 0-127 evenly
        return { msb: Math.min(127, Math.round((v + 0.5) * 128 / p.options.length)) };

    if (p.type === 'bip') {
        const b = 2 * (v - p.min) / (p.max - p.min) - 1;
        if (p.fine) {
            const raw = Math.max(0, Math.min(16383, Math.round(8192 + b * 8191)));
            return { msb: raw >> 7, lsb: raw & 0x7f };
        }
        return { msb: Math.max(0, Math.min(127, Math.round(64 + b * 63))) };
    }

    const u = valueToPos(p, v);
    if (p.fine) {
        const raw = Math.max(0, Math.min(16383, Math.round(u * 16383)));
        return { msb: raw >> 7, lsb: raw & 0x7f };
    }
    return { msb: Math.max(0, Math.min(127, Math.round(u * 127))) };
}

// MIDI -> value, for CCs coming back from the synth
function midiToValue(p, msb, lsb) {
    const fine = p.fine && lsb !== undefined;
    switch (p.type) {
    case 'switch':
        return msb >= 64 ? 1 : 0;
    case 'enum':
        return Math.min(p.options.length - 1, Math.floor(msb * p.options.length / 128));
    case 'bip': {
        const b = fine ? ((msb << 7 | lsb) - 8192) / 8191 : (msb - 64) / 63;
        return p.min + (Math.max(-1, Math.min(1, b)) + 1) / 2 * (p.max - p.min);
    }
    default:
        return posToValue(p, fine ? (msb << 7 | lsb) / 16383 : msb / 127);
    }
}

function formatValue(p, v) {
    switch (p.unit) {
    case 's':
        return v < 1 ? `${Math.round(v * 1000)} ms` : `${v.toFixed(2)} s`;
    case 'Hz':
        return v < 1000 ? `${Math.round(v)} Hz` : `${(v / 1000).toFixed(2)} kHz`;
    case 'st':
        return `${v >= 0 ? '+' : ''}${v.toFixed(2)} st`;
    case 'level':
        return (v * 10).toFixed(1);
    case 'mix':
        return v < 0.02 ? 'Osc 3' : v > 0.98 ? 'Noise' : `${Math.round((1 - v) * 100)}/${Math.round(v * 100)}`;
    case 'pan':
        return Math.abs(v) < 0.02 ? 'C' : v < 0 ? `L${Math.round(-v * 100)}` : `R${Math.round(v * 100)}`;
    }
    if (p.type === 'enum')
        return p.options[v];
    return v ? 'On' : 'Off';
}

/* --------------------------------------------------------------------------
 * Patches (one per channel)
 * ------------------------------------------------------------------------ */

const STORAGE_KEY = 'stm32world-minimoog-patches';

function defaultPatch() {
    return Object.fromEntries(CONTROLS.map(p => [p.id, p.def]));
}

let patches = Array.from({ length: 16 }, defaultPatch);
let channel = 0;

try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (Array.isArray(saved) && saved.length === 16)
        patches = saved.map(s => Object.assign(defaultPatch(), s));
} catch (e) {
    // No storage, keep the defaults
}

let saveTimer = null;
function savePatches() {
    clearTimeout(saveTimer);
    saveTimer = setTimeout(() => {
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify(patches));
        } catch (e) {
            // Storage unavailable
        }
    }, 300);
}

/* --------------------------------------------------------------------------
 * MIDI
 * ------------------------------------------------------------------------ */

let midiAccess = null;
let output = null;
let keyboardInput = null;
let synthInput = null;
const lsbPending = {}; // Incoming 14-bit MSBs per channel/cc

function send(bytes) {
    if (output)
        output.send(bytes);
}

function sendParam(ch, p, v) {
    const m = valueToMidi(p, v);
    send([0xb0 | ch, p.cc, m.msb]);
    if (m.lsb !== undefined)
        send([0xb0 | ch, p.cc + 32, m.lsb]);
}

function sendPatch(ch) {
    for (const p of CONTROLS)
        sendParam(ch, p, patches[ch][p.id]);
}

function setStatus() {
    document.getElementById('midi-status').classList.toggle('ok', !!output);
    document.getElementById('midi-status').title = output ? `Connected to ${output.name}` : 'No MIDI output';
}

function fillSelect(select, ports, keep) {
    const current = keep || select.value;
    select.length = 1;
    for (const port of ports) {
        const o = document.createElement('option');
        o.value = port.id;
        o.textContent = port.name;
        select.appendChild(o);
    }
    if ([...select.options].some(o => o.value === current))
        select.value = current;
}

function isSynth(port) {
    return /minimoog/i.test(port.name || '');
}

function refreshPorts() {
    const outs = [...midiAccess.outputs.values()];
    const ins = [...midiAccess.inputs.values()];
    const outSel = document.getElementById('midi-out');
    const inSel = document.getElementById('midi-in');

    const preferred = output ? output.id : (outs.find(isSynth) || {}).id;
    fillSelect(outSel, outs, preferred);
    fillSelect(inSel, ins.filter(p => !isSynth(p)));

    selectOutput(outSel.value);
    selectKeyboardInput(inSel.value);

    // Listen to the synth itself for parameter echoes
    const si = ins.find(isSynth) || null;
    if (si !== synthInput) {
        if (synthInput)
            synthInput.onmidimessage = null;
        synthInput = si;
        if (synthInput)
            synthInput.onmidimessage = onSynthMessage;
    }
}

function selectOutput(id) {
    const next = id ? midiAccess.outputs.get(id) || null : null;
    const changed = next !== output;
    output = next;
    setStatus();
    if (changed && output)
        sendPatch(channel);
}

function selectKeyboardInput(id) {
    if (keyboardInput)
        keyboardInput.onmidimessage = null;
    keyboardInput = id ? midiAccess.inputs.get(id) || null : null;
    if (keyboardInput)
        keyboardInput.onmidimessage = onKeyboardMessage;
}

// A hardware keyboard plays the selected channel
function onKeyboardMessage(e) {
    const [status, d1, d2] = e.data;
    const type = status & 0xf0;
    if (type === 0x90 || type === 0x80) {
        send([type | channel, d1, d2]);
        showKey(d1, type === 0x90 && d2 > 0);
    } else if (type === 0xe0 || type === 0xb0 && (d1 === 1 || d1 === 33 || d1 === 64)) {
        send([type | channel, d1, d2]);
    }
}

// CCs from the synth (parameter echo / state dump) update the panel
function onSynthMessage(e) {
    const [status, cc, value] = e.data;
    if ((status & 0xf0) !== 0xb0)
        return;
    const ch = status & 0x0f;
    const key = ch * 128;

    if (cc >= 32 && cc < 64) {
        const p = CONTROLS.find(q => q.fine && q.cc === cc - 32);
        if (p && lsbPending[key + p.cc] !== undefined)
            applyIncoming(ch, p, midiToValue(p, lsbPending[key + p.cc], value));
        return;
    }
    const p = CONTROLS.find(q => q.cc === cc);
    if (!p)
        return;
    if (p.fine)
        lsbPending[key + cc] = value;
    applyIncoming(ch, p, midiToValue(p, value));
}

function applyIncoming(ch, p, v) {
    patches[ch][p.id] = v;
    savePatches();
    if (ch === channel)
        widgets[p.id].update(v);
}

async function initMidi() {
    if (!navigator.requestMIDIAccess) {
        document.getElementById('midi-status').title = 'Web MIDI is not supported by this browser';
        return;
    }
    try {
        midiAccess = await navigator.requestMIDIAccess();
    } catch (e) {
        document.getElementById('midi-status').title = 'MIDI access refused';
        return;
    }
    midiAccess.onstatechange = refreshPorts;
    refreshPorts();
}

/* --------------------------------------------------------------------------
 * Widgets
 * ------------------------------------------------------------------------ */

const SVG = 'http://www.w3.org/2000/svg';
const widgets = {};

function svg(tag, attrs, parent) {
    const el = document.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs))
        el.setAttribute(k, v);
    if (parent)
        parent.appendChild(el);
    return el;
}

function changeParam(p, v) {
    if (patches[channel][p.id] === v)
        return;
    patches[channel][p.id] = v;
    widgets[p.id].update(v);
    sendParam(channel, p, v);
    savePatches();
}

// Rotary knob (continuous) or rotary selector (enum) with 270 degree travel
function makeKnob(p) {
    const wrap = document.createElement('div');
    wrap.className = 'control';
    wrap.innerHTML = `<div class="name">${p.name}</div>`;

    const knob = svg('svg', { class: 'knob' + (p.type === 'enum' ? ' selector' : ''), viewBox: '0 0 46 46',
        tabindex: 0, role: 'slider', 'aria-label': p.name });
    // Scale ticks
    const ticks = p.type === 'enum' ? p.options.length : 11;
    for (let i = 0; i < ticks; i++) {
        const a = (-135 + 270 * i / (ticks - 1)) * Math.PI / 180;
        svg('line', { class: 'scale', x1: 23 + 19 * Math.sin(a), y1: 23 - 19 * Math.cos(a),
            x2: 23 + 22 * Math.sin(a), y2: 23 - 22 * Math.cos(a) }, knob);
    }
    svg('circle', { class: 'body', cx: 23, cy: 23, r: 16 }, knob);
    svg('circle', { class: 'cap', cx: 23, cy: 23, r: 8 }, knob);
    const pointer = svg('line', { class: 'pointer', x1: 23, y1: 21, x2: 23, y2: 8 }, knob);
    wrap.appendChild(knob);

    const value = document.createElement('div');
    value.className = 'value';
    wrap.appendChild(value);

    let pos = 0;
    const update = v => {
        pos = valueToPos(p, v);
        pointer.setAttribute('transform', `rotate(${-135 + 270 * pos} 23 23)`);
        value.textContent = formatValue(p, v);
        knob.setAttribute('aria-valuetext', formatValue(p, v));
    };

    const steps = p.type === 'enum' ? p.options.length - 1 : 0;
    const nudge = (delta) => {
        if (steps)
            changeParam(p, Math.max(0, Math.min(steps, patches[channel][p.id] + Math.sign(delta))));
        else
            changeParam(p, posToValue(p, pos + delta));
    };

    // Drag up/down: 200 px for the full range, shift for fine
    let drag = null;
    knob.addEventListener('pointerdown', e => {
        knob.setPointerCapture(e.pointerId);
        drag = { y: e.clientY, pos, acc: 0 };
    });
    knob.addEventListener('pointermove', e => {
        if (!drag)
            return;
        const scale = e.shiftKey ? 1000 : 200;
        if (steps) {
            drag.acc += (drag.y - e.clientY);
            drag.y = e.clientY;
            if (Math.abs(drag.acc) > 18) {
                nudge(drag.acc);
                drag.acc = 0;
            }
        } else {
            drag.pos += (drag.y - e.clientY) / scale;
            drag.y = e.clientY;
            changeParam(p, posToValue(p, drag.pos));
        }
    });
    knob.addEventListener('pointerup', () => drag = null);
    knob.addEventListener('pointercancel', () => drag = null);
    knob.addEventListener('wheel', e => {
        e.preventDefault();
        nudge((e.deltaY < 0 ? 1 : -1) * (e.shiftKey ? 0.002 : 0.02));
    }, { passive: false });
    knob.addEventListener('dblclick', () => changeParam(p, p.def));
    knob.addEventListener('keydown', e => {
        const d = { ArrowUp: 1, ArrowRight: 1, ArrowDown: -1, ArrowLeft: -1 }[e.key];
        if (d) {
            e.preventDefault();
            nudge(d * (e.shiftKey ? 0.002 : 0.02));
        }
    });

    return { el: wrap, update };
}

function makeSwitch(p) {
    const wrap = document.createElement('div');
    wrap.className = 'control switch';
    wrap.innerHTML = `<div class="name">${p.name}</div>`;
    const btn = document.createElement('button');
    btn.className = 'rocker';
    btn.setAttribute('role', 'switch');
    btn.setAttribute('aria-label', p.name);
    btn.addEventListener('click', () => changeParam(p, patches[channel][p.id] ? 0 : 1));
    wrap.appendChild(btn);
    const value = document.createElement('div');
    value.className = 'value';
    wrap.appendChild(value);
    const update = v => {
        btn.setAttribute('aria-checked', v ? 'true' : 'false');
        value.textContent = v ? 'On' : 'Off';
    };
    return { el: wrap, update };
}

function buildPanel() {
    // Shared gradient for the knob caps
    const defs = svg('svg', { width: 0, height: 0, style: 'position:absolute' }, document.body);
    const grad = svg('radialGradient', { id: 'cap-gradient', cx: '40%', cy: '35%', r: '70%' }, svg('defs', {}, defs));
    svg('stop', { offset: '0%', 'stop-color': '#f4f4f4' }, grad);
    svg('stop', { offset: '100%', 'stop-color': '#8a8a8a' }, grad);

    for (const p of PARAMS) {
        const grid = p.sec === 'perf'
            ? document.getElementById('perf-switches')
            : document.querySelector(`#sec-${p.sec} .grid`);
        if (p.label) {
            const l = document.createElement('div');
            l.className = 'sublabel';
            l.textContent = p.label;
            grid.appendChild(l);
            continue;
        }
        if (p.blank) {
            const s = document.createElement('div');
            s.className = 'spacer';
            grid.appendChild(s);
            continue;
        }
        const w = p.type === 'switch' ? makeSwitch(p) : makeKnob(p);
        widgets[p.id] = w;
        grid.appendChild(w.el);
    }
}

function showPatch() {
    for (const p of CONTROLS)
        widgets[p.id].update(patches[channel][p.id]);
}

function buildChannels() {
    const box = document.getElementById('channels');
    for (let i = 0; i < 16; i++) {
        const b = document.createElement('button');
        b.textContent = i + 1;
        b.setAttribute('role', 'radio');
        b.setAttribute('aria-label', `Channel ${i + 1}`);
        b.addEventListener('click', () => selectChannel(i));
        box.appendChild(b);
    }
}

function selectChannel(ch) {
    allNotesOff();
    channel = ch;
    [...document.getElementById('channels').children].forEach((b, i) =>
        b.setAttribute('aria-checked', i === ch ? 'true' : 'false'));
    showPatch();
    sendPatch(ch); // Keeps the synth in step with the panel, e.g. after a board reset
    try {
        localStorage.setItem(STORAGE_KEY + '-channel', ch);
    } catch (e) {
        // Ignore
    }
}

/* --------------------------------------------------------------------------
 * Keyboard and wheels
 * ------------------------------------------------------------------------ */

const KEYS = 37;            // C to C, three octaves
let baseNote = 36;          // C2
const held = new Set();
const keyEls = {};

const COMPUTER_KEYS = 'zsxdcvgbhnjmq2w3er5t6y7ui';

function noteName(n) {
    return ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'][n % 12] + (Math.floor(n / 12) - 1);
}

function noteOn(n) {
    if (n < 0 || n > 127 || held.has(n))
        return;
    held.add(n);
    send([0x90 | channel, n, 100]);
    showKey(n, true);
}

function noteOff(n) {
    if (!held.has(n))
        return;
    held.delete(n);
    send([0x80 | channel, n, 0]);
    showKey(n, false);
}

function allNotesOff() {
    for (const n of [...held])
        noteOff(n);
}

function showKey(n, down) {
    const el = keyEls[n];
    if (el)
        el.classList.toggle('down', down);
}

function buildKeyboard() {
    const kb = document.getElementById('keyboard');
    kb.innerHTML = '';
    for (const k in keyEls)
        delete keyEls[k];

    const whites = [];
    for (let i = 0; i < KEYS; i++)
        if (![1, 3, 6, 8, 10].includes(i % 12))
            whites.push(i);
    const w = 100 / whites.length;

    for (let i = 0; i < KEYS; i++) {
        const n = baseNote + i;
        const black = [1, 3, 6, 8, 10].includes(i % 12);
        const el = document.createElement('div');
        el.className = 'key ' + (black ? 'black' : 'white');
        el.dataset.note = n;
        if (black) {
            const left = whites.filter(x => x < i).length;
            el.style.left = `calc(${left * w}% - ${w * 0.3}%)`;
            el.style.width = `${w * 0.6}%`;
        } else {
            el.style.left = `${whites.indexOf(i) * w}%`;
            el.style.width = `${w}%`;
            if (i % 12 === 0)
                el.innerHTML = `<span class="key-label">${noteName(n)}</span>`;
        }
        if (held.has(n))
            el.classList.add('down');
        kb.appendChild(el);
        keyEls[n] = el;
    }
    document.getElementById('oct-label').textContent = noteName(baseNote);
}

function setupKeyboardInput() {
    const kb = document.getElementById('keyboard');
    const pointers = new Map(); // pointerId -> note

    const noteAt = (x, y) => {
        const el = document.elementFromPoint(x, y);
        return el && el.closest('.key') ? +el.closest('.key').dataset.note : null;
    };
    kb.addEventListener('pointerdown', e => {
        kb.setPointerCapture(e.pointerId);
        const n = noteAt(e.clientX, e.clientY);
        if (n !== null) {
            pointers.set(e.pointerId, n);
            noteOn(n);
        }
    });
    kb.addEventListener('pointermove', e => {
        if (!pointers.has(e.pointerId))
            return;
        const n = noteAt(e.clientX, e.clientY);
        const old = pointers.get(e.pointerId);
        if (n !== null && n !== old) {
            noteOn(n);
            noteOff(old); // On first, so glide connects the notes
            pointers.set(e.pointerId, n);
        }
    });
    const release = e => {
        if (pointers.has(e.pointerId)) {
            noteOff(pointers.get(e.pointerId));
            pointers.delete(e.pointerId);
        }
    };
    kb.addEventListener('pointerup', release);
    kb.addEventListener('pointercancel', release);

    // Computer keyboard: z-m lower octave, q-i upper octave
    const down = new Set();
    window.addEventListener('keydown', e => {
        if (e.repeat || e.ctrlKey || e.metaKey || e.altKey || e.target.matches('input, select'))
            return;
        const i = COMPUTER_KEYS.indexOf(e.key.toLowerCase());
        if (i < 0)
            return;
        const offset = i < 12 ? i : i - 12 + 12;
        const n = baseNote + 12 + offset;
        down.add(e.key.toLowerCase());
        noteOn(n);
    });
    window.addEventListener('keyup', e => {
        const i = COMPUTER_KEYS.indexOf(e.key.toLowerCase());
        if (i < 0)
            return;
        down.delete(e.key.toLowerCase());
        noteOff(baseNote + 12 + i);
    });
    window.addEventListener('blur', allNotesOff);

    document.getElementById('oct-down').addEventListener('click', () => {
        allNotesOff();
        baseNote = Math.max(0, baseNote - 12);
        buildKeyboard();
    });
    document.getElementById('oct-up').addEventListener('click', () => {
        allNotesOff();
        baseNote = Math.min(127 - KEYS + 1, baseNote + 12);
        buildKeyboard();
    });

    document.getElementById('sustain').addEventListener('change', e =>
        send([0xb0 | channel, 64, e.target.checked ? 127 : 0]));
}

// Pitch bend springs back to the centre, mod wheel stays where it is left
function setupWheel(id, onValue, spring) {
    const wheel = document.getElementById(id);
    const thumb = wheel.querySelector('.wheel-thumb');
    let pos = spring ? 0.5 : 0;
    const draw = () => thumb.style.top = `${(1 - pos) * (wheel.clientHeight - 10)}px`;
    let drag = null;
    wheel.addEventListener('pointerdown', e => {
        wheel.setPointerCapture(e.pointerId);
        drag = { y: e.clientY, pos };
    });
    wheel.addEventListener('pointermove', e => {
        if (!drag)
            return;
        pos = Math.max(0, Math.min(1, drag.pos + (drag.y - e.clientY) / (wheel.clientHeight - 10)));
        draw();
        onValue(pos);
    });
    const end = () => {
        drag = null;
        if (spring) {
            pos = 0.5;
            draw();
            onValue(pos);
        }
    };
    wheel.addEventListener('pointerup', end);
    wheel.addEventListener('pointercancel', end);
    draw();
}

/* --------------------------------------------------------------------------
 * Patch buttons
 * ------------------------------------------------------------------------ */

function setupActions() {
    document.getElementById('btn-send').addEventListener('click', () => sendPatch(channel));
    document.getElementById('btn-init').addEventListener('click', () => {
        patches[channel] = defaultPatch();
        showPatch();
        sendPatch(channel);
        savePatches();
    });
    document.getElementById('btn-save').addEventListener('click', () => {
        const blob = new Blob([JSON.stringify(patches[channel], null, 2)], { type: 'application/json' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = `minimoog-channel-${channel + 1}.json`;
        a.click();
        URL.revokeObjectURL(a.href);
    });
    const file = document.getElementById('file-load');
    document.getElementById('btn-load').addEventListener('click', () => file.click());
    file.addEventListener('change', async () => {
        if (!file.files[0])
            return;
        try {
            const data = JSON.parse(await file.files[0].text());
            patches[channel] = Object.assign(defaultPatch(), data);
            showPatch();
            sendPatch(channel);
            savePatches();
        } catch (e) {
            alert('Not a MiniMoog patch file');
        }
        file.value = '';
    });

    document.getElementById('midi-out').addEventListener('change', e => selectOutput(e.target.value));
    document.getElementById('midi-in').addEventListener('change', e => selectKeyboardInput(e.target.value));
}

/* --------------------------------------------------------------------------
 * Start
 * ------------------------------------------------------------------------ */

buildPanel();
buildChannels();
buildKeyboard();
setupKeyboardInput();
setupActions();
setupWheel('wheel-bend', pos => {
    const v = Math.max(0, Math.min(16383, Math.round(pos * 16383)));
    send([0xe0 | channel, v & 0x7f, v >> 7]);
}, true);
setupWheel('wheel-mod', pos => {
    const v = Math.round(pos * 16383);
    send([0xb0 | channel, 1, v >> 7]);
    send([0xb0 | channel, 33, v & 0x7f]);
}, false);

let startChannel = 0;
try {
    startChannel = Math.max(0, Math.min(15, +localStorage.getItem(STORAGE_KEY + '-channel') || 0));
} catch (e) {
    // Ignore
}
selectChannel(startChannel);
initMidi();

// For automated tests
window.minimoog = { PARAMS, valueToMidi, midiToValue, patches: () => patches, selectChannel, changeParam: (id, v) => changeParam(PARAM_BY_ID[id], v), noteOn, noteOff };
