#pragma once
#include <Arduino.h>

namespace clock {
  // Call once in setup()
  void init();

  // Call every loop()
  void service();

  // Control
  void hardResetCounters();
  void forceStop();

  // ---------- Config knobs ----------
  // MIDI channel to use elsewhere (set this in your .ino via #define; see template below)
  extern uint8_t midiChannel;                // MUST be set in your .ino (1..16)

  // Apply note-division changes exactly on next realm boundary:
  // - straight/dotted divisions on 16th grid (6 clocks), triplets on 8-clock grid
  extern bool    quantizeDivChangeToGrid;    // default true

  // (Optional fallback) Apply on next MIDI Start (FA) if you prefer
  extern bool    quantizeDivChangeToStart;   // default false

  // Reserved for future "downbeat" features; not required by grid-quantize
  extern uint8_t beatsPerBar;                // default 4 (unused here)

  // ---------- Exposed knobs (read/write) ----------
  extern volatile bool usingExt;             // true = follows external MIDI clock
  extern uint16_t      bpm;                  // internal BPM when not using ext clock

  // ---------- Telemetry ----------
  extern volatile unsigned long lastF8Us;    // micros() timestamp of last F8
  extern volatile unsigned long f8IntervalUs;// delta between consecutive F8s

  // ---------- Events/flags ----------
  // Set true whenever a sequencer step is fired (cleared by UI after consuming)
  extern volatile bool stepJustFired;

  // Set true on MIDI Stop/forceStop. Main loop should send CC123 (All Notes Off)
  // on midiChannel and then clear this flag.
  extern volatile bool pendingAllNotesOff;
}
