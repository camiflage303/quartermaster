// ---------------- clock_engine.h (updated) ---------------------------------
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

  // Exposed knobs (read/write)
  extern volatile bool usingExt;    // true = follows external MIDI clock
  extern uint16_t      bpm;         // internal BPM when not using ext clock

  // Optional telemetry (for profiling)
  extern volatile unsigned long lastF8Us;      // micros() timestamp of last F8
  extern volatile unsigned long f8IntervalUs;  // delta between consecutive F8s

  // NEW: UI/loop coordination flags
  extern volatile bool stepJustFired;      // set true right after seq::nextStep(); UI may clear it
  extern volatile bool pendingAllNotesOff; // set on MIDI Stop; main loop sends CC123 and clears
}
