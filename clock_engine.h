#pragma once
#include <Arduino.h>

namespace clock {
  // Call once
  void init();

  // Call every loop()
  void service();

  // Control
  void hardResetCounters();
  void forceStop();

  // Exposed knobs (read/write)
  extern volatile bool usingExt;  // true=follows external clock
  extern uint16_t      bpm;       // internal BPM

  // Optional telemetry (for profiling)
  extern volatile unsigned long lastF8Us;
  extern volatile unsigned long f8IntervalUs;
}
