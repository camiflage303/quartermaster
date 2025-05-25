#pragma once
#include <Arduino.h>

namespace clock {

    // call once from setup()
    void init();

    // call each loop()  – handles both ext & int timing
    void service();

    void hardResetCounters();
    void forceStop();

    // the app can read or set these (but doesn’t have to)
    extern bool     usingExt;         // true = follow external clock
    extern uint16_t bpm;              // beats per minute (30-300)
    extern uint8_t  pulsesPerStep;    // 24, 12, 6 … (PPQN ÷ divider)
}
