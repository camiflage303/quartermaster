#pragma once
#include <Arduino.h>
#include "hw_inputs.h"      // for pots + buttons

namespace seq {

    // Renamed Acc -> VSel (Velocity selector: 0 = V1, 1 = V2)
    enum class Aspect : uint8_t { Pitch, Vel, Oct, VSel, Count };

    void init();
    void nextStep();
    void forceStep(uint8_t step);   // 0-15
    void regenerateAll(uint8_t probability);
    void commitProspect();
    void rotateAllLeft();
    void rotateAllRight();
    void armReset();
    // NEW: non-tick background work
    void serviceBackground();
    void markLoopBoundsDirty();

    /* expose read-only state for UI */
    uint8_t stepNow();               // 0-15
    uint8_t pitch(uint8_t i);        // helpers
    uint8_t vel  (uint8_t i);
    uint8_t oct  (uint8_t i);
    uint8_t acc  (uint8_t i);
}
