#pragma once
#include <Arduino.h>
#include "hw_inputs.h"      // for pots + buttons

namespace seq {

    enum class Aspect : uint8_t { Pitch, Vel, Oct, Acc, Count }; //Dbl

    void init();
    void nextStep();
    void forceStep(uint8_t step);   // 0-15

    /* expose read-only state for UI */
    uint8_t stepNow();               // 0-15
    uint8_t pitch(uint8_t i);        // helpers
    uint8_t vel  (uint8_t i);
    uint8_t oct  (uint8_t i);
    uint8_t acc  (uint8_t i);
}
