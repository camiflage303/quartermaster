#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;

namespace ui {
    void init();
    void refresh();

    // Kept for compatibility; now just commits if pending.
    void commitAfterStepIfNeededExt();

    // Force an immediate LED commit right now if anything is dirty.
    void commitNow();
}
