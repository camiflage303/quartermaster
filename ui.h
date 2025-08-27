#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;

namespace ui {
    void init();
    void refresh();

    // Still exposed for compatibility; now just commits immediately if needed.
    void commitAfterStepIfNeededExt();

    // New: force an immediate LED commit right now if anything is dirty.
    void commitNow();
}
