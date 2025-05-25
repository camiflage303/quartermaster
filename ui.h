#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;

namespace ui {

    /* call once from setup() */
    void init();

    /* call every loop() – cheap; only touches the strip when something changed */
    void refresh();
}
