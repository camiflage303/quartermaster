#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel strip;

namespace ui {
  void init();
  void refresh();                    // buffer-only work
  void commitAfterStepIfNeededExt(); // call once per step (ext clock safe)
}
