#pragma once
#include <Arduino.h>
#include <Adafruit_DotStar.h>

extern Adafruit_DotStar strip;

namespace ui {
  void init();
  void refresh();                    // buffer-only work
}
