#pragma once
#include <Arduino.h>

namespace seq {
  enum class Aspect : uint8_t { Pitch, Vel, Oct, VSel, Count };

  void init();
  void nextStep();
  void forceStep(uint8_t step);
  void regenerateAll(uint8_t probability);
  void commitProspect();
  void rotateAllLeft();
  void rotateAllRight();
  void armReset();

  // Live UI reads for drawing
  uint8_t stepNow();
  uint8_t pitch(uint8_t i);
  uint8_t vel  (uint8_t i);
  uint8_t oct  (uint8_t i);
  uint8_t acc  (uint8_t i);
}
