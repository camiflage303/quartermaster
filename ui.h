#pragma once
#include <Arduino.h>

/* ================================
   Hard kill-switch for LEDs / UI.
   1 = disable all NeoPixel work (no interrupts from WS2812)
   0 = use the original NeoPixel UI
   ================================= */
#define DISABLE_LEDS 1

#if DISABLE_LEDS

// Minimal dummy "strip" so other files link without changes
struct DummyStrip {
  void begin() {}
  void setBrightness(uint8_t) {}
  void show() {}
  uint32_t Color(uint8_t, uint8_t, uint8_t) { return 0; }
  uint32_t getPixelColor(uint16_t) { return 0; }
  void setPixelColor(uint16_t, uint32_t) {}
};
extern DummyStrip strip;

#else

#include <Adafruit_NeoPixel.h>
extern Adafruit_NeoPixel strip;

#endif

namespace ui {

    /* call once from setup() */
    void init();

    /* call every loop() – cheap; only touches the strip when something changed */
    void refresh();

    void commitAfterStepIfNeededExt();  // safe per-step commit when using external clock
}
