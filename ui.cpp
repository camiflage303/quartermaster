// --- ui.cpp (updated) ------------------------------------------------------
// Key changes:
// - Added rate-limited LED flushes (DotStar .show()) while following ext MIDI.
// - Skips one flush immediately after a sequencer step (reduces clock starvation).
// - No UI work happens inside seq::nextStep(); main loop calls ui::refresh().
//
// Requires clock_engine.h to expose:
//   extern volatile bool usingExt;
//   extern volatile bool stepJustFired;

#include "ui.h"
#include "sequencer.h"
#include "hw_inputs.h"
#include "clock_engine.h"   // <-- for usingExt + stepJustFired
#include <Adafruit_DotStar.h>
#include <Arduino.h>

constexpr uint8_t NUM_LEDS = 16;

// Use hardware SPI (Nano: D11 = SDI, D13 = CKI)
Adafruit_DotStar strip(NUM_LEDS, DOTSTAR_BGR);

// ---------- Internal UI state ----------
static uint8_t prevStep = 255;
static uint8_t prevLoopLo = 0, prevLoopHi = 15;
static uint8_t prevType[NUM_LEDS] = {0};
static bool    ledsDirty = false;

// Flush governor (to avoid starving MIDI.read())
static uint32_t lastFlushUs = 0;
static constexpr uint32_t kMinFlushIntervalUs = 2000;  // 2 ms ≈ safe gap between F8 bursts

// ---------- Small helpers ----------
inline void px(uint8_t logicalIndex, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t physicalIndex = (NUM_LEDS - 1) - logicalIndex;  // REVERSE ORDER HERE
  uint32_t c = strip.Color(r, g, b);
  if (strip.getPixelColor(physicalIndex) != c) { 
      strip.setPixelColor(physicalIndex, c); 
      ledsDirty = true; 
  }
}

struct RGB { uint8_t r, g, b; };
constexpr RGB CLR_OFF {0,0,0}, CLR_L{8,0,0}, CLR_R{8,0,0};
constexpr RGB CLR_LOOP{80,80,40}, CLR_GEN{40,40,20};

static inline void readBounds(uint8_t &s, uint8_t &e) {
  s = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
  e = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
}
static inline void bandLoHi(uint8_t &lo, uint8_t &hi) {
  uint8_t s, e; readBounds(s, e);
  if (s <= e) { lo = s; hi = e; } else { lo = e; hi = s; }
}
static inline bool inBand(uint8_t i, uint8_t lo, uint8_t hi) { return i >= lo && i <= hi; }

// Return index of the left and right loop markers, or -1 if full-span/no marker
static inline int8_t markL() {
  uint8_t lo, hi; bandLoHi(lo, hi);
  if (hi - lo + 1 == 16) return -1;
  uint8_t s, e; readBounds(s, e);
  if (s == 0 || e == 0) return -1;
  return int8_t((lo + 15) & 0x0F);
}
static inline int8_t markR() {
  uint8_t lo, hi; bandLoHi(lo, hi);
  if (hi - lo + 1 == 16) return -1;
  uint8_t s, e; readBounds(s, e);
  if (e == 15 || s == 15) return -1;
  return int8_t((hi + 1) & 0x0F);
}

static inline RGB vColour(bool acc) {
  auto heat = [&](uint8_t v) -> RGB {
    if (!v) return RGB{0,0,0};
    uint8_t hue = map(v, 0, 127, 85, 255);
    auto wheel = [&](uint8_t p) -> RGB {
      if (p < 85)   return RGB{uint8_t(255 - p * 3), 0,                 uint8_t(p * 3)};
      if (p < 170) { p -= 85; return RGB{0,                 uint8_t(p * 3), uint8_t(255 - p * 3)}; }
      p -= 170;     return RGB{uint8_t(p * 3), uint8_t(255 - p * 3), 0};
    };
    RGB b = wheel(hue);
    uint16_t br = map(v, 0, 127, 5, 120);
    b.r = b.r * br / 255; b.g = b.g * br / 255; b.b = b.b * br / 255;
    return b;
  };
  return acc ? heat(hw::pots.accentVel) : heat(hw::pots.velocity);
}

static void paintStatic() {
  uint8_t lo, hi; bandLoHi(lo, hi);
  int8_t ixL = markL(), ixR = markR();

  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    RGB c;
    if      (i == ixL)                c = CLR_L;
    else if (i == ixR)                c = CLR_R;
    else if (inBand(i, lo, hi))       c = seq::vel(i) ? (seq::acc(i) ? vColour(true) : vColour(false)) : CLR_OFF;
    else                              c = CLR_OFF;

    px(i, c.r, c.g, c.b);
    prevType[i] = (i == ixL) ? 3 : (i == ixR) ? 4 : (seq::vel(i) ? (seq::acc(i) ? 2 : 1) : 0);
  }
  prevLoopLo = lo; prevLoopHi = hi;
}

// Decide whether to flush the LED buffer now; rate-limit under external clock
static inline void maybeFlush() {
  if (!ledsDirty) return;

  // Skip one flush immediately after a step while slaved, to avoid coinciding
  // with the tightest timing moment (F8 boundary + nextStep()).
  // Always clear stepJustFired so it doesn't go stale across mode switches.
  bool stepFired = false;
  noInterrupts();
  stepFired = clock::stepJustFired;
  clock::stepJustFired = false;
  interrupts();

  if (clock::usingExt) {
    if (stepFired) return;  // skip flush right at the step boundary
    uint32_t now = micros();
    if (lastFlushUs != 0 && (uint32_t)(now - lastFlushUs) < kMinFlushIntervalUs) return;
    lastFlushUs = now;
  }

  strip.show();
  ledsDirty = false;
}

// ---------- Public API ----------
void ui::init() {
  strip.begin();
  strip.setBrightness(50);
  strip.show();
  lastFlushUs = micros();
}

void ui::refresh() {
  bool on = hw::btnOnOff.level;

  if (!on) {
    paintStatic();
    prevStep = 255;
    ledsDirty = true;
    maybeFlush();
    return;
  }

  bool needFull = false;
  // reflect V1/V2 pot changes immediately
  static uint8_t pV1 = 255, pV2 = 255;
  if (hw::pots.velocity != pV1 || hw::pots.accentVel != pV2) {
    pV1 = hw::pots.velocity; pV2 = hw::pots.accentVel; needFull = true;
  }

  uint8_t lo, hi; bandLoHi(lo, hi);
  if (lo != prevLoopLo || hi != prevLoopHi) {
    needFull = true;
  } else {
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
      uint8_t now = seq::vel(i) ? (seq::acc(i) ? 2 : 1) : 0;
      if (now != prevType[i]) { needFull = true; break; }
    }
  }

  if (needFull) {
    paintStatic();
    prevStep = 255;
    ledsDirty = true;
  }

  // Playhead
  uint8_t step = seq::stepNow();
  if (needFull || step != prevStep || prevLoopLo == prevLoopHi) {
    // erase old
    if (prevStep < NUM_LEDS) {
      int8_t ixL = markL(), ixR = markR(); RGB c;
      if      (prevStep == ixL) c = CLR_L;
      else if (prevStep == ixR) c = CLR_R;
      else if (prevStep >= prevLoopLo && prevStep <= prevLoopHi) {
        c = seq::vel(prevStep) ? (seq::acc(prevStep) ? vColour(true) : vColour(false)) : CLR_OFF;
      } else c = CLR_OFF;
      px(prevStep, c.r, c.g, c.b);
      prevType[prevStep] = (prevStep == ixL) ? 3 : (prevStep == ixR) ? 4 : (seq::vel(prevStep) ? (seq::acc(prevStep) ? 2 : 1) : 0);
    }

    // draw new head
    RGB head = seq::vel(step) ? CLR_LOOP : CLR_GEN;
    if (hw::btnInstant.edge)       head = RGB{60,60,0};
    else if (hw::btnDestruct.edge) head = RGB{60,0,0};

    px(step, head.r, head.g, head.b);
    prevStep = step;
    ledsDirty = true;
  }

  // Flush with DotStar (rate-limited under external clock)
  maybeFlush();
}
