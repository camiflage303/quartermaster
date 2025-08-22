#include "ui.h"
#include "sequencer.h"
#include "hw_inputs.h"
#include "clock_engine.h"

#if DISABLE_LEDS

/* ----------------------------------------------------
   LED/UI fully disabled build
   ---------------------------------------------------- */
DummyStrip strip;

void ui::init() {}
void ui::refresh() {}
void ui::commitAfterStepIfNeededExt() {}

#else

/* ===== Original UI with NeoPixel LEDs (unchanged) ===== */
#include <Adafruit_NeoPixel.h>

/* ───────── NeoPixel hardware ───────── */
constexpr uint8_t LED_PIN   = 6;      // same as your old build
constexpr uint8_t NUM_LEDS  = 16;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ───────── cached previous state ───── */
static uint8_t  prevStep      = 255;      // invalid → forces first paint
static uint8_t  prevLoopLo    = 0;        // last painted lo (0..15)
static uint8_t  prevLoopHi    = 15;       // last painted hi (0..15)
static uint8_t  prevVel[NUM_LEDS] = {0};  // 0=off,1=v1,2=v2,3=left marker,4=right marker
static bool     ledsDirty = false;

/* quick helpers */
inline void px(uint8_t i,uint8_t r,uint8_t g,uint8_t b){
    uint32_t newClr = strip.Color(r,g,b);
    if (strip.getPixelColor(i) != newClr) {   // only if color actually changes
        strip.setPixelColor(i,newClr);
        ledsDirty = true;                     // request a strip.show()
    }
}

/* colour palette (tweak to taste) */
struct RGB { uint8_t r,g,b; };
constexpr RGB CLR_OFF       {  0,  0,  0};
constexpr RGB CLR_OUTSIDE   {  0,  0,  0};     // off
constexpr RGB CLR_MARK_L    {  8,  0,  0};     // dim red  (left/outside-before band)
constexpr RGB CLR_MARK_R    {  8,  0,  0};     // dim red  (right/outside-after band)
constexpr RGB CLR_PLAY_LOOP { 80, 80, 40};     // warm white
constexpr RGB CLR_PLAY_GEN  { 40, 40, 20};     // dim warm white

/* ---------- heat-map with full blue→red sweep + brightness ramp ---------- */
constexpr uint8_t MAX_BRIGHT = 127;     // peak LED intensity (0-255)

/* 0-255 colour wheel straight from Adafruit_NeoPixel examples */
static inline RGB wheel(uint8_t pos)
{
    uint8_t r, g, b;
    if (pos < 85) {                    // red ↘ blue ↗
        r = 255 - pos * 3;
        g = 0;
        b = pos * 3;
    } else if (pos < 170) {            // blue ↘ green ↗
        pos -= 85;
        r = 0;
        g = pos * 3;
        b = 255 - pos * 3;
    } else {                           // green ↘ red ↗
        pos -= 170;
        r = pos * 3;
        g = 255 - pos * 3;
        b = 0;
    }
    return { r, g, b };
}

/* velocity (0-127) → RGB with both hue AND brightness scaling */
static inline RGB heatColor(uint8_t vel)
{
    if (!vel) return {0,0,0};                          // completely off

    /* ❶ Hue: map 0-127 → wheel  85 (blue) .. 255 (red)          */
    uint8_t hue   = map(vel, 0, 127, 85, 255);
    RGB base      = wheel(hue);

    /* ❷ Brightness: 5 → MAX_BRIGHT linearly with velocity       */
    uint16_t bright = map(vel, 0, 127, 5, MAX_BRIGHT);

    base.r = base.r * bright / 255;
    base.g = base.g * bright / 255;
    base.b = base.b * bright / 255;
    return base;
}

/* Helpers used everywhere else in ui.cpp */
inline RGB colour_v1() { return heatColor(hw::pots.velocity ); }
inline RGB colour_v2() { return heatColor(hw::pots.accentVel); }

inline void flashLed(uint8_t ledIdx, RGB colour, uint8_t frames=4)
{
    static uint8_t timer[8]={0};
    if (ledIdx>=8) return;
    timer[ledIdx] = frames;
    px(ledIdx, colour.r, colour.g, colour.b);
    // count-down in refresh() tail
    for(uint8_t i=0;i<8;i++){
        if(timer[i] && --timer[i]==0) px(i,0,0,0);
    }
}

/* ───────────────────────────────────── */
void ui::init(){
    strip.begin();
    strip.setBrightness(50);
    strip.show();                 // clear
}

/* ── loop/marker helpers (deterministic, non-wrapping band) ───────── */

/* Read pots → 0..15 indices */
static inline void readBounds(uint8_t &s, uint8_t &e){
    s = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
    e = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
}

/* Compute non-wrapped band [lo..hi] regardless of which bound is “start” */
static inline void bandLoHi(uint8_t &lo, uint8_t &hi){
    uint8_t s,e; readBounds(s,e);
    if (s <= e) { lo = s; hi = e; }
    else        { lo = e; hi = s; }
}

/* Length (inclusive) of the non-wrapped band */
static inline uint8_t bandLen(){
    uint8_t lo,hi; bandLoHi(lo,hi);
    return uint8_t(hi - lo + 1);    // 1..16
}

/* Outside markers: left = lo-1, right = hi+1 (mod 16). Hidden when band is full-width. */
static inline int8_t markerLeftIx(){
    if (bandLen() == 16) return -1;                 // full width ⇒ no markers
    uint8_t s,e; readBounds(s,e);
    uint8_t lo,hi; bandLoHi(lo,hi);
    if (s == 0 || e == 0) return -1;                // hide when either bound == 1
    return int8_t((lo + 15) & 0x0F);                // one BEFORE band (wrap-safe)
}
static inline int8_t markerRightIx(){
    if (bandLen() == 16) return -1;                 // full width ⇒ no markers
    uint8_t s,e; readBounds(s,e);
    uint8_t lo,hi; bandLoHi(lo,hi);
    if (e == 15 || s == 15) return -1;              // hide when either bound == 16
    return int8_t((hi + 1) & 0x0F);                 // one AFTER band (wrap-safe)
}

/* Test if index is inside the band (non-wrapped) */
static inline bool inBand(uint8_t idx, uint8_t lo, uint8_t hi){
    return (idx >= lo && idx <= hi);
}

static void paintStaticRegion()
{
    uint8_t lo,hi; bandLoHi(lo,hi);
    int8_t ixL = markerLeftIx();
    int8_t ixR = markerRightIx();

    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        RGB c;
        if (i == ixL) {                     // left/outside-before marker
            c = CLR_MARK_L;
        } else if (i == ixR) {              // right/outside-after marker
            c = CLR_MARK_R;
        } else if (inBand(i,lo,hi)) {       // inside the band
            if (seq::vel(i)) {
                c = seq::acc(i) ? colour_v2() : colour_v1();
            } else {
                c = CLR_OFF;                // rest inside band
            }
        } else {
            c = CLR_OUTSIDE;                // completely outside band
        }

        px(i, c.r, c.g, c.b);

        uint8_t type = 0;                   // 0 = off/rest
        if      (i == ixL) type = 3;        // left marker
        else if (i == ixR) type = 4;        // right marker
        else if (seq::vel(i)) type = seq::acc(i) ? 2 : 1;
        prevVel[i] = type;
    }

    prevLoopLo = lo;
    prevLoopHi = hi;
}

void ui::refresh()
{
    // Hide playhead when transport is OFF, and suppress head on first ON frame
    static bool     prevOn = true;
    static bool     waitingForFirstStep = false;
    static uint8_t  stepAtOn = 0;   // step we were parked at when ON was pressed
    bool on = hw::btnOnOff.level;

    if (!on) {
        paintStaticRegion();         // show markers + gates only
        prevStep = 255;              // forget old head
        ledsDirty = true;
        if (!clock::usingExt) {
            strip.show();
            ledsDirty = false;
        }
        prevOn = on;
        waitingForFirstStep = false; // reset any arming
        return;                      // no head while OFF
    }

    // OFF → ON edge: arm "wait for first step change" and paint static once
    if (on && !prevOn) {
        waitingForFirstStep = true;
        stepAtOn = seq::stepNow();   // typically the parked position
        paintStaticRegion();
        prevStep = 255;
        ledsDirty = true;
        if (!clock::usingExt) {
            strip.show();
            ledsDirty = false;
        }
        prevOn = on;
        return;
    }
    prevOn = on;

    // While armed, suppress head until the sequencer actually advances
    if (waitingForFirstStep) {
        if (seq::stepNow() == stepAtOn) {
            paintStaticRegion();
            ledsDirty = true;
            if (!clock::usingExt) {
                strip.show();
                ledsDirty = false;
            }
            return;
        }
        waitingForFirstStep = false;
        prevStep = 255;  // force head repaint on first visible step
    }

    bool needFull = false;

    static uint8_t prevPotV1 = 255, prevPotV2 = 255;
    if (hw::pots.velocity  != prevPotV1 ||
        hw::pots.accentVel != prevPotV2) {
        needFull  = true;
        prevPotV1 = hw::pots.velocity;
        prevPotV2 = hw::pots.accentVel;
    }

    /* 1. detect whether static region must be repainted ── */
    uint8_t lo,hi; bandLoHi(lo,hi);

    if (lo != prevLoopLo || hi != prevLoopHi) {
        needFull = true;                 /* pots moved → repaint band   */
    } else {
        for (uint8_t i = 0; i < NUM_LEDS; ++i) {
            uint8_t nowType = seq::vel(i) ? (seq::acc(i) ? 2 : 1) : 0;
            if (nowType != prevVel[i]) { needFull = true; break; }
        }
    }
    if (needFull) {
        paintStaticRegion();
        prevStep = 255;                  /* force head redraw too      */
        ledsDirty = true;

        if (!clock::usingExt){
            strip.show();
            ledsDirty = false;
        }
    }

    /* 2. head / play-cursor ───────────────────────────────── */
    uint8_t step = seq::stepNow();           // 0-15
    bool oneStepLoop = (prevLoopLo == prevLoopHi);   // 1-step band

    if (needFull || step != prevStep || oneStepLoop) {
        /* erase old -------- */
        if (prevStep < NUM_LEDS) {
            uint8_t i = prevStep;
            int8_t ixL = markerLeftIx();
            int8_t ixR = markerRightIx();

            RGB c;
            if (i == ixL) {
                c = CLR_MARK_L;
            } else if (i == ixR) {
                c = CLR_MARK_R;
            } else if (inBand(i, prevLoopLo, prevLoopHi)) {
                if (seq::vel(i)) {
                    c = seq::acc(i) ? colour_v2() : colour_v1();
                } else {
                    c = CLR_OFF;
                }
            } else {
                c = CLR_OUTSIDE;
            }
            px(i, c.r, c.g, c.b);

            uint8_t type = 0;
            if      (i == ixL) type = 3;
            else if (i == ixR) type = 4;
            else if (seq::vel(i)) type = seq::acc(i) ? 2 : 1;
            prevVel[i] = type;
        }

        /* draw new -------- */
        RGB head = seq::vel(step) ? CLR_PLAY_LOOP : CLR_PLAY_GEN;
        if (hw::btnInstant.edge)       head = (RGB{60,60, 0});   // yellow flash
        else if (hw::btnDestruct.edge) head = (RGB{60, 0, 0});   // red flash
        px(step, head.r, head.g, head.b);

        prevStep = step;
    }

    /* ---------- commit to strip ---------- */
    if (ledsDirty) {
        if (!clock::usingExt) {
            /* internal-clock mode – safe to block right now */
            strip.show();
            ledsDirty = false;
        } else {
            /* external sync: defer the blocking call until the exact
               instant a new step has *already* arrived → we piggy-back
               on the gap we know is safe (Option 1 throttle). */
        }
    }
}

/* Commit exactly once per step when using external clock (safe timing window). */
void ui::commitAfterStepIfNeededExt() {
    if (!clock::usingExt) return;
    if (ledsDirty) {
        strip.show();     // ~0.4 ms; call this right after a step edge
        ledsDirty = false;
    }
}

#endif // DISABLE_LEDS
