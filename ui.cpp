#include "ui.h"
#include "sequencer.h"
#include "hw_inputs.h"
#include <Adafruit_NeoPixel.h>
#include "clock_engine.h"

/* ───────── NeoPixel hardware ───────── */
constexpr uint8_t LED_PIN   = 6;      // same as your old build
constexpr uint8_t NUM_LEDS  = 16;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ───────── cached previous state ───── */
static uint8_t  prevStep      = 255;      // invalid → forces first paint
static uint8_t  prevLoopMin   = 0;
static uint8_t  prevLoopMax   = 0;
static uint8_t  prevVel[NUM_LEDS] = {0};

static bool ledsDirty = false;           // set → something changed this frame

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
constexpr RGB CLR_OFF      {  0,  0,  0};
constexpr RGB CLR_OUTSIDE  {  0,  0,  0};     // off
constexpr RGB CLR_MARK_END {  8,  0,  0};     // dim red  (loop-end marker)
constexpr RGB CLR_MARK_ST  {  8,  0,  0};     // dim red  (loop-start marker)
constexpr RGB CLR_PLAY_LOOP{ 80, 80, 40};     // warm white
constexpr RGB CLR_PLAY_GEN { 40, 40, 20};     // dim warm white

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

/* ── marker helpers ───────────────────────────────────────────────
   Return -1 when the marker should be hidden (start=1 or end=16). */

inline int8_t markerStartIx()      /* dim-magenta */
{
    uint8_t s = hw::pots.loopStart ? hw::pots.loopStart-1 : 0;
    uint8_t e = hw::pots.loopEnd   ? hw::pots.loopEnd  -1 : 0;
    bool wrap = s > e;

    if (!wrap && s == 0) return -1;            // hide when start = 1
    return wrap ? (s + 1)  & 0x0F              // one AFTER when wrapped
                : (s + 15) & 0x0F;             // one BEFORE otherwise
}

inline int8_t markerEndIx()        /* dim-white  */
{
    uint8_t s = hw::pots.loopStart ? hw::pots.loopStart-1 : 0;
    uint8_t e = hw::pots.loopEnd   ? hw::pots.loopEnd  -1 : 0;
    bool wrap = s > e;

    if (!wrap && e == 15) return -1;           // hide when end = 16
    return wrap ? (e + 15) & 0x0F              // one BEFORE when wrapped
                : (e + 1)  & 0x0F;             // one AFTER otherwise
}

static void paintStaticRegion()
{
    /* loop range */
    uint8_t lo = hw::pots.loopStart-1;        // 0-15
    uint8_t hi = hw::pots.loopEnd  -1;
    if (lo > hi) { uint8_t tmp = lo; lo = hi; hi = tmp; }

    /*for(uint8_t i=0;i<NUM_LEDS;i++){
        RGB c = (i<lo || i>hi) ? CLR_OUTSIDE
                               : (seq::vel(i)? CLR_INSIDE : CLR_OFF);
        px(i,c.r,c.g,c.b);
        prevVel[i] = seq::vel(i);
    }*/

    int8_t ixSt  = markerStartIx();
    int8_t ixEnd = markerEndIx();

    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        RGB c;
        if (i == ixSt) {                 // marker BEFORE start
            c = CLR_MARK_ST;
        } else if (i == ixEnd) {            // marker AFTER end
            c = CLR_MARK_END;
        } else if (i < lo || i > hi) {        // completely outside
            c = CLR_OUTSIDE;
        } else if (seq::vel(i)) {             // gate present?
            c = seq::acc(i) ? colour_v2()     // Velocity-2 hit (yellow, scaled)
                            : colour_v1();    // Velocity-1 hit (green,  scaled)
        } else {                              // rest
            c = CLR_OFF;
        }
        px(i, c.r, c.g, c.b);

        uint8_t type = 0;                          // 0 = off/rest
        if      (i == ixSt)  type = 3;             // magenta marker
        else if (i == ixEnd) type = 4;             // white   marker
        else if (seq::vel(i)) type = seq::acc(i) ? 2 : 1;
        prevVel[i] = type;
    }

    prevLoopMin = lo;
    prevLoopMax = hi;
}

void ui::refresh()
{
    //if (clock::usingExt) return;      // ❶ DON’T touch LEDs while external sync is active
    /* flash on edges ------------------------------------------------ */
//if(hw::btnCycleL.edge) flashLed(3, {0,60,0});       // green
//if(hw::btnCycleR.edge) flashLed(5, {0,60,0});
//if(hw::btnReset .edge) flashLed(4, {0, 0,60});      // blue

    bool needFull = false;

    static uint8_t prevPotV1 = 255, prevPotV2 = 255;
    if (hw::pots.velocity  != prevPotV1 ||
        hw::pots.accentVel != prevPotV2) {
        needFull  = true;
        prevPotV1 = hw::pots.velocity;
        prevPotV2 = hw::pots.accentVel;
    }

    /* 1. detect whether static region must be repainted ── */
    uint8_t lo = hw::pots.loopStart - 1;
    uint8_t hi = hw::pots.loopEnd   - 1;
    if (lo > hi) { uint8_t t = lo; lo = hi; hi = t; }   // swap for reverse

    if (lo != prevLoopMin || hi != prevLoopMax) {
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

        strip.show();                 // one 0.4 ms block – happens rarely
        ledsDirty = false;            // buffer is now clean
    }

    /* 2. head / play-cursor ───────────────────────────────── */
    uint8_t step = seq::stepNow();           // 0-15
    bool oneStepLoop = (prevLoopMin == prevLoopMax);

    if (needFull || step != prevStep || oneStepLoop) {
        /* erase old -------- */
        if (prevStep < NUM_LEDS) {
            uint8_t i = prevStep;
            int8_t ixSt  = markerStartIx();
            int8_t ixEnd = markerEndIx();

            RGB c;
            if (i == ixSt) {                   // magenta marker
                c = CLR_MARK_ST;
            } else if (i == ixEnd) {         // white marker
                c = CLR_MARK_END;
            } else if (i < prevLoopMin || i > prevLoopMax) {
                c = CLR_OUTSIDE;
            } else if (seq::vel(i)) {
                c = seq::acc(i) ? colour_v2()                  // Velocity-2 (yellow, scaled)
                                : colour_v1();                 // Velocity-1 (green,  scaled)
            } else {                                           // rest
                c = CLR_OFF;
            }
            px(i, c.r, c.g, c.b);
            uint8_t type = 0;                    // 0 = off/rest
            if      (i == ixSt)      type = 3;   // magenta marker
            else if (i == ixEnd)     type = 4;   // white   marker
            else if (seq::vel(i))    type = seq::acc(i) ? 2 : 1;
            prevVel[i] = type;
        }

        /* draw new -------- */
        RGB head = seq::vel(step) ? CLR_PLAY_LOOP : CLR_PLAY_GEN;
        if (hw::btnInstant.edge)       head = {60,60, 0};   // yellow flash
        else if (hw::btnDestruct.edge) head = {60, 0, 0};   // red flash
        px(step, head.r, head.g, head.b);

        prevStep = step;
    }

    /* ---------- commit to strip ---------- */
    if (ledsDirty) {

        if (!clock::usingExt) {
            /* internal-clock mode – safe to block right now */
            strip.show();
            ledsDirty = false;
        }
        else {
            /* external sync: defer the blocking call until the exact
               instant a new step has *already* arrived → we piggy-back
               on the gap we know is safe (Option 1 throttle).           */
        }
    }
}

