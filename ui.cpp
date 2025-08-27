#include "ui.h"
#include "sequencer.h"
#include "hw_inputs.h"
#include "clock_engine.h"
#include <Adafruit_NeoPixel.h>

/* ───────── NeoPixel hardware ───────── */
constexpr uint8_t LED_PIN  = 6;
constexpr uint8_t NUM_LEDS = 16;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ───────── cached previous state ───── */
static uint8_t  prevStep        = 255;   // invalid → forces first paint
static uint8_t  prevLoopLo      = 0;
static uint8_t  prevLoopHi      = 15;
static uint8_t  prevVel[NUM_LEDS] = {0}; // 0=off,1=v1,2=v2,3=Lmark,4=Rmark
static bool     ledsDirty       = false;

/* quick helpers */
inline void px(uint8_t i,uint8_t r,uint8_t g,uint8_t b){
    uint32_t nc = strip.Color(r,g,b);
    if (strip.getPixelColor(i) != nc) { strip.setPixelColor(i,nc); ledsDirty = true; }
}

/* colour palette */
struct RGB { uint8_t r,g,b; };
constexpr RGB CLR_OFF       {  0,  0,  0};
constexpr RGB CLR_OUTSIDE   {  0,  0,  0};
constexpr RGB CLR_MARK_L    {  8,  0,  0};
constexpr RGB CLR_MARK_R    {  8,  0,  0};
constexpr RGB CLR_PLAY_LOOP { 80, 80, 40};
constexpr RGB CLR_PLAY_GEN  { 40, 40, 20};

/* heat-map (velocity-based colour) */
constexpr uint8_t MAX_BRIGHT = 127;
static inline RGB wheel(uint8_t pos){
    uint8_t r,g,b;
    if (pos < 85)      { r = 255 - pos * 3; g = 0; b = pos * 3; }
    else if (pos < 170){ pos -= 85; r = 0; g = pos * 3; b = 255 - pos * 3; }
    else               { pos -= 170; r = pos * 3; g = 255 - pos * 3; b = 0; }
    return {r,g,b};
}
static inline RGB heatColor(uint8_t vel){
    if (!vel) return {0,0,0};
    uint8_t hue = map(vel, 0,127, 85,255);
    RGB base = wheel(hue);
    uint16_t br = map(vel, 0,127, 5, MAX_BRIGHT);
    base.r = base.r * br / 255;
    base.g = base.g * br / 255;
    base.b = base.b * br / 255;
    return base;
}
inline RGB colour_v1() { return heatColor(hw::pots.velocity ); }
inline RGB colour_v2() { return heatColor(hw::pots.accentVel); }

/* ---- loop / marker helpers ---- */
static inline void readBounds(uint8_t &s, uint8_t &e){
    s = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
    e = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
}
static inline void bandLoHi(uint8_t &lo, uint8_t &hi){
    uint8_t s,e; readBounds(s,e);
    if (s <= e) { lo = s; hi = e; } else { lo = e; hi = s; }
}
static inline uint8_t bandLen(){ uint8_t lo,hi; bandLoHi(lo,hi); return uint8_t(hi-lo+1); }

static inline int8_t markerLeftIx(){
    if (bandLen() == 16) return -1;
    uint8_t s,e; readBounds(s,e);
    uint8_t lo,hi; bandLoHi(lo,hi);
    if (s == 0 || e == 0) return -1;
    return int8_t((lo + 15) & 0x0F);
}
static inline int8_t markerRightIx(){
    if (bandLen() == 16) return -1;
    uint8_t s,e; readBounds(s,e);
    uint8_t lo,hi; bandLoHi(lo,hi);
    if (e == 15 || s == 15) return -1;
    return int8_t((hi + 1) & 0x0F);
}
static inline bool inBand(uint8_t idx, uint8_t lo, uint8_t hi){
    return (idx >= lo && idx <= hi);
}

/* ---- full static repaint ---- */
static void paintStaticRegion()
{
    uint8_t lo,hi; bandLoHi(lo,hi);
    int8_t ixL = markerLeftIx();
    int8_t ixR = markerRightIx();

    for (uint8_t i = 0; i < NUM_LEDS; ++i) {
        RGB c;
        if      (i == ixL)         c = CLR_MARK_L;
        else if (i == ixR)         c = CLR_MARK_R;
        else if (inBand(i,lo,hi)) {
            if (seq::vel(i))       c = seq::acc(i) ? colour_v2() : colour_v1();
            else                   c = CLR_OFF;
        } else                     c = CLR_OUTSIDE;

        px(i, c.r, c.g, c.b);

        uint8_t type = 0;
        if      (i == ixL) type = 3;
        else if (i == ixR) type = 4;
        else if (seq::vel(i)) type = seq::acc(i) ? 2 : 1;
        prevVel[i] = type;
    }
    prevLoopLo = lo;
    prevLoopHi = hi;
}

/* ───────── API ───────── */
void ui::init(){
    strip.begin();
    strip.setBrightness(50);
    strip.show(); // clear
}

void ui::refresh()
{
    static bool     prevOn = true;
    static bool     waitingForFirstStep = false;
    static uint8_t  stepAtOn = 0;
    bool on = hw::btnOnOff.level;

    if (!on) {
        paintStaticRegion();
        prevStep = 255;
        ledsDirty = true;
        //if (ledsDirty) { strip.show(); ledsDirty = false; }
        prevOn = on;
        waitingForFirstStep = false;
        return;
    }

    if (on && !prevOn) {
        waitingForFirstStep = true;
        stepAtOn = seq::stepNow();
        paintStaticRegion();
        prevStep = 255;
        ledsDirty = true;
        //if (ledsDirty) { strip.show(); ledsDirty = false; }
        prevOn = on;
        return;
    }
    prevOn = on;

    if (waitingForFirstStep) {
        if (seq::stepNow() == stepAtOn) {
            paintStaticRegion();
            ledsDirty = true;
            //if (ledsDirty) { strip.show(); ledsDirty = false; }
            return;
        }
        waitingForFirstStep = false;
        prevStep = 255;
    }

    bool needFull = false;

    // Immediate repaint when V1/V2 pots move
    static uint8_t prevPotV1 = 255, prevPotV2 = 255;
    if (hw::pots.velocity  != prevPotV1 ||
        hw::pots.accentVel != prevPotV2) {
        needFull  = true;
        prevPotV1 = hw::pots.velocity;
        prevPotV2 = hw::pots.accentVel;
    }

    // Repaint when loop bounds or per-step gate/accent types change
    uint8_t lo,hi; bandLoHi(lo,hi);
    if (lo != prevLoopLo || hi != prevLoopHi) {
        needFull = true;
    } else {
        for (uint8_t i = 0; i < NUM_LEDS; ++i) {
            uint8_t nowType = seq::vel(i) ? (seq::acc(i) ? 2 : 1) : 0;
            if (nowType != prevVel[i]) { needFull = true; break; }
        }
    }

    if (needFull) {
        paintStaticRegion();
        prevStep = 255;
        ledsDirty = true;
        //if (ledsDirty) { strip.show(); ledsDirty = false; }
    }

    static uint8_t lastStepPainted = 255;
    bool ext = clock::usingExt;

    /*if (needFull && ext) {
        uint8_t step = seq::stepNow();
        if (step == lastStepPainted) {
            needFull = false; // coalesce until next step boundary
        } else {
            lastStepPainted = step;
        }
    }*/

    // Playhead / head highlight
    uint8_t step = seq::stepNow();
    bool oneStepLoop = (prevLoopLo == prevLoopHi);

    if (needFull || step != prevStep || oneStepLoop) {
        // erase old
        if (prevStep < NUM_LEDS) {
            int8_t ixL = markerLeftIx();
            int8_t ixR = markerRightIx();
            RGB c;
            if      (prevStep == ixL) c = CLR_MARK_L;
            else if (prevStep == ixR) c = CLR_MARK_R;
            else if (inBand(prevStep, prevLoopLo, prevLoopHi)) {
                if (seq::vel(prevStep)) c = seq::acc(prevStep) ? colour_v2() : colour_v1();
                else                    c = CLR_OFF;
            } else                    c = CLR_OUTSIDE;
            px(prevStep, c.r, c.g, c.b);

            uint8_t type = 0;
            if      (prevStep == ixL) type = 3;
            else if (prevStep == ixR) type = 4;
            else if (seq::vel(prevStep)) type = seq::acc(prevStep) ? 2 : 1;
            prevVel[prevStep] = type;
        }

        // draw new head
        RGB head = seq::vel(step) ? CLR_PLAY_LOOP : CLR_PLAY_GEN;
        if (hw::btnInstant.edge)       head = (RGB{60,60, 0}); // yellow flash
        else if (hw::btnDestruct.edge) head = (RGB{60, 0, 0}); // red flash
        px(step, head.r, head.g, head.b);

        prevStep = step;
        ledsDirty = true;
    }

    // Commit immediately (both internal and external)
    //if (ledsDirty) { strip.show(); ledsDirty = false; }
}

void ui::commitAfterStepIfNeededExt() {
    // Now simply commits right away if something is pending.
    //if (ledsDirty) { strip.show(); ledsDirty = false; }
}

void ui::commitNow(){
    if (ledsDirty) {
        strip.show();
        ledsDirty = false;
    }
}

