#include "ui.h"
#include "sequencer.h"
#include "hw_inputs.h"
#include <Adafruit_NeoPixel.h>

/* ───────── NeoPixel hardware ───────── */
constexpr uint8_t LED_PIN   = 6;      // same as your old build
constexpr uint8_t NUM_LEDS  = 16;
static Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ───────── cached previous state ───── */
static uint8_t  prevStep      = 255;      // invalid → forces first paint
static uint8_t  prevLoopMin   = 0;
static uint8_t  prevLoopMax   = 0;
static uint8_t  prevVel[NUM_LEDS] = {0};

/* quick helpers */
inline void px(uint8_t i,uint8_t r,uint8_t g,uint8_t b){
    strip.setPixelColor(i,strip.Color(r,g,b));
}

/* colour palette (tweak to taste) */
struct RGB { uint8_t r,g,b; };
constexpr RGB CLR_OFF      {  0,  0,  0};
constexpr RGB CLR_OUTSIDE  { 16,  0,  0};     // dim red
constexpr RGB CLR_INSIDE   {  0, 20,  0};     // dim green
constexpr RGB CLR_PLAY_LOOP{  0,  0, 40};     // blue
constexpr RGB CLR_PLAY_GEN { 40,  0, 40};     // magenta

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

static void paintStaticRegion()
{
    /* loop range */
    uint8_t lo = hw::pots.loopStart-1;        // 0-15
    uint8_t hi = hw::pots.loopEnd  -1;
    if (lo > hi) { uint8_t tmp = lo; lo = hi; hi = tmp; }

    for(uint8_t i=0;i<NUM_LEDS;i++){
        RGB c = (i<lo || i>hi) ? CLR_OUTSIDE
                               : (seq::vel(i)? CLR_INSIDE : CLR_OFF);
        px(i,c.r,c.g,c.b);
        prevVel[i] = seq::vel(i);
    }
    prevLoopMin = lo;
    prevLoopMax = hi;
}

void ui::refresh()
{
    /* flash on edges ------------------------------------------------ */
//if(hw::btnCycleL.edge) flashLed(3, {0,60,0});       // green
//if(hw::btnCycleR.edge) flashLed(5, {0,60,0});
//if(hw::btnReset .edge) flashLed(4, {0, 0,60});      // blue

    
    /* 1. detect whether static region must be repainted ── */
    bool needFull = false;

    uint8_t lo = hw::pots.loopStart - 1;
    uint8_t hi = hw::pots.loopEnd   - 1;
    if (lo > hi) { uint8_t t = lo; lo = hi; hi = t; }   // swap for reverse

    if (lo != prevLoopMin || hi != prevLoopMax) {
        needFull = true;                 /* pots moved → repaint band   */
    } else {
        for (uint8_t i = 0; i < NUM_LEDS; ++i){
            if (seq::vel(i) != prevVel[i]) { needFull = true; break; }
        }
    }
    if (needFull) {
        paintStaticRegion();
        prevStep = 255;                  /* force head redraw too      */
    }

    /* 2. head / play-cursor ───────────────────────────────── */
    uint8_t step = seq::stepNow();           // 0-15
    bool oneStepLoop = (prevLoopMin == prevLoopMax);

    if (needFull || step != prevStep || oneStepLoop) {
        /* erase old -------- */
        if (prevStep < NUM_LEDS) {
            uint8_t i = prevStep;
            RGB c = (i < prevLoopMin || i > prevLoopMax)
                    ? CLR_OUTSIDE
                    : (seq::vel(i) ? CLR_INSIDE : CLR_OFF);
            px(i,c.r,c.g,c.b);
        }

        /* draw new -------- */
        RGB head = seq::vel(step) ? CLR_PLAY_LOOP : CLR_PLAY_GEN;
        if (hw::btnInstant.edge)       head = {60,60, 0};   // yellow flash
        else if (hw::btnDestruct.edge) head = {60, 0, 0};   // red flash
        px(step, head.r, head.g, head.b);

        prevStep = step;
    }

    strip.show();
}

