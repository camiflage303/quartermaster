/*  clock_engine.cpp  ─────────────────────────────────────────────────────
    External-MIDI and internal-tempo clock engine
    ---------------------------------------------------------------------- */

#include "clock_engine.h"
#include "hw_inputs.h"     // buttons & pots
#include "sequencer.h"
#include <MIDI.h>

/* ---- exact extern declared by the FortySevenEffects library ---- */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

/* ───────── constants ─────────────────────────────────────────────────── */
constexpr uint8_t PPQN      = 24;   // MIDI clocks per quarter-note
volatile uint8_t  pulsesPerStepISR = 6;   // was STEP_DIV const

/* ───────── state shared with ISRs  ───────────────────────────────────── */
namespace {
    /* external-clock path */
    volatile uint8_t  extTickCtr   = 0;   // clocks inside current step
    volatile bool     extStepFlag  = false;
    volatile bool     transportRun = false;  // set by Start / Stop

    /* internal-clock path (only used in main loop) */
    uint8_t intTickCtr = 0;

    /* microsecond phase accumulator for internal clock */
    unsigned long lastIntUs = 0;
}

/* ───────── public globals ────────────────────────────────────────────── */
bool     clock::usingExt = false;   // true ⇢ follow external clock
uint16_t clock::bpm      = 120;

/* ───────── helpers ──────────────────────────────────────────────────── */
static inline void raiseStepFlag()
{
    extTickCtr  = 0;
    extStepFlag = true;             // main loop will service this
}

/* ───────── MIDI ISR callbacks ───────────────────────────────────────── */
static void isrClock()
{
    if (!clock::usingExt || !transportRun) return;

    if (++extTickCtr >= pulsesPerStepISR) {
        raiseStepFlag();
    }
}

static void isrStart()    // 0xFA
{
    transportRun = true;
    raiseStepFlag();      // beat-1 right away
}
static void isrContinue() { transportRun = true; }
static void isrStop()     { 
    transportRun = false; 
    extStepFlag = false; 
    MIDI.sendControlChange(123, 0, 1);
}

/* ───────── init ─────────────────────────────────────────────────────── */
void clock::init()
{
    MIDI.setHandleClock   (isrClock);
    MIDI.setHandleStart   (isrStart);
    MIDI.setHandleContinue(isrContinue);
    MIDI.setHandleStop    (isrStop);
    MIDI.begin(MIDI_CHANNEL_OMNI);

    lastIntUs = micros();
}

/*  OPTIONAL: expose two tiny helpers for other modules  */
void clock::hardResetCounters()
{
    noInterrupts();
    extTickCtr  = 0;
    extStepFlag = false;
    intTickCtr  = 0;
    interrupts();
}
void clock::forceStop()   // call if you need an emergency kill
{
    noInterrupts();
    transportRun = false;
    extStepFlag  = false;
    interrupts();
}

/* ───────── service() – call every loop() ────────────────────────────── */
void clock::service()
{
    /* 0. Snapshot panel controls (cheap; does NOT block interrupts) */
    usingExt = hw::btnExtMidi.level;
    bool on  = hw::btnOnOff.level;
    bpm      = hw::pots.bpm;

    /* -------- update pulses-per-step (pot) ------------- */
    uint8_t uiPPS = constrain(hw::pots.pulsesPerStep, 1, 24);  // you choose range
    static uint8_t prevPPS = 6;

    if (uiPPS != prevPPS) {                // pot moved → copy to ISR
        noInterrupts();
        pulsesPerStepISR = uiPPS;          // single 8-bit store (atomic)
        extTickCtr  = 0;                   // avoid half-step after change
        intTickCtr  = 0;
        interrupts();
        prevPPS = uiPPS;
    }

    /* If the user flipped Ext-Sync, flush the external counters so
       we never reuse stale ticks when we switch modes. */
    static bool prevUsingExt = usingExt;
    if (usingExt != prevUsingExt) {
        hardResetCounters();
        lastIntUs = micros();
        prevUsingExt = usingExt;
    }

    /* Transport OFF ⇒ everything frozen except MIDI parser in loop() */
    if (!on) { transportRun = false; return; }

    /* =============================================================
       A.  External-clock branch
       ============================================================= */
    if (usingExt)
    {
        bool fire = false;

        /* grab & clear the flag with interrupts masked for 1-2 µs */
        noInterrupts();
        if (extStepFlag) {
            extStepFlag = false;
            fire = true;
        }
        interrupts();

        if (fire && transportRun) seq::nextStep();
        return;                       // no internal clock math
    }

    /* =============================================================
       B.  Internal-clock branch
       ============================================================= */
    unsigned long now = micros();
    const float usPerQuarter = 60.0f / bpm * 1e6f;
    const float usPerTick    = usPerQuarter / PPQN;

    if (now - lastIntUs >= usPerTick)
    {
        lastIntUs += usPerTick;           // maintain phase
        MIDI.sendRealTime(midi::Clock);   // keep downstream gear happy

        if (++intTickCtr >= pulsesPerStepISR) {
            intTickCtr = 0;
            seq::nextStep();
        }
    }
}
