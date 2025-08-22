/*  clock_engine.cpp — F8-driven external clock with in-ISR tick nudge
    - External: MIDI F8 clocks drive a tiny step queue.
    - Immediate first step on Start (FA) or ON-in-Ext (not nudged).
    - Subsequent steps are delayed by N F8 ticks (EXT_NUDGE_TICKS) inside the F8 ISR.
    - Bounded catch-up (max 2 steps per loop).
    - Internal clock path unchanged.
*/

#include "clock_engine.h"
#include "hw_inputs.h"
#include "sequencer.h"
#include <MIDI.h>

/* ---- extern from FortySevenEffects library ---- */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

/* ───────── config ───────── */
// Delay *normal* external steps by this many F8 clocks after the boundary.
// 0 = original behavior; 1 usually fixes the "1 tick early" feel at slow tempos.
#define EXT_NUDGE_TICKS 1

/* ───────── constants ───────── */
constexpr uint8_t PPQN       = 24;   // MIDI clocks per quarter-note
constexpr uint8_t STEP_Q_MAX = 8;    // cap queued external steps

/* ───────── ISR-shared state ───────── */
namespace {
    /* External clock path */
    volatile uint8_t extTickCtr    = 0;     // F8 clocks inside current step
    volatile uint8_t extStepCount  = 0;     // queued step edges (consumed in service)
    volatile bool    transportRun  = false; // set by Start/Continue/Stop
    volatile bool    firstAfterStart = false;  // next step should fire immediately
    volatile uint8_t nudgeCtr      = 0;     // countdown in F8s AFTER a step boundary

    /* Internal clock path */
    uint8_t       intTickCtr   = 0;
    unsigned long lastIntUs    = 0;
    bool          intStartFlag = false;
}

/* ───────── public globals ───────── */
volatile bool     clock::usingExt        = false;   // true ⇢ follow external clock
uint16_t          clock::bpm             = 120;
uint8_t           clock::pulsesPerStep   = 6;       // UI copy
volatile unsigned long clock::lastF8Us   = 0;       // time of last F8 (µs)
volatile unsigned long clock::f8IntervalUs = 0;     // filtered F8 period (µs)

/* separate ISR copy of pulses-per-step (8-bit ⇒ atomic on AVR) */
volatile uint8_t  pulsesPerStepISR = 6;

/* ───────── helpers ───────── */
static inline void zeroExtPhaseAndQueue()
{
    noInterrupts();
    extTickCtr     = 0;
    extStepCount   = 0;
    nudgeCtr       = 0;
    interrupts();
}

/* ───────── MIDI “ISR” callbacks (invoked inside MIDI.read()) ───────── */
static void isrClock()   // F8 (0xF8)
{
    // Track F8 period (IIR average; useful for diagnostics/LED heuristics)
    unsigned long now = micros();
    if (clock::lastF8Us != 0) {
        unsigned long meas = now - clock::lastF8Us;
        clock::f8IntervalUs = clock::f8IntervalUs ? ((clock::f8IntervalUs * 7 + meas) / 8) : meas;
    }
    clock::lastF8Us = now;

    if (!transportRun) return;

    // 1) Advance within-step F8 counter
    if (++extTickCtr >= pulsesPerStepISR) {
        // We've reached the *boundary* for the next sequencer step.
        extTickCtr = 0;

        // If we're using external clock:
        if (clock::usingExt) {
            // For the *first* step after Start/ON, fire immediately (no nudge).
            if (firstAfterStart) {
                firstAfterStart = false;
                if (extStepCount < STEP_Q_MAX) ++extStepCount;   // enqueue now
                nudgeCtr = 0;                                     // clear any residual
            } else {
                // For normal steps: arm a post-boundary nudge countdown in F8 ticks.
                // We do NOT enqueue yet. We wait EXT_NUDGE_TICKS F8s, then enqueue once.
                nudgeCtr = EXT_NUDGE_TICKS;
            }
        } else {
            // Not following external → ignore
            nudgeCtr = 0;
        }
    } else {
        // Between boundaries: if a nudge countdown is active, count it down.
        if (nudgeCtr > 0) {
            if (--nudgeCtr == 0) {
                // The post-boundary nudge delay elapsed → enqueue exactly one step.
                if (clock::usingExt && extStepCount < STEP_Q_MAX) ++extStepCount;
            }
        }
    }
}

static void isrStart()   // FA
{
    transportRun = true;
    noInterrupts();
    extTickCtr      = 0;               // phase-align to beat
    extStepCount    = 0;               // drop stale
    nudgeCtr        = 0;               // cancel any pending nudge
    firstAfterStart = true;            // next step: immediate (no nudge)
    // Queue that immediate first step now:
    if (clock::usingExt && extStepCount < STEP_Q_MAX) ++extStepCount;
    interrupts();
}

static void isrContinue() // FB
{
    transportRun = true;
    // Treat Continue like Start for "immediate resume"
    noInterrupts();
    firstAfterStart = true;
    nudgeCtr        = 0;
    if (clock::usingExt && extStepCount < STEP_Q_MAX) ++extStepCount;
    interrupts();
}

static void isrStop()     // FC
{
    transportRun = false;
    firstAfterStart = false;
    zeroExtPhaseAndQueue();
    MIDI.sendControlChange(123, 0, 1);      // All Notes Off for safety
}

/* ───────── init ───────── */
void clock::init()
{
    MIDI.setHandleClock   (isrClock);
    MIDI.setHandleStart   (isrStart);
    MIDI.setHandleContinue(isrContinue);
    MIDI.setHandleStop    (isrStop);
    MIDI.begin(MIDI_CHANNEL_OMNI);

    lastIntUs = micros();
}

/* optional utility; safe default */
bool clock::safeToBlockForLeds() { return true; }

/* tiny helpers */
void clock::hardResetCounters()
{
    noInterrupts();
    extTickCtr     = 0;
    extStepCount   = 0;
    intTickCtr     = 0;
    nudgeCtr       = 0;
    interrupts();
    lastIntUs = micros();
}

void clock::forceStop()
{
    noInterrupts();
    transportRun   = false;
    firstAfterStart= false;
    extStepCount   = 0;
    extTickCtr     = 0;
    nudgeCtr       = 0;
    interrupts();
}

/* ───────── service() – call from loop() every pass ───────── */
void clock::service()
{
    /* 0) Snapshot panel controls (cheap) */
    usingExt = hw::btnExtMidi.level;
    const bool on = hw::btnOnOff.level;
    bpm      = hw::pots.bpm;

    /* 1) Update pulses-per-step from pot and mirror to ISR */
    uint8_t uiPPS = constrain(hw::pots.pulsesPerStep, 1, 96);
    static uint8_t prevPPS = 6;
    if (uiPPS != prevPPS) {
        noInterrupts();
        pulsesPerStepISR = uiPPS;      // atomic
        extTickCtr   = 0;              // avoid half-step after change
        intTickCtr   = 0;
        nudgeCtr     = 0;
        interrupts();
        clock::pulsesPerStep = uiPPS;
        prevPPS = uiPPS;
    }

    /* 2) Handle switching Ext <-> Int */
    static bool prevUsingExt = usingExt;
    if (usingExt != prevUsingExt) {
        if (usingExt) {
            // TO external: clear internal counters/queue; wait for FA/Continue or ON
            zeroExtPhaseAndQueue();
            firstAfterStart = false;
            transportRun    = false;   // wait for FA/Continue or ON
        } else {
            // TO internal: reset everything; next loop handles internal start
            hardResetCounters();
            firstAfterStart = false;
        }
        prevUsingExt = usingExt;
    }

    /* 3) Transport OFF ⇒ freeze (MIDI parser still runs in loop()) */
    static bool prevOn = false;
    if (!on) { transportRun = false; prevOn = on; return; }

    /* 4) OFF → ON edge */
    if (on && !prevOn) {
        hardResetCounters();
        if (!usingExt) {
            intStartFlag = true;           // internal: start immediately in B-branch
        } else {
            // external: immediate first step like FA
            noInterrupts();
            transportRun    = true;
            firstAfterStart = true;
            nudgeCtr        = 0;
            if (extStepCount < STEP_Q_MAX) ++extStepCount;
            interrupts();
        }
    }
    prevOn = on;

    /* =============================================================
       A) External-clock branch (F8-queued with in-ISR nudge)
       ============================================================= */
    if (usingExt)
    {
        // Process up to 2 queued steps per pass (bounded catch-up)
        uint8_t fires = 0;
        while (fires < 2) {
            bool fire = false;
            noInterrupts();
            if (extStepCount) { --extStepCount; fire = true; }
            interrupts();

            if (fire && transportRun) {
                seq::nextStep();
                ++fires;
            } else {
                break;
            }
        }
        return;  // external branch done
    }

    /* =============================================================
       B) Internal-clock branch (unchanged)
       ============================================================= */
    unsigned long now = micros();
    const float usPerQuarter = 60.0f / bpm * 1e6f;
    const float usPerTick    = usPerQuarter / PPQN;

    if (intStartFlag) {
        intStartFlag = false;
        lastIntUs = now;
        intTickCtr = 0;
        seq::armReset();
        seq::nextStep();
        return;
    }

    // Saturation guard: if we fell behind, drop phase (don’t burst)
    if (now - lastIntUs > 2 * usPerTick) {
        lastIntUs = now;
    }

    if (now - lastIntUs >= usPerTick)
    {
        lastIntUs += usPerTick;           // maintain phase
        MIDI.sendRealTime(midi::Clock);   // echo clock for downstream gear

        if (++intTickCtr >= pulsesPerStepISR) {
            intTickCtr = 0;
            seq::nextStep();
        }
    }
}
