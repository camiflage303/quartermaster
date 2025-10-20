// ---------------- clock_engine.cpp (updated) -------------------------------
#include "clock_engine.h"
#include "hw_inputs.h"
#include "sequencer.h"
#include <MIDI.h>

extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

namespace {
  constexpr uint8_t PPQN = 24;

  // Delay the external step trigger by this many MIDI clocks (F8)
  // AFTER each subdivision boundary. 0 = no delay (step at boundary).
  // 1 = step on the first clock AFTER the boundary.
  constexpr uint8_t kExtPhaseTicks = 1;

  // ---------------- Shared with “ISR” callbacks (MIDI.read-driven) --------
  volatile uint8_t  pulsesPerStepISR = 6; // PPQN / divider (e.g. 24,12,6,...)
  volatile uint8_t  extTickCtr   = 0;     // ticks within step (0..pulsesPerStepISR-1)
  volatile bool     extStepFlag  = false; // one-shot: "service a step now"
  volatile bool     transportRun = false;

  // Phase-delay state: arm at boundary, count down ticks, then raise step flag once.
  volatile uint8_t  extPhaseWait = 0;     // remaining clocks to wait before firing
  volatile bool     extPhaseArmed = false;

  // Request an immediate step on MIDI Start (handled in main loop, not ISR)
  volatile bool     extStartKick = false;

  // queue next subdivision; 0 = no pending change
  volatile uint8_t  nextPPSISR   = 0;

  // Internal clock state
  uint8_t       intTickCtr = 0;
  unsigned long lastIntUs  = 0;
}

// public
volatile bool     clock::usingExt = false;
uint16_t          clock::bpm      = 120;
volatile unsigned long clock::lastF8Us     = 0;
volatile unsigned long clock::f8IntervalUs = 0;

// NEW: public flags
volatile bool clock::stepJustFired      = false;
volatile bool clock::pendingAllNotesOff = false;

// MIDI “ISR” callbacks (invoked from MIDI.read())
static void isrClock()
{
  if (!transportRun) return;

  unsigned long now = micros();
  static unsigned long prev = 0;
  clock::lastF8Us = now;
  if (prev) clock::f8IntervalUs = now - prev;
  prev = now;

  // Advance F8 counter and wrap at subdivision boundary
  if (++extTickCtr >= pulsesPerStepISR) {
    extTickCtr = 0;

    // Adopt new subdivision exactly at the boundary
    if (nextPPSISR && nextPPSISR != pulsesPerStepISR) {
      pulsesPerStepISR = nextPPSISR;
      nextPPSISR = 0;
    }

    // Arm a delayed step fire for external mode; do NOT change the period.
    uint8_t phase = (kExtPhaseTicks < pulsesPerStepISR) ? kExtPhaseTicks : 0;
    extPhaseWait  = phase;
    extPhaseArmed = true;
  }

  // If using external clock, handle the phase-delayed step flagging
  if (clock::usingExt && extPhaseArmed) {
    if (extPhaseWait > 0) {
      --extPhaseWait;                  // wait N clocks after boundary
    } else {
      extStepFlag  = true;             // fire exactly once
      extPhaseArmed = false;
    }
  }
}

static void isrStart()
{
  transportRun = true;

  // Reset tick counter to boundary.
  extTickCtr = 0;

  // Schedule NEXT step for a full subdivision after start, plus phase.
  // (We will fire ONE immediate step in the main loop via extStartKick.)
  uint8_t phase = (kExtPhaseTicks < pulsesPerStepISR) ? kExtPhaseTicks : 0;
  // Wait a full pulsesPerStep + phase before the next flagged step:
  extPhaseWait  = (uint8_t)((pulsesPerStepISR + phase) % 255); // safe wrap
  extPhaseArmed = true;

  // Ask main loop to emit an immediate step (keeps first beat tight with master)
  extStartKick = true;
}

static void isrContinue()
{
  transportRun = true;
  // Do not force a boundary/phase jump here; most masters resume mid-cycle.
}

static void isrStop()
{
  // Defer any MIDI sends to main loop to avoid starving the RX path.
  transportRun        = false;
  extStepFlag         = false;
  extPhaseArmed       = false;
  extStartKick        = false;
  clock::pendingAllNotesOff = true;   // main loop will send CC123
}

void clock::init()
{
  MIDI.setHandleClock   (isrClock);
  MIDI.setHandleStart   (isrStart);
  MIDI.setHandleContinue(isrContinue);
  MIDI.setHandleStop    (isrStop);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  lastIntUs = micros();
}

void clock::hardResetCounters(){
  noInterrupts();
  extTickCtr    = 0;
  extStepFlag   = false;
  intTickCtr    = 0;
  extPhaseWait  = 0;
  extPhaseArmed = false;
  extStartKick  = false;
  interrupts();
  lastIntUs = micros();
}

void clock::forceStop(){
  noInterrupts();
  transportRun  = false;
  extStepFlag   = false;
  extPhaseArmed = false;
  extStartKick  = false;
  interrupts();
}

void clock::service()
{
  // 0) snapshot UI knobs cheaply
  usingExt = hw::btnExtMidi.level;
  bool on  = hw::btnOnOff.level;
  bpm      = hw::pots.bpm;

  // pulses-per-step updates: queue and apply at next step boundary
  static uint8_t prevUiPPS = 6;
  uint8_t uiPPS = constrain(hw::pots.pulsesPerStep, 1, 96);
  if (uiPPS != prevUiPPS) {
    noInterrupts();
    nextPPSISR = uiPPS;     // defer until boundary
    interrupts();
    prevUiPPS = uiPPS;
  }

  // Transport OFF ⇒ pause everything locally
  if (!on) {
    noInterrupts(); extStepFlag = false; extPhaseArmed = false; extStartKick = false; interrupts();
    return;
  }

  if (usingExt) {
    bool fireStart = false;
    bool fireStep  = false;

    noInterrupts();
    if (extStartKick) { extStartKick = false; fireStart = true; }
    // Only step when the external transport is actually running
    if (transportRun && extStepFlag) { extStepFlag = false; fireStep = true; }
    interrupts();

    // Immediate first step on Start (and reset sequencer position cleanly)
    if (fireStart) {
      seq::armReset();      // start loop at current LS/LE
      seq::nextStep();      // fire first step immediately
      stepJustFired = true; // let UI defer a flush this loop turn
      return;               // wait for the scheduled next step (full interval later)
    }

    if (fireStep) {
      seq::nextStep();
      stepJustFired = true;
    }
    return;
  }

  // ---------------- Internal clock path ----------------
  unsigned long now = micros();
  const float usPerQuarter = 60.0f / bpm * 1e6f;
  const float usPerTick    = usPerQuarter / PPQN;

  if (now - lastIntUs > 2 * usPerTick) lastIntUs = now; // saturate drift
  if (now - lastIntUs >= usPerTick) {
    lastIntUs += usPerTick;
    MIDI.sendRealTime(midi::Clock);
    if (++intTickCtr >= pulsesPerStepISR) {
      intTickCtr = 0;

      // adopt new subdivision exactly at boundary
      noInterrupts();
      if (nextPPSISR && nextPPSISR != pulsesPerStepISR) {
        pulsesPerStepISR = nextPPSISR;
        nextPPSISR = 0;
      }
      interrupts();

      seq::nextStep();
      stepJustFired = true;
    }
  }
}
