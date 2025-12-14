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
  constexpr uint8_t kExtPhaseTicks = 1;

  // ---------------- Shared with ISR ----------------
  volatile uint8_t  pulsesPerStepISR = 6; // clocks per sequencer step (24/4=6 → 16ths)
  volatile uint8_t  extTickCtr   = 0;     // ticks within current step (0..pulsesPerStepISR-1)
  volatile bool     extStepFlag  = false; // one-shot: "service a step now"
  volatile bool     transportRun = false;

  // Phase-delay state: arm at boundary, count down ticks, then raise step flag once.
  volatile uint8_t  extPhaseWait = 0;
  volatile bool     extPhaseArmed = false;

  // Request an immediate step on MIDI Start (handled in main loop, not ISR)
  volatile bool     extStartKick = false;

  // Queue next subdivision change; 0 = no pending change (boundary-safe change)
  volatile uint8_t  nextPPSISR   = 0;

  // Quantize-to-Start queue (applied exactly in isrStart if nonzero)
  volatile uint8_t  queuedPPSOnStart = 0;

  // ---------- Realm quantization ----------
  // Realm grid counters (modulo MIDI clocks)
  volatile uint8_t f8Mod6 = 0;  // 16th grid (6 clocks per 16th)
  volatile uint8_t f8Mod8 = 0;  // triplet grid (8 clocks per triplet-16th)

  // Queued "apply PPS" on a specific realm boundary
  enum : uint8_t { GRID_NONE=0, GRID_SIXTEENTH=1, GRID_TRIPLET=2 };
  volatile uint8_t queuedPPSOnGrid = 0;
  volatile uint8_t queuedGridKind  = GRID_NONE;

  // Internal clock state
  uint8_t       intTickCtr = 0;
  unsigned long lastIntUs  = 0;
}

// ---------- public state ----------
volatile bool           clock::usingExt = false;
uint16_t                clock::bpm      = 120;
volatile unsigned long  clock::lastF8Us     = 0;
volatile unsigned long  clock::f8IntervalUs = 0;

// Set in main.ino
uint8_t clock::midiChannel;

bool    clock::quantizeDivChangeToGrid  = true;
bool    clock::quantizeDivChangeToStart = false;
uint8_t clock::beatsPerBar              = 4;

volatile bool clock::stepJustFired      = false;
volatile bool clock::pendingAllNotesOff = false;

// ---------- MIDI ISR callbacks ----------
static void isrClock()
{
  if (!transportRun) return;

  unsigned long now = micros();
  static unsigned long prev = 0;
  clock::lastF8Us = now;
  if (prev) clock::f8IntervalUs = now - prev;
  prev = now;

  // Advance realm counters each F8 (for grid-quantized division changes)
  f8Mod6 = (uint8_t)((f8Mod6 + 1) % 6);
  f8Mod8 = (uint8_t)((f8Mod8 + 1) % 8);

  // If a realm-quantized PPS change is queued, apply it exactly on the realm boundary
  if (queuedPPSOnGrid && clock::usingExt) {
    const bool hit16th = (queuedGridKind == GRID_SIXTEENTH) && (f8Mod6 == 0);
    const bool hitTrip = (queuedGridKind == GRID_TRIPLET)   && (f8Mod8 == 0);
    if (hit16th || hitTrip) {
      pulsesPerStepISR = queuedPPSOnGrid;

      // Hard boundary at this tick: restart step phase
      extTickCtr = 0;

      // Arm the phase-delayed step so the next step fires kExtPhaseTicks after this boundary
      uint8_t phase = (kExtPhaseTicks < pulsesPerStepISR) ? kExtPhaseTicks : 0;
      extPhaseWait  = phase;
      extPhaseArmed = true;

      // Clear queues
      queuedPPSOnGrid = 0;
      queuedGridKind  = GRID_NONE;
      nextPPSISR = 0; // avoid double-apply
    }
  }

  // Advance F8 counter and wrap at subdivision boundary
  if (++extTickCtr >= pulsesPerStepISR) {
    extTickCtr = 0;

    // Adopt new subdivision exactly at the boundary (non-quantized path)
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
      --extPhaseWait;
    } else {
      extStepFlag   = true;
      extPhaseArmed = false;
    }
  }
}

static void isrStart()
{
  transportRun = true;

  // Reset tick counter to boundary and reset realm counters (align downbeat)
  extTickCtr = 0;
  f8Mod6 = 0;
  f8Mod8 = 0;

  // Apply quantized PPS change exactly on Start (if requested)
  if (queuedPPSOnStart) {
    pulsesPerStepISR = queuedPPSOnStart;
    queuedPPSOnStart = 0;
  }

  // Schedule NEXT step for a full subdivision after start, plus phase.
  uint8_t phase = (kExtPhaseTicks < pulsesPerStepISR) ? kExtPhaseTicks : 0;
  extPhaseWait  = (uint8_t)((pulsesPerStepISR + phase) % 255);
  extPhaseArmed = true;

  // Ask main loop to emit an immediate step
  extStartKick = true;
}

static void isrContinue()
{
  transportRun = true;
}

static void isrStop()
{
  transportRun  = false;
  extStepFlag   = false;
  extPhaseArmed = false;
  extStartKick  = false;

  // Defer All Notes Off to main loop on user-selected channel
  clock::pendingAllNotesOff = true;
}

// ---------- public API ----------
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
  nextPPSISR    = 0;
  queuedPPSOnStart = 0;
  queuedPPSOnGrid  = 0;
  queuedGridKind   = GRID_NONE;
  f8Mod6 = 0;
  f8Mod8 = 0;
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

  pendingAllNotesOff = true;
}

void clock::service()
{
  // 0) snapshot UI knobs cheaply
  usingExt = hw::btnExtMidi.level;
  bool on  = hw::btnOnOff.level;
  bpm      = hw::pots.bpm;

  // 0a) Master transport (send Start/Stop) ONLY when internal master
  static bool prevOn = false;
  if (!usingExt) {
    if (on && !prevOn) {
      MIDI.sendRealTime(midi::Start);
      // reset internal clock phase for tight start
      intTickCtr = 0;
      lastIntUs = micros();
      transportRun = true;
    }
    if (!on && prevOn) {
      MIDI.sendRealTime(midi::Stop);
      transportRun = false;
      pendingAllNotesOff = true;
    }
  }
  prevOn = on;

  // pulses-per-step from the UI (note division selector)
  static uint8_t prevUiPPS = 6;
  uint8_t uiPPS = (uint8_t)constrain(hw::pots.pulsesPerStep, 1, 96);

  auto isTripletPPS = [](uint8_t p)->bool {
    return (p == 16) || (p == 8) || (p == 4) || (p == 2);
  };

  if (uiPPS != prevUiPPS) {
    noInterrupts();
    if (usingExt && clock::quantizeDivChangeToGrid) {
      const bool toTrip = isTripletPPS(uiPPS);
      queuedPPSOnGrid = uiPPS;
      queuedGridKind  = toTrip ? GRID_TRIPLET : GRID_SIXTEENTH;

      queuedPPSOnStart = 0;
      nextPPSISR       = 0;
    } else if (usingExt && clock::quantizeDivChangeToStart) {
      queuedPPSOnStart = uiPPS;
      nextPPSISR       = 0;
      queuedPPSOnGrid  = 0;
      queuedGridKind   = GRID_NONE;
    } else {
      // Internal master, or external without quantize: apply at next boundary
      nextPPSISR       = uiPPS;
      queuedPPSOnStart = 0;
      queuedPPSOnGrid  = 0;
      queuedGridKind   = GRID_NONE;
    }
    interrupts();
    prevUiPPS = uiPPS;
  }

  // Transport OFF ⇒ pause everything locally
  if (!on) {
    noInterrupts();
    extStepFlag   = false;
    extPhaseArmed = false;
    extStartKick  = false;
    interrupts();
    return;
  }

  // ---------------- External clock path ----------------
  if (usingExt) {
    bool fireStart = false;
    bool fireStep  = false;

    noInterrupts();
    if (extStartKick) { extStartKick = false; fireStart = true; }
    if (transportRun && extStepFlag) { extStepFlag = false; fireStep = true; }
    interrupts();

    if (fireStart) {
      seq::armReset();
      seq::nextStep();
      stepJustFired = true;
      return;
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

  if (now - lastIntUs > 2 * usPerTick) lastIntUs = now;
  if (now - lastIntUs >= usPerTick) {
    lastIntUs += usPerTick;
    MIDI.sendRealTime(midi::Clock);

    // Apply queued PPS change exactly at boundary
    if (++intTickCtr >= pulsesPerStepISR) {
      intTickCtr = 0;

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
