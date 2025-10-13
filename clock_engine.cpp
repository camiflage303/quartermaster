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

  // Shared with ISR
  volatile uint8_t  pulsesPerStepISR = 6; // PPQN / divider (e.g. 24,12,6,...)
  volatile uint8_t  extTickCtr   = 0;     // ticks within step
  volatile bool     extStepFlag  = false; // one-shot: "service a step now"
  volatile bool     transportRun = false;

  // Internal clock state
  uint8_t       intTickCtr = 0;
  unsigned long lastIntUs  = 0;

  inline void flagStep() { extTickCtr = 0; extStepFlag = true; }
}

// public
volatile bool     clock::usingExt = false;
uint16_t          clock::bpm      = 120;
volatile unsigned long clock::lastF8Us     = 0;
volatile unsigned long clock::f8IntervalUs = 0;

// MIDI ISR callbacks
static void isrClock()
{
  if (!transportRun) return;
  unsigned long now = micros();
  static unsigned long prev = 0;
  clock::lastF8Us = now;
  if (prev) clock::f8IntervalUs = now - prev;
  prev = now;

  if (++extTickCtr >= pulsesPerStepISR) {
    extTickCtr = 0;
    if (clock::usingExt) extStepFlag = true;
  }
}
static void isrStart()   { transportRun = true; extTickCtr = 0; if (clock::usingExt) extStepFlag = true; }
static void isrContinue(){ transportRun = true; }
static void isrStop()    { transportRun = false; extStepFlag = false; MIDI.sendControlChange(123,0,1); }

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
  extTickCtr   = 0;
  extStepFlag  = false;
  intTickCtr   = 0;
  interrupts();
  lastIntUs = micros();
}

void clock::forceStop(){
  noInterrupts();
  transportRun = false;
  extStepFlag  = false;
  interrupts();
}

void clock::service()
{
  // 0) snapshot UI knobs cheaply
  usingExt = hw::btnExtMidi.level;
  bool on  = hw::btnOnOff.level;
  bpm      = hw::pots.bpm;

  // pulses-per-step updates (atomic write)
  static uint8_t prevPPS = 6;
  uint8_t uiPPS = constrain(hw::pots.pulsesPerStep, 1, 96);
  if (uiPPS != prevPPS) {
    noInterrupts();
    pulsesPerStepISR = uiPPS;
    extTickCtr = 0; intTickCtr = 0;
    interrupts();
    prevPPS = uiPPS;
  }

  // Transport OFF ⇒ pause everything locally (don’t force RUN=TRUE here)
  if (!on) {
    // ensure we’re not stepping
    noInterrupts(); extStepFlag = false; interrupts();
    return;
  }

  if (usingExt) {
    bool fire = false;
    noInterrupts();
    // Only step when the external transport is actually running
    if (transportRun && extStepFlag) { extStepFlag = false; fire = true; }
    interrupts();

    if (fire) seq::nextStep();
    return;
  }

  // Internal clock
  unsigned long now = micros();
  const float usPerQuarter = 60.0f / bpm * 1e6f;
  const float usPerTick    = usPerQuarter / PPQN;

  if (now - lastIntUs > 2 * usPerTick) lastIntUs = now; // saturate drift
  if (now - lastIntUs >= usPerTick) {
    lastIntUs += usPerTick;
    MIDI.sendRealTime(midi::Clock);
    if (++intTickCtr >= pulsesPerStepISR) {
      intTickCtr = 0;
      seq::nextStep();
    }
  }
}
