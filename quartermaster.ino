// --- main.ino -------------------------------------------------
// Last modified: 2026-05-31
// ========= USER SETTINGS =========
#define QM_MIDI_CHANNEL 1   // ← set your global MIDI channel here (1..16)
// =================================

#include <MIDI.h>
#include "hw_inputs.h"
#include "clock_engine.h"
#include "sequencer.h"
#include "ui.h"

MIDI_CREATE_DEFAULT_INSTANCE();

void setup() {
  // Authoritative global MIDI channel for this device
  clock::midiChannel = (uint8_t)constrain(QM_MIDI_CHANNEL, 1, 16);

  // MIDI baud for DIN
  Serial.begin(31250);

  // Random seed for engines
  randomSeed(analogRead(A7));

  // Hardware + UI init
  hw::initPins();
  hw::scanInputs();      // prime the first read
  seq::armReset();
  clock::init();
  seq::init();
  ui::init();
}

void loop()
{
  // --- 0) Pump MIDI first: drain all pending bytes so callbacks stay timely
  while (MIDI.read()) {
    // Clock / Start / Stop handlers run in callbacks in clock_engine.cpp
  }

  // If the clock engine requested an All Notes Off (e.g., on Stop), send it now.
  if (clock::pendingAllNotesOff) {
    MIDI.sendControlChange(123, 0, clock::midiChannel);
    // Clear the flag atomically
    noInterrupts();
    clock::pendingAllNotesOff = false;
    interrupts();
  }

  // --- 1) Scan hardware (banked; cheap)
  hw::scanInputs();

  // --- 2) Immediate actions (no long work here)
  if (hw::btnInstant.edge) {
    seq::regenerateAll(hw::pots.instChance);
    seq::commitProspect();
    ui::refresh();                 // immediate flush handled in ui.cpp (rate limited there)
    hw::btnInstant.edge = false;
  }

  if (hw::btnCopy.edge) {          // Commit prospective layer via Non-Destruct button
    seq::commitProspect();
  }

  /* ---- Performance buttons ---- */
  if (hw::btnCycleL.edge) {
    seq::rotateAllLeft();
    hw::btnCycleL.edge = false;
  }

  if (hw::btnCycleR.edge) {
    seq::rotateAllRight();
    hw::btnCycleR.edge = false;
  }

  if (hw::btnReset.edge) {
    seq::armReset();               // will take effect on next tick
    hw::btnReset.edge = false;
  }

  // --- 3) Transport ON/OFF handling
  static bool prevOn = false;
  bool on = hw::btnOnOff.level;

  /* rising edge (OFF → ON) */
  if ( on && !prevOn ) {
    // Don’t pre-advance; let nextStep render the loop-start step
    seq::armReset();
  }

  /* falling edge (ON → OFF) */
  if (!on &&  prevOn ) {
    // Local All Notes Off when user stops from panel
    MIDI.sendControlChange(123, 0, clock::midiChannel);

    // Park the playhead at the last step of the current loop
    uint8_t target = hw::pots.loopEnd ? hw::pots.loopEnd - 1 : 15;
    seq::forceStep(target);
  }

  /* run clock only while ON */
  if (on) {
    clock::service();
  }

  // --- 4) Refresh UI once per loop (DotStar flush is rate-limited in ui.cpp)
  ui::refresh();

  prevOn = on;

}
