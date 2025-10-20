// --- main.ino (updated) -------------------------------------------------
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
  clock::midiChannel = (uint8_t)constrain(QM_MIDI_CHANNEL, 1, 16);  // ✅ authoritative channel

  Serial.begin(31250);
  randomSeed(analogRead(A7));
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
    while (MIDI.read()) { /* handlers (F8/Start/Stop) run in callbacks */ }

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

    if (hw::btnCopy.edge) {            // Commit prospective layer
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
        MIDI.sendControlChange(123, 0, 1);    // local all notes off on user stop
        uint8_t target = hw::pots.loopEnd ? hw::pots.loopEnd - 1 : 15;
        seq::forceStep(target);               // park at last step
    }

    /* run clock only while ON */
    if (on) {
        clock::service();
    }

    // --- 4) Refresh UI once per loop (DotStar flush is rate-limited in ui.cpp)
    ui::refresh();

    prevOn = on;

    // dbgPrint();   // optional: enable if you want periodic debug prints
}

/* ---------- Optional debug helper ---------- */
void dbgPrint()
{
    static unsigned long lastMs = 0;
    static uint8_t prevStart = 0, prevEnd = 0;
    static uint8_t prevPP    = 0;
    static bool    pendingEdge = false;   // latch Destruct press

    /* latch the edge immediately */
    if (hw::btnDestruct.edge) pendingEdge = true;

    /* only print a few times per second */
    if (millis() - lastMs < 300) return;
    lastMs = millis();

    bool changed = false;

    /* loop-pots */
    if (hw::pots.loopStart != prevStart || hw::pots.loopEnd != prevEnd) {
        prevStart = hw::pots.loopStart;
        prevEnd   = hw::pots.loopEnd;
        changed = true;
    }

    /* first pitch-prob slider */
    if (hw::pots.pitchProb[0] != prevPP) {
        prevPP = hw::pots.pitchProb[0];
        changed = true;
    }

    /* latched Destruct edge */
    if (pendingEdge) changed = true;

    if (!changed) return;

    /* -------- pretty print -------- */
    /*
    Serial.print(F("Start="));  Serial.print(prevStart);
    Serial.print(F(" End="));   Serial.print(prevEnd);
    Serial.print(F(" PProb1="));Serial.print(prevPP);
    Serial.print(F(" Destruct="));Serial.print(hw::btnDestruct.level);
    if (pendingEdge) Serial.print(F("  [EDGE]"));
    Serial.println();
    */

    pendingEdge = false;   // clear after showing
}
