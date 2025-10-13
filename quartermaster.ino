#include <MIDI.h>
#include "hw_inputs.h"
#include "clock_engine.h"
#include "sequencer.h"
#include "ui.h"

MIDI_CREATE_DEFAULT_INSTANCE();

void setup(){
    Serial.begin(31250);
    hw::initPins();
    hw::scanInputs();      // prime the first read
    seq::armReset();
    clock::init();
    seq::init();
    ui::init();
}

void loop()
{
    hw::scanInputs();

    // Drain all pending MIDI bytes this pass so callbacks stay timely
    while (MIDI.read()) { /* handlers (F8/Start/Stop) run in callbacks */ }

    // Immediate actions
    if (hw::btnInstant.edge) {
        seq::regenerateAll(hw::pots.instChance);
        seq::commitProspect();
        ui::refresh();                 // immediate flush handled in ui.cpp
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

    static bool prevOn = false;
    bool on = hw::btnOnOff.level;

    /* rising edge (OFF → ON) */
    if ( on && !prevOn ) {
        // Don’t pre-advance; let nextStep render the loop-start step
        seq::armReset();
    }

    /* falling edge (ON → OFF) */
    if (!on &&  prevOn ) {
        MIDI.sendControlChange(123, 0, 1);    // all notes off
        uint8_t target = hw::pots.loopEnd ? hw::pots.loopEnd - 1 : 15;
        seq::forceStep(target);               // park at last step
    }

    /* run clock only while ON */
    if (on) {
        clock::service();
    }

    // Refresh UI every loop; it flushes immediately when dirty
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
