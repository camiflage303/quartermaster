#include <MIDI.h>
#include "hw_inputs.h"
#include "clock_engine.h"
#include "sequencer.h"
#include "ui.h"

MIDI_CREATE_DEFAULT_INSTANCE();

void setup(){
    Serial.begin(31250);
    hw::initPins();
    hw::scanInputs();
    seq::armReset();
    clock::init();
    seq::init();
    ui::init();
}


void loop()
{
    hw::scanInputs();
    // Drain all pending MIDI bytes this pass so callbacks stay timely
    while (MIDI.read()) { /* handlers will run here */ }

    // Do background (non-timing-critical) sequencer work *between* ticks
    //seq::serviceBackground();

    if (hw::btnInstant.edge) {                         
        seq::regenerateAll(hw::pots.instChance);
        seq::commitProspect();
        ui::refresh();
        strip.show();
        hw::btnInstant.edge = false;
    }


    if (hw::btnCopy.edge) {                            //   BTN_COMMIT (global)
        seq::commitProspect();                         //   write the 16-step prospective layer to regular
    }

    /* ----------------------------------------------
   Immediate performance buttons
   ---------------------------------------------- */
    if (hw::btnCycleL.edge) {
        seq::rotateAllLeft();
        hw::btnCycleL.edge = false;
        //flashLed(3, {0,60,0});            // same green wink
    }

    if (hw::btnCycleR.edge) {
        seq::rotateAllRight();
        hw::btnCycleR.edge = false;
        //flashLed(5, {0,60,0});
    }

    if (hw::btnReset.edge) {
        seq::armReset();                  // will take effect on next tick
        //flashLed(4, {0,0,60});            // blue wink
    }

    static bool prevOn = false;
    bool        on     = hw::btnOnOff.level;

    /* ---------- rising edge  (OFF → ON)  ------------------- */
    if ( on && !prevOn ) {
        // Don’t pre-advance; let nextStep render the loop-start step
        seq::armReset();
    }

    /* ---------- falling edge  (ON → OFF) -------------------- */
    if (!on &&  prevOn ) {
        MIDI.sendControlChange(123, 0, 1);    // all notes off
        uint8_t target = hw::pots.loopEnd ? hw::pots.loopEnd - 1 : 15;
        seq::forceStep(target);            // park at last step
    }

    /* ---------- run clock only while ON --------------------- */
    if (on) {
        // usingExt is managed inside clock::service()
        clock::service();
    }

    ui::refresh();
   // main loop() – after ui::refresh():
    static unsigned long lastCommitF8Us = 0;

    /*if (clock::safeToBlockForLeds()) {
        // Only one commit per F8 period to coalesce paints
        if (clock::lastF8Us != lastCommitF8Us) {
            ui::commitNow();                 // this calls strip.show() if ledsDirty
            lastCommitF8Us = clock::lastF8Us;
        }
    }*/

    prevOn = on;

    //dbgPrint();
}

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
    /*Serial.print(F("Start="));  Serial.print(prevStart);
    Serial.print(F(" End="));   Serial.print(prevEnd);
    Serial.print(F(" PProb1="));Serial.print(prevPP);
    Serial.print(F(" Destruct="));Serial.print(hw::btnDestruct.level);
    if (pendingEdge) Serial.print(F("  [EDGE]"));
    Serial.println();*/

    pendingEdge = false;   // clear after showing*/
}