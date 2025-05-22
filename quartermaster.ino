#include <MIDI.h>
#include "hw_inputs.h"
#include "clock_engine.h"
#include "sequencer.h"
#include "ui.h"

void setup(){
    Serial.begin(31250);
    hw::initPins();
    hw::scanInputs();
    seq::forceStep(hw::pots.loopStart - 1);
    clock::init();
    seq::init();
    ui::init();
}


void loop()
{
    hw::scanInputs();

    if (hw::btnInstant.edge) {                         //   BTN_INST
        seq::regenerateAll(hw::pots.instChance);       //   make 16 new prospect notes
        seq::commitProspect();                         //   and commit at once
    }

    if (hw::btnCopy.edge) {                            //   BTN_NONDEST
        seq::commitProspect();                         //   promote last 16 temp steps
    }

    /* ----------------------------------------------
   Immediate performance buttons
   ---------------------------------------------- */
    if (hw::btnCycleL.edge) {
        seq::rotateAllLeft();
        //flashLed(3, {0,60,0});            // same green wink
    }

    if (hw::btnCycleR.edge) {
        seq::rotateAllRight();
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
        uint8_t target = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        seq::forceStep(target);            // jump to first step *before* clock runs
    }

    /* ---------- falling edge  (ON → OFF) -------------------- */
    if (!on &&  prevOn ) {
        uint8_t target = hw::pots.loopEnd ? hw::pots.loopEnd - 1 : 15;
        seq::forceStep(target);            // park at last step
    }

    /* ---------- run clock only while ON --------------------- */
    if (on) {
        clock::usingExt = hw::btnExtMidi.level;
        clock::service();
    }

    ui::refresh();
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
    Serial.print(F("Start="));  Serial.print(prevStart);
    Serial.print(F(" End="));   Serial.print(prevEnd);
    Serial.print(F(" PProb1="));Serial.print(prevPP);
    Serial.print(F(" Destruct="));Serial.print(hw::btnDestruct.level);
    if (pendingEdge) Serial.print(F("  [EDGE]"));
    Serial.println();

    pendingEdge = false;   // clear after showing*/
}