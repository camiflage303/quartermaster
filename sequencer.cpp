#include "sequencer.h"
#include "clock_engine.h"
#include "ui.h"
#include <MIDI.h>

/*  exact extern using the namespace chosen by the library  */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

/* ---------- internal storage ---------- */
namespace {

    static uint8_t weightedRandomSelection(uint8_t len, const uint8_t* w)
    {
        uint16_t total=0; for(uint8_t i=0;i<len;i++) total += w[i];
        if(!total) return 0;
        uint16_t r = random(total);
        for(uint8_t i=0;i<len;i++){ if(r < w[i]) return i; r -= w[i]; }
        return 0;
    }

    constexpr uint8_t kSteps = 16;

    static uint8_t advanceWithin(uint8_t s, uint8_t a, uint8_t b) //bounded advance
    {
        if (a == b) return a;                 // 1-step loop
        if (a < b) {                          // forward
            return (s < b) ? (uint8_t)(s + 1) : a;
        } else {                              // reverse (start > end)
            return (s > b) ? (uint8_t)(s - 1) : a;
        }
    }

    struct Track { uint8_t regularSequence[kSteps]={0}; uint8_t prospectiveSequence[kSteps]={0}; };

    Track trPitch, trVel, trOct, trAcc;

    uint8_t curStep = 0;

    // Latched loop bounds used for timing (sound); UI may show live pots.
    static uint8_t lsLat = 0;   // 0..15
    static uint8_t leLat = 15;  // 0..15
    static volatile bool loopDirty = false; // request to latch at next step edge

    // track last-sounding note to send a lightweight note-off (vel=0)
    static int16_t prevNote = -1;

    inline Track& track(seq::Aspect a){
        switch(a){
            case seq::Aspect::Pitch: return trPitch;
            case seq::Aspect::Vel:   return trVel;
            case seq::Aspect::Oct:   return trOct;
            default:                 return trAcc; // VSel
        }
    }

    /* octave displacement  (-1 / 0 / +1) from per-degree pot (0..127 mapped) */
    static int8_t octaveDisplacement(uint8_t degree)
    {
        uint16_t v = hw::pots.octaveProb[degree];      // 0..128
        if (v <= 2)   return -1;
        if (v >= 126) return +1;

        const int MID = 64, DZ = 8;
        if (v < MID - DZ) {
            uint8_t chance = map((int)v, 0, MID - DZ - 1, 127, 0);
            return (random(128) < chance) ? -1 : 0;
        } else if (v > MID + DZ) {
            uint8_t chance = map((int)v, MID + DZ + 1, 128, 0, 127);
            return (random(128) < chance) ? +1 : 0;
        } else {
            return 0;
        }
    }

    /* simple generators */
    static uint8_t genPitch(){
        return weightedRandomSelection(8, hw::pots.pitchProb);
    }
    static uint8_t genGate (){
        return (random(128) < hw::pots.density); // 0/1
    }
    static uint8_t genOct  (){
        uint8_t deg = trPitch.prospectiveSequence[curStep] & 0x07;
        return (uint8_t)(octaveDisplacement(deg) + 1); // 0,1,2
    }
    static uint8_t genVSel (){
        if (!trVel.prospectiveSequence[curStep]) return 0; // only accent if gated
        return (random(128) < hw::pots.accentChance);      // 0/1
    }
}

/* ---------- public accessors ---------- */
uint8_t seq::stepNow(){ return curStep; }
uint8_t seq::pitch(uint8_t i){ return trPitch.regularSequence[i]; }
uint8_t seq::vel  (uint8_t i){ return trVel  .regularSequence[i]; }
uint8_t seq::oct  (uint8_t i){ return trOct  .regularSequence[i]; }
uint8_t seq::acc  (uint8_t i){ return trAcc  .regularSequence[i]; }

void seq::forceStep(uint8_t s){ curStep = s % 16; }

/* ---------- hooks ---------- */
void seq::serviceBackground() { /* no-op */ }
void seq::markLoopBoundsDirty(){ loopDirty = true; }

/* ---------- init() ---------- */
void seq::init(){
    for(uint8_t i=0;i<kSteps;i++){
        trPitch.regularSequence[i]=0; trVel.regularSequence[i]=1;
        trOct  .regularSequence[i]=1; trAcc  .regularSequence[i]=0;
        trPitch.prospectiveSequence[i]=0; trVel.prospectiveSequence[i]=1;
        trOct  .prospectiveSequence[i]=1; trAcc.prospectiveSequence[i]=0;
    }
    lsLat = 0; leLat = 15; loopDirty = true;
    prevNote = -1;
}

/* ===========================================================
   Fill 16 prospective steps once (lock-aware; does NOT touch regular)
   =========================================================== */
void seq::regenerateAll(uint8_t probability /*0-127*/)
{
    using namespace hw;
    const Aspect order[4] = { Aspect::Pitch, Aspect::Vel, Aspect::Oct, Aspect::VSel };

    for (uint8_t s = 0; s < kSteps; ++s) {
        uint8_t saved = curStep; curStep = s;
        for (uint8_t i = 0; i < 4; ++i) {
            Aspect asp = order[i];
            uint8_t lock = 0;
            switch (asp){
                case Aspect::Pitch: lock = pots.deltaProb[0]; break;
                case Aspect::Vel:   lock = pots.deltaProb[1]; break;
                case Aspect::Oct:   lock = pots.deltaProb[2]; break;
                case Aspect::VSel:  lock = pots.deltaProb[3]; break;
                default: break;
            }
            if (random(128) < lock) continue;                 // locked ⇒ skip
            if (random(128) >= probability) continue;         // chance gate

            uint8_t v = 0;
            switch (asp){
                case Aspect::Pitch: v = genPitch(); break;
                case Aspect::Vel:   v = genGate (); break;
                case Aspect::Oct:   v = genOct  (); break;
                case Aspect::VSel:  v = genVSel (); break;
                default: break;
            }
            track(asp).prospectiveSequence[s] = v;
        }
        curStep = saved;
    }
}

/* ===========================================================
   Promote prospect → regular
   =========================================================== */
void seq::commitProspect()
{
    for (uint8_t s = 0; s < kSteps; ++s) {
        trPitch.regularSequence[s] = trPitch.prospectiveSequence[s];
        trVel  .regularSequence[s] = trVel  .prospectiveSequence[s];
        trOct  .regularSequence[s] = trOct  .prospectiveSequence[s];
        trAcc  .regularSequence[s] = trAcc  .prospectiveSequence[s];
    }
}

/* ===============================================================
   rotation helpers – work on BOTH regular + prospect arrays
   =============================================================== */
static void rotateLeft (Track& T)
{
    uint8_t first = T.regularSequence[0];
    for (uint8_t i = 0; i < kSteps-1; ++i)
        T.regularSequence[i] = T.regularSequence[i+1];
    T.regularSequence[kSteps-1] = first;

    first = T.prospectiveSequence[0];
    for (uint8_t i = 0; i < kSteps-1; ++i)
        T.prospectiveSequence[i] = T.prospectiveSequence[i+1];
    T.prospectiveSequence[kSteps-1] = first;
}

static void rotateRight(Track& T)
{
    uint8_t last = T.regularSequence[kSteps-1];
    for (int8_t i = kSteps-1; i > 0; --i)
        T.regularSequence[i] = T.regularSequence[i-1];
    T.regularSequence[0] = last;

    last = T.prospectiveSequence[kSteps-1];
    for (int8_t i = kSteps-1; i > 0; --i)
        T.prospectiveSequence[i] = T.prospectiveSequence[i-1];
    T.prospectiveSequence[0] = last;
}

/* public wrappers ------------------------------------------------ */
void seq::rotateAllLeft ()
{
    rotateLeft (trPitch); rotateLeft (trVel);
    rotateLeft (trOct  ); rotateLeft (trAcc);
}

void seq::rotateAllRight()
{
    rotateRight(trPitch); rotateRight(trVel);
    rotateRight(trOct  ); rotateRight(trAcc);
}

static bool resetPending = false;
void  seq::armReset() { resetPending = true; }

/* ---------- nextStep() – timing-first, lock-first ---------- */
void seq::nextStep()
{
    using namespace hw;

    /* 0) Latch loop bounds at step edge (stable advance math) */
    // Also self-detect: if live mapped values differ from latched, mark dirty.
    uint8_t liveLS = pots.loopStart ? pots.loopStart - 1 : 0;
    uint8_t liveLE = pots.loopEnd   ? pots.loopEnd   - 1 : 15;
    if (liveLS != lsLat || liveLE != leLat) loopDirty = true;

    if (loopDirty) {
        noInterrupts(); loopDirty = false; interrupts();
        lsLat = liveLS; leLat = liveLE;
    }

    /* 1) Advance using LATCHED bounds */
    if (resetPending) {
        curStep = lsLat;                 // start of latched band
        resetPending = false;
    } else {
        curStep = advanceWithin(curStep, lsLat, leLat);
    }

    /* 2) Snapshot engine inputs once per step */
    const bool     instEdge      = hw::btnInstant.edge;
    const bool     destructOn    = hw::btnDestruct.level;
    const uint8_t  instChance    = hw::pots.instChance;
    const uint8_t  destrChance   = hw::pots.destructiveChance;
    const uint8_t  nondestChance = hw::pots.nondestChance;

    /* 3) Aspect logic: lock first, then engines; no auto-regeneration */
    auto doAspect = [&](seq::Aspect asp, uint8_t lockVal, uint8_t (*genFn)()->uint8_t){
        Track& T = track(asp);
        const bool locked = (random(128) < lockVal);

        if (locked) {
            T.prospectiveSequence[curStep] = T.regularSequence[curStep];
            return;
        }

        if (instEdge && random(128) < instChance) {
            uint8_t v = genFn();
            T.regularSequence   [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            return;
        }
        if (destructOn && random(128) < destrChance) {
            uint8_t v = genFn();
            T.regularSequence   [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            return;
        }
        if (random(128) < nondestChance) {
            T.prospectiveSequence[curStep] = genFn(); // preview only
            return;
        }
        T.prospectiveSequence[curStep] = T.regularSequence[curStep];
    };

    // Pitch → Gate → Oct → VSel
    doAspect(seq::Aspect::Pitch, hw::pots.deltaProb[0], genPitch);
    doAspect(seq::Aspect::Vel,   hw::pots.deltaProb[1], genGate );
    doAspect(seq::Aspect::Oct,   hw::pots.deltaProb[2], genOct  );
    doAspect(seq::Aspect::VSel,  hw::pots.deltaProb[3], genVSel );

    /* 4) Build and send MIDI (no CC-123 per step!) */
    static const uint8_t modes[7][8] = {
        {0,2,4,5,7,9,11,12},{0,2,3,5,7,9,10,12},{0,1,3,5,7,8,10,12},
        {0,2,4,6,7,9,11,12},{0,2,4,5,7,9,10,12},{0,2,3,5,7,8,10,12},
        {0,1,3,5,6,8,10,12}
    };

    uint8_t degree = trPitch.prospectiveSequence[curStep] & 0x07;
    uint8_t scale  = constrain(pots.scale, 1, 7) - 1;
    int8_t  octDisp= int8_t(trOct.prospectiveSequence[curStep]) - 1;
    uint8_t midiPitch = pots.root + modes[scale][degree] + octDisp * 12;

    uint8_t baseVel = trVel.prospectiveSequence[curStep] ? pots.velocity : 0;
    if (trAcc.prospectiveSequence[curStep]) baseVel = pots.accentVel;
    uint8_t midiVel = constrain(baseVel, 0, 127);

    // Lightweight note-off of previous note, then current NoteOn
    if (prevNote >= 0) MIDI.sendNoteOn((uint8_t)prevNote, 0, 1); // vel=0 NoteOff
    MIDI.sendNoteOn(midiPitch, midiVel, 1);
    prevNote = (midiVel ? midiPitch : -1);

    /* 5) UI (commit policy handled in ui.cpp) */
    ui::refresh();

    /* 6) Clear Instant only after all aspects had a chance to use it */
    if (instEdge) hw::btnInstant.edge = false;
}
