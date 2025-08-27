#include "hw_inputs.h"
#include "sequencer.h"
#include "clock_engine.h"
#include "ui.h"
#include <MIDI.h>

/* exact extern using the namespace chosen by the library */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

/* ---------- constants / storage ---------- */
namespace {
    constexpr uint8_t kSteps = 16;

    struct Track {
        uint8_t regularSequence[kSteps]    = {0};
        uint8_t prospectiveSequence[kSteps]= {0};
    };

    // Tracks
    Track trPitch, trVel, trOct, trAcc;   // Vel = gate(0/1), Acc=VSel(0/1)

    // State
    uint8_t curStep = 0;
    bool    resetPending = false;

    /* Utility: advance within loop [a..b], supporting reversed loops (a>b) */
    static uint8_t advanceWithin(uint8_t s, uint8_t a, uint8_t b)
    {
        if (a == b) return a; // single-step loop ⇒ park
        const bool inBand = (a <= b) ? (s >= a && s <= b)
                                     : (s >= b && s <= a);
        if (!inBand) return a;                 // snap into band on first advance
        if (a < b)   return (s < b) ? uint8_t(s + 1) : a;   // forward
        else         return (s > b) ? uint8_t(s - 1) : a;   // reverse
    }

    /* Weighted random pick for 8-degree pitch */
    static uint8_t pickPitchDegree()
    {
        const uint8_t* w = hw::pots.pitchProb;
        uint16_t total = 0;
        for (uint8_t i=0;i<8;i++) total += w[i];
        if (!total) return 0;
        uint16_t r = random(total);
        for (uint8_t i=0;i<8;i++){ if (r < w[i]) return i; r -= w[i]; }
        return 0;
    }

    /* Octave displacement (-1/0/+1) based on per-degree sliders */
    static int8_t octaveDisplacement(uint8_t degree)
    {
        uint16_t v = hw::pots.octaveProb[degree];   // 0..128

        if (v <= 2)   return -1;   // hard down
        if (v >= 126) return +1;   // hard up

        const int MID = 64, DZ = 8;
        if (v < MID - DZ) {
            uint8_t chance = map((int)v, 0, MID - DZ - 1, 127, 0);
            return (random(128) < chance) ? -1 : 0;
        } else if (v > MID + DZ) {
            uint8_t chance = map((int)v, MID + DZ + 1, 128, 0, 127);
            return (random(128) < chance) ? +1 : 0;
        }
        return 0;
    }

    /* Per-aspect raw generators (no pools) */
    static uint8_t genPitch() { return pickPitchDegree(); }         // 0..7
    static uint8_t genGate () { return (random(128) < hw::pots.density) ? 1 : 0; }
    static uint8_t genOct  ()
    {
        uint8_t deg = trPitch.prospectiveSequence[curStep] & 0x07;
        return uint8_t(octaveDisplacement(deg) + 1);                // store 0,1,2 as (−1,0,+1)+1
    }
    static uint8_t genVSel()
    {
        if (!trVel.prospectiveSequence[curStep]) return 0;          // no gate → V1
        return (random(128) < hw::pots.accentChance) ? 1 : 0;       // 0=V1, 1=V2
    }

    static inline Track& track(seq::Aspect a){
        switch(a){
            case seq::Aspect::Pitch: return trPitch;
            case seq::Aspect::Vel:   return trVel;
            case seq::Aspect::Oct:   return trOct;
            case seq::Aspect::VSel:  return trAcc;
            default:                 return trAcc;
        }
    }

} // namespace

/* ---------- public accessors ---------- */
uint8_t seq::stepNow(){ return curStep; }
uint8_t seq::pitch(uint8_t i){ return trPitch.regularSequence[i]; }
uint8_t seq::vel  (uint8_t i){ return trVel  .regularSequence[i]; }
uint8_t seq::oct  (uint8_t i){ return trOct  .regularSequence[i]; }
uint8_t seq::acc  (uint8_t i){ return trAcc  .regularSequence[i]; }

void seq::forceStep(uint8_t s){ curStep = s % 16; }

/* ---------- init() ---------- */
void seq::init(){
    for(uint8_t i=0;i<kSteps;i++){
        trPitch.regularSequence[i]=0; trVel.regularSequence[i]=1;
        trOct.regularSequence[i]=1;   trAcc.regularSequence[i]=0;

        trPitch.prospectiveSequence[i]=0; trVel.prospectiveSequence[i]=1;
        trOct.prospectiveSequence[i]=1;   trAcc.prospectiveSequence[i]=0;
    }
}

/* Regenerate ALL 16 prospective steps once (unchanged feel) */
void seq::regenerateAll(uint8_t probability /*0-127*/, bool respectLocks /*=true*/)
{
    using namespace hw;
    const Aspect order[4] = { Aspect::Pitch, Aspect::Vel, Aspect::Oct, Aspect::VSel };

    for (uint8_t s = 0; s < kSteps; ++s) {
        uint8_t saved = curStep; curStep = s;

        for (uint8_t i = 0; i < 4; ++i) {
            Aspect asp = order[i];
            if (respectLocks) {
                // Per-aspect lock as a probability (same as nextStep)
                const uint8_t lockVal = hw::pots.deltaProb[(uint8_t)asp];
                if (random(128) < lockVal) {
                    // keep regular → prospect mirrors regular
                    track(asp).prospectiveSequence[s] = track(asp).regularSequence[s];
                    continue;
                }
            }
            if (random(128) >= probability) continue;   // only sometimes change

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

/* Promote prospect → regular immediately */
void seq::commitProspect()
{
    for (uint8_t s = 0; s < kSteps; ++s) {
        trPitch.regularSequence[s] = trPitch.prospectiveSequence[s];
        trVel  .regularSequence[s] = trVel  .prospectiveSequence[s];
        trOct  .regularSequence[s] = trOct  .prospectiveSequence[s];
        trAcc  .regularSequence[s] = trAcc  .prospectiveSequence[s];
    }
}

/* rotations – operate on both regular and prospect arrays */
static void rotateLeft (Track& T)
{
    uint8_t first = T.regularSequence[0];
    for (uint8_t i = 0; i < kSteps-1; ++i) T.regularSequence[i] = T.regularSequence[i+1];
    T.regularSequence[kSteps-1] = first;

    first = T.prospectiveSequence[0];
    for (uint8_t i = 0; i < kSteps-1; ++i) T.prospectiveSequence[i] = T.prospectiveSequence[i+1];
    T.prospectiveSequence[kSteps-1] = first;
}

static void rotateRight(Track& T)
{
    uint8_t last = T.regularSequence[kSteps-1];
    for (int8_t i = kSteps-1; i > 0; --i) T.regularSequence[i] = T.regularSequence[i-1];
    T.regularSequence[0] = last;

    last = T.prospectiveSequence[kSteps-1];
    for (int8_t i = kSteps-1; i > 0; --i) T.prospectiveSequence[i] = T.prospectiveSequence[i-1];
    T.prospectiveSequence[0] = last;
}

void seq::rotateAllLeft ()  { rotateLeft (trPitch); rotateLeft (trVel); rotateLeft (trOct); rotateLeft (trAcc); }
void seq::rotateAllRight()  { rotateRight(trPitch); rotateRight(trVel); rotateRight(trOct); rotateRight(trAcc); }

void  seq::armReset() { resetPending = true; }

/* ---------- nextStep() – delta-locks are absolute ---------- */
void seq::nextStep()
{
    MIDI.sendControlChange(123,0,1); // optional safety

    using namespace hw;

    // 0) advance step / honor reset
    if (resetPending) {
        curStep = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        resetPending = false;
    } else {
        curStep = advanceWithin(
            curStep,
            hw::pots.loopStart ? hw::pots.loopStart - 1 : 0,
            hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15
        );
    }

    // 1) Snapshot engine inputs ONCE so Instant can affect all free aspects
    const bool     instEdge      = hw::btnInstant.edge;
    const bool     destructOn    = hw::btnDestruct.level;
    const uint8_t  instChance    = hw::pots.instChance;
    const uint8_t  destrChance   = hw::pots.destructiveChance;
    const uint8_t  nondestChance = hw::pots.nondestChance;

    // helper: absolute-lock per aspect (lock evaluated BEFORE engines)
    auto doAspect = [&](seq::Aspect asp, uint8_t lockVal, uint8_t (*genFn)()->uint8_t){
        Track& T = track(asp);

        // Lock pot is a probability of being LOCKED (0..127).
        const bool locked = (random(128) < lockVal);

        if (locked) {
            // Absolutely locked → engines are ignored; use REGULAR.
            T.prospectiveSequence[curStep] = T.regularSequence[curStep];
            return;
        }

        // Free this step → engines MAY act; if none fires, keep REGULAR (no regen).
        // Engines are gated equally by the lock; order here is just priority.
        if (instEdge && random(128) < instChance) {
            uint8_t v = genFn();
            T.regularSequence   [curStep] = v;  // commit pattern
            T.prospectiveSequence[curStep] = v; // play now
            return;
        }
        if (destructOn && random(128) < destrChance) {
            uint8_t v = genFn();
            T.regularSequence   [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            return;
        }
        if (destructOn && random(128) < nondestChance) {
            // Preview only — pattern unchanged
            T.prospectiveSequence[curStep] = genFn();
            return;
        }

        // No engine fired → play stored pattern
        T.prospectiveSequence[curStep] = T.regularSequence[curStep];
    };

    // Order: Pitch → Gate → Oct → VSel (so Oct/VSel see just-made Pitch/Gate)
    doAspect(seq::Aspect::Pitch, hw::pots.deltaProb[0], []()->uint8_t{ return genPitch(); });
    doAspect(seq::Aspect::Vel,   hw::pots.deltaProb[1], []()->uint8_t{ return genGate (); });
    doAspect(seq::Aspect::Oct,   hw::pots.deltaProb[2], []()->uint8_t{ return genOct  (); });
    doAspect(seq::Aspect::VSel,  hw::pots.deltaProb[3], []()->uint8_t{ return genVSel (); });

    // 2) Build & send MIDI
    static const uint8_t modes[7][8] = {
        {0,2,4,5,7,9,11,12},{0,2,3,5,7,9,10,12},{0,1,3,5,7,8,10,12},
        {0,2,4,6,7,9,11,12},{0,2,4,5,7,9,10,12},{0,2,3,5,7,8,10,12},
        {0,1,3,5,6,8,10,12}
    };
    uint8_t degree = trPitch.prospectiveSequence[curStep] & 0x07;
    uint8_t scale  = constrain(hw::pots.scale, 1, 7) - 1;
    int8_t  octDisp= int8_t(trOct.prospectiveSequence[curStep]) - 1;
    uint8_t midiPitch = hw::pots.root + modes[scale][degree] + octDisp * 12;

    uint8_t baseVel = 0;
    if (trVel.prospectiveSequence[curStep]) {
        baseVel = hw::pots.velocity;
        if (trAcc.prospectiveSequence[curStep]) baseVel = hw::pots.accentVel;
    }
    uint8_t midiVel = constrain(baseVel, 0, 127);

    MIDI.sendNoteOn(midiPitch, midiVel, 1);

    // 3) UI commit (your ui.cpp now commits immediately)
    ui::refresh();

    // 4) Clear Instant only AFTER all aspects had a chance to use it
    if (instEdge) hw::btnInstant.edge = false;
}


