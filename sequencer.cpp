#include "sequencer.h"
#include "clock_engine.h"

#include <MIDI.h>

/*  exact extern using the namespace chosen by the library  */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;          // lets you write just “MIDI.send…”


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
        /* a = start pot-1,  b = end pot-1   (0-15)            */
        if (a == b) return a;                 // 1-step loop

        if (a < b) {                          // forward
            return (s < b) ? s + 1 : a;
        } else {                              // reverse (start > end)
            return (s > b) ? s - 1 : a;
        }
    }

    struct Track { uint8_t regularSequence[kSteps]={0}; uint8_t prospectiveSequence[kSteps]={0}; };

    Track trPitch, trVel, trOct, trAcc;

    uint8_t curStep = 0;

    inline Track& track(seq::Aspect a){
        switch(a){
            case seq::Aspect::Pitch: return trPitch;
            case seq::Aspect::Vel:   return trVel;
            case seq::Aspect::Oct:   return trOct;
            default:                 return trAcc;
        }
    }

    /* ------------------------------------------------------
   helper:  per-degree octave displacement  (-1 / 0 / +1)
   pot = 0..1023.  0..511 ⇒ favour -1,  512..1023 ⇒ favour +1.
   at exactly 512 there is 0 % chance of either ±1.
    ------------------------------------------------------ */
    static int8_t octaveDisplacement(uint8_t degree)
    {
        uint16_t v = hw::pots.octaveProb[degree];   // raw pot
        if (v < 512){
            uint16_t chance = map(v, 0,511, 127,0);         // 0 ➜ 100 %,  511 ➜ 0 %
            return (random(128) < chance) ? -1 : 0;
        }else if (v > 512){
            uint16_t chance = map(v, 513,1023, 0,127);      // 513 ➜ 0 %,  1023 ➜ 100 %
            return (random(128) < chance) ? +1 : 0;
        }
        return 0;   // mid detent
    }

    uint8_t generate(seq::Aspect a)
    {
        using namespace hw;
        switch(a){
            case seq::Aspect::Pitch:
                return weightedRandomSelection(8, pots.pitchProb);   // helper below
            case seq::Aspect::Vel:
                return (random(128) < hw::pots.density);   // 0/1
            case seq::Aspect::Oct: {
                /* return -1 / 0 / +1 weighted by that degree’s pot */
                uint8_t deg = trPitch.prospectiveSequence[curStep] & 0x07;   // use the *prospective* degree
                return octaveDisplacement(deg) + 1;           // store as 0,1,2  (-1,0,+1)
            }
            default: /* Accent */ 
                return random(128) < pots.accentChance;
        }
    }

}

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
    }
}

/* ---------- nextStep() – main logic ---------- */
void seq::nextStep()
{
    MIDI.sendControlChange(123,0,1);             // all notes off
    
    using namespace hw;

    /* 1. advance step counter */
    curStep = advanceWithin(curStep,
                        hw::pots.loopStart - 1,
                        hw::pots.loopEnd   - 1);

    /* 2. loop over four aspects */
    for(uint8_t a=0; a < (uint8_t)Aspect::Count; ++a)
    {
        Aspect asp = (Aspect)a;
        Track& T   = track(asp);

        /* ─ Δ-lock ─ */
        bool delta = (random(128) < pots.deltaProb[a]);
        if (delta){
            T.prospectiveSequence[curStep] = T.regularSequence[curStep];
            continue;
        }

        /* ─ Instantaneous (highest priority) ─ */
        if (btnInstant.edge && random(128) < pots.instChance){
            uint8_t v = generate(asp);
            T.reg [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            continue;
        }

        /* ─ Destructive ─ */
        if (btnDestruct.level && random(128) < pots.destructiveChance){
            uint8_t v = generate(asp);

                //debug
                /*Serial.print(F("Vel = ["));
                for (uint8_t i=0;i<16;i++){ Serial.print(trVel.regularSequence[i]); Serial.print(' '); }
                Serial.println(']');*/
            
            T.reg [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            continue;
        }

        /* ─ Nondestructive ─ */
        if (random(128) < pots.nondestChance){
            T.prospectiveSequence[curStep] = generate(asp);
            continue;
        }

        /* ─ default: copy regular → prospect ─ */
        T.prospectiveSequence[curStep] = T.regularSequence[curStep];
    }

    /* 3. Commit prospect → regular if Copy button edged */
    /* 3-A  Instantaneous button  – regenerate *all* 16 prospect steps once */
    if (hw::btnInstant.edge){
        for(uint8_t s=0;s<kSteps;s++){
            for(uint8_t a=0;a<(uint8_t)Aspect::Count;a++){
                track(Aspect(a)).prospectiveSequence[s] = generate(Aspect(a));
            }
        }
    }

    /* 3-B  Copy button  – commit prospect → regular */
    if (hw::btnCopy.edge){
        for(uint8_t s=0;s<kSteps;s++){
            trPitch.regularSequence[s]=trPitch.prospectiveSequence[s];
            trVel  .regularSequence[s]=trVel  .prospectiveSequence[s];
            trOct  .regularSequence[s]=trOct  .prospectiveSequence[s];
            trAcc  .regularSequence[s]=trAcc  .prospectiveSequence[s];
        }
    }


    /* 4. Build and send MIDI note (simple demo) */
    static const uint8_t modes[7][8] = {
        {0,2,4,5,7,9,11,12},   // Ionian
        {0,2,3,5,7,9,10,12},   // Dorian
        {0,1,3,5,7,8,10,12},   // Phrygian
        {0,2,4,6,7,9,11,12},   // Lydian
        {0,2,4,5,7,9,10,12},   // Mixolydian
        {0,2,3,5,7,8,10,12},   // Aeolian
        {0,1,3,5,6,8,10,12}    // Locrian
    };

    //Set pitch
    uint8_t degree = trPitch.prospectiveSequence[curStep] & 0x07;          // 0-7
    uint8_t scale  = constrain(pots.scale, 1, 7) - 1;       // 0-6
    int8_t octDisp = int8_t(trOct.prospectiveSequence[curStep]) - 1;     // 0,1,2 → -1..+1
    uint8_t midiPitch = pots.root
                    + modes[scale][degree]
                    + octDisp * 12;


    //Set velocity/accent
    uint8_t baseVel = trVel.prospectiveSequence[curStep] ? pots.velocity : 0;
    if (trAcc.prospectiveSequence[curStep] && baseVel > 0) {
         baseVel = pots.accentVel;   // accent hit
    }
    uint8_t midiVel = constrain(baseVel, 0, 127);



    MIDI.sendNoteOn(midiPitch, midiVel, 1);      // new note

    //TESTING
    //Serial.print(F("STEP ")); Serial.println(curStep);

    /* ---------------- DEBUG DUMP -------------------------------- */
    /*
    Serial.print(F("S="));  Serial.print(curStep);
    Serial.print(F("  P:["));
    for(uint8_t i=0;i<16;i++){ Serial.print(trPitch.regularSequence[i]); Serial.print(' ');}
    Serial.print(F("] V:["));
    for(uint8_t i=0;i<16;i++){ Serial.print(trVel.regularSequence[i]);  Serial.print(' ');}
    Serial.print(F("] O:["));
    for(uint8_t i=0;i<16;i++){ Serial.print(trOct.regularSequence[i]);  Serial.print(' ');}
    Serial.print(F("] A:["));
    for(uint8_t i=0;i<16;i++){ Serial.print(trAcc.regularSequence[i]);  Serial.print(' ');}
    Serial.println(']');
    */
}
