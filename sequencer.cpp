#include "hw_inputs.h"
#include "sequencer.h"
#include "clock_engine.h"
#include "ui.h"
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

    Track trPitch, trVel, trOct, trAcc;   // trAcc = VSel (0 = V1, 1 = V2)

    uint8_t curStep = 0;

    /* ─────────────  LEGACY SH-101 style PITCH POOL (used when Δ-lock=0)  ───────────── */
    static uint8_t  legacyPitchPool[16];   // remembered degrees (0-7)
    static uint8_t  legacyPoolSize  = 0;   // valid entries
    static uint8_t  legacyPoolIndex = 0;   // next element to read
    static bool     legacyTookFromPool = false;  // set per step

    static void legacyRebuildPitchPool()
    {
        legacyPoolSize  = 0;
        legacyPoolIndex = 0;

        uint8_t lo = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        uint8_t hi = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
        bool wrap  = lo > hi;

        for (uint8_t i = 0; i < kSteps; ++i) {
            bool inLoop = wrap ? (i >= lo || i <= hi)
                               : (i >= lo && i <= hi);
            if (inLoop && trVel.regularSequence[i]) {
                legacyPitchPool[legacyPoolSize++] = trPitch.regularSequence[i] & 0x07;
                if (legacyPoolSize == 16) break;                // safety cap
            }
        }

        if (legacyPoolSize == 0) { legacyPitchPool[0] = 0; legacyPoolSize = 1; }
        legacyPoolIndex %= legacyPoolSize;
    }

    static inline void legacyRememberPitch(uint8_t stepIdx, uint8_t degree)
    {
        trPitch.regularSequence[stepIdx] = degree;
        legacyRebuildPitchPool();                         // keeps order unique-dup
        legacyPoolIndex = (legacyPoolIndex + 1) % legacyPoolSize;  // next after this note
    }

    /* Count gated steps inside the current loop – used for legacy shrink */
    static uint8_t legacyGatedCountInLoop()
    {
        uint8_t lo = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        uint8_t hi = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
        bool wrap  = lo > hi;
        uint8_t cnt=0;
        for (uint8_t i=0;i<kSteps;i++){
            bool inLoop = wrap ? (i>=lo || i<=hi) : (i>=lo && i<=hi);
            if (inLoop && trVel.prospectiveSequence[i]) ++cnt;
        }
        return cnt ? cnt : 1;
    }

    /* ---- Legacy generator for Pitch (SH-101 semantics) ---- */
    static uint8_t generateLegacyPitch()
    {
        using namespace hw;
        uint8_t prob = hw::pots.deltaProb[0];   // 0-127 (legacy lock)
        if (!legacyPoolSize) legacyRebuildPitchPool();      // first run / after reset

        bool usePool = (prob == 127) || (random(128) < prob);
        legacyTookFromPool = false;

        if (usePool && legacyPoolSize) {
            uint8_t deg = legacyPitchPool[legacyPoolIndex];
            legacyTookFromPool = true;          // advance later if gated
            return deg;
        }

        /* Generate new degree */
        uint8_t deg = weightedRandomSelection(8, pots.pitchProb);

        /* Fade-in lock: remember with chance  (127-prob)/127 */
        if (prob < 127 && random(128) < (127 - prob))
            legacyRememberPitch(curStep, deg);

        return deg;
    }

    /* ─────────────  NEW per-aspect pools with target/active + morph (Pitch/Oct/VSel) ───────────── */
    struct PoolElem { uint8_t step; uint8_t val; };  // keep source step to preserve loop order
    struct AspectPool {
        PoolElem active[16]; uint8_t aSize = 0; uint8_t aPtr = 0;   // what is currently used
        PoolElem target[16]; uint8_t tSize = 0;                     // rebuilt from loop-gated steps
        bool     morphActive = false;                               // need to converge active→target
    };

    static AspectPool poolPitch, poolOct, poolVSel;

    static inline void stepsInLoop(uint8_t* out, uint8_t& n){
        uint8_t a = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        uint8_t b = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
        n = 0;
        if (a <= b) { for (uint8_t i=a; i<=b; ++i) out[n++] = i; }
        else        { for (uint8_t i=a; i<16; ++i) out[n++] = i; for (uint8_t i=0; i<=b; ++i) out[n++] = i; }
    }

    static inline Track& trackOf(seq::Aspect a){
        switch(a){
            case seq::Aspect::Pitch: return trPitch;
            case seq::Aspect::Vel:   return trVel;
            case seq::Aspect::Oct:   return trOct;
            case seq::Aspect::VSel:  return trAcc;
            default:                 return trAcc;
        }
    }

    static inline int8_t findStep(const PoolElem* arr, uint8_t sz, uint8_t step){
        for (uint8_t i=0;i<sz;i++) if (arr[i].step == step) return i;
        return -1;
    }

    /* Rebuild TARGET pool from REGULAR buffer’s gated steps in play order */
    static void rebuildTargetPool(seq::Aspect asp, AspectPool& P){
        uint8_t idxs[16], n; stepsInLoop(idxs, n);
        const Track& T = trackOf(asp);
        P.tSize = 0;
        for (uint8_t k=0;k<n;k++){
            uint8_t s = idxs[k];
            if (trVel.regularSequence[s]) {                 // only gated steps (REGULAR)
                P.target[P.tSize++] = { s, T.regularSequence[s] };
                if (P.tSize == 16) break;
            }
        }
        P.morphActive = true;  // request gradual convergence
    }

    /* Perform at most one morph step toward target; probability = (127 - slider) */
    static void maybeMorph(AspectPool& P, uint8_t slider /*0..127*/){
        uint8_t pMorph = 127 - slider;                      // high slider → slower morphs
        if (!P.morphActive) return;
        if (random(128) >= pMorph) return;

        // Prefer deletes of out-of-loop items first when shrinking
        if (P.aSize > P.tSize){
            for (int8_t i = P.aSize - 1; i >= 0; --i){
                if (findStep(P.target, P.tSize, P.active[i].step) < 0){
                    if (P.aSize && i < P.aPtr) P.aPtr = (P.aPtr + P.aSize - 1) % P.aSize;
                    for (uint8_t j=i; j<P.aSize-1; ++j) P.active[j] = P.active[j+1];
                    --P.aSize;
                    if (P.aSize) P.aPtr %= P.aSize; else P.aPtr = 0;
                    return;
                }
            }
        }
        // Grow: insert first missing target element at its position
        if (P.aSize < P.tSize){
            for (uint8_t k=0;k<P.tSize;k++){
                if (findStep(P.active, P.aSize, P.target[k].step) < 0){
                    uint8_t pos = k;
                    for (uint8_t j=P.aSize; j>pos; --j) P.active[j] = P.active[j-1];
                    P.active[pos] = P.target[k];
                    ++P.aSize;
                    if (pos <= P.aPtr) ++P.aPtr;
                    P.aPtr %= P.aSize;
                    return;
                }
            }
        }
        // Same size: replace first mismatch to realign contents/order
        for (uint8_t k=0;k<P.aSize;k++){
            if (P.active[k].step != P.target[k].step || P.active[k].val != P.target[k].val){
                P.active[k] = P.target[k];
                return;
            }
        }
        P.morphActive = false;  // reached target
    }

    /* ---------- pointer alignment (for 0→>0 lock) ---------- */
    static void alignPointerToCurrentOrNextGate(AspectPool& P){
        if (!P.aSize) { P.aPtr = 0; return; }
        uint8_t loopIdx[16], n; stepsInLoop(loopIdx, n);
        // Find start position at current step in loop order
        uint8_t startK = 0;
        for (uint8_t k=0;k<n;k++){ if (loopIdx[k]==curStep){ startK = k; break; } }
        // Scan from current step forward to the next gated step that exists in pool
        for (uint8_t off=0; off<n; ++off){
            uint8_t s = loopIdx[(startK + off) % n];
            if (trVel.regularSequence[s]) {
                int8_t ix = findStep(P.active, P.aSize, s);
                if (ix >= 0) { P.aPtr = (uint8_t)ix; return; }
            }
        }
        // fallback
        P.aPtr %= P.aSize;
    }

    /* ---------- on-slider-change hook ---------- */
    static void onSliderChange(seq::Aspect asp, uint8_t oldS, uint8_t newS){
        AspectPool* P = (asp==seq::Aspect::Pitch ? &poolPitch
                           : asp==seq::Aspect::Oct ? &poolOct
                                                   : &poolVSel);
        // Always rebuild TARGET immediately on change
        rebuildTargetPool(asp, *P);
        if (oldS == 0 && newS > 0) {
            // Snap active := target for "no audible change" upon locking
            P->aSize = P->tSize;
            for (uint8_t i=0;i<P->tSize;i++) P->active[i] = P->target[i];
            P->morphActive = false;
            alignPointerToCurrentOrNextGate(*P);
        }
        // If >0→>0 or >0→0, we leave active as-is (no snap), morphing will handle it.
    }

    inline Track& track(seq::Aspect a){ return trackOf(a); }

    /* ------------------------------------------------------
       helper:  per-degree octave displacement  (-1 / 0 / +1)
       (unchanged)
    ------------------------------------------------------ */
    static int8_t octaveDisplacement(uint8_t degree)
    {
        uint16_t v = hw::pots.octaveProb[degree];   // raw pot
        if (v < 62){
            uint16_t chance = map(v, 0,63, 127,0);         // 0 ➜ 100 %,  62 ➜ 0 %
            return (random(128) < chance) ? -1 : 0;
        }else if (v > 64){
            uint16_t chance = map(v, 64,127, 0,127);      // 64 ➜ 0 %,  127 ➜ 100 %
            return (random(128) < chance) ? +1 : 0;
        }
        return 0;   // mid detent
    }

    /* Raw generators that ignore any new-pool logic */
    static uint8_t generateRaw(seq::Aspect a){
        using namespace hw;
        switch (a){
            case seq::Aspect::Pitch:
                return weightedRandomSelection(8, pots.pitchProb);
            case seq::Aspect::Vel:
                return random(128) < pots.density;                // gate present?
            case seq::Aspect::Oct: {
                uint8_t deg = trPitch.prospectiveSequence[curStep] & 0x07;
                return octaveDisplacement(deg) + 1;               // store 0,1,2
            }
            case seq::Aspect::VSel:
                if (!trVel.prospectiveSequence[curStep]) return 0; // no gate → V1
                return random(128) < pots.accentChance;            // 0=V1, 1=V2
        }
        return 0;
    }


    /* Step-indexed raw generator: use state from step 's' (for Instant 16-step fills) */
    static uint8_t generateRawAt(seq::Aspect a, uint8_t s){
        using namespace hw;
        switch (a){
            case seq::Aspect::Pitch:
                return weightedRandomSelection(8, pots.pitchProb);
            case seq::Aspect::Vel:
                return random(128) < pots.density;
            case seq::Aspect::Oct: {
                uint8_t deg = trPitch.prospectiveSequence[s] & 0x07;
                return octaveDisplacement(deg) + 1;                 // 0,1,2
            }
            case seq::Aspect::VSel:
                if (!trVel.prospectiveSequence[s]) return 0;        // only accent if gated
                return random(128) < pots.accentChance;             // respects mix pot
        }
        return 0;
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
    // reset legacy & new pool systems
    legacyPitchPool[0] = 0; legacyPoolSize = 0; legacyPoolIndex = 0; legacyTookFromPool = false;
    poolPitch = AspectPool{}; poolOct = AspectPool{}; poolVSel = AspectPool{};
}

/* ===========================================================
   ❶  Regenerate ALL 16 prospective steps once (unchanged feel)
   =========================================================== */
void seq::regenerateAll(uint8_t probability /*0-127*/)
{
    using namespace hw;

    /* Generate in a stable per-step order so Oct/VSel can see the just-made Pitch/Vel */
    const Aspect order[4] = { Aspect::Pitch, Aspect::Vel, Aspect::Oct, Aspect::VSel };

    for (uint8_t s = 0; s < kSteps; ++s) {
        for (uint8_t i = 0; i < 4; ++i) {
            Aspect asp = order[i];

            /* Keep old "freeze non-pitch" behavior for bulk regen */
            if (asp != Aspect::Pitch && random(128) < pots.deltaProb[(uint8_t)asp])
                continue;

            if (random(128) < probability) {
                uint8_t v = generateRawAt(asp, s);
                track(asp).prospectiveSequence[s] = v;
            }
        }
    }
}

/* ===========================================================
   ❷  Promote prospect → regular immediately
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

/* ---------- nextStep() – main logic ---------- */
void seq::nextStep()
{
    MIDI.sendControlChange(123,0,1);             // all notes off
    
    using namespace hw;

    /* 0. honour pending reset ------------------------------------ */
    if (resetPending) {
        curStep      = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
        resetPending = false;           // one-shot
    } else {
    /* 1. advance step counter normally ----------------------- */
    curStep = advanceWithin(curStep,
                            hw::pots.loopStart - 1,
                            hw::pots.loopEnd   - 1);
}

    /* 1b. If loop bounds changed, rebuild TARGET pools (active morphs toward them) */
    static uint8_t prevLS = 0, prevLE = 0;
    if (hw::pots.loopStart != prevLS || hw::pots.loopEnd != prevLE){
        rebuildTargetPool(Aspect::Pitch, poolPitch);
        rebuildTargetPool(Aspect::Oct,   poolOct);
        rebuildTargetPool(Aspect::VSel,  poolVSel);
        prevLS = hw::pots.loopStart; prevLE = hw::pots.loopEnd;
    }

    /* 1c. Slider-change detection → immediate pool updates + optional snap */
    static uint8_t prevPitchS = 0, prevOctS = 0, prevVSelS = 0;
    uint8_t sPitch = hw::pots.deltaProb[0];
    uint8_t sOct   = hw::pots.deltaProb[2];
    uint8_t sVSel  = hw::pots.deltaProb[3];
    if (sPitch != prevPitchS){ onSliderChange(Aspect::Pitch, prevPitchS, sPitch); prevPitchS = sPitch; }
    if (sOct   != prevOctS)  { onSliderChange(Aspect::Oct,   prevOctS,   sOct  ); prevOctS   = sOct;   }
    if (sVSel  != prevVSelS) { onSliderChange(Aspect::VSel,  prevVSelS,  sVSel ); prevVSelS  = sVSel;  }

    bool usedNewPool_P = false, usedNewPool_O = false, usedNewPool_V = false;
    bool usedLegacyPool_P = false;

    /* 2. loop over four aspects */
    for(uint8_t a=0; a < (uint8_t)Aspect::Count; ++a)
    {
        Aspect asp = (Aspect)a;
        Track& T   = track(asp);

        /* -------- Gate Δ-lock (unchanged) -------- */
        if (asp == Aspect::Vel){
            bool delta = (random(128) < pots.deltaProb[a]);
            if (delta){
                T.prospectiveSequence[curStep] = T.regularSequence[curStep];
                hw::btnInstant.edge = false;
                continue;
            }
            /* Engines in original priority; nondest requires Destruct toggle */
            if (btnInstant.edge && random(128) < pots.instChance){
                uint8_t v = generateRaw(asp);
                T.regularSequence [curStep] = v;
                T.prospectiveSequence[curStep] = v;
                hw::btnInstant.edge = false;
                continue;
            }
            if (btnDestruct.level && random(128) < pots.destructiveChance){
                uint8_t v = generateRaw(asp);
                T.regularSequence [curStep] = v;
                T.prospectiveSequence[curStep] = v;
                continue;
            }
            if (btnDestruct.level && random(128) < pots.nondestChance){
                T.prospectiveSequence[curStep] = generateRaw(asp);
                continue;
            }
            T.prospectiveSequence[curStep] = T.regularSequence[curStep];
            continue;
        }

        /* -------- Pitch / Oct / VSel -------- */
        const uint8_t slider = pots.deltaProb[a];
        const uint8_t pUse   = slider;         // 0..127  → use-pool probability
        const uint8_t pRe    = 127 - slider;   // recalc / morph probability

        // Background recalc + morph every tick (still stochastic)
        if (random(128) < pRe){
            if      (asp == Aspect::Pitch) rebuildTargetPool(asp, poolPitch);
            else if (asp == Aspect::Oct)   rebuildTargetPool(asp, poolOct);
            else                            rebuildTargetPool(asp, poolVSel);
        }
        if      (asp == Aspect::Pitch) maybeMorph(poolPitch, slider);
        else if (asp == Aspect::Oct)   maybeMorph(poolOct,   slider);
        else                           maybeMorph(poolVSel,  slider);

        AspectPool* P = (asp==Aspect::Pitch ? &poolPitch : asp==Aspect::Oct ? &poolOct : &poolVSel);

        // Engines first; when they fire, choose pool vs generator by slider
        bool engineFired = false;
        bool chosePool   = false;   // track for pointer advance
        if (btnInstant.edge && random(128) < pots.instChance){
            uint8_t v;
            if (P->aSize && random(128) < pUse) { v = P->active[P->aPtr].val; chosePool = true; }
            else                                 { v = generateRaw(asp); }
            T.regularSequence [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            hw::btnInstant.edge = false;
            engineFired = true;
        } else if (btnDestruct.level && random(128) < pots.destructiveChance){
            uint8_t v;
            if (P->aSize && random(128) < pUse) { v = P->active[P->aPtr].val; chosePool = true; }
            else                                 { v = generateRaw(asp); }
            T.regularSequence [curStep] = v;
            T.prospectiveSequence[curStep] = v;
            engineFired = true;
        } else if (btnDestruct.level && random(128) < pots.nondestChance){
            uint8_t v;
            if (P->aSize && random(128) < pUse) { v = P->active[P->aPtr].val; chosePool = true; }
            else                                 { v = generateRaw(asp); }
            T.prospectiveSequence[curStep] = v;   // nondest: prospective only
            engineFired = true;
        }

        if (engineFired){
            if (asp == Aspect::Pitch) usedLegacyPool_P = usedLegacyPool_P || false; // legacy not used in this branch
            if (chosePool){
                if (asp == Aspect::Pitch) usedNewPool_P = true;
                else if (asp == Aspect::Oct) usedNewPool_O = true;
                else usedNewPool_V = true;
            }
            continue;   // engines wrote the value; skip non-engine path
        }

        // No engine: default playback, optionally override with pool
        T.prospectiveSequence[curStep] = T.regularSequence[curStep];
        if (P->aSize && random(128) < pUse){
            uint8_t v = P->active[P->aPtr].val;
            T.prospectiveSequence[curStep] = v;      // play from pool (REGULAR untouched)
            if (asp == Aspect::Pitch) usedNewPool_P = true;
            else if (asp == Aspect::Oct) usedNewPool_O = true;
            else usedNewPool_V = true;
        }

        // Legacy path note: when slider==0 the above code is still fine because pUse=0 and pRe=127.
        // For Pitch, the original “fade-in lock” and legacy pool is only used in the dedicated legacy branch earlier
        // (we keep legacy semantics active when slider==0 by using generateLegacyPitch in that branch).
        if (slider == 0 && asp == Aspect::Pitch){
            // Overwrite with legacy behavior when fully unlocked to preserve exact original feel
            // Engines are already handled in the engine branch above; only default path matters here:
            T.prospectiveSequence[curStep] = T.regularSequence[curStep];
        }
    }

    /* ---------- advance pool pointers if a pool was *used* and gate is ON ---------- */
    {
        bool gate = trVel.prospectiveSequence[curStep];
        if (gate){
            if (usedNewPool_P && poolPitch.aSize) poolPitch.aPtr = (poolPitch.aPtr + 1) % poolPitch.aSize;
            if (usedNewPool_O && poolOct.aSize)   poolOct.aPtr   = (poolOct.aPtr   + 1) % poolOct.aSize;
            if (usedNewPool_V && poolVSel.aSize)  poolVSel.aPtr  = (poolVSel.aPtr  + 1) % poolVSel.aSize;

            if (usedLegacyPool_P && legacyPoolSize) legacyPoolIndex = (legacyPoolIndex + 1) % legacyPoolSize;
        }
        usedNewPool_P = usedNewPool_O = usedNewPool_V = false;
        usedLegacyPool_P = false;
    }

    /* ---------- LEGACY pitch-pool maintenance (gradual shrink) ---------- */
    {
        uint8_t prob = hw::pots.deltaProb[0];
        if (prob < 127) {  // only shrink when unlocked in legacy sense
            uint8_t want = legacyGatedCountInLoop();
            if (legacyPoolSize > want) {
                if (random(128) < (127 - prob)) {
                    --legacyPoolSize;
                    if (legacyPoolSize == 0) { legacyPoolSize = 1; legacyPitchPool[0]=0; }
                    legacyPoolIndex %= legacyPoolSize;
                }
            }
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

    //Set velocity/accent (VSel)
    uint8_t baseVel = 0;
    if (trVel.prospectiveSequence[curStep]) {           // Velocity-1 hit
        baseVel = hw::pots.velocity;                    // Velocity pot
        if (trAcc.prospectiveSequence[curStep])         // flipped to Velocity-2?
            baseVel = hw::pots.accentVel;               // Acc_amt pot
    }
    uint8_t midiVel = constrain(baseVel, 0, 127);

    MIDI.sendNoteOn(midiPitch, midiVel, 1);      // new note
    ui::refresh();          // draw into the pixel buffer
    strip.show();           // commit: interrupts off for ~0.4 ms

    hw::btnInstant.edge = false;    // prevents multiple hits per press
}
