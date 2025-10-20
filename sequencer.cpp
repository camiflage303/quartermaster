// --- sequencer.cpp (use-next-in-pool playback, hard-lock invariant, gradual morph)
// What’s new in this build:
// • Pool playback always takes the *next* element (pre-increment) instead of the
//   one under the cursor. This removes the audible “repeat the last one” feel
//   at loop/Δ transitions without needing last-played guards.
// • Pointer advance now sets aPtr := playIdx (since we pre-increment on read),
//   so we still advance exactly one slot per gated step.
// • Fully Δ-locked remains invariant: no pointer realign on loop changes; when
//   entering hard lock, we freeze the current ACTIVE set (target := active).
// • Easing off hard lock resumes gradual morphing; background rebuild ≤1/step.

#include "sequencer.h"
#include "hw_inputs.h"
#include "clock_engine.h"
#include "ui.h"
#include <MIDI.h>

extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

namespace {
  constexpr uint8_t kSteps = 16;

  // ======= Δ tuning bands =======
  constexpr uint8_t  kHardLockThreshold = 125; // Δ >= 125: pUse=127, pRe=0
  constexpr uint8_t  kSoftLockStart     = 121; // 121..124: pUse=127, pRe=1
  constexpr uint8_t  kSoftLock_pRe      = 1;
  constexpr uint8_t  kRecurveShift      = 0;   // keep linear pRe

  struct Track { uint8_t regular[kSteps]={0}; uint8_t prospect[kSteps]={0}; };
  Track trPitch, trVel, trOct, trAcc;   // trAcc = VSel (0=V1, 1=V2)

  uint8_t  curStep = 0;
  bool     resetPending = false;

  // Latched loop bounds for timing math
  uint8_t lsLat = 0, leLat = 15;

  // ---------- Small helpers ----------
  static inline bool isHardLocked(uint8_t slider){ return slider >= kHardLockThreshold; }

  uint8_t advWithin(uint8_t s, uint8_t a, uint8_t b) {
    if (a == b) return a;
    if (a < b)  return (s < b) ? uint8_t(s+1) : a;
    else        return (s > b) ? uint8_t(s-1) : a;
  }

  inline uint8_t pick(uint8_t len, const uint8_t* w){
    uint16_t total=0; for(uint8_t i=0;i<len;i++) total += w[i];
    if(!total) return 0;
    uint16_t r = random(total);
    for(uint8_t i=0;i<len;i++){ if(r < w[i]) return i; r -= w[i]; }
    return 0;
  }

  static inline void loopBoundsRaw(uint8_t &s, uint8_t &e){
    s = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
    e = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
  }
  static inline void stepsInLoop(uint8_t* out, uint8_t& n){
    uint8_t s,e; loopBoundsRaw(s,e);
    n = 0;
    if (s <= e) { for (uint8_t i=s; i<=e; ++i) out[n++] = i; }
    else        { for (uint8_t i=s; i<16; ++i) out[n++] = i; for (uint8_t i=0; i<=e; ++i) out[n++] = i; }
  }

  inline uint8_t& regOf(seq::Aspect a, uint8_t i){
    switch(a){
      case seq::Aspect::Pitch: return trPitch.regular[i];
      case seq::Aspect::Vel:   return trVel  .regular[i];
      case seq::Aspect::Oct:   return trOct  .regular[i];
      default:                 return trAcc  .regular[i];
    }
  }
  inline uint8_t& proOf(seq::Aspect a, uint8_t i){
    switch(a){
      case seq::Aspect::Pitch: return trPitch.prospect[i];
      case seq::Aspect::Vel:   return trVel  .prospect[i];
      case seq::Aspect::Oct:   return trOct  .prospect[i];
      default:                 return trAcc  .prospect[i];
    }
  }

  // ============================================================
  // Unified morphing pools (independent per aspect)
  // ============================================================
  struct PoolElem { uint8_t step; uint8_t val; };  // preserve loop order
  struct AspectPool {
    PoolElem active[16]; uint8_t aSize = 0; uint8_t aPtr = 0;   // cursor (points to the element BEFORE the next-to-play)
    PoolElem target[16]; uint8_t tSize = 0;
    bool     morphActive = false;
    bool     usedThisStep = false;
    uint8_t  playIdxThisStep = 255;   // index actually used this step
  };
  static AspectPool poolPitch, poolOct, poolVSel;

  static inline int8_t findStep(const PoolElem* arr, uint8_t sz, uint8_t step){
    for (uint8_t i=0;i<sz;i++) if (arr[i].step == step) return i;
    return -1;
  }

  static inline Track& trackOf(seq::Aspect a){
    switch(a){
      case seq::Aspect::Pitch: return trPitch;
      case seq::Aspect::Vel:   return trVel;
      case seq::Aspect::Oct:   return trOct;
      default:                 return trAcc;
    }
  }

  // Rebuild TARGET pool from REGULAR buffer’s gated steps in loop order
  static void rebuildTargetPool(seq::Aspect asp, AspectPool& P){
    uint8_t idxs[16], n; stepsInLoop(idxs, n);
    const Track& T = trackOf(asp);
    P.tSize = 0;
    for (uint8_t k=0;k<n;k++){
      uint8_t s = idxs[k];
      if (trVel.regular[s]) {  // gated steps only
        P.target[P.tSize++] = { s, T.regular[s] };
        if (P.tSize == 16) break;
      }
    }
    P.morphActive = true;  // convergence requested (might be ignored by pRe==0)
  }

  // Perform at most one morph operation toward target; rate ~ pRe
  static void maybeMorph(AspectPool& P, uint8_t pRe /*0..127*/){
    if (!P.morphActive) return;
    if (pRe == 0) return;              // hard-locked ⇒ no background morph
    if (random(128) >= pRe) return;

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
          // keep aPtr pointing to the element BEFORE the next to play:
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

  // Align pointer so that the *next* playback will match current/next gated step.
  // Because we use "take NEXT", we align aPtr to the element BEFORE the one we want.
  static void alignPointerToCurrentOrNextGate(AspectPool& P){
    if (!P.aSize) { P.aPtr = 0; return; }
    uint8_t loopIdx[16], n; stepsInLoop(loopIdx, n);
    uint8_t startK = 0;
    for (uint8_t k=0;k<n;k++){ if (loopIdx[k]==curStep){ startK = k; break; } }
    for (uint8_t off=0; off<n; ++off){
      uint8_t s = loopIdx[(startK + off) % n];
      if (trVel.regular[s]) {
        int8_t ix = findStep(P.active, P.aSize, s);
        if (ix >= 0) {
          // We want NEXT read to produce 'ix', so set aPtr := ix-1 (mod aSize)
          P.aPtr = (uint8_t)((ix + P.aSize - 1) % P.aSize);
          return;
        }
      }
    }
    P.aPtr %= P.aSize;
  }

  // Take a value from a pool using "next element" semantics.
  // aPtr points to the element BEFORE the one to play; we pre-increment.
  static inline uint8_t takeFromPool(AspectPool& P, uint8_t fallbackVal){
    if (!P.aSize) return fallbackVal;
    uint8_t ix = (P.aSize == 1) ? 0 : (uint8_t)((P.aPtr + 1) % P.aSize);
    P.usedThisStep = true;
    P.playIdxThisStep = ix;
    return P.active[ix].val;
  }

  // Slider-change handling:
  // - 0 -> >0 : snap active := target + align once (legacy "lock-in" feel).
  // - ENTERING hard-lock: FREEZE current active set (target := active),
  //   DO NOT realign pointer, so content+phase freeze exactly.
  // - leaving hard-lock : DO NOT snap; optional pointer align; gradual morph.
  static void onSliderChange(seq::Aspect asp, uint8_t oldS, uint8_t newS){
    AspectPool* P = (asp==seq::Aspect::Pitch ? &poolPitch
                     : asp==seq::Aspect::Oct ? &poolOct : &poolVSel);

    const bool enteringHard = (oldS <  kHardLockThreshold) && (newS >= kHardLockThreshold);
    const bool leavingHard  = (oldS >= kHardLockThreshold) && (newS <  kHardLockThreshold);
    const bool lowRise      = (oldS == 0) && (newS > 0);

    if (enteringHard) {
      // Freeze: target := active; keep pointer/phase as-is; stop morphing.
      P->tSize = P->aSize;
      for (uint8_t i=0;i<P->aSize;i++) P->target[i] = P->active[i];
      P->morphActive = false;
      return;
    }

    if (lowRise) {
      // First rise from 0: snap to current target built from REGULAR + gate
      rebuildTargetPool(asp, *P);
      P->aSize = P->tSize;
      for (uint8_t i=0;i<P->tSize;i++) P->active[i] = P->target[i];
      P->morphActive = false;
      alignPointerToCurrentOrNextGate(*P);
      return;
    }

    // Other cases: rebuild target and let gradual morph handle convergence.
    rebuildTargetPool(asp, *P);
    if (leavingHard) {
      // Optionally align once for sane phase, but do not snap content.
      if (P->aSize) alignPointerToCurrentOrNextGate(*P);
    }
  }

  // ---------- Raw generators (used when not taking from pool) ----------
  int8_t octaveDisp(uint8_t degree){
    uint16_t v = hw::pots.octaveProb[degree]; // 0..128
    if (v <= 2) return -1; if (v >= 126) return +1;
    const int MID=64, DZ=8;
    if (v < MID - DZ) { uint8_t p = map((int)v,0,MID-DZ-1,127,0); return (random(128)<p)?-1:0; }
    if (v > MID + DZ) { uint8_t p = map((int)v,MID+DZ+1,128,0,127); return (random(128)<p)?+1:0; }
    return 0;
  }
  static uint8_t genPitchRaw(){ return pick(8, hw::pots.pitchProb); }
  static uint8_t genGateRaw (){ return (random(128) < hw::pots.density); }
  static uint8_t genOctRaw  (){
    uint8_t deg = trPitch.prospect[curStep] & 0x07;
    return (uint8_t)(octaveDisp(deg) + 1);  // 0,1,2
  }
  static uint8_t genVSelRaw (){
    if (!trVel.prospect[curStep]) return 0;
    return (random(128) < hw::pots.accentChance);
  }

  // Map Δ slider to probabilities
  struct DeltaTuning { uint8_t pUse; uint8_t pRe; };
  static inline DeltaTuning deltaTuning(uint8_t slider /*0..127*/){
    if (slider >= kHardLockThreshold) { return {127, 0}; }           // hard band
    if (slider >= kSoftLockStart)     { return {127, kSoftLock_pRe}; } // soft band
    uint8_t pUse   = slider;                    // 0..127
    uint8_t baseRe = (uint8_t)(127 - slider);   // 127..0
    uint8_t pRe    = baseRe >> kRecurveShift;   // optional easing
    if (pRe == 0) pRe = 1;
    return {pUse, pRe};
  }
}

// ---------- public getters ----------
uint8_t seq::stepNow(){ return curStep; }
uint8_t seq::pitch(uint8_t i){ return trPitch.regular[i]; }
uint8_t seq::vel  (uint8_t i){ return trVel  .regular[i]; }
uint8_t seq::oct  (uint8_t i){ return trOct  .regular[i]; }
uint8_t seq::acc  (uint8_t i){ return trAcc  .regular[i]; }

void seq::forceStep(uint8_t s){ curStep = s % 16; }
void seq::armReset(){ resetPending = true; }

void seq::init(){
  for(uint8_t i=0;i<kSteps;i++){
    trPitch.regular[i]=0; trVel.regular[i]=1; trOct.regular[i]=1; trAcc.regular[i]=0;
    trPitch.prospect[i]=0; trVel.prospect[i]=1; trOct.prospect[i]=1; trAcc.prospect[i]=0;
  }
  lsLat=0; leLat=15; resetPending=false;

  // Reset pools
  poolPitch = AspectPool{}; poolOct = AspectPool{}; poolVSel = AspectPool{};
}

void seq::regenerateAll(uint8_t probability){
  const Aspect order[4] = { Aspect::Pitch, Aspect::Vel, Aspect::Oct, Aspect::VSel };
  for (uint8_t s=0;s<kSteps;s++){
    uint8_t save = curStep; curStep = s;
    for (uint8_t k=0;k<4;k++){
      Aspect a = order[k];
      // In regen, keep non-pitch "frozen" when Δ high (legacy vibe)
      uint8_t lock = (a==Aspect::Pitch) ? 0 : hw::pots.deltaProb[(uint8_t)a];
      if (random(128) < lock) continue;
      if (random(128) >= probability) continue;
      uint8_t v =
        (a==Aspect::Pitch)? genPitchRaw() :
        (a==Aspect::Vel)  ? genGateRaw()  :
        (a==Aspect::Oct)  ? genOctRaw()   :
                            genVSelRaw();
      proOf(a,s)=v;
    }
    curStep = save;
  }
}

void seq::commitProspect(){
  for (uint8_t s=0;s<kSteps;s++){
    trPitch.regular[s]=trPitch.prospect[s];
    trVel  .regular[s]=trVel  .prospect[s];
    trOct  .regular[s]=trOct  .prospect[s];
    trAcc  .regular[s]=trAcc  .prospect[s];
  }
  // Keep pools synced with REGULAR
  rebuildTargetPool(Aspect::Pitch, poolPitch);
  rebuildTargetPool(Aspect::Oct,   poolOct);
  rebuildTargetPool(Aspect::VSel,  poolVSel);
}

static void rotL(Track& T){
  uint8_t f=T.regular[0]; for(uint8_t i=0;i<kSteps-1;i++) T.regular[i]=T.regular[i+1]; T.regular[kSteps-1]=f;
       f=T.prospect[0]; for(uint8_t i=0;i<kSteps-1;i++) T.prospect[i]=T.prospect[i+1]; T.prospect[kSteps-1]=f;
}
static void rotR(Track& T){
  uint8_t l=T.regular[kSteps-1]; for(int8_t i=kSteps-1;i>0;i--) T.regular[i]=T.regular[i-1]; T.regular[0]=l;
       l=T.prospect[kSteps-1]; for(int8_t i=kSteps-1;i>0;i--) T.prospect[i]=T.prospect[i-1]; T.prospect[0]=l;
}
void seq::rotateAllLeft (){ rotL(trPitch); rotL(trVel); rotL(trOct); rotL(trAcc);
  rebuildTargetPool(Aspect::Pitch, poolPitch);
  rebuildTargetPool(Aspect::Oct,   poolOct);
  rebuildTargetPool(Aspect::VSel,  poolVSel);
}
void seq::rotateAllRight(){ rotR(trPitch); rotR(trVel); rotR(trOct); rotR(trAcc);
  rebuildTargetPool(Aspect::Pitch, poolPitch);
  rebuildTargetPool(Aspect::Oct,   poolOct);
  rebuildTargetPool(Aspect::VSel,  poolVSel);
}

void seq::nextStep()
{
  // Snapshot current Δ sliders early so we can branch loop-change behavior
  static uint8_t prevPitchS = 0, prevOctS = 0, prevVSelS = 0;
  uint8_t sPitch = hw::pots.deltaProb[0];
  uint8_t sOct   = hw::pots.deltaProb[2];
  uint8_t sVSel  = hw::pots.deltaProb[3];

  const bool hardPitch = isHardLocked(sPitch);
  const bool hardOct   = isHardLocked(sOct);
  const bool hardVSel  = isHardLocked(sVSel);

  // 0) latch loop bounds ONCE per step
  uint8_t liveLS = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
  uint8_t liveLE = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
  if (resetPending) { lsLat = liveLS; leLat = liveLE; }

  // 1) advance within LATCHED band
  if (resetPending) { curStep = lsLat; resetPending=false; }
  else              { curStep = advWithin(curStep, lsLat, leLat); }

  // Also align latched bounds when user moved them (at the step)
  if (liveLS != lsLat || liveLE != leLat) { lsLat = liveLS; leLat = liveLE; }

  // 1a) Slider-change detection → immediate pool updates/freeze/snap logic
  if (sPitch != prevPitchS){ onSliderChange(Aspect::Pitch, prevPitchS, sPitch); prevPitchS = sPitch; }
  if (sOct   != prevOctS)  { onSliderChange(Aspect::Oct,   prevOctS,   sOct  ); prevOctS   = sOct;   }
  if (sVSel  != prevVSelS) { onSliderChange(Aspect::VSel,  prevVSelS,  sVSel ); prevVSelS  = sVSel;  }

  // 1b) Detect loop bound changes → rebuild targets + conditional pointer align
  static uint8_t prevLS = 0, prevLE = 0;
  if (hw::pots.loopStart != prevLS || hw::pots.loopEnd != prevLE){
    rebuildTargetPool(Aspect::Pitch, poolPitch);
    rebuildTargetPool(Aspect::Oct,   poolOct);
    rebuildTargetPool(Aspect::VSel,  poolVSel);

    // If an aspect is FULLY hard-locked, DO NOT realign its pointer.
    if (!hardPitch && poolPitch.aSize) alignPointerToCurrentOrNextGate(poolPitch);
    if (!hardOct   && poolOct  .aSize) alignPointerToCurrentOrNextGate(poolOct);
    if (!hardVSel  && poolVSel .aSize) alignPointerToCurrentOrNextGate(poolVSel);

    prevLS = hw::pots.loopStart; prevLE = hw::pots.loopEnd;
  }

  // 2) snapshot engine inputs for this step (for morph maintenance)
  const auto tP = deltaTuning(sPitch);
  const auto tO = deltaTuning(sOct);
  const auto tV = deltaTuning(sVSel);

  // --- Throttle background target rebuilds to ONE per step -------------
  bool didBgRebuild = false;
  auto maybeBgRebuild = [&](seq::Aspect asp, AspectPool& P, uint8_t pRe){
    if (didBgRebuild) return;
    if (pRe && random(128) < pRe) {
      rebuildTargetPool(asp, P);
      didBgRebuild = true;
    }
  };
  // Priority order: Pitch → Oct → VSel
  maybeBgRebuild(Aspect::Pitch, poolPitch, tP.pRe);
  maybeBgRebuild(Aspect::Oct,   poolOct,   tO.pRe);
  maybeBgRebuild(Aspect::VSel,  poolVSel,  tV.pRe);
  // ---------------------------------------------------------------------

  maybeMorph(poolPitch, tP.pRe);
  maybeMorph(poolOct,   tO.pRe);
  maybeMorph(poolVSel,  tV.pRe);

  // Clear per-step flags
  poolPitch.usedThisStep = poolOct.usedThisStep = poolVSel.usedThisStep = false;
  poolPitch.playIdxThisStep = poolOct.playIdxThisStep = poolVSel.playIdxThisStep = 255;

  // Helper: choose from pool or generator (by pUse)
  auto chooseFromPoolOrGen = [&](seq::Aspect a, uint8_t pUse)->uint8_t{
    if (a == seq::Aspect::Pitch){
      if (poolPitch.aSize && random(128) < pUse) return takeFromPool(poolPitch, genPitchRaw());
      return genPitchRaw();
    } else if (a == seq::Aspect::Oct){
      if (poolOct.aSize   && random(128) < pUse) return takeFromPool(poolOct,   genOctRaw());
      return genOctRaw();
    } else if (a == seq::Aspect::VSel){
      if (poolVSel.aSize  && random(128) < pUse) return takeFromPool(poolVSel,  genVSelRaw());
      return genVSelRaw();
    }
    return 0;
  };

  // Force-from-pool (used when Δ-lock triggers for Pitch/Oct/VSel)
  auto forceFromPoolOrFallbackR = [&](seq::Aspect a, uint8_t& R)->uint8_t{
    if (a == seq::Aspect::Pitch && poolPitch.aSize) return takeFromPool(poolPitch, R);
    if (a == seq::Aspect::Oct   && poolOct.aSize)   return takeFromPool(poolOct,   R);
    if (a == seq::Aspect::VSel  && poolVSel.aSize)  return takeFromPool(poolVSel,  R);
    return R; // fallback to regular if pool empty
  };

  // Core per-aspect runner (engines only when not locked)
  auto runAspect = [&](Aspect a, uint8_t lockProb, uint8_t (*genRaw)()->uint8_t,
                       uint8_t pUse, bool allowPoolPlayback)
  {
    uint8_t& R = regOf(a, curStep);
    uint8_t& P = proOf(a, curStep);

    const bool locked = (random(128) < lockProb);

    // Velocity: legacy freeze when locked
    if (a == Aspect::Vel && locked) { P = R; return; }

    // Pitch/Oct/VSel: if locked, force playback from pool (engines skipped)
    if (allowPoolPlayback && locked) { P = forceFromPoolOrFallbackR(a, R); return; }

    // Not locked → engines may fire (Instant / Destructive / Non-destructive)
    const bool     instEdge      = hw::btnInstant.edge;
    const bool     destructOn    = hw::btnDestruct.level;
    const uint8_t  instChance    = hw::pots.instChance;
    const uint8_t  destrChance   = hw::pots.destructiveChance;
    const uint8_t  nondestChance = hw::pots.nondestChance;   // tied to Destruct toggle

    if (instEdge && random(128) < instChance){
      uint8_t v = allowPoolPlayback ? chooseFromPoolOrGen(a, pUse) : genRaw();
      R = v; P = v; return;
    }
    if (destructOn && random(128) < destrChance){
      uint8_t v = allowPoolPlayback ? chooseFromPoolOrGen(a, pUse) : genRaw();
      R = v; P = v; return;
    }
    if (destructOn && random(128) < nondestChance){
      uint8_t v = allowPoolPlayback ? chooseFromPoolOrGen(a, pUse) : genRaw();
      P = v; return;
    }

    // Default playback (no engine fired)
    if (allowPoolPlayback && random(128) < pUse){
      P = forceFromPoolOrFallbackR(a, R);
      return;
    }
    P = R;
  };

  // Run aspects in order
  runAspect(Aspect::Pitch, hw::pots.deltaProb[0], genPitchRaw, tP.pUse, true );
  runAspect(Aspect::Vel,   hw::pots.deltaProb[1], genGateRaw , 0      , false);
  runAspect(Aspect::Oct,   hw::pots.deltaProb[2], genOctRaw  , tO.pUse, true );
  runAspect(Aspect::VSel,  hw::pots.deltaProb[3], genVSelRaw , tV.pUse, true );

  // Advance pool pointers only on gated steps and only if that pool was used.
  // Because we *played* index ix = (aPtr+1)%size, we now set aPtr := ix
  // so that next step will naturally read the next element again.
  if (trVel.prospect[curStep]){
    if (poolPitch.usedThisStep && poolPitch.aSize) {
      poolPitch.aPtr = poolPitch.playIdxThisStep;
    }
    if (poolOct.usedThisStep && poolOct.aSize) {
      poolOct.aPtr = poolOct.playIdxThisStep;
    }
    if (poolVSel.usedThisStep && poolVSel.aSize) {
      poolVSel.aPtr = poolVSel.playIdxThisStep;
    }
  }

  // -------- MIDI build & send (clamped, proper NoteOff, per-step retrigger) --------
  static const uint8_t modes[7][8] = {
    {0,2,4,5,7,9,11,12},{0,2,3,5,7,9,10,12},{0,1,3,5,7,8,10,12},
    {0,2,4,6,7,9,11,12},{0,2,4,5,7,9,10,12},{0,2,3,5,7,8,10,12},
    {0,1,3,5,6,8,10,12}
  };
  uint8_t degree  = trPitch.prospect[curStep] & 0x07;
  uint8_t scale   = constrain(hw::pots.scale, 1, 7) - 1;
  int8_t  octDisp = int8_t(trOct.prospect[curStep]) - 1;

  int pitchTmp = int(hw::pots.root) + int(modes[scale][degree]) + int(octDisp) * 12;
  pitchTmp = constrain(pitchTmp, 0, 127);
  uint8_t midiPitch = (uint8_t)pitchTmp;

  bool gateNow = (trVel.prospect[curStep] != 0);
  uint8_t baseVel = gateNow ? hw::pots.velocity : 0;
  if (trAcc.prospect[curStep]) baseVel = hw::pots.accentVel;
  uint8_t midiVel = constrain(baseVel, 0, 127);

  static int8_t prevPitch = -1;
  static bool   prevGate  = false;

  if (!gateNow && prevGate && prevPitch >= 0) {
    MIDI.sendNoteOff((uint8_t)prevPitch, 0, clock::midiChannel);
    prevPitch = -1;
  }

  if (gateNow) {
    if (prevPitch < 0) {
      MIDI.sendNoteOn(midiPitch, midiVel, clock::midiChannel);
      prevPitch = midiPitch;
    } else if (midiPitch != prevPitch) {
      MIDI.sendNoteOff((uint8_t)prevPitch, 0, clock::midiChannel);
      MIDI.sendNoteOn(midiPitch, midiVel, clock::midiChannel);
      prevPitch = midiPitch;
    } else {
      // Retrigger same note each step (comment out next two lines for legato)
      MIDI.sendNoteOff((uint8_t)prevPitch, 0, clock::midiChannel);
      MIDI.sendNoteOn(midiPitch, midiVel, clock::midiChannel);
    }
  }

  prevGate = gateNow;

  // NOTE: UI refresh is handled exclusively in the main loop.
}
