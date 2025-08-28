#include "sequencer.h"
#include "hw_inputs.h"
#include "ui.h"
#include <MIDI.h>

extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;

namespace {
  constexpr uint8_t kSteps = 16;

  struct Track { uint8_t regular[kSteps]={0}; uint8_t prospect[kSteps]={0}; };
  Track trPitch, trVel, trOct, trAcc;

  uint8_t curStep = 0;
  static bool resetPending = false;

  // Latched loop bounds used for timing math
  uint8_t lsLat = 0, leLat = 15;

  // Track previous note to send vel=0 NoteOff
  int16_t prevNote = -1;

  // Helpers
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

  int8_t octaveDisp(uint8_t degree){
    uint16_t v = hw::pots.octaveProb[degree]; // 0..128
    if (v <= 2) return -1; if (v >= 126) return +1;
    const int MID=64, DZ=8;
    if (v < MID - DZ) { uint8_t p = map((int)v,0,MID-DZ-1,127,0); return (random(128)<p)?-1:0; }
    if (v > MID + DZ) { uint8_t p = map((int)v,MID+DZ+1,128,0,127); return (random(128)<p)?+1:0; }
    return 0;
  }

  uint8_t genPitch(){ return pick(8, hw::pots.pitchProb); }
  uint8_t genGate (){ return (random(128) < hw::pots.density); }
  uint8_t genOct  (){
    uint8_t deg = trPitch.prospect[curStep] & 0x07;
    return (uint8_t)(octaveDisp(deg) + 1);  // 0,1,2
  }
  uint8_t genVSel (){
    if (!trVel.prospect[curStep]) return 0;
    return (random(128) < hw::pots.accentChance);
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
}

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
  lsLat=0; leLat=15; prevNote=-1; resetPending=false;
}

void seq::regenerateAll(uint8_t probability){
  const Aspect order[4] = { Aspect::Pitch, Aspect::Vel, Aspect::Oct, Aspect::VSel };
  for (uint8_t s=0;s<kSteps;s++){
    uint8_t save = curStep; curStep = s;
    for (uint8_t k=0;k<4;k++){
      Aspect a = order[k];
      uint8_t lock = 0;
      switch(a){ case Aspect::Pitch: lock=hw::pots.deltaProb[0]; break;
                 case Aspect::Vel:   lock=hw::pots.deltaProb[1]; break;
                 case Aspect::Oct:   lock=hw::pots.deltaProb[2]; break;
                 case Aspect::VSel:  lock=hw::pots.deltaProb[3]; break; }
      if (random(128) < lock) continue;                // locked: skip
      if (random(128) >= probability) continue;        // instChance gate
      uint8_t v = (a==Aspect::Pitch)?genPitch() : (a==Aspect::Vel)?genGate() :
                  (a==Aspect::Oct)?genOct() : genVSel();
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
}
static void rotL(Track& T){
  uint8_t f=T.regular[0]; for(uint8_t i=0;i<kSteps-1;i++) T.regular[i]=T.regular[i+1]; T.regular[kSteps-1]=f;
       f=T.prospect[0]; for(uint8_t i=0;i<kSteps-1;i++) T.prospect[i]=T.prospect[i+1]; T.prospect[kSteps-1]=f;
}
static void rotR(Track& T){
  uint8_t l=T.regular[kSteps-1]; for(int8_t i=kSteps-1;i>0;i--) T.regular[i]=T.regular[i-1]; T.regular[0]=l;
       l=T.prospect[kSteps-1]; for(int8_t i=kSteps-1;i>0;i--) T.prospect[i]=T.prospect[i-1]; T.prospect[0]=l;
}
void seq::rotateAllLeft (){ rotL(trPitch); rotL(trVel); rotL(trOct); rotL(trAcc); }
void seq::rotateAllRight(){ rotR(trPitch); rotR(trVel); rotR(trOct); rotR(trAcc); }

void seq::nextStep()
{
  // 0) latch loop bounds ONCE per step
  uint8_t liveLS = hw::pots.loopStart ? hw::pots.loopStart - 1 : 0;
  uint8_t liveLE = hw::pots.loopEnd   ? hw::pots.loopEnd   - 1 : 15;
  if (resetPending) { lsLat = liveLS; leLat = liveLE; }

  // 1) advance within LATCHED band
  if (resetPending) { curStep = lsLat; resetPending=false; }
  else              { curStep = advWithin(curStep, lsLat, leLat); }

  // Also align latched bounds when user moved them (at the step)
  if (liveLS != lsLat || liveLE != leLat) { lsLat = liveLS; leLat = liveLE; }

  // 2) snapshot engine inputs once for this step
  const bool     instEdge      = hw::btnInstant.edge;
  const bool     destructOn    = hw::btnDestruct.level;
  const uint8_t  instChance    = hw::pots.instChance;
  const uint8_t  destrChance   = hw::pots.destructiveChance;
  const uint8_t  nondestChance = hw::pots.nondestChance;

  auto runAspect = [&](Aspect a, uint8_t lock, uint8_t (*gen)()->uint8_t){
    uint8_t& R = regOf(a, curStep);
    uint8_t& P = proOf(a, curStep);
    // Lock FIRST: if locked, engines are ignored
    if (random(128) < lock) { P = R; return; }
    if (instEdge && random(128) < instChance) { uint8_t v=gen(); R=v; P=v; return; }
    if (destructOn && random(128) < destrChance){ uint8_t v=gen(); R=v; P=v; return; }
    if (random(128) < nondestChance){ P = gen(); return; }
    P = R;
  };

  runAspect(Aspect::Pitch, hw::pots.deltaProb[0], genPitch);
  runAspect(Aspect::Vel,   hw::pots.deltaProb[1], genGate );
  runAspect(Aspect::Oct,   hw::pots.deltaProb[2], genOct  );
  runAspect(Aspect::VSel,  hw::pots.deltaProb[3], genVSel );

  // 3) Build and send MIDI (No CC-123 here!)
  static const uint8_t modes[7][8] = {
    {0,2,4,5,7,9,11,12},{0,2,3,5,7,9,10,12},{0,1,3,5,7,8,10,12},
    {0,2,4,6,7,9,11,12},{0,2,4,5,7,9,10,12},{0,2,3,5,7,8,10,12},
    {0,1,3,5,6,8,10,12}
  };
  uint8_t degree  = trPitch.prospect[curStep] & 0x07;
  uint8_t scale   = constrain(hw::pots.scale, 1, 7) - 1;
  int8_t  octDisp = int8_t(trOct.prospect[curStep]) - 1;
  uint8_t midiPitch = hw::pots.root + modes[scale][degree] + octDisp * 12;

  uint8_t baseVel = trVel.prospect[curStep] ? hw::pots.velocity : 0;
  if (trAcc.prospect[curStep]) baseVel = hw::pots.accentVel;
  uint8_t midiVel = constrain(baseVel, 0, 127);

  if (prevNote >= 0) MIDI.sendNoteOn((uint8_t)prevNote, 0, 1); // vel=0 off
  MIDI.sendNoteOn(midiPitch, midiVel, 1);
  prevNote = (midiVel ? midiPitch : -1);

  // 4) UI buffer update; commit policy lives in ui.cpp
  ui::refresh();
  ui::commitAfterStepIfNeededExt(); // ext: one commit per step; int: no-op
}
