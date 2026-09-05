#pragma once
#include <Arduino.h>

namespace hw {

struct PotValues {
  uint8_t  pitchProb[8];   // sliders 1..8
  uint8_t  octaveProb[8];  // octave bias per degree
  uint8_t  deltaProb [4];  // locks: Pitch, Vel, Oct, Accent
  uint8_t  density;        // gate density
  uint8_t  destructiveChance;
  uint8_t  nondestChance;
  uint8_t  instChance;
  uint8_t  accentChance;
  uint16_t bpm;            // internal BPM
  uint8_t  loopStart;      // 1..16
  uint8_t  loopEnd;        // 1..16
  uint8_t  root;           // MIDI root note
  uint8_t  velocity;       // base velocity
  uint8_t  accentVel;      // accent velocity
  uint8_t  scale;          // 1..7
  uint8_t  pulsesPerStep;  // how many MIDI clocks per sequencer step
};

struct ButtonState { bool level; bool edge; };

extern PotValues  pots;
extern ButtonState btnOnOff, btnExtMidi, btnDestruct, btnInstant,
                   btnCopy,  btnCycleL,  btnCycleR,   btnReset;

void initPins();        // call once in setup()
void scanInputs();      // call each loop()

} // namespace hw
