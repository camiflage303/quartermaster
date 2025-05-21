#pragma once
#include <Arduino.h>

namespace hw {

struct PotValues {
  uint8_t  pitchProb[8];
  uint8_t  octaveProb[8];
  uint8_t  deltaProb [4];
  uint8_t  density;
  uint8_t  destructiveChance;
  uint8_t  nondestChance;
  uint8_t  instChance;
  uint8_t  accentChance;
  uint16_t bpm;
  uint8_t  loopStart;     // 1-16
  uint8_t  loopEnd;       // 1-16
  uint8_t  root;
  uint8_t  velocity;
  uint8_t  accentVel;
  uint8_t  scale;
  uint8_t  pulsesPerStep; // how many MIDI clocks per sequencer step
};

struct ButtonState { bool level; bool edge; };

extern PotValues pots;
extern ButtonState btnOnOff, btnExtMidi, btnDestruct, btnInstant,
                   btnCopy,  btnCycleL,  btnCycleR,   btnReset;

void initPins();        // call once in setup()
void scanInputs();      // call each loop()

} // namespace hw
