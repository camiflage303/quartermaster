#include "hw_inputs.h"
#include "sequencer.h"   // for seq::markLoopBoundsDirty() (optional)
#include <Arduino.h>

/* ───────────────── 1) Physical pin mapping ──────────────────────────
 * MUX select pins unchanged; MUX signals map to A5/A6/A4
 * Panel LEDs remapped per your new wiring:
 *
 *   D6  -> L6
 *   D7  -> L2
 *   D8  -> L3
 *   D9  -> L4
 *   D10 -> L5
 *   D12 -> L7
 *   A0  -> L1
 *   A1  -> spare (or L2 on some builds — see note below)
 *
 * NOTE: If your actual build uses A1 for L2 instead of D7,
 *       just swap LED_PINS[1] and LED_PINS[7].
 */
constexpr uint8_t MUX_S0 = 5,  MUX_S1 = 4,  MUX_S2 = 3,  MUX_S3 = 2;
constexpr uint8_t MUX_SIG[3] = { A5, A6, A4 };

constexpr uint8_t LED_PINS[8] = {
  A0,  // [0] L1
  7,   // [1] L2  (swap with A1 if your board uses A1 for L2)
  8,   // [2] L3
  9,   // [3] L4
  10,  // [4] L5
  6,   // [5] L6
  12,  // [6] L7
  A1   // [7] spare (or L2 on alternate wiring)
};
static uint8_t    ledTimer[8] = {0};

/* UI PPS positions (locked divisors of 96 PPQN) */
constexpr uint8_t kPpsLocked[] = {96,48,24,12,6};

/* ───────────────── 2) Raw-input descriptor ────────────────────────── */
struct Input {
  uint8_t mux, ch;     // which MUX (0..2) and which channel (0..15)
  bool    isButton;    // analog-thresholded button vs pot/slider
  int     lastVal;     // last raw analog value OR debounced bool for buttons
  bool    led;         // for toggle buttons: latched ON/OFF state
  int8_t  ledIdx;      // panel LED index to toggle (-1 = none)
};

/* ───────────────── 3) Symbolic indexes (abbrev) ──────────────────── */
enum InIdx {
  IDX_LOOP_END , IDX_DESTRUCT_POT , IDX_BTN_DESTRUCT , IDX_BTN_CYC_L ,
  IDX_SLIDE_8  , IDX_SLIDE_7 , IDX_SLIDE_6 , IDX_SLIDE_5 , IDX_SLIDE_4 ,
  IDX_OCT_4 , IDX_OCT_5 , IDX_OCT_6 , IDX_OCT_7 , IDX_OCT_8 ,

  IDX_OCT_3 , IDX_OCT_2 , IDX_OCT_1 ,
  IDX_SLIDE_3 , IDX_SLIDE_2 , IDX_SLIDE_1 ,
  IDX_ACC_PROB_POT , IDX_ACC_AMT_POT , IDX_DENSITY_POT , IDX_SCALE_POT ,
  IDX_VELOCITY_POT , IDX_TEMPO_POT , IDX_ROOT_POT , IDX_BTN_ONOFF_TOG , IDX_BTN_EXTMIDI_TOG ,
  IDX_LOOP_START ,

  IDX_DELTA_PITCH , IDX_DELTA_VEL , IDX_DELTA_OCT , IDX_DELTA_ACC ,
  IDX_BTN_CYC_R , IDX_BTN_RESET , IDX_BTN_INST , IDX_BTN_NONDEST ,
  IDX_INST_POT , IDX_NONDEST_POT ,

  N_RAW_INPUTS
};

/* ───────────────── 4) Physical lookup table ─────────────────────────
 * Updated for the fixed PCB: RESET button is now on MUX2 ch=4 (was 9).
 * Other channel positions match your prior design.
 */
static Input inputs[N_RAW_INPUTS] = {
  // MUX0 (A5)
  {0,0 ,false,-1,false,-1}, // IDX_LOOP_END
  {0,3 ,false,-1,false,-1}, // IDX_DESTRUCT_POT
  {0,4 ,true ,-1,false, 0}, // IDX_BTN_DESTRUCT (toggle; ties LED via code)
  {0,5 ,true ,-1,false, 3}, // IDX_BTN_CYC_L  -> flash L4
  {0,6 ,false,-1,false,-1}, // IDX_SLIDE_8
  {0,7 ,false,-1,false,-1}, // IDX_SLIDE_7
  {0,8 ,false,-1,false,-1}, // IDX_SLIDE_6
  {0,9 ,false,-1,false,-1}, // IDX_SLIDE_5
  {0,10,false,-1,false,-1}, // IDX_SLIDE_4
  {0,11,false,-1,false,-1}, // IDX_OCT_4
  {0,12,false,-1,false,-1}, // IDX_OCT_5
  {0,13,false,-1,false,-1}, // IDX_OCT_6
  {0,14,false,-1,false,-1}, // IDX_OCT_7
  {0,15,false,-1,false,-1}, // IDX_OCT_8

  // MUX1 (A6)
  {1,0 ,false,-1,false,-1}, // IDX_OCT_3
  {1,1 ,false,-1,false,-1}, // IDX_OCT_2
  {1,2 ,false,-1,false,-1}, // IDX_OCT_1
  {1,3 ,false,-1,false,-1}, // IDX_SLIDE_3
  {1,4 ,false,-1,false,-1}, // IDX_SLIDE_2
  {1,5 ,false,-1,false,-1}, // IDX_SLIDE_1
  {1,6 ,false,-1,false,-1}, // IDX_ACC_PROB_POT
  {1,7 ,false,-1,false,-1}, // IDX_ACC_AMT_POT
  {1,8 ,false,-1,false,-1}, // IDX_DENSITY_POT
  {1,9 ,false,-1,false,-1}, // IDX_SCALE_POT
  {1,10,false,-1,false,-1}, // IDX_VELOCITY_POT
  {1,11,false,-1,false,-1}, // IDX_TEMPO_POT
  {1,12,false,-1,false,-1}, // IDX_ROOT_POT
  {1,13,true ,-1,false, 6}, // IDX_BTN_ONOFF_TOG  -> toggles L7
  {1,14,true ,-1,false, 7}, // IDX_BTN_EXTMIDI_TOG-> toggles spare (A1) or L2 if swapped
  {1,15,false,-1,false,-1}, // IDX_LOOP_START

  // MUX2 (A4)
  {2,0 ,false,-1,false,-1}, // IDX_DELTA_PITCH
  {2,2 ,false,-1,false,-1}, // IDX_DELTA_VEL
  {2,4 ,false,-1,false,-1}, // IDX_DELTA_OCT
  {2,6 ,false,-1,false,-1}, // IDX_DELTA_ACC
  {2,8 ,true ,-1,false, 5}, // IDX_BTN_CYC_R  -> flash L6
  {2,9 ,true ,-1,false,-1}, // IDX_BTN_RESET  -> *** FIXED to channel 4 on new PCB ***
  {2,10,true ,-1,false, 2}, // IDX_BTN_INST   -> flash L3
  {2,11,true ,-1,false, 1}, // IDX_BTN_NONDEST-> flash L2
  {2,14,false,-1,false,-1}, // IDX_INST_POT
  {2,15,false,-1,false,-1}  // IDX_NONDEST_POT
};

/* ───────────────── 5) Public globals ──────────────────────────────── */
namespace hw {
  PotValues   pots;
  ButtonState btnOnOff, btnExtMidi, btnDestruct, btnInstant,
              btnCopy,  btnCycleL,  btnCycleR,   btnReset;
}

/* ───────────────── 6) Helpers ─────────────────────────────────────── */
static inline int readMux(uint8_t m, uint8_t ch) {
  digitalWrite(MUX_S0, bitRead(ch,0));
  digitalWrite(MUX_S1, bitRead(ch,1));
  digitalWrite(MUX_S2, bitRead(ch,2));
  digitalWrite(MUX_S3, bitRead(ch,3));
  delayMicroseconds(4);
  return analogRead(MUX_SIG[m]);
}

/* ───────────────── 7) initPins() ──────────────────────────────────── */
void hw::initPins() {
  pinMode(MUX_S0, OUTPUT); pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT); pinMode(MUX_S3, OUTPUT);
  for (uint8_t p : LED_PINS) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
}

/* ───────────────── 8) scanInputs() ────────────────────────────────── */
void hw::scanInputs() {
  /* Banked scan: touch ~1/3 of inputs per loop() pass */
  static uint8_t bank = 0;               // 0,1,2
  constexpr uint8_t  BANKS      = 3;
  constexpr uint16_t BANK_SIZE  = (N_RAW_INPUTS + BANKS - 1) / BANKS;
  uint16_t start = bank * BANK_SIZE;
  uint16_t end   = (start + BANK_SIZE < N_RAW_INPUTS) ? (start + BANK_SIZE) : N_RAW_INPUTS;

  /* 1) Read this slice (analog + button debounce/edge) */
  for (uint16_t i = start; i < end; ++i) {
    int v = readMux(inputs[i].mux, inputs[i].ch);

    if (inputs[i].isButton) {
      bool pressed = (v > 512);
      if (pressed && inputs[i].lastVal == 0) {         // rising edge
        inputs[i].led = !inputs[i].led;                // latch toggle buttons
        if (inputs[i].ledIdx >= 0)
          digitalWrite(LED_PINS[inputs[i].ledIdx], inputs[i].led);
      }
      inputs[i].lastVal = pressed;
    } else {
      if (abs(v - inputs[i].lastVal) > 10)
        inputs[i].lastVal = v;
    }
  }

  /* 2) Advance bank pointer */
  bank = (bank + 1) % BANKS;

  /* 3) Only rebuild Pots/Buttons after all 3 slices were read */
  if (bank != 0) return;

  /* ---- Pots mapping ---- */
  auto pot = [&](InIdx idx){ return inputs[idx].lastVal; };

  // Pitch-prob sliders (invert 0..1024 -> 127..0 like original)
  pots.pitchProb[0] = map(pot(IDX_SLIDE_1), 0,1024,127,-1);
  pots.pitchProb[1] = map(pot(IDX_SLIDE_2), 0,1024,127,-1);
  pots.pitchProb[2] = map(pot(IDX_SLIDE_3), 0,1024,127,-1);
  pots.pitchProb[3] = map(pot(IDX_SLIDE_4), 0,1024,127,-1);
  pots.pitchProb[4] = map(pot(IDX_SLIDE_5), 0,1024,127,-1);
  pots.pitchProb[5] = map(pot(IDX_SLIDE_6), 0,1024,127,-1);
  pots.pitchProb[6] = map(pot(IDX_SLIDE_7), 0,1024,127,-1);
  pots.pitchProb[7] = map(pot(IDX_SLIDE_8), 0,1024,127,-1);

  // Octave bias pots 0..128
  pots.octaveProb[0] = map(pot(IDX_OCT_1), 0,1023, 0,128);
  pots.octaveProb[1] = map(pot(IDX_OCT_2), 0,1023, 0,128);
  pots.octaveProb[2] = map(pot(IDX_OCT_3), 0,1023, 0,128);
  pots.octaveProb[3] = map(pot(IDX_OCT_4), 0,1023, 0,128);
  pots.octaveProb[4] = map(pot(IDX_OCT_5), 0,1023, 0,128);
  pots.octaveProb[5] = map(pot(IDX_OCT_6), 0,1023, 0,128);
  pots.octaveProb[6] = map(pot(IDX_OCT_7), 0,1023, 0,128);
  pots.octaveProb[7] = map(pot(IDX_OCT_8), 0,1023, 0,128);

  // Density with top snap + hysteresis
  {
    int raw = pot(IDX_DENSITY_POT);
    const int SAT_RAW    = 990;  // go to 100% if ADC >= this
    const int UNSNAP_RAW = 975;  // drop below 100% only if ADC <= this
    static bool sat = false;
    if (raw >= SAT_RAW)         sat = true;
    else if (raw <= UNSNAP_RAW) sat = false;

    pots.density = sat ? 128 : (uint8_t)constrain(map(raw, 0, SAT_RAW, 0, 127), 0, 127);
  }

  // Delta-lock probabilities (invert like original)
  pots.deltaProb[0] = map(pot(IDX_DELTA_PITCH), 0,1024,127,-1);
  pots.deltaProb[1] = map(pot(IDX_DELTA_VEL  ), 0,1024,127,-1);
  pots.deltaProb[2] = map(pot(IDX_DELTA_OCT  ), 0,1024,127,-1);
  pots.deltaProb[3] = map(pot(IDX_DELTA_ACC  ), 0,1024,127,-1);

  // Engine chance pots
  pots.destructiveChance = map(pot(IDX_DESTRUCT_POT), 0,1023, 0,128);
  pots.nondestChance     = map(pot(IDX_NONDEST_POT ), 0,1023, 0,128);
  pots.instChance        = map(pot(IDX_INST_POT    ), 0,1023, 0,128);
  pots.accentChance      = map(pot(IDX_ACC_PROB_POT), 0,1023, 0,128);

  // Tempo & pulses-per-step selector
  pots.bpm = map(pot(IDX_TEMPO_POT), 0,1023, 3,303);
  {
    uint8_t ix = map(pot(IDX_TEMPO_POT), 0,1024, 0,5);   // 0..5 → clamp to 0..4 later
    if (ix > 4) ix = 4;
    pots.pulsesPerStep = kPpsLocked[ix];
  }

  // Loop bounds 1..16
  pots.loopStart = map(pot(IDX_LOOP_START), 0,1024, 1,17);
  pots.loopEnd   = map(pot(IDX_LOOP_END  ), 0,1024, 1,17);

  // Optionally notify sequencer if you want off-tick rebuilds
  {
    static uint8_t _prevLS = 0, _prevLE = 0;
    if (pots.loopStart != _prevLS || pots.loopEnd != _prevLE) {
      // seq::markLoopBoundsDirty();   // keep commented unless implemented
      _prevLS = pots.loopStart; _prevLE = pots.loopEnd;
    }
  }

  // Musical pots
  pots.root     = map(pot(IDX_ROOT_POT   ), 0,1023, 12,108);
  pots.velocity = map(pot(IDX_VELOCITY_POT), 0,1024, 0,128);
  pots.accentVel= map(pot(IDX_ACC_AMT_POT ), 0,1024, 0,128);
  pots.scale    = map(pot(IDX_SCALE_POT  ), 0,1024, 1,  8);

  /* ---- Buttons: level + edge ---- */
  auto mapBtn = [&](ButtonState& b, InIdx idx){
    bool now = inputs[idx].lastVal;
    b.edge  = now && !b.level;
    b.level = now;
  };
  auto mapTgl = [&](ButtonState& b, InIdx idx){
    bool now = inputs[idx].led;  // latched ON/OFF (for toggle switches)
    b.edge  = now && !b.level;   // edge when it turns ON
    b.level = now;
  };

  mapTgl(btnOnOff   , IDX_BTN_ONOFF_TOG   );
  mapTgl(btnDestruct, IDX_BTN_DESTRUCT    );
  mapTgl(btnExtMidi , IDX_BTN_EXTMIDI_TOG );

  mapBtn(btnInstant , IDX_BTN_INST        );
  mapBtn(btnCopy    , IDX_BTN_NONDEST     );
  mapBtn(btnCycleL  , IDX_BTN_CYC_L       );
  mapBtn(btnCycleR  , IDX_BTN_CYC_R       );
  mapBtn(btnReset   , IDX_BTN_RESET       );

  /* Small event flashes on panel LEDs */
  auto flash = [&](uint8_t idx, uint8_t dur=4){
    if (idx < 8) { ledTimer[idx] = dur; digitalWrite(LED_PINS[idx], HIGH); }
  };
  if (btnCycleL.edge)  flash(3);  // L4
  if (btnCycleR.edge)  flash(5);  // L6
  if (btnReset .edge)  flash(4);  // L5
  if (btnInstant.edge) flash(2);  // L3
  if (btnCopy  .edge)  flash(1);  // L2

  /* Run-down LED timers */
  for (uint8_t i=0; i<8; ++i) {
    if (ledTimer[i] && --ledTimer[i] == 0)
      digitalWrite(LED_PINS[i], LOW);
  }
}
