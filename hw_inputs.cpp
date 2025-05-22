#include "hw_inputs.h"
#include <Arduino.h>

/* ───────────── 1. Physical pin mapping  ──────────────────── */
constexpr uint8_t MUX_S0 = 5,  MUX_S1 = 4,  MUX_S2 = 3,  MUX_S3 = 2;
constexpr uint8_t MUX_SIG[3] = {A5, A6, A4};
constexpr uint8_t LED_PINS[8] = {A7, 7, 8, 9, 10, 11, 12, 13};
static uint8_t    ledTimer[8] = {0};
constexpr uint8_t kPpsLookup[9] = {96,72,48,32,24,18,12,9,6}; // 1, d2,2,d4,4,d8,8,d16,16

/* ───────────── 2. Raw-input descriptor  ──────────────────── */
struct Input { uint8_t mux, ch; bool isButton; int lastVal; bool led; int8_t ledIdx; };

/* ───────────── 3. Symbolic indexes  (shortened) ──────────── */
enum InIdx {
  IDX_LOOP_END , IDX_DESTRUCT_POT , IDX_BTN_DESTRUCT , IDX_BTN_CYC_L ,
  IDX_SLIDE_8  , IDX_SLIDE_7 , IDX_SLIDE_6 , IDX_SLIDE_5 , IDX_SLIDE_4 ,
  IDX_OCT_4 , IDX_OCT_5 , IDX_OCT_6 , IDX_OCT_7 , IDX_OCT_8 ,
  IDX_OCT_3 , IDX_OCT_2 , IDX_OCT_1 ,
  IDX_SLIDE_3 , IDX_SLIDE_2 , IDX_SLIDE_1 ,
  IDX_ACC_PROB_POT , IDX_ACC_AMT_POT , IDX_DENSITY_POT , IDX_SCALE_POT ,
  IDX_VELOCITY_POT , IDX_TEMPO_POT , IDX_ROOT_POT , IDX_BTN_ONOFF_TOG , IDX_BTN_EXTMIDI_TOG ,
  IDX_LOOP_START , IDX_DELTA_PITCH , IDX_DELTA_VEL , IDX_DELTA_OCT , IDX_DELTA_ACC ,
  IDX_BTN_CYC_R , IDX_BTN_RESET , IDX_BTN_INST , IDX_BTN_NONDEST , IDX_INST_POT , IDX_NONDEST_POT ,
  N_RAW_INPUTS
};


/* ───────────── 4. Physical lookup table  (abbrev) ────────── */
Input inputs[N_RAW_INPUTS] = {
  //MUX0
  {0,0 ,false,-1,false,-1}, //IDX_LOOP_END
  {0,3 ,false,-1,false,-1}, //IDX_DESTRUCT_POT
  {0,4 ,true ,-1,false,-1}, //IDX_BTN_DESTRUCT ***A7 LED MISTAKE***
  {0,5 ,true ,-1,false, 3}, //IDX_BTN_CYC_L
  {0,6 ,false,-1,false,-1}, //IDX_SLIDE_8
  {0,7 ,false,-1,false,-1}, //IDX_SLIDE_7
  {0,8 ,false,-1,false,-1}, //IDX_SLIDE_6
  {0,9 ,false,-1,false,-1}, //IDX_SLIDE_5
  {0,10,false,-1,false,-1}, //IDX_SLIDE_4
  {0,11,false,-1,false,-1}, //IDX_OCT_4
  {0,12,false,-1,false,-1}, //IDX_OCT_5
  {0,13,false,-1,false,-1}, //IDX_OCT_6
  {0,14,false,-1,false,-1}, //IDX_OCT_7
  {0,15,false,-1,false,-1}, //IDX_OCT_8

  //MUX1
  {1,0 ,false,-1,false,-1}, //IDX_OCT_3
  {1,1 ,false,-1,false,-1}, //IDX_OCT_2
  {1,2 ,false,-1,false,-1}, //IDX_OCT_1
  {1,3 ,false,-1,false,-1}, //IDX_SLIDE_3
  {1,4 ,false,-1,false,-1}, //IDX_SLIDE_2
  {1,5 ,false,-1,false,-1}, //IDX_SLIDE_1
  {1,6 ,false,-1,false,-1}, //IDX_ACC_PROB_POT
  {1,7 ,false,-1,false,-1}, //IDX_ACC_AMT_POT
  {1,8 ,false,-1,false,-1}, //IDX_DENSITY_POT
  {1,9 ,false,-1,false,-1}, //IDX_SCALE_POT
  {1,10,false,-1,false,-1}, //IDX_VELOCITY_POT
  {1,11,false,-1,false,-1}, //IDX_TEMPO_POT
  {1,12,false,-1,false,-1}, //IDX_ROOT_POT
  {1,13,true ,-1,false, 6}, //IDX_BTN_ONOFF_TOG
  {1,14,true ,-1,false, 7}, //IDX_BTN_EXTMIDI_TOG
  {1,15,false,-1,false,-1}, //IDX_LOOP_START

  //MUX2
  {2,0 ,false,-1,false,-1}, //IDX_DELTA_PITCH
  {2,2 ,false,-1,false,-1}, //IDX_DELTA_VEL
  {2,4 ,false,-1,false,-1}, //IDX_DELTA_OCT
  {2,6 ,false,-1,false,-1}, //IDX_DELTA_ACC
  {2,8 ,true ,-1,false, 5}, //IDX_BTN_CYC_R
  {2,9 ,true ,-1,false,-1}, // IDX_BTN_RESET   (should be 4 on a working pcb)
  {2,10,true ,-1,false, 2}, //IDX_BTN_INST
  {2,11,true ,-1,false, 1}, //IDX_BTN_NONDEST
  {2,14,false,-1,false,-1}, //IDX_INST_POT
  {2,15,false,-1,false,-1}  //IDX_NONDEST_POT
};

/* ───────────── 5. Public globals ------------------------------------ */
namespace hw {
    PotValues    pots;
    ButtonState  btnOnOff,          // ← NEW
                 btnExtMidi, btnDestruct, btnInstant, btnCopy,
                 btnCycleL,  btnCycleR,  btnReset;
}

/* ───────────── 6. Internal helpers  ───────────────────────────────── */
static inline int readMux(uint8_t m,uint8_t ch){
    digitalWrite(MUX_S0, bitRead(ch,0));
    digitalWrite(MUX_S1, bitRead(ch,1));
    digitalWrite(MUX_S2, bitRead(ch,2));
    digitalWrite(MUX_S3, bitRead(ch,3));
    delayMicroseconds(4);
    return analogRead(MUX_SIG[m]);
}

/* ───────────── 7. initPins()  ─────────────────────────────────────── */
void hw::initPins(){
    pinMode(MUX_S0,OUTPUT); pinMode(MUX_S1,OUTPUT);
    pinMode(MUX_S2,OUTPUT); pinMode(MUX_S3,OUTPUT);
    for(uint8_t p:LED_PINS){ pinMode(p,OUTPUT); digitalWrite(p,LOW); }
}

/* ───────────── 8. scanInputs()  ───────────────────────────────────── */
void hw::scanInputs()
{
    /* 8-A  read every control into inputs[] ------------------------- */
    for(int i=0;i<N_RAW_INPUTS;++i){
        int v = readMux(inputs[i].mux, inputs[i].ch);

        if(inputs[i].isButton){
            bool pressed = (v > 512);
            // edge detect for LED toggle buttons
            if(pressed && inputs[i].lastVal==0){
                inputs[i].led = !inputs[i].led;
                if(inputs[i].ledIdx >= 0)             //    only drive HW if it exists
                   digitalWrite(LED_PINS[inputs[i].ledIdx], inputs[i].led);
            }
            inputs[i].lastVal = pressed;
        }else{
            if(abs(v - inputs[i].lastVal) > 10) inputs[i].lastVal = v;
        }
    }

    /* 8-B  map raw pots to 0-100/127/…, fill PotValues -------------- */
    auto pot = [&](InIdx idx){ return inputs[idx].lastVal; };

    pots.pitchProb[0]          = map(pot(IDX_SLIDE_1), 0,1024,127,-1);
    pots.pitchProb[1]          = map(pot(IDX_SLIDE_2), 0,1024,127,-1);
    pots.pitchProb[2]          = map(pot(IDX_SLIDE_3), 0,1024,127,-1);
    pots.pitchProb[3]          = map(pot(IDX_SLIDE_4), 0,1024,127,-1);
    pots.pitchProb[4]          = map(pot(IDX_SLIDE_5), 0,1024,127,-1);
    pots.pitchProb[5]          = map(pot(IDX_SLIDE_6), 0,1024,127,-1);
    pots.pitchProb[6]          = map(pot(IDX_SLIDE_7), 0,1024,127,-1);
    pots.pitchProb[7]          = map(pot(IDX_SLIDE_8), 0,1024,127,-1);

    pots.octaveProb[0]         = map(pot(IDX_OCT_1),   0,1024, 0, 128);
    pots.octaveProb[1]         = map(pot(IDX_OCT_2),   0,1024, 0, 128);
    pots.octaveProb[2]         = map(pot(IDX_OCT_3),   0,1024, 0, 128);
    pots.octaveProb[3]         = map(pot(IDX_OCT_4),   0,1024, 0, 128);
    pots.octaveProb[4]         = map(pot(IDX_OCT_5),   0,1024, 0, 128);
    pots.octaveProb[5]         = map(pot(IDX_OCT_6),   0,1024, 0, 128);
    pots.octaveProb[6]         = map(pot(IDX_OCT_7),   0,1024, 0, 128);
    pots.octaveProb[7]         = map(pot(IDX_OCT_8),   0,1024, 0, 128);

    pots.density               = map(pot(IDX_DENSITY_POT), 0,1024, 0, 128);

    pots.deltaProb[0]          = map(pot(IDX_DELTA_PITCH), 0,1024, 127,-1);
    pots.deltaProb[1]          = map(pot(IDX_DELTA_VEL  ), 0,1024, 127,-1);
    pots.deltaProb[2]          = map(pot(IDX_DELTA_OCT  ), 0,1024, 127,-1);
    pots.deltaProb[3]          = map(pot(IDX_DELTA_ACC  ), 0,1024, 127,-1);

    pots.destructiveChance     = map(pot(IDX_DESTRUCT_POT), 0,1024, 0,128);
    pots.nondestChance         = map(pot(IDX_NONDEST_POT ), 0,1024, 0,128);
    pots.instChance            = map(pot(IDX_INST_POT    ), 0,1024, 0,128);
    pots.accentChance          = map(pot(IDX_ACC_PROB_POT ), 0,1024, 0,128);

    pots.bpm                   = map(pot(IDX_TEMPO_POT   ), 0,1023, 3,303);
    uint8_t ix = map(pot(IDX_TEMPO_POT), 0,1023, 0,8);   // 0-8
    pots.pulsesPerStep = kPpsLookup[ix];

    pots.loopStart             = map(pot(IDX_LOOP_START  ), 0,1024, 1, 17);
    pots.loopEnd               = map(pot(IDX_LOOP_END    ), 0,1024, 1, 17);
    pots.root                  = map(pot(IDX_ROOT_POT    ), 0,1024, 0,128);
    pots.velocity              = map(pot(IDX_VELOCITY_POT), 0,1024, 0,128);
    pots.accentVel             = map(pot(IDX_ACC_AMT_POT ), 0,1024, 0,128);
    pots.scale                 = map(pot(IDX_SCALE_POT   ), 0,1024, 1,  8);


    /* 8-C  buttons: level + edge ----------------------------------- */
    auto mapBtn = [&](ButtonState& b, InIdx idx){
        bool now = inputs[idx].lastVal;
        b.edge  = now && !b.level;
        b.level = now;
    };

    auto mapTgl = [&](ButtonState& b, InIdx idx){
        bool now = inputs[idx].led;         // latched ON / OFF
        b.edge  = now && !b.level;          // edge when it turns ON
        b.level = now;
    };

    mapTgl(btnOnOff    , IDX_BTN_ONOFF_TOG  );
    mapTgl(btnDestruct , IDX_BTN_DESTRUCT   );
    mapTgl(btnExtMidi  , IDX_BTN_EXTMIDI_TOG);

    mapBtn(btnInstant  , IDX_BTN_INST       );

    mapBtn(btnCopy     , IDX_BTN_NONDEST    );

    mapBtn(btnCycleL   , IDX_BTN_CYC_L      );
    mapBtn(btnCycleR   , IDX_BTN_CYC_R      );
    mapBtn(btnReset    , IDX_BTN_RESET      );

    auto flash = [&](uint8_t idx, uint8_t dur=4){
    if(idx < 8){ ledTimer[idx] = dur;
                 digitalWrite(LED_PINS[idx], HIGH); }
    };

    if(btnCycleL.edge) flash(3);
    if(btnCycleR.edge) flash(5);
    if(btnReset .edge) flash(4);
    if(btnInstant.edge)flash(2);
    if(btnCopy  .edge) flash(1);

    /* 8-D  tie Destructive ON to reset-LED ---------------------- */
    digitalWrite(LED_PINS[4], btnDestruct.level ? HIGH : LOW);   //  LED 5

    // run-down the timers
    for(uint8_t i=0;i<8;i++){
    if(ledTimer[i] && --ledTimer[i]==0)
        digitalWrite(LED_PINS[i], LOW);
}
}
