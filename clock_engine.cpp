#include "clock_engine.h"
#include "hw_inputs.h"
#include "sequencer.h"
#include <MIDI.h>

/*  exact extern using the namespace chosen by the library  */
extern MIDI_NAMESPACE::MidiInterface<
           MIDI_NAMESPACE::SerialMIDI<HardwareSerial>
       > MIDI;
using namespace MIDI_NAMESPACE;          // lets you write just “MIDI.send…”



namespace {

/* -------------------- constants --------------------- */
constexpr uint8_t PPQN = 24;                // MIDI clocks per quarter note

/* -------------------- state ------------------------- */
volatile uint8_t  extClkCount = 0;          // bumps in ISR
unsigned long     lastIntMicros = 0;        // phase accumulator

/* fast helpers */
inline void sendClockTick() { MIDI.sendRealTime(midi::Clock); }

} // unnamed namespace


/* ====================================================
   Public variables – initial defaults
   ==================================================== */
bool     clock::usingExt      = false;
uint16_t clock::bpm           = 120;
uint8_t  clock::pulsesPerStep = 6;          // default = 16th note


/* ====================================================
   MIDI-clock interrupt handlers
   ==================================================== */
static void handleMidiClock()   { extClkCount++; }
static void handleMidiStart()   { extClkCount = 0; }
static void handleMidiStop()    { /* do nothing – keeps usingExt untouched */ }


/* ====================================================
   init()
   ==================================================== */
void clock::init()
{
    MIDI.setHandleClock   (handleMidiClock);
    MIDI.setHandleStart   (handleMidiStart);
    MIDI.setHandleStop    (handleMidiStop);
    MIDI.begin(MIDI_CHANNEL_OMNI);
    lastIntMicros = micros();
}


/* ====================================================
   service()  – call every main loop()
   ==================================================== */
void clock::service()
{
    /* ---------- 1. Update settings from the UI layer ---------- */
    usingExt      = hw::btnExtMidi.level;          // EXT-MIDI toggle
    bpm           = hw::pots.bpm;                 // 30-303 mapped earlier
    pulsesPerStep = hw::pots.pulsesPerStep;       // filled in hw_inputs

    /* ---------- 2. External clock branch ---------------------- */
    if (usingExt)
    {
        if (extClkCount >= pulsesPerStep)
        {
            extClkCount = 0;
            seq::nextStep();
        }
        return;   // nothing else to do until next tick
    }

    /* ---------- 3. Internal clock branch ---------------------- */
    unsigned long now = micros();
    // microseconds per MIDI clock tick
    float usPerQuarter = 60.0f / bpm * 1e6f;
    float usPerTick    = usPerQuarter / PPQN;

    if (now - lastIntMicros >= usPerTick)
    {
        lastIntMicros += usPerTick;   // maintain phase
        sendClockTick();
        if (++extClkCount >= pulsesPerStep)
        {
            extClkCount = 0;
            seq::nextStep();
        }
    }
}
