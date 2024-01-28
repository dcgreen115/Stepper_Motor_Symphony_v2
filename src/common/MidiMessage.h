
// A message encoding a MIDI event, to be sent over serial
typedef struct MidiMessage {
    uint8_t status;
    uint8_t data[2];
} MidiMessage;

// A message's function represents a command for the receiving device. These will
// make up the upper 4 bits of the status byte
// Values taken from https://www.midi.org/specifications/midi-reference-tables/expanded-midi-1-0-messages-list-status-bytes
typedef enum Function {
    NOTE_OFF = 0x80,
    NOTE_ON = 0x90,
    POLYPHONIC_AFTERTOUCH = 0xA0,
    CONTROL_MODE_CHANGE = 0xB0,
    PROGRAM_CHANGE = 0xC0,
    CHANNEL_AFTERTOUCH = 0xD0,
    PITCH_BEND_CHANGE = 0xE0,
} Function;

// The channel associated with some MIDI functions. These will make up the lower
// 4 bits of the status byte
typedef enum Channel {
    CHANNEL0,
    CHANNEL1,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10,
    CHANNEL11,
    CHANNEL12,
    CHANNEL13,
    CHANNEL14,
    CHANNEL15,
} Channel;
