/*
  MaschineMk1UsbMidiBridge.ino

  Teensy 4.1 USB host-to-USB-MIDI bridge for Native Instruments Maschine Mk1.

  Arduino IDE compile settings:
    - Board: Teensy 4.1
    - USB Type: Serial + MIDI

  Host side:
    - USBHost_t36 on the Teensy 4.1 host port.
    - Custom USBDriver for the Maschine Mk1 vendor-specific interface.
    - EP1 handles buttons, encoders, LEDs, and DIN MIDI.
    - EP4 handles pad pressure frames.

  Device side:
    - Built-in Teensy usbMIDI object when Tools > USB Type includes MIDI.
*/

#include <Arduino.h>
#include <USBHost_t36.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// --------------------------------------------------------------------------
// Build-time configuration
// --------------------------------------------------------------------------

#define ENABLE_MIDI_TRANSLATION 1
#define ENABLE_DIN_MIDI_BRIDGE 1

#define ROUTE_CONTROL_MIDI_TO_USB 1
#define ROUTE_CONTROL_MIDI_TO_DIN_OUT 1
#define ROUTE_USB_MIDI_IN_TO_DIN_OUT 0
#define ROUTE_DIN_MIDI_IN_TO_USB 0
#define ROUTE_DIN_MIDI_IN_TO_DIN_OUT 0

#define LOG_RAW_PACKETS 0
#define LOG_RAW_EP1_PACKETS 0
#define LOG_RAW_EP4_PACKETS 0
#define SUPPRESS_DUPLICATE_RAW_LOGS 1
#define RAW_LOG_MAX_BYTES 256
#define LOG_BUTTONS 0
#define LOG_ENCODER_DECODING 0
#define LOG_ENCODER_PACKET_BYTES 0
#define LOG_PAD_PRESSURE_CHANGES 0
#define LOG_DIN_MIDI 0
#define LOG_INCOMING_LED_MIDI 0
#define LOG_INCOMING_ENCODER_MIDI 0
#define ENABLE_DISPLAY_METERS 1

#define LOG_DISPLAY_WRITES 0
#define LOG_DISPLAY_PROGRESS 0
#define DISPLAY_FRAME_PACKET_DELAY_MS 0
#define DISPLAY_OUT_TIMEOUT_MS 2000
#define DISPLAY_STRICT_TRANSFER_ERRORS 1
#define DISPLAY_METER_REFRESH_MIN_MS 33

// If EP1 GET_DEVICE_INFO never receives a reply, try setting this to 1.
// Linux writes len+1 bytes; some host stacks prefer fixed 64-byte frames.
#define SEND_EP1_COMMANDS_AS_64_BYTES 0

// Pad aftertouch modes. Maschine Mk1 EP4 pad frames carry continuous 12-bit
// pressure after the initial strike.
#define PAD_AFTERTOUCH_OFF 0
#define PAD_AFTERTOUCH_POLY_PRESSURE 1
#define PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD 2
#define PAD_AFTERTOUCH_CHANNEL_MAX 3
#define PAD_AFTERTOUCH_MODE PAD_AFTERTOUCH_POLY_PRESSURE

#if (PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_OFF) && \
    (PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_POLY_PRESSURE) && \
    (PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD) && \
    (PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_CHANNEL_MAX)
#error "PAD_AFTERTOUCH_MODE must be one of the PAD_AFTERTOUCH_* constants"
#endif

#define ENCODER_REVERSE_DIRECTION 0
#define ENABLE_ENCODER_JITTER_FILTER 1
#define ENABLE_ENCODER_SENSITIVITY_CURVE 1

// --------------------------------------------------------------------------
// Maschine Mk1 USB protocol constants
// --------------------------------------------------------------------------

static constexpr uint16_t MASCHINE_VID = 0x17CC;
static constexpr uint16_t MASCHINE_PID = 0x0808;

static constexpr uint8_t EP1_OUT_ADDR = 0x01;
static constexpr uint8_t EP1_IN_ADDR = 0x81;
static constexpr uint8_t EP4_IN_ADDR = 0x84;
static constexpr uint8_t EP_DISPLAY_OUT_ADDR = 0x08;

static constexpr uint16_t EP1_BUFSIZE = 64;
static constexpr uint16_t EP4_BUFSIZE = 512;

static constexpr uint8_t EP1_CMD_GET_DEVICE_INFO = 0x01;
static constexpr uint8_t EP1_CMD_READ_ERP = 0x02;
static constexpr uint8_t EP1_CMD_READ_ANALOG = 0x03;
static constexpr uint8_t EP1_CMD_READ_IO = 0x04;
static constexpr uint8_t EP1_CMD_WRITE_IO = 0x05;
static constexpr uint8_t EP1_CMD_MIDI_READ = 0x06;
static constexpr uint8_t EP1_CMD_MIDI_WRITE = 0x07;
static constexpr uint8_t EP1_CMD_AUTO_MSG = 0x0B;
static constexpr uint8_t EP1_CMD_DIMM_LEDS = 0x0C;

// Linux snd-usb-caiaq uses AUTO_MSG(1,10,5) for Maschine.
static constexpr uint8_t AUTO_MSG_DIGITAL = 1;
static constexpr uint8_t AUTO_MSG_ANALOG = 10;
static constexpr uint8_t AUTO_MSG_ERP = 5;

// Maschine DIN MIDI jacks use the same vendor EP1 command pipe as controls:
//   DIN IN from Maschine: [0x06, port, len, midi bytes...]
//   DIN OUT to Maschine:  [0x07, port, len, midi bytes...]
static constexpr uint8_t DIN_MIDI_PORT = 0;
static constexpr uint8_t DIN_MIDI_MAX_MESSAGE_BYTES = 3;
static constexpr uint8_t DIN_MIDI_OUT_QUEUE_SIZE = 64;
static constexpr uint8_t DIN_MIDI_OUT_MAX_RETRIES = 3;

// Linux caiaq control.c sends Maschine LED dimmer banks as:
//   [0x0C, 0x00, 32 LED bytes] for indices 0..31
//   [0x0C, 0x1E, 32 LED bytes] for indices 32..63
static constexpr uint8_t MASCHINE_LED_COUNT = 64;
static constexpr uint8_t MASCHINE_LED_BANK_SIZE = 32;
static constexpr uint8_t MASCHINE_LED_BANK0_ID = 0x00;
static constexpr uint8_t MASCHINE_LED_BANK1_ID = 0x1E;
static constexpr uint8_t MASCHINE_LED_MAX_BRIGHTNESS = 63;
static constexpr uint8_t MASCHINE_LED_INDEX_NONE = 0xFF;
static constexpr uint8_t MASCHINE_LED_BRIGHTNESS_UNKNOWN = 0xFF;
static constexpr uint8_t MASCHINE_DISPLAY_BACKLIGHT_LED_INDEX = 59;
static constexpr uint8_t MASCHINE_DISPLAY_BACKLIGHT_BRIGHTNESS = 63;

// Display protocol based on CABL MaschineMK1::initDisplay()/sendFrame().
static constexpr uint8_t MASCHINE_DISPLAYS = 2;
static constexpr uint16_t DISPLAY_WIDTH = 255;
static constexpr uint8_t DISPLAY_HEIGHT = 64;
static constexpr uint16_t DISPLAY_ROW_BYTES = 170;
static constexpr uint16_t DISPLAY_FRAME_BYTES = DISPLAY_ROW_BYTES * DISPLAY_HEIGHT;
static constexpr uint16_t DISPLAY_CHUNK_DATA_BYTES = 502;
static constexpr uint16_t DISPLAY_LAST_CHUNK_BYTES = 338;
static constexpr uint8_t DISPLAY_FRAME_CHUNK_COUNT = 22;
static constexpr uint8_t DISPLAY_INIT_STEPS = 22;
static constexpr uint16_t DISPLAY_OUT_BUFSIZE = 512;

static_assert(((uint32_t)(DISPLAY_FRAME_CHUNK_COUNT - 1) * DISPLAY_CHUNK_DATA_BYTES +
               DISPLAY_LAST_CHUNK_BYTES) == DISPLAY_FRAME_BYTES,
              "Maschine LCD frame chunking must cover exactly one full frame");

static constexpr uint8_t MASCHINE_PADS = 16;
static constexpr uint16_t PAD_PRESS_THRESHOLD = 200;
static constexpr uint16_t PAD_PEAK_CAPTURE_MS = 5;
static constexpr uint8_t PAD_ATTACK_MIN_CAPTURE_MS = 1;
static constexpr uint16_t PAD_ATTACK_PEAK_FALL_THRESHOLD = 24;
static constexpr uint8_t PAD_VELOCITY_PEAK_WEIGHT = 60;
static constexpr uint8_t PAD_VELOCITY_RISE_WEIGHT = 40;
static constexpr uint16_t PAD_VELOCITY_MAX_RISE = 4095 - PAD_PRESS_THRESHOLD;
static constexpr uint8_t PAD_MIDI_CHANNEL = 10;
static constexpr uint8_t PAD_BANK_COUNT = 8;
static constexpr uint8_t PAD_BANK_DEFAULT = 0;
static constexpr uint8_t PAD_BANK_LED_BRIGHTNESS = MASCHINE_LED_MAX_BRIGHTNESS;
static constexpr uint8_t PAD_LOCAL_FEEDBACK_BRIGHTNESS = MASCHINE_LED_MAX_BRIGHTNESS;
static constexpr uint8_t PAD_NOTE_NONE = 0xFF;
static constexpr uint8_t PAD_CHANNEL_NONE = 0;
static constexpr uint8_t PAD_AFTERTOUCH_VALUE_NONE = 0xFF;
static constexpr uint16_t PAD_AFTERTOUCH_PRESSURE_FLOOR = PAD_PRESS_THRESHOLD;
static constexpr uint8_t PAD_AFTERTOUCH_MIN_VALUE_DELTA = 2;
static constexpr uint16_t PAD_AFTERTOUCH_MIN_INTERVAL_MS = 12;
static constexpr uint8_t PAD_AFTERTOUCH_SMOOTHING_NUMERATOR = 4;
static constexpr uint8_t PAD_AFTERTOUCH_SMOOTHING_DENOMINATOR = 8;
static constexpr uint8_t PAD_FRAMES_PER_EP4_PACKET = 2;

static_assert(PAD_PRESS_THRESHOLD < 4095,
              "PAD_PRESS_THRESHOLD must leave usable pressure range for velocity");
static_assert((PAD_VELOCITY_PEAK_WEIGHT + PAD_VELOCITY_RISE_WEIGHT) > 0,
              "Pad velocity weights must not both be zero");
static_assert(PAD_AFTERTOUCH_SMOOTHING_DENOMINATOR != 0,
              "PAD_AFTERTOUCH_SMOOTHING_DENOMINATOR must be non-zero");

// CABL MaschineMK1::led(unsigned) maps incoming pad IDs to the matching LED.
// The high nibble in each pad pressure word is the same incoming pad ID.
static const uint8_t PAD_LED_INDEX[MASCHINE_PADS] = {
  15, 14, 13, 12,
  11, 10, 9, 8,
  7, 6, 5, 4,
  3, 2, 1, 0
};

static const uint8_t PAD_BANK_NOTE[PAD_BANK_COUNT][MASCHINE_PADS] = {
  {36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51},
  {52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67},
  {68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83},
  {84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99},
  {100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115},
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
  {16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31},
  {32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47}
};

static const uint8_t PAD_BANK_MIDI_CHANNEL[PAD_BANK_COUNT] = {
  10, 10, 10, 10, 10, 10, 10, 10
};

enum MaschineButton : uint8_t {
  BTN_MUTE = 0,
  BTN_SOLO = 1,
  BTN_SELECT = 2,
  BTN_DUPLICATE = 3,
  BTN_NAVIGATE = 4,
  BTN_PAD_MODE = 5,
  BTN_PATTERN = 6,
  BTN_SCENE = 7,
  BTN_REC = 9,
  BTN_ERASE = 10,
  BTN_SHIFT = 11,
  BTN_GRID = 12,
  BTN_TRANSPORT_RIGHT = 13,
  BTN_TRANSPORT_LEFT = 14,
  BTN_LOOP = 15,
  BTN_GROUP_E = 16,
  BTN_GROUP_F = 17,
  BTN_GROUP_G = 18,
  BTN_GROUP_H = 19,
  BTN_GROUP_D = 20,
  BTN_GROUP_C = 21,
  BTN_GROUP_B = 22,
  BTN_GROUP_A = 23,
  BTN_CONTROL = 24,
  BTN_BROWSE = 25,
  BTN_BROWSE_LEFT = 26,
  BTN_SNAP = 27,
  BTN_AUTOWRITE = 28,
  BTN_BROWSE_RIGHT = 29,
  BTN_SAMPLING = 30,
  BTN_STEP = 31,
  BTN_DISPLAY_8 = 32,
  BTN_DISPLAY_7 = 33,
  BTN_DISPLAY_6 = 34,
  BTN_DISPLAY_5 = 35,
  BTN_DISPLAY_4 = 36,
  BTN_DISPLAY_3 = 37,
  BTN_DISPLAY_2 = 38,
  BTN_DISPLAY_1 = 39,
  BTN_NOTE_REPEAT = 40,
  BTN_PLAY = 41,
  BTN_COUNT = 42
};

static constexpr uint8_t BUTTON_LOCAL_FEEDBACK_BRIGHTNESS = MASCHINE_LED_MAX_BRIGHTNESS;

static const uint8_t BUTTON_LED_INDEX[BTN_COUNT] = {
  16, 17, 18, 19, 20, 21, 22, 23,
  MASCHINE_LED_INDEX_NONE,
  28, 25, 24, 26, 27, 32, 33,
  39, 38, 35, 34, 36, 37, 40, 41,
  49, 47, 45, 43, 42, 44, 46, 48,
  50, 51, 52, 53, 54, 55, 56, 57,
  58, 29
};

struct PadBankButtonMap {
  uint8_t button;
  const char *name;
  uint8_t bank;
};

static const PadBankButtonMap PAD_BANK_BUTTONS[] = {
  {BTN_GROUP_A, "Group A", 0},
  {BTN_GROUP_B, "Group B", 1},
  {BTN_GROUP_C, "Group C", 2},
  {BTN_GROUP_D, "Group D", 3},
  {BTN_GROUP_E, "Group E", 4},
  {BTN_GROUP_F, "Group F", 5},
  {BTN_GROUP_G, "Group G", 6},
  {BTN_GROUP_H, "Group H", 7}
};

enum MidiMapKind : uint8_t {
  MAP_DISABLED = 0,
  MAP_CC = 1,
  MAP_NOTE = 2
};

static constexpr uint8_t TOP_BUTTON_CC_FIRST = 14;
static constexpr uint8_t TOP_BUTTON_CC_LAST = 21;
static constexpr uint8_t TOP_BUTTON_MIDI_CHANNEL = 1;
static constexpr uint8_t TOP_BUTTON_LED_MIDI_CHANNEL = 0;
static constexpr uint8_t TOP_BUTTON_1_LED_INDEX = 57;
static constexpr uint8_t SHIFT_BUTTON_INDEX = BTN_SHIFT;
static constexpr uint8_t SHIFTED_BUTTON_MIDI_CHANNEL = 2;
static constexpr uint8_t EXTRA_BUTTON_CC_FIRST = 30;
static constexpr uint8_t CONTROL_BUTTON_CC = 45;
static constexpr uint8_t BROWSE_BUTTON_CC = 46;
static constexpr uint8_t EXTRA_ENCODER_CC_FIRST = 42;

static constexpr uint8_t ENCODER_METER_FIRST_LOGICAL = 1;
static constexpr uint8_t ENCODER_METER_COUNT = 8;
static constexpr uint8_t ENCODER_LOGICAL_COUNT = 11;
static constexpr uint8_t ENCODER_VALUE_CHANNEL_COUNT = 2;
static constexpr uint8_t ENCODER_METER_CC_FIRST = 22;
static constexpr uint8_t ENCODER_METER_MIDI_CHANNEL = 1;
static constexpr uint8_t ENCODER_METER_DEFAULT_VALUE = 64;
static constexpr uint16_t ENCODER_POSITION_RANGE = 1000;
static constexpr uint16_t ENCODER_POSITION_HALF_RANGE = ENCODER_POSITION_RANGE / 2;
static constexpr uint8_t ENCODER_POSITION_UNITS_PER_MIDI_STEP = 7;
static constexpr uint8_t ENCODER_JITTER_POSITION_UNITS = 10;
static constexpr uint8_t ENCODER_DIRECTION_CHANGE_POSITION_UNITS = 24;
static constexpr uint16_t ENCODER_DIRECTION_LOCK_MS = 90;
static constexpr uint16_t ENCODER_ALIAS_RECOVERY_POSITION_UNITS = 430;
static constexpr uint16_t ENCODER_GLITCH_POSITION_UNITS = 800;
static constexpr uint16_t ENCODER_CURVE_MEDIUM_UNITS_PER_SECOND = 4500;
static constexpr uint16_t ENCODER_CURVE_FAST_UNITS_PER_SECOND = 9000;
static constexpr uint16_t ENCODER_CURVE_VERY_FAST_UNITS_PER_SECOND = 15000;
static constexpr uint8_t ENCODER_CURVE_MEDIUM_MULTIPLIER = 2;
static constexpr uint8_t ENCODER_CURVE_FAST_MULTIPLIER = 3;
static constexpr uint8_t ENCODER_CURVE_VERY_FAST_MULTIPLIER = 4;
static constexpr int16_t ENCODER_MAX_MIDI_DELTA_PER_UPDATE = 24;
static constexpr int32_t ENCODER_MAX_PENDING_MIDI_DELTA = 512;
static constexpr uint16_t ENCODER_OUTPUT_MIN_INTERVAL_MS = 1;

struct ButtonMidiMap {
  uint8_t button;
  const char *name;
  MidiMapKind kind;
  uint8_t number;
  uint8_t channel;
};

static const ButtonMidiMap BUTTON_MAP[] = {
  {BTN_DISPLAY_1, "Display 1", MAP_CC, TOP_BUTTON_CC_FIRST + 0, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_2, "Display 2", MAP_CC, TOP_BUTTON_CC_FIRST + 1, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_3, "Display 3", MAP_CC, TOP_BUTTON_CC_FIRST + 2, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_4, "Display 4", MAP_CC, TOP_BUTTON_CC_FIRST + 3, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_5, "Display 5", MAP_CC, TOP_BUTTON_CC_FIRST + 4, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_6, "Display 6", MAP_CC, TOP_BUTTON_CC_FIRST + 5, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_7, "Display 7", MAP_CC, TOP_BUTTON_CC_FIRST + 6, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_DISPLAY_8, "Display 8", MAP_CC, TOP_BUTTON_CC_FIRST + 7, TOP_BUTTON_MIDI_CHANNEL},
  {BTN_MUTE, "Mute", MAP_CC, 102, 1},
  {BTN_SOLO, "Solo", MAP_CC, 103, 1},
  {BTN_SELECT, "Select", MAP_CC, 104, 1},
  {BTN_DUPLICATE, "Duplicate", MAP_CC, EXTRA_BUTTON_CC_FIRST + 2, 1},
  {BTN_NAVIGATE, "Navigate", MAP_CC, EXTRA_BUTTON_CC_FIRST + 3, 1},
  {BTN_PAD_MODE, "Pad Mode", MAP_CC, 105, 1},
  {BTN_PATTERN, "Pattern", MAP_CC, EXTRA_BUTTON_CC_FIRST + 4, 1},
  {BTN_SCENE, "Scene", MAP_CC, EXTRA_BUTTON_CC_FIRST + 5, 1},
  {BTN_PLAY, "Play", MAP_NOTE, 117, 1},
  {BTN_REC, "Rec", MAP_NOTE, 118, 1},
  {BTN_ERASE, "Erase", MAP_CC, EXTRA_BUTTON_CC_FIRST + 0, 1},
  {BTN_GRID, "Grid", MAP_CC, EXTRA_BUTTON_CC_FIRST + 1, 1},
  {BTN_LOOP, "Loop/Restart", MAP_NOTE, 119, 1},
  {BTN_TRANSPORT_LEFT, "Transport Left", MAP_CC, 110, 1},
  {BTN_TRANSPORT_RIGHT, "Transport Right", MAP_CC, 111, 1},
  {BTN_CONTROL, "Control", MAP_CC, CONTROL_BUTTON_CC, 1},
  {BTN_BROWSE, "Browse", MAP_CC, BROWSE_BUTTON_CC, 1},
  {BTN_BROWSE_LEFT, "Browse Left", MAP_CC, EXTRA_BUTTON_CC_FIRST + 6, 1},
  {BTN_SNAP, "Snap", MAP_CC, EXTRA_BUTTON_CC_FIRST + 7, 1},
  {BTN_AUTOWRITE, "Autowrite", MAP_CC, EXTRA_BUTTON_CC_FIRST + 8, 1},
  {BTN_BROWSE_RIGHT, "Browse Right", MAP_CC, EXTRA_BUTTON_CC_FIRST + 9, 1},
  {BTN_SAMPLING, "Sampling", MAP_CC, EXTRA_BUTTON_CC_FIRST + 10, 1},
  {BTN_STEP, "Step", MAP_CC, EXTRA_BUTTON_CC_FIRST + 11, 1},
  {BTN_NOTE_REPEAT, "Note Repeat", MAP_CC, 112, 1}
};

struct EncoderMidiMap {
  uint8_t encoder;
  const char *name;
  uint8_t cc;
  uint8_t channel;
};

// Encoder packet format is CABL-derived:
//   report byte 0 == 0x02
//   eleven 16-bit values start at bytes 1..22
//   CABL maps raw pair index -> logical encoder as shown below.
static const uint8_t RAW_ENCODER_TO_LOGICAL[11] = {
  8, 4, 10, 7, 3, 9, 6, 2, 0, 5, 1
};

static const EncoderMidiMap ENCODER_MAP[] = {
  {0, "Encoder 0", EXTRA_ENCODER_CC_FIRST + 0, ENCODER_METER_MIDI_CHANNEL},
  {1, "Encoder 1", ENCODER_METER_CC_FIRST + 0, ENCODER_METER_MIDI_CHANNEL},
  {2, "Encoder 2", ENCODER_METER_CC_FIRST + 1, ENCODER_METER_MIDI_CHANNEL},
  {3, "Encoder 3", ENCODER_METER_CC_FIRST + 2, ENCODER_METER_MIDI_CHANNEL},
  {4, "Encoder 4", ENCODER_METER_CC_FIRST + 3, ENCODER_METER_MIDI_CHANNEL},
  {5, "Encoder 5", ENCODER_METER_CC_FIRST + 4, ENCODER_METER_MIDI_CHANNEL},
  {6, "Encoder 6", ENCODER_METER_CC_FIRST + 5, ENCODER_METER_MIDI_CHANNEL},
  {7, "Encoder 7", ENCODER_METER_CC_FIRST + 6, ENCODER_METER_MIDI_CHANNEL},
  {8, "Encoder 8", ENCODER_METER_CC_FIRST + 7, ENCODER_METER_MIDI_CHANNEL},
  {9, "Encoder 9", EXTRA_ENCODER_CC_FIRST + 1, ENCODER_METER_MIDI_CHANNEL},
  {10, "Encoder 10", EXTRA_ENCODER_CC_FIRST + 2, ENCODER_METER_MIDI_CHANNEL}
};

template <typename T, size_t N>
static constexpr size_t countof(const T (&)[N]) {
  return N;
}

#if ENABLE_DISPLAY_METERS
namespace MidiTranslation {
  const uint8_t *encoderMeterValues();
}

namespace DisplayGfx {
static void setDisplayPixel5Bit(uint8_t *frame, uint16_t x, uint8_t y, uint8_t mono) {
  if (!frame || x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT) return;

  uint8_t pixelValue = ((uint16_t)mono * 31 + 127) / 255;
  uint16_t byteIndex = (DISPLAY_ROW_BYTES * y) + ((x / 3) * 2);

  switch (x % 3) {
    case 0:
      frame[byteIndex] |= 0xF8;
      frame[byteIndex] &= (uint8_t)~(pixelValue << 3);
      break;
    case 1:
      frame[byteIndex] |= 0x07;
      frame[byteIndex + 1] |= 0xC0;
      frame[byteIndex] &= (uint8_t)~(pixelValue >> 2);
      frame[byteIndex + 1] &= (uint8_t)~(pixelValue << 6);
      break;
    default:
      frame[byteIndex + 1] |= 0x1F;
      frame[byteIndex + 1] &= (uint8_t)~pixelValue;
      break;
  }
}

static void drawDisplayPixel(uint8_t *frame, int16_t x, int16_t y) {
  if (x < 0 || y < 0) return;
  setDisplayPixel5Bit(frame, (uint16_t)x, (uint8_t)y, 255);
}

static void fillDisplayRect(uint8_t *frame, int16_t x, int16_t y, uint16_t w, uint8_t h) {
  for (uint8_t yy = 0; yy < h; yy++) {
    for (uint16_t xx = 0; xx < w; xx++) {
      drawDisplayPixel(frame, x + xx, y + yy);
    }
  }
}

static void drawDisplayRect(uint8_t *frame, int16_t x, int16_t y, uint16_t w, uint8_t h) {
  if (w == 0 || h == 0) return;
  fillDisplayRect(frame, x, y, w, 1);
  fillDisplayRect(frame, x, y + h - 1, w, 1);
  fillDisplayRect(frame, x, y, 1, h);
  fillDisplayRect(frame, x + w - 1, y, 1, h);
}

static uint8_t glyph5x7Row(char c, uint8_t row) {
  if (row >= 7) return 0;
  if (c >= 'a' && c <= 'z') c -= 32;

#define GLYPH(ch, r0, r1, r2, r3, r4, r5, r6) \
  case ch: { \
    static const uint8_t rows[7] = {r0, r1, r2, r3, r4, r5, r6}; \
    return rows[row]; \
  }

  switch (c) {
    GLYPH(' ', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
    GLYPH('-', 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00)
    GLYPH('#', 0x0A, 0x1F, 0x0A, 0x0A, 0x1F, 0x0A, 0x00)
    GLYPH('?', 0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04)
    GLYPH('0', 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E)
    GLYPH('1', 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E)
    GLYPH('2', 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F)
    GLYPH('3', 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E)
    GLYPH('4', 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02)
    GLYPH('5', 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E)
    GLYPH('6', 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E)
    GLYPH('7', 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08)
    GLYPH('8', 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E)
    GLYPH('9', 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C)
    GLYPH('A', 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11)
    GLYPH('B', 0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E)
    GLYPH('C', 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E)
    GLYPH('D', 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E)
    GLYPH('E', 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F)
    GLYPH('F', 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10)
    GLYPH('G', 0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E)
    GLYPH('H', 0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11)
    GLYPH('I', 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E)
    GLYPH('K', 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11)
    GLYPH('N', 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11)
    GLYPH('O', 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E)
    GLYPH('P', 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10)
    GLYPH('R', 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11)
    GLYPH('T', 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04)
    GLYPH('V', 0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04)
    default:
      return 0;
  }

#undef GLYPH
}

static void drawText5x7(uint8_t *frame, const char *text, int16_t x, int16_t y, uint8_t scale = 1) {
  if (!frame || !text || scale == 0) return;
  int16_t cursorX = x;
  while (*text) {
    char c = *text++;
    if (c == '\n') {
      cursorX = x;
      y += 8 * scale;
      continue;
    }

    for (uint8_t row = 0; row < 7; row++) {
      uint8_t bits = glyph5x7Row(c, row);
      for (uint8_t col = 0; col < 5; col++) {
        if (bits & (1 << (4 - col))) {
          fillDisplayRect(frame,
                          cursorX + (col * scale),
                          y + (row * scale),
                          scale,
                          scale);
        }
      }
    }
    cursorX += 6 * scale;
  }
}
}

namespace DisplayFrames {
static void drawCenteredText(uint8_t *frame, const char *text, uint16_t slotX, uint16_t slotW, int16_t y) {
  if (!text) return;
  uint16_t textW = strlen(text) ? ((uint16_t)strlen(text) * 6 - 1) : 0;
  int16_t x = slotX + ((slotW > textW) ? ((slotW - textW) / 2) : 0);
  DisplayGfx::drawText5x7(frame, text, x, y, 1);
}

static void buildEncoderMeterFrame(uint8_t *frame, uint8_t displayIndex) {
  memset(frame, 0xFF, DISPLAY_FRAME_BYTES);

  const uint8_t *values = MidiTranslation::encoderMeterValues();
  static constexpr uint8_t METERS_PER_DISPLAY = 4;
  static constexpr uint16_t SLOT_W = DISPLAY_WIDTH / METERS_PER_DISPLAY;
  static constexpr uint8_t BAR_Y = 47;
  static constexpr uint8_t BAR_W = 48;
  static constexpr uint8_t BAR_H = 10;
  static constexpr int16_t BUTTON_CC_Y = 1;
  static constexpr int16_t ENCODER_CC_Y = 38;

  for (uint8_t slot = 0; slot < METERS_PER_DISPLAY; slot++) {
    uint8_t meter = (displayIndex * METERS_PER_DISPLAY) + slot;
    if (meter >= ENCODER_METER_COUNT) continue;

    uint8_t value = values[meter];
    uint16_t slotX = slot * SLOT_W;
    uint16_t slotW = (slot == METERS_PER_DISPLAY - 1) ? (DISPLAY_WIDTH - slotX) : SLOT_W;
    uint16_t barX = slotX + ((slotW - BAR_W) / 2);

    char buttonCcLabel[8];
    snprintf(buttonCcLabel, sizeof(buttonCcLabel), "CC %u", (unsigned)(TOP_BUTTON_CC_FIRST + meter));
    drawCenteredText(frame, buttonCcLabel, slotX, slotW, BUTTON_CC_Y);

    char encoderCcLabel[8];
    snprintf(encoderCcLabel, sizeof(encoderCcLabel), "CC %u", (unsigned)(ENCODER_METER_CC_FIRST + meter));
    drawCenteredText(frame, encoderCcLabel, slotX, slotW, ENCODER_CC_Y);

    DisplayGfx::drawDisplayRect(frame, barX, BAR_Y, BAR_W, BAR_H);

    uint8_t innerW = BAR_W - 2;
    uint8_t filled = ((uint32_t)value * innerW + 63) / 127;
    if (filled > 0) {
      DisplayGfx::fillDisplayRect(frame, barX + 1, BAR_Y + 1, filled, BAR_H - 2);
    }

    uint16_t midX = barX + 1 + (innerW / 2);
    DisplayGfx::fillDisplayRect(frame, midX, BAR_Y + 1, 1, BAR_H - 2);
  }
}
}

static void buildDisplayMeterFrame(uint8_t *frame, uint8_t displayIndex) {
  DisplayFrames::buildEncoderMeterFrame(frame, displayIndex);
}
#endif

// --------------------------------------------------------------------------
// Diagnostic helpers
// --------------------------------------------------------------------------

static void printHex2(uint8_t b) {
  if (b < 16) Serial.print('0');
  Serial.print(b, HEX);
}

static void printHex4(uint16_t w) {
  if (w < 0x1000) Serial.print('0');
  if (w < 0x0100) Serial.print('0');
  if (w < 0x0010) Serial.print('0');
  Serial.print(w, HEX);
}

static const char *endpointTypeName(uint8_t attr) {
  switch (attr & 0x03) {
    case 0: return "CONTROL";
    case 1: return "ISO";
    case 2: return "BULK";
    case 3: return "INTERRUPT";
    default: return "?";
  }
}

static void printPacketLine(uint16_t offset, const uint8_t *data, uint16_t count) {
  if (offset < 0x1000) Serial.print('0');
  if (offset < 0x0100) Serial.print('0');
  if (offset < 0x0010) Serial.print('0');
  Serial.print(offset, HEX);
  Serial.print(": ");

  for (uint8_t i = 0; i < 16; i++) {
    if (i < count) {
      printHex2(data[i]);
      Serial.print(' ');
    } else {
      Serial.print("   ");
    }
  }

  Serial.print(" |");
  for (uint8_t i = 0; i < count; i++) {
    char c = (char)data[i];
    Serial.print((c >= 32 && c <= 126) ? c : '.');
  }
  Serial.println('|');
}

static void dumpBytes(const char *label, const uint8_t *data, uint32_t len, uint32_t maxBytes) {
  Serial.print(label);
  Serial.print(" len=");
  Serial.println(len);

  uint32_t show = len;
  if (show > maxBytes) show = maxBytes;
  for (uint32_t offset = 0; offset < show; offset += 16) {
    uint16_t lineCount = (show - offset >= 16) ? 16 : (show - offset);
    printPacketLine((uint16_t)offset, data + offset, lineCount);
  }
  if (show < len) {
    Serial.print("... truncated ");
    Serial.print(len - show);
    Serial.println(" bytes");
  }
}

static void printDescriptorChain(const uint8_t *descriptors, uint32_t len) {
  Serial.print("Descriptor chain bytes=");
  Serial.println(len);

  uint32_t i = 0;
  uint8_t currentInterface = 0xFF;
  uint8_t currentAlt = 0xFF;

  while (i + 2 <= len) {
    uint8_t bLength = descriptors[i];
    uint8_t bType = descriptors[i + 1];
    if (bLength == 0 || i + bLength > len) {
      Serial.println("  malformed descriptor chain");
      break;
    }

    Serial.print("  desc @");
    Serial.print(i);
    Serial.print(" len=");
    Serial.print(bLength);
    Serial.print(" type=0x");
    printHex2(bType);

    if (bType == 0x04 && bLength >= 9) {
      currentInterface = descriptors[i + 2];
      currentAlt = descriptors[i + 3];
      Serial.print(" INTERFACE if=");
      Serial.print(currentInterface);
      Serial.print(" alt=");
      Serial.print(currentAlt);
      Serial.print(" eps=");
      Serial.print(descriptors[i + 4]);
      Serial.print(" class=0x");
      printHex2(descriptors[i + 5]);
      Serial.print(" subclass=0x");
      printHex2(descriptors[i + 6]);
      Serial.print(" proto=0x");
      printHex2(descriptors[i + 7]);
    } else if (bType == 0x05 && bLength >= 7) {
      uint8_t addr = descriptors[i + 2];
      uint8_t attr = descriptors[i + 3];
      uint16_t mps = descriptors[i + 4] | ((uint16_t)descriptors[i + 5] << 8);
      Serial.print(" ENDPOINT if=");
      Serial.print(currentInterface);
      Serial.print(" alt=");
      Serial.print(currentAlt);
      Serial.print(" addr=0x");
      printHex2(addr);
      Serial.print(' ');
      Serial.print(endpointTypeName(attr));
      Serial.print(" wMaxPacket=");
      Serial.print(mps);
      Serial.print(" interval=");
      Serial.print(descriptors[i + 6]);
    } else if (bType == 0x21 && bLength >= 9) {
      Serial.print(" HID");
      Serial.print(" reportType=0x");
      printHex2(descriptors[i + 6]);
      Serial.print(" reportLen=");
      Serial.print(descriptors[i + 7] | ((uint16_t)descriptors[i + 8] << 8));
    } else if (bType == 0x24) {
      Serial.print(" CLASS-SPECIFIC");
    }
    Serial.println();

    i += bLength;
  }
}

// --------------------------------------------------------------------------
// MIDI translation layer
// --------------------------------------------------------------------------

static void setMaschineLed(uint8_t ledIndex, uint8_t brightness);
static bool queueMaschineDinMidiOut(const uint8_t *midi, uint8_t len);

namespace MidiTranslation {
  enum PadRuntimeState : uint8_t {
    PAD_IDLE = 0,
    PAD_CAPTURING = 1,
    PAD_HELD = 2
  };

  static uint16_t padPressure[MASCHINE_PADS];
  static PadRuntimeState padState[MASCHINE_PADS];
  static uint16_t padPeakPressure[MASCHINE_PADS];
  static uint16_t padPreviousPressure[MASCHINE_PADS];
  static uint16_t padAttackMaxRise[MASCHINE_PADS];
  static uint8_t padPeakElapsedMs[MASCHINE_PADS];
  static uint32_t padCaptureStartedMs[MASCHINE_PADS];
  static uint8_t padActiveNote[MASCHINE_PADS];
  static uint8_t padActiveChannel[MASCHINE_PADS];
  static uint8_t padLedBase[MASCHINE_PADS];
  static uint8_t padLedEffective[MASCHINE_PADS];
  static bool padLedLocalActive[MASCHINE_PADS];
  static bool padAftertouchActive[MASCHINE_PADS];
  static uint8_t padAftertouchSmoothed[MASCHINE_PADS];
  static uint8_t padAftertouchLastSent[MASCHINE_PADS];
  static uint32_t padAftertouchLastSentMs[MASCHINE_PADS];
  static uint8_t selectedPadBank = PAD_BANK_DEFAULT;

  static bool buttonPressed[BTN_COUNT];
  static uint8_t buttonActiveMidiChannel[BTN_COUNT];
  static uint8_t buttonLedBase[BTN_COUNT];
  static uint8_t buttonLedEffective[BTN_COUNT];
  static bool buttonLedLocalActive[BTN_COUNT];

  static uint8_t channelAftertouchLastSent[16];
  static uint32_t channelAftertouchLastSentMs[16];
  static uint16_t encoderValue[11];
  static uint16_t encoderPosition[11];
  static int16_t encoderPositionRemainder[11];
  static uint32_t encoderLastMoveMs[11];
  static int8_t encoderLastDirection[11];
  static int16_t encoderSuppressedReverseUnits[11];
  static int8_t encoderRemainderDirection[11];
  static int32_t encoderMidiDeltaPending[11];
  static uint32_t encoderLastOutputMs[11];
  static uint8_t encoderMidiValue[ENCODER_VALUE_CHANNEL_COUNT][ENCODER_LOGICAL_COUNT];
  static bool encoderMeterDirty = true;
  static bool encodersInitialized = false;

  static void drainQueuedEncoderMidiDeltas();

#if ENABLE_MIDI_TRANSLATION
  static uint8_t pressureToVelocity(uint16_t pressure) {
    if (pressure <= PAD_PRESS_THRESHOLD) return 1;
    if (pressure > 4095) pressure = 4095;
    long v = map((long)pressure, (long)PAD_PRESS_THRESHOLD, 4095L, 1L, 127L);
    if (v < 1) v = 1;
    if (v > 127) v = 127;
    return (uint8_t)v;
  }

  static uint8_t pressureRiseToVelocity(uint16_t rise) {
    if (rise == 0) return 1;
    if (rise > PAD_VELOCITY_MAX_RISE) rise = PAD_VELOCITY_MAX_RISE;
    long v = map((long)rise, 0L, (long)PAD_VELOCITY_MAX_RISE, 1L, 127L);
    if (v < 1) v = 1;
    if (v > 127) v = 127;
    return (uint8_t)v;
  }
#endif

  static uint8_t pressureToAftertouchValue(uint16_t pressure) {
    if (pressure <= PAD_AFTERTOUCH_PRESSURE_FLOOR) return 0;
    if (pressure > 4095) pressure = 4095;
    long v = map((long)pressure, (long)PAD_AFTERTOUCH_PRESSURE_FLOOR, 4095L, 0L, 127L);
    if (v < 0) v = 0;
    if (v > 127) v = 127;
    return (uint8_t)v;
  }

  static uint8_t smoothAftertouchValue(uint8_t current, uint8_t target) {
    int16_t delta = (int16_t)target - (int16_t)current;
    if (delta == 0) return current;

    int16_t step = (int16_t)(((int32_t)delta * PAD_AFTERTOUCH_SMOOTHING_NUMERATOR) /
                             PAD_AFTERTOUCH_SMOOTHING_DENOMINATOR);
    if (step == 0) step = (delta > 0) ? 1 : -1;

    int16_t next = (int16_t)current + step;
    if (next < 0) next = 0;
    if (next > 127) next = 127;
    return (uint8_t)next;
  }

  static bool captureWindowExpired(uint8_t pad) {
    return (uint32_t)(millis() - padCaptureStartedMs[pad]) >= PAD_PEAK_CAPTURE_MS;
  }

  static void beginPadAttackCapture(uint8_t pad, uint16_t pressure) {
    padState[pad] = PAD_CAPTURING;
    padPeakPressure[pad] = pressure;
    padPreviousPressure[pad] = pressure;
    padAttackMaxRise[pad] = (pressure > PAD_PRESS_THRESHOLD) ?
                             (uint16_t)(pressure - PAD_PRESS_THRESHOLD) : 0;
    padPeakElapsedMs[pad] = 0;
    padCaptureStartedMs[pad] = millis();
  }

  static void updatePadAttackCapture(uint8_t pad, uint16_t pressure) {
    uint32_t elapsed = millis() - padCaptureStartedMs[pad];
    if (elapsed > 255) elapsed = 255;

    if (pressure > padPreviousPressure[pad]) {
      uint16_t rise = pressure - padPreviousPressure[pad];
      if (rise > padAttackMaxRise[pad]) {
        padAttackMaxRise[pad] = rise;
      }
    }

    if (pressure > padPeakPressure[pad]) {
      padPeakPressure[pad] = pressure;
      padPeakElapsedMs[pad] = (uint8_t)elapsed;
    }

    padPreviousPressure[pad] = pressure;
  }

  static bool padAttackReady(uint8_t pad, uint16_t pressure) {
    if (captureWindowExpired(pad)) return true;

    uint32_t elapsed = millis() - padCaptureStartedMs[pad];
    if (elapsed < PAD_ATTACK_MIN_CAPTURE_MS) return false;
    if (pressure >= padPeakPressure[pad]) return false;

    return (uint16_t)(padPeakPressure[pad] - pressure) >= PAD_ATTACK_PEAK_FALL_THRESHOLD;
  }

  static void clearPadAttackCapture(uint8_t pad) {
    padPeakPressure[pad] = 0;
    padPreviousPressure[pad] = 0;
    padAttackMaxRise[pad] = 0;
    padPeakElapsedMs[pad] = 0;
    padCaptureStartedMs[pad] = 0;
  }

#if ENABLE_MIDI_TRANSLATION
  static uint16_t padAttackRiseForVelocity(uint8_t pad) {
    uint16_t rise = padAttackMaxRise[pad];
    uint16_t peakRise = (padPeakPressure[pad] > PAD_PRESS_THRESHOLD) ?
                        (uint16_t)(padPeakPressure[pad] - PAD_PRESS_THRESHOLD) : 0;
    uint8_t peakElapsed = padPeakElapsedMs[pad];

    // A fast strike can peak before the capture window ends. Scaling the
    // observed rise by elapsed time keeps short, hard taps from feeling weak.
    uint32_t timeScaledRise = peakRise;
    if (peakElapsed > 0 && peakElapsed < PAD_PEAK_CAPTURE_MS) {
      timeScaledRise = ((uint32_t)peakRise * PAD_PEAK_CAPTURE_MS + (peakElapsed / 2)) /
                       peakElapsed;
    }
    if (timeScaledRise > PAD_VELOCITY_MAX_RISE) {
      timeScaledRise = PAD_VELOCITY_MAX_RISE;
    }
    if (timeScaledRise > rise) rise = (uint16_t)timeScaledRise;
    return rise;
  }

  static uint8_t padAttackToVelocity(uint8_t pad) {
    uint8_t peakVelocity = pressureToVelocity(padPeakPressure[pad]);
    uint8_t riseVelocity = pressureRiseToVelocity(padAttackRiseForVelocity(pad));
    uint16_t weightTotal = PAD_VELOCITY_PEAK_WEIGHT + PAD_VELOCITY_RISE_WEIGHT;
    uint16_t weighted =
      (uint16_t)peakVelocity * PAD_VELOCITY_PEAK_WEIGHT +
      (uint16_t)riseVelocity * PAD_VELOCITY_RISE_WEIGHT;
    uint16_t velocity = (weighted + (weightTotal / 2)) / weightTotal;
    if (velocity < 1) velocity = 1;
    if (velocity > 127) velocity = 127;
    return (uint8_t)velocity;
  }
#endif

  static uint8_t clampLedBrightness(uint8_t brightness) {
    return (brightness > MASCHINE_LED_MAX_BRIGHTNESS) ?
           MASCHINE_LED_MAX_BRIGHTNESS : brightness;
  }

  static bool padToLedIndex(uint8_t pad, uint8_t *ledIndex) {
    if (pad >= MASCHINE_PADS) return false;
    uint8_t index = PAD_LED_INDEX[pad];
    if (index == MASCHINE_LED_INDEX_NONE) return false;
    if (ledIndex) *ledIndex = index;
    return true;
  }

  static bool buttonToLedIndex(uint8_t button, uint8_t *ledIndex) {
    if (button >= BTN_COUNT) return false;
    uint8_t index = BUTTON_LED_INDEX[button];
    if (index == MASCHINE_LED_INDEX_NONE) return false;
    if (ledIndex) *ledIndex = index;
    return true;
  }

  static void applyPadLed(uint8_t pad) {
    uint8_t ledIndex = 0;
    if (!padToLedIndex(pad, &ledIndex)) return;

    uint8_t brightness = padLedLocalActive[pad] ?
                         PAD_LOCAL_FEEDBACK_BRIGHTNESS : padLedBase[pad];
    brightness = clampLedBrightness(brightness);
    if (padLedEffective[pad] == brightness) return;

    padLedEffective[pad] = brightness;
    setMaschineLed(ledIndex, brightness);
  }

  static void applyButtonLed(uint8_t button) {
    uint8_t ledIndex = 0;
    if (!buttonToLedIndex(button, &ledIndex)) return;

    uint8_t brightness = buttonLedLocalActive[button] ?
                         BUTTON_LOCAL_FEEDBACK_BRIGHTNESS : buttonLedBase[button];
    brightness = clampLedBrightness(brightness);
    if (buttonLedEffective[button] == brightness) return;

    buttonLedEffective[button] = brightness;
    setMaschineLed(ledIndex, brightness);
  }

  static bool setPadLedBase(uint8_t pad, uint8_t brightness) {
    if (pad >= MASCHINE_PADS) return false;
    brightness = clampLedBrightness(brightness);
    bool changed = (padLedBase[pad] != brightness);
    padLedBase[pad] = brightness;
    if (changed || padLedEffective[pad] == MASCHINE_LED_BRIGHTNESS_UNKNOWN) {
      applyPadLed(pad);
    }
    return changed;
  }

  static bool setButtonLedBase(uint8_t button, uint8_t brightness) {
    if (button >= BTN_COUNT) return false;
    brightness = clampLedBrightness(brightness);
    bool changed = (buttonLedBase[button] != brightness);
    buttonLedBase[button] = brightness;
    if (changed || buttonLedEffective[button] == MASCHINE_LED_BRIGHTNESS_UNKNOWN) {
      applyButtonLed(button);
    }
    return changed;
  }

  bool setPadLedBaseFromMidi(uint8_t pad, uint8_t brightness) {
    return setPadLedBase(pad, brightness);
  }

  bool setButtonLedBaseFromMidi(uint8_t button, uint8_t brightness) {
    return setButtonLedBase(button, brightness);
  }

  static void setPadLocalFeedback(uint8_t pad, bool active) {
    if (pad >= MASCHINE_PADS) return;
    if (padLedLocalActive[pad] == active &&
        padLedEffective[pad] != MASCHINE_LED_BRIGHTNESS_UNKNOWN) {
      return;
    }

    padLedLocalActive[pad] = active;
    applyPadLed(pad);
  }

  static void setButtonLocalFeedback(uint8_t button, bool active) {
    if (button >= BTN_COUNT) return;
    if (buttonLedLocalActive[button] == active &&
        buttonLedEffective[button] != MASCHINE_LED_BRIGHTNESS_UNKNOWN) {
      return;
    }

    buttonLedLocalActive[button] = active;
    applyButtonLed(button);
  }

#if ENABLE_MIDI_TRANSLATION
  static uint8_t midiChannelToStatus(uint8_t baseStatus, uint8_t channel) {
    if (channel < 1 || channel > 16) channel = 1;
    return (uint8_t)(baseStatus | ((channel - 1) & 0x0F));
  }

  static void routeControlRawMidi(const uint8_t *msg, uint8_t len) {
#if ROUTE_CONTROL_MIDI_TO_DIN_OUT
    queueMaschineDinMidiOut(msg, len);
#else
    (void)msg;
    (void)len;
#endif
  }

  static void sendControlNoteOn(uint8_t note, uint8_t velocity, uint8_t channel) {
    uint8_t msg[3] = {
      midiChannelToStatus(0x90, channel),
      (uint8_t)(note & 0x7F),
      (uint8_t)(velocity & 0x7F)
    };
    routeControlRawMidi(msg, sizeof(msg));
#if ROUTE_CONTROL_MIDI_TO_USB
    usbMIDI.sendNoteOn(msg[1], msg[2], channel);
#endif
  }

  static void sendControlNoteOff(uint8_t note, uint8_t velocity, uint8_t channel) {
    uint8_t msg[3] = {
      midiChannelToStatus(0x80, channel),
      (uint8_t)(note & 0x7F),
      (uint8_t)(velocity & 0x7F)
    };
    routeControlRawMidi(msg, sizeof(msg));
#if ROUTE_CONTROL_MIDI_TO_USB
    usbMIDI.sendNoteOff(msg[1], msg[2], channel);
#endif
  }

  static void sendControlChange(uint8_t cc, uint8_t value, uint8_t channel) {
    uint8_t msg[3] = {
      midiChannelToStatus(0xB0, channel),
      (uint8_t)(cc & 0x7F),
      (uint8_t)(value & 0x7F)
    };
    routeControlRawMidi(msg, sizeof(msg));
#if ROUTE_CONTROL_MIDI_TO_USB
    usbMIDI.sendControlChange(msg[1], msg[2], channel);
#endif
  }

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_POLY_PRESSURE
  static void sendControlPolyPressure(uint8_t note, uint8_t pressure, uint8_t channel) {
    uint8_t msg[3] = {
      midiChannelToStatus(0xA0, channel),
      (uint8_t)(note & 0x7F),
      (uint8_t)(pressure & 0x7F)
    };
    routeControlRawMidi(msg, sizeof(msg));
#if ROUTE_CONTROL_MIDI_TO_USB
    usbMIDI.sendAfterTouchPoly(msg[1], msg[2], channel);
#endif
  }
#endif

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD || \
    PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
  static void sendControlChannelPressure(uint8_t pressure, uint8_t channel) {
    uint8_t msg[2] = {
      midiChannelToStatus(0xD0, channel),
      (uint8_t)(pressure & 0x7F)
    };
    routeControlRawMidi(msg, sizeof(msg));
#if ROUTE_CONTROL_MIDI_TO_USB
    usbMIDI.sendAfterTouch(msg[1], channel);
#endif
  }
#endif
#endif

  static const PadBankButtonMap *findPadBankButton(uint8_t button) {
    for (size_t i = 0; i < countof(PAD_BANK_BUTTONS); i++) {
      if (PAD_BANK_BUTTONS[i].button == button) return &PAD_BANK_BUTTONS[i];
    }
    return nullptr;
  }

  static uint8_t selectedPadBankSafe() {
    return (selectedPadBank < PAD_BANK_COUNT) ? selectedPadBank : PAD_BANK_DEFAULT;
  }

  static uint8_t padNoteForCurrentBank(uint8_t pad) {
    if (pad >= MASCHINE_PADS) return 0;
    return PAD_BANK_NOTE[selectedPadBankSafe()][pad];
  }

  static uint8_t padChannelForBank(uint8_t bank) {
    if (bank >= PAD_BANK_COUNT) bank = PAD_BANK_DEFAULT;
    uint8_t channel = PAD_BANK_MIDI_CHANNEL[bank];
    if (channel < 1 || channel > 16) channel = PAD_MIDI_CHANNEL;
    if (channel < 1 || channel > 16) channel = 10;
    return channel;
  }

  static uint8_t padChannelForCurrentPad(uint8_t pad) {
    (void)pad;
    return padChannelForBank(selectedPadBankSafe());
  }

  static void updatePadBankLeds() {
    for (size_t i = 0; i < countof(PAD_BANK_BUTTONS); i++) {
      uint8_t brightness = (PAD_BANK_BUTTONS[i].bank == selectedPadBankSafe()) ?
                           PAD_BANK_LED_BRIGHTNESS : 0;
      setButtonLedBase(PAD_BANK_BUTTONS[i].button, brightness);
    }
  }

  void refreshPadBankLeds() {
    updatePadBankLeds();
  }

  void forceRefreshControlLeds() {
    for (uint8_t pad = 0; pad < MASCHINE_PADS; pad++) {
      padLedEffective[pad] = MASCHINE_LED_BRIGHTNESS_UNKNOWN;
    }
    for (uint8_t button = 0; button < BTN_COUNT; button++) {
      buttonLedEffective[button] = MASCHINE_LED_BRIGHTNESS_UNKNOWN;
    }

    updatePadBankLeds();

    for (uint8_t pad = 0; pad < MASCHINE_PADS; pad++) {
      applyPadLed(pad);
    }
    for (uint8_t button = 0; button < BTN_COUNT; button++) {
      applyButtonLed(button);
    }
  }

  static bool handlePadBankButton(uint8_t button, bool pressed) {
    const PadBankButtonMap *bankButton = findPadBankButton(button);
    if (!bankButton) return false;

    if (pressed && bankButton->bank != selectedPadBank) {
      selectedPadBank = bankButton->bank;
      updatePadBankLeds();

#if LOG_BUTTONS
      Serial.print("[PAD BANK] selected ");
      Serial.println(bankButton->name);
#endif
    } else if (pressed) {
      updatePadBankLeds();
    }

    return true;
  }

  static void sendPadNoteOnFromAttack(uint8_t pad) {
#if ENABLE_MIDI_TRANSLATION
    uint8_t note = padNoteForCurrentBank(pad);
    uint8_t channel = padChannelForCurrentPad(pad);
    uint8_t velocity = padAttackToVelocity(pad);
    padActiveNote[pad] = note;
    padActiveChannel[pad] = channel;
    sendControlNoteOn(note, velocity, channel);
#else
    (void)pad;
#endif
  }

  static void sendPadNoteOff(uint8_t pad) {
#if ENABLE_MIDI_TRANSLATION
    uint8_t note = padActiveNote[pad];
    if (note == PAD_NOTE_NONE) {
      note = padNoteForCurrentBank(pad);
    }
    uint8_t channel = padActiveChannel[pad];
    if (channel < 1 || channel > 16) {
      channel = padChannelForCurrentPad(pad);
    }
    sendControlNoteOff(note, 0, channel);
    padActiveNote[pad] = PAD_NOTE_NONE;
    padActiveChannel[pad] = PAD_CHANNEL_NONE;
#else
    (void)pad;
#endif
  }

  static uint8_t absDiff8(uint8_t a, uint8_t b) {
    return (a > b) ? (a - b) : (b - a);
  }

  static bool aftertouchValueShouldSend(uint8_t lastValue,
                                        uint32_t lastSentMs,
                                        uint8_t value,
                                        bool force) {
    if (lastValue != PAD_AFTERTOUCH_VALUE_NONE && lastValue == value) return false;
    if (value == 0 && lastValue == PAD_AFTERTOUCH_VALUE_NONE) return false;
    if (force) return true;
    if (lastValue == PAD_AFTERTOUCH_VALUE_NONE) return true;
    if (absDiff8(lastValue, value) < PAD_AFTERTOUCH_MIN_VALUE_DELTA) return false;
    return (uint32_t)(millis() - lastSentMs) >= PAD_AFTERTOUCH_MIN_INTERVAL_MS;
  }

  static uint8_t activeChannelForPad(uint8_t pad) {
    if (pad >= MASCHINE_PADS) return padChannelForCurrentPad(0);
    uint8_t channel = padActiveChannel[pad];
    if (channel < 1 || channel > 16) channel = padChannelForCurrentPad(pad);
    if (channel < 1 || channel > 16) channel = 10;
    return channel;
  }

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_POLY_PRESSURE
  static bool activeNoteAndChannelForPad(uint8_t pad, uint8_t *note, uint8_t *channel) {
    if (pad >= MASCHINE_PADS || !padAftertouchActive[pad]) return false;
    uint8_t activeNote = padActiveNote[pad];
    if (activeNote == PAD_NOTE_NONE) return false;
    if (note) *note = activeNote;
    if (channel) *channel = activeChannelForPad(pad);
    return true;
  }

  static void maybeSendPolyAftertouch(uint8_t pad, uint8_t value, bool force) {
    uint8_t note = 0;
    uint8_t channel = 0;
    if (!activeNoteAndChannelForPad(pad, &note, &channel)) return;
    if (!aftertouchValueShouldSend(padAftertouchLastSent[pad],
                                   padAftertouchLastSentMs[pad],
                                   value,
                                   force)) {
      return;
    }

    padAftertouchLastSent[pad] = value;
    padAftertouchLastSentMs[pad] = millis();
#if ENABLE_MIDI_TRANSLATION
    sendControlPolyPressure(note, value, channel);
#endif
  }
#endif

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD || \
    PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
  static void maybeSendChannelAftertouch(uint8_t channel, uint8_t value, bool force) {
    if (channel < 1 || channel > 16) return;
    uint8_t index = channel - 1;
    if (!aftertouchValueShouldSend(channelAftertouchLastSent[index],
                                   channelAftertouchLastSentMs[index],
                                   value,
                                   force)) {
      return;
    }

    channelAftertouchLastSent[index] = value;
    channelAftertouchLastSentMs[index] = millis();
#if ENABLE_MIDI_TRANSLATION
    sendControlChannelPressure(value, channel);
#endif
  }
#endif

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
  static uint8_t aggregateChannelAftertouchValue(uint8_t channel) {
    uint8_t value = 0;
    if (channel < 1 || channel > 16) return value;
    for (uint8_t pad = 0; pad < MASCHINE_PADS; pad++) {
      if (!padAftertouchActive[pad]) continue;
      if (activeChannelForPad(pad) != channel) continue;
      if (padAftertouchSmoothed[pad] > value) value = padAftertouchSmoothed[pad];
    }
    return value;
  }
#endif

  static void sendAftertouchForPad(uint8_t pad, bool force) {
    if (pad >= MASCHINE_PADS || !padAftertouchActive[pad]) return;

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_POLY_PRESSURE
    maybeSendPolyAftertouch(pad, padAftertouchSmoothed[pad], force);
#elif PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD
    maybeSendChannelAftertouch(activeChannelForPad(pad), padAftertouchSmoothed[pad], force);
#elif PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
    uint8_t channel = activeChannelForPad(pad);
    maybeSendChannelAftertouch(channel, aggregateChannelAftertouchValue(channel), force);
#else
    (void)force;
#endif
  }

  static void beginPadAftertouch(uint8_t pad, uint16_t pressure) {
#if PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_OFF
    uint8_t value = pressureToAftertouchValue(pressure);
    padAftertouchActive[pad] = true;
    padAftertouchSmoothed[pad] = value;
    padAftertouchLastSent[pad] = PAD_AFTERTOUCH_VALUE_NONE;
    padAftertouchLastSentMs[pad] = 0;
    sendAftertouchForPad(pad, false);
#else
    (void)pad;
    (void)pressure;
#endif
  }

  static void updatePadAftertouch(uint8_t pad, uint16_t pressure) {
#if PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_OFF
    if (pad >= MASCHINE_PADS || !padAftertouchActive[pad]) return;
    uint8_t target = pressureToAftertouchValue(pressure);
    padAftertouchSmoothed[pad] = smoothAftertouchValue(padAftertouchSmoothed[pad], target);
    sendAftertouchForPad(pad, false);
#else
    (void)pad;
    (void)pressure;
#endif
  }

  static void releasePadAftertouch(uint8_t pad) {
#if PAD_AFTERTOUCH_MODE != PAD_AFTERTOUCH_OFF
    if (pad >= MASCHINE_PADS || !padAftertouchActive[pad]) return;

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_POLY_PRESSURE
    maybeSendPolyAftertouch(pad, 0, true);
#elif PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD
    uint8_t channel = activeChannelForPad(pad);
#elif PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
    uint8_t channel = activeChannelForPad(pad);
#endif

    padAftertouchActive[pad] = false;
    padAftertouchSmoothed[pad] = 0;
    padAftertouchLastSent[pad] = PAD_AFTERTOUCH_VALUE_NONE;
    padAftertouchLastSentMs[pad] = 0;

#if PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_ACTIVE_PAD
    maybeSendChannelAftertouch(channel, 0, true);
#elif PAD_AFTERTOUCH_MODE == PAD_AFTERTOUCH_CHANNEL_MAX
    maybeSendChannelAftertouch(channel, aggregateChannelAftertouchValue(channel), true);
#endif
#else
    (void)pad;
#endif
  }

  static const ButtonMidiMap *findButtonMap(uint8_t button) {
    for (size_t i = 0; i < countof(BUTTON_MAP); i++) {
      if (BUTTON_MAP[i].button == button) return &BUTTON_MAP[i];
    }
    return nullptr;
  }

  static const EncoderMidiMap *findEncoderMap(uint8_t encoder) {
    for (size_t i = 0; i < countof(ENCODER_MAP); i++) {
      if (ENCODER_MAP[i].encoder == encoder) return &ENCODER_MAP[i];
    }
    return nullptr;
  }

  static bool encoderMidiChannelToValueBank(uint8_t channel, uint8_t *bank);

  static const EncoderMidiMap *findEncoderMapByCc(uint8_t channel, uint8_t cc) {
    if (!encoderMidiChannelToValueBank(channel, nullptr)) return nullptr;
    for (size_t i = 0; i < countof(ENCODER_MAP); i++) {
      if (ENCODER_MAP[i].cc != cc) continue;
      return &ENCODER_MAP[i];
    }
    return nullptr;
  }

  static bool logicalEncoderToMeterIndex(uint8_t logical, uint8_t *index) {
    if (logical < ENCODER_METER_FIRST_LOGICAL) return false;
    uint8_t local = logical - ENCODER_METER_FIRST_LOGICAL;
    if (local >= ENCODER_METER_COUNT) return false;
    if (index) *index = local;
    return true;
  }

  static bool logicalEncoderToValueIndex(uint8_t logical, uint8_t *index) {
    if (logical >= ENCODER_LOGICAL_COUNT) return false;
    if (index) *index = logical;
    return true;
  }

  static uint8_t activeEncoderValueBank() {
    return buttonPressed[SHIFT_BUTTON_INDEX] ? 1 : 0;
  }

  static bool encoderMidiChannelToValueBank(uint8_t channel, uint8_t *bank) {
    if (channel == ENCODER_METER_MIDI_CHANNEL) {
      if (bank) *bank = 0;
      return true;
    }
    if (channel == SHIFTED_BUTTON_MIDI_CHANNEL) {
      if (bank) *bank = 1;
      return true;
    }
    return false;
  }

  bool setMappedEncoderValueFromUsbMidi(uint8_t channel, uint8_t cc, uint8_t value) {
    const EncoderMidiMap *mapEntry = findEncoderMapByCc(channel, cc);
    if (!mapEntry) return false;

    uint8_t valueBank = 0;
    if (!encoderMidiChannelToValueBank(channel, &valueBank)) return false;

    uint8_t index = 0;
    if (!logicalEncoderToValueIndex(mapEntry->encoder, &index)) return false;
    value &= 0x7F;
    if (encoderMidiValue[valueBank][index] == value) return true;
    encoderMidiValue[valueBank][index] = value;
    if (valueBank == activeEncoderValueBank() && logicalEncoderToMeterIndex(mapEntry->encoder, nullptr)) {
      encoderMeterDirty = true;
    }
    if (mapEntry->encoder < countof(encoderPositionRemainder)) {
      encoderPositionRemainder[mapEntry->encoder] = 0;
      encoderRemainderDirection[mapEntry->encoder] = 0;
      encoderMidiDeltaPending[mapEntry->encoder] = 0;
    }
    return true;
  }

  // Linux snd-usb-caiaq input.c decode_erp(): two 90-degree phase-shifted
  // analog tapers decoded to one circular 0..999 position.
  static uint16_t decodeMaschineErp(uint8_t a, uint8_t b) {
    static constexpr int HIGH_PEAK = 268;
    static constexpr int LOW_PEAK = -7;

    int range = HIGH_PEAK - LOW_PEAK;
    int deg90 = range / 2;
    int deg180 = range;
    int deg270 = deg90 + deg180;
    int deg360 = deg180 * 2;
    int midValue = (HIGH_PEAK + LOW_PEAK) / 2;

    int weightB = abs(midValue - (int)a) - (range / 2 - 100) / 2;
    if (weightB < 0) weightB = 0;
    if (weightB > 100) weightB = 100;
    int weightA = 100 - weightB;

    int posB;
    if (a < midValue) {
      posB = (int)b - LOW_PEAK + deg270;
      if (posB >= deg360) posB -= deg360;
    } else {
      posB = HIGH_PEAK - (int)b + deg90;
    }

    int posA;
    if (b > midValue) {
      posA = (int)a - LOW_PEAK;
    } else {
      posA = HIGH_PEAK - (int)a + deg180;
    }

    int ret = posA * weightA + posB * weightB;
    ret *= 10;
    ret /= deg360;
    if (ret < 0) ret += ENCODER_POSITION_RANGE;
    if (ret >= ENCODER_POSITION_RANGE) ret -= ENCODER_POSITION_RANGE;
    return (uint16_t)ret;
  }

  static int16_t abs16(int16_t v) {
    return (v < 0) ? -v : v;
  }

  static int8_t sign16(int16_t v) {
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
  }

  static int16_t encoderCircularDelta(uint16_t current, uint16_t previous) {
    int16_t delta = (int16_t)current - (int16_t)previous;
    if (delta > (int16_t)ENCODER_POSITION_HALF_RANGE) {
      delta -= ENCODER_POSITION_RANGE;
    } else if (delta < -(int16_t)ENCODER_POSITION_HALF_RANGE) {
      delta += ENCODER_POSITION_RANGE;
    }
#if ENCODER_REVERSE_DIRECTION
    delta = -delta;
#endif
    return delta;
  }

  static int16_t recoverEncoderAliasedDelta(uint8_t raw, int16_t delta, bool *aliasRecovered) {
    if (aliasRecovered) *aliasRecovered = false;
    if (raw >= countof(encoderLastDirection) || delta == 0) return delta;

    int16_t magnitude = abs16(delta);
    if (magnitude < (int16_t)ENCODER_ALIAS_RECOVERY_POSITION_UNITS) return delta;

    int8_t direction = sign16(delta);
    int8_t lastDirection = encoderLastDirection[raw];
    if (lastDirection == 0 || direction == lastDirection) return delta;

    uint32_t now = millis();
    if ((uint32_t)(now - encoderLastMoveMs[raw]) > ENCODER_DIRECTION_LOCK_MS) return delta;

    // Near the 0/999 wrap point, a rapid encoder movement can look like a huge
    // jump in the opposite direction. When timing and direction history agree,
    // use the alternate wrapped delta instead of treating it as reverse motion.
    int16_t alternate = (delta < 0) ?
                        (int16_t)(delta + ENCODER_POSITION_RANGE) :
                        (int16_t)(delta - ENCODER_POSITION_RANGE);
    if (sign16(alternate) != lastDirection) return delta;

    if (aliasRecovered) *aliasRecovered = true;
    return alternate;
  }

  static int16_t filterEncoderPositionDelta(uint8_t raw,
                                            int16_t delta,
                                            bool *jitterSuppressed,
                                            bool *glitchSuppressed) {
    if (jitterSuppressed) *jitterSuppressed = false;
    if (glitchSuppressed) *glitchSuppressed = false;
    if (delta == 0 || raw >= countof(encoderLastDirection)) return delta;

    int16_t magnitude = abs16(delta);
    if (magnitude > (int16_t)ENCODER_GLITCH_POSITION_UNITS) {
      if (glitchSuppressed) *glitchSuppressed = true;
      encoderSuppressedReverseUnits[raw] = 0;
      return 0;
    }

#if ENABLE_ENCODER_JITTER_FILTER
    uint32_t now = millis();
    int8_t direction = sign16(delta);
    int8_t lastDirection = encoderLastDirection[raw];
    bool opposite = (lastDirection != 0 && direction != 0 && direction != lastDirection);
    bool locked = (uint32_t)(now - encoderLastMoveMs[raw]) <= ENCODER_DIRECTION_LOCK_MS;

    if (opposite && locked && magnitude <= (int16_t)ENCODER_JITTER_POSITION_UNITS) {
      encoderSuppressedReverseUnits[raw] += delta;
      if (abs16(encoderSuppressedReverseUnits[raw]) < (int16_t)ENCODER_DIRECTION_CHANGE_POSITION_UNITS) {
        if (jitterSuppressed) *jitterSuppressed = true;
        return 0;
      }

      delta = encoderSuppressedReverseUnits[raw];
      encoderSuppressedReverseUnits[raw] = 0;
      direction = sign16(delta);
    } else {
      encoderSuppressedReverseUnits[raw] = 0;
    }

    if (direction != 0) {
      encoderLastDirection[raw] = direction;
      encoderLastMoveMs[raw] = now;
    }
#else
    encoderLastDirection[raw] = sign16(delta);
    encoderLastMoveMs[raw] = millis();
#endif

    return delta;
  }

  static uint32_t encoderSpeedUnitsPerSecond(int16_t positionDelta, uint32_t elapsedMs) {
    if (elapsedMs == 0) elapsedMs = 1;
    return ((uint32_t)abs16(positionDelta) * 1000UL) / elapsedMs;
  }

  static uint8_t encoderSensitivityMultiplier(int16_t positionDelta, uint32_t elapsedMs) {
#if ENABLE_ENCODER_SENSITIVITY_CURVE
    uint32_t speed = encoderSpeedUnitsPerSecond(positionDelta, elapsedMs);
    if (speed >= ENCODER_CURVE_VERY_FAST_UNITS_PER_SECOND) return ENCODER_CURVE_VERY_FAST_MULTIPLIER;
    if (speed >= ENCODER_CURVE_FAST_UNITS_PER_SECOND) return ENCODER_CURVE_FAST_MULTIPLIER;
    if (speed >= ENCODER_CURVE_MEDIUM_UNITS_PER_SECOND) return ENCODER_CURVE_MEDIUM_MULTIPLIER;
#else
    (void)positionDelta;
    (void)elapsedMs;
#endif
    return 1;
  }

  static int16_t encoderPositionDeltaToMidiDelta(uint8_t logical,
                                                 int16_t positionDelta,
                                                 uint8_t multiplier) {
    if (logical >= countof(encoderPositionRemainder)) return 0;
    if (ENCODER_POSITION_UNITS_PER_MIDI_STEP == 0) return 0;
    if (multiplier == 0) multiplier = 1;

    int8_t direction = sign16(positionDelta);
    if (direction != 0 && encoderRemainderDirection[logical] != 0 &&
        direction != encoderRemainderDirection[logical]) {
      encoderPositionRemainder[logical] = 0;
    }
    if (direction != 0) encoderRemainderDirection[logical] = direction;

    int32_t scaledPositionDelta = (int32_t)positionDelta * multiplier;
    int32_t accumulated = (int32_t)encoderPositionRemainder[logical] + scaledPositionDelta;
    int16_t midiDelta = (int16_t)(accumulated / ENCODER_POSITION_UNITS_PER_MIDI_STEP);
    encoderPositionRemainder[logical] =
      (int16_t)(accumulated - ((int32_t)midiDelta * ENCODER_POSITION_UNITS_PER_MIDI_STEP));
    return midiDelta;
  }

  static void queueEncoderMidiDelta(uint8_t logical, int16_t midiDelta) {
    if (logical >= countof(encoderMidiDeltaPending) || midiDelta == 0) return;
    int32_t pending = encoderMidiDeltaPending[logical] + (int32_t)midiDelta;
    if (pending > ENCODER_MAX_PENDING_MIDI_DELTA) {
      pending = ENCODER_MAX_PENDING_MIDI_DELTA;
    } else if (pending < -ENCODER_MAX_PENDING_MIDI_DELTA) {
      pending = -ENCODER_MAX_PENDING_MIDI_DELTA;
    }
    encoderMidiDeltaPending[logical] = pending;
  }

  static int16_t takeQueuedEncoderMidiDelta(uint8_t logical, bool force) {
    if (logical >= countof(encoderMidiDeltaPending)) return 0;
    int32_t pending = encoderMidiDeltaPending[logical];
    if (pending == 0) return 0;

    uint32_t now = millis();
    if (!force && encoderLastOutputMs[logical] != 0 &&
        (uint32_t)(now - encoderLastOutputMs[logical]) < ENCODER_OUTPUT_MIN_INTERVAL_MS) {
      return 0;
    }

    int16_t chunk;
    if (pending > ENCODER_MAX_MIDI_DELTA_PER_UPDATE) {
      chunk = ENCODER_MAX_MIDI_DELTA_PER_UPDATE;
    } else if (pending < -ENCODER_MAX_MIDI_DELTA_PER_UPDATE) {
      chunk = -ENCODER_MAX_MIDI_DELTA_PER_UPDATE;
    } else {
      chunk = (int16_t)pending;
    }

    encoderMidiDeltaPending[logical] = pending - chunk;
    encoderLastOutputMs[logical] = now;
    return chunk;
  }

  static bool applyEncoderDelta(uint8_t logical, uint8_t valueBank, int16_t delta, uint8_t *newValue) {
    uint8_t index = 0;
    if (valueBank >= ENCODER_VALUE_CHANNEL_COUNT) return false;
    if (!logicalEncoderToValueIndex(logical, &index)) return false;
    if (delta == 0) {
      if (newValue) *newValue = encoderMidiValue[valueBank][index];
      return false;
    }

    uint8_t oldValue = encoderMidiValue[valueBank][index];
    int16_t value = (int16_t)oldValue + delta;
    if (value < 0) value = 0;
    if (value > 127) value = 127;

    if (newValue) *newValue = (uint8_t)value;
    if (value == oldValue) return false;

    encoderMidiValue[valueBank][index] = (uint8_t)value;
    if (valueBank == activeEncoderValueBank() && logicalEncoderToMeterIndex(logical, nullptr)) {
      encoderMeterDirty = true;
    }
    return true;
  }

  const uint8_t *encoderMeterValues() {
    return encoderMidiValue[activeEncoderValueBank()] + ENCODER_METER_FIRST_LOGICAL;
  }

  bool takeEncoderMeterDirty() {
    bool dirty = encoderMeterDirty;
    encoderMeterDirty = false;
    return dirty;
  }

  static bool dispatchEncoderMidiDelta(uint8_t logical,
                                       int16_t midiDelta,
                                       uint8_t *midiValue,
                                       bool *valueChanged) {
    if (midiValue) *midiValue = 0;
    if (valueChanged) *valueChanged = false;
    if (midiDelta == 0) return false;

    const EncoderMidiMap *mapEntry = findEncoderMap(logical);
    uint8_t localMidiValue = 0;
    uint8_t *outValue = midiValue ? midiValue : &localMidiValue;
    uint8_t valueBank = activeEncoderValueBank();
    bool changed = applyEncoderDelta(logical, valueBank, midiDelta, outValue);
    if (valueChanged) *valueChanged = changed;
    if (!changed && logical < countof(encoderMidiDeltaPending)) {
      encoderMidiDeltaPending[logical] = 0;
    }
#if ENABLE_MIDI_TRANSLATION
    if (mapEntry && changed) {
      uint8_t channel = buttonPressed[SHIFT_BUTTON_INDEX] ? SHIFTED_BUTTON_MIDI_CHANNEL : mapEntry->channel;
      sendControlChange(mapEntry->cc, *outValue, channel);
    }
#else
    (void)mapEntry;
#endif
    return changed;
  }

  static void drainQueuedEncoderMidiDeltas() {
    for (uint8_t logical = 0; logical < countof(encoderMidiDeltaPending); logical++) {
      int16_t midiDelta = takeQueuedEncoderMidiDelta(logical, false);
      if (midiDelta == 0) continue;
      dispatchEncoderMidiDelta(logical, midiDelta, nullptr, nullptr);
    }
  }

  void reset() {
    memset(padPressure, 0, sizeof(padPressure));
    memset(padState, 0, sizeof(padState));
    memset(padPeakPressure, 0, sizeof(padPeakPressure));
    memset(padPreviousPressure, 0, sizeof(padPreviousPressure));
    memset(padAttackMaxRise, 0, sizeof(padAttackMaxRise));
    memset(padPeakElapsedMs, 0, sizeof(padPeakElapsedMs));
    memset(padCaptureStartedMs, 0, sizeof(padCaptureStartedMs));
    memset(padActiveNote, PAD_NOTE_NONE, sizeof(padActiveNote));
    memset(padActiveChannel, PAD_CHANNEL_NONE, sizeof(padActiveChannel));
    memset(padLedBase, 0, sizeof(padLedBase));
    memset(padLedEffective, MASCHINE_LED_BRIGHTNESS_UNKNOWN, sizeof(padLedEffective));
    memset(padLedLocalActive, 0, sizeof(padLedLocalActive));
    memset(padAftertouchActive, 0, sizeof(padAftertouchActive));
    memset(padAftertouchSmoothed, 0, sizeof(padAftertouchSmoothed));
    memset(padAftertouchLastSent, PAD_AFTERTOUCH_VALUE_NONE, sizeof(padAftertouchLastSent));
    memset(padAftertouchLastSentMs, 0, sizeof(padAftertouchLastSentMs));
    selectedPadBank = PAD_BANK_DEFAULT;

    memset(buttonPressed, 0, sizeof(buttonPressed));
    memset(buttonActiveMidiChannel, 0, sizeof(buttonActiveMidiChannel));
    memset(buttonLedBase, 0, sizeof(buttonLedBase));
    memset(buttonLedEffective, MASCHINE_LED_BRIGHTNESS_UNKNOWN, sizeof(buttonLedEffective));
    memset(buttonLedLocalActive, 0, sizeof(buttonLedLocalActive));

    memset(channelAftertouchLastSent, PAD_AFTERTOUCH_VALUE_NONE, sizeof(channelAftertouchLastSent));
    memset(channelAftertouchLastSentMs, 0, sizeof(channelAftertouchLastSentMs));
    memset(encoderValue, 0, sizeof(encoderValue));
    memset(encoderPosition, 0, sizeof(encoderPosition));
    memset(encoderPositionRemainder, 0, sizeof(encoderPositionRemainder));
    memset(encoderLastMoveMs, 0, sizeof(encoderLastMoveMs));
    memset(encoderLastDirection, 0, sizeof(encoderLastDirection));
    memset(encoderSuppressedReverseUnits, 0, sizeof(encoderSuppressedReverseUnits));
    memset(encoderRemainderDirection, 0, sizeof(encoderRemainderDirection));
    memset(encoderMidiDeltaPending, 0, sizeof(encoderMidiDeltaPending));
    memset(encoderLastOutputMs, 0, sizeof(encoderLastOutputMs));
    for (uint8_t bank = 0; bank < ENCODER_VALUE_CHANNEL_COUNT; bank++) {
      for (uint8_t i = 0; i < ENCODER_LOGICAL_COUNT; i++) {
        encoderMidiValue[bank][i] = ENCODER_METER_DEFAULT_VALUE;
      }
    }
    encoderMeterDirty = true;
    encodersInitialized = false;
  }

  void update() {
    for (uint8_t pad = 0; pad < MASCHINE_PADS; pad++) {
      if (padState[pad] == PAD_CAPTURING && captureWindowExpired(pad)) {
        sendPadNoteOnFromAttack(pad);
        beginPadAftertouch(pad, padPressure[pad]);
        padState[pad] = PAD_HELD;
      }
    }
    drainQueuedEncoderMidiDeltas();
  }

  static void processPadFrame(const uint8_t *data, uint8_t frameIndex) {
    // One pad frame is exactly 16 little-endian words:
    //   word = (pad_id << 12) | pressure_12_bit
    for (uint8_t i = 0; i < MASCHINE_PADS; i++) {
      uint16_t word = (uint16_t)data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8);
      uint8_t pad = (word >> 12) & 0x0F;
      uint16_t pressure = word & 0x0FFF;
      if (pad >= MASCHINE_PADS) continue;

#if LOG_PAD_PRESSURE_CHANGES
      if (pressure != padPressure[pad]) {
        Serial.print("[PAD] pad=");
        Serial.print(pad);
        Serial.print(" frame=");
        Serial.print(frameIndex);
        Serial.print(" pressure=");
        Serial.println(pressure);
      }
#else
      (void)frameIndex;
#endif

      if (pressure > PAD_PRESS_THRESHOLD) {
        if (padState[pad] == PAD_IDLE) {
          setPadLocalFeedback(pad, true);
          beginPadAttackCapture(pad, pressure);
        } else if (padState[pad] == PAD_CAPTURING) {
          updatePadAttackCapture(pad, pressure);
          if (padAttackReady(pad, pressure)) {
            sendPadNoteOnFromAttack(pad);
            beginPadAftertouch(pad, pressure);
            padState[pad] = PAD_HELD;
          }
        } else if (padState[pad] == PAD_HELD) {
          updatePadAftertouch(pad, pressure);
        }
      } else {
        if (padState[pad] == PAD_CAPTURING) {
          updatePadAttackCapture(pad, pressure);
          sendPadNoteOnFromAttack(pad);
          sendPadNoteOff(pad);
          setPadLocalFeedback(pad, false);
        } else if (padState[pad] == PAD_HELD) {
          releasePadAftertouch(pad);
          sendPadNoteOff(pad);
          setPadLocalFeedback(pad, false);
        }
        padState[pad] = PAD_IDLE;
        clearPadAttackCapture(pad);
      }
      padPressure[pad] = pressure;
    }
  }

  static void processPads(const uint8_t *data, uint32_t len) {
    static constexpr uint16_t PAD_FRAME_BYTES = MASCHINE_PADS * 2;
    if (len < PAD_FRAME_BYTES) return;

    uint8_t availableFrames = len / PAD_FRAME_BYTES;
    uint8_t framesToProcess = availableFrames;
    if (framesToProcess > PAD_FRAMES_PER_EP4_PACKET) {
      framesToProcess = PAD_FRAMES_PER_EP4_PACKET;
    }

    for (uint8_t frame = 0; frame < framesToProcess; frame++) {
      processPadFrame(data + (frame * PAD_FRAME_BYTES), frame);
    }
  }

  static bool buttonBit(const uint8_t *data, uint32_t len, uint8_t button) {
    uint32_t pos = 1 + (button >> 3);
    if (pos >= len) return false;
    return (data[pos] & (1u << (button & 7))) != 0;
  }

  static void sendButtonMidi(uint8_t button, bool pressed) {
    const ButtonMidiMap *mapEntry = findButtonMap(button);
    if (!mapEntry || mapEntry->kind == MAP_DISABLED) return;

#if ENABLE_MIDI_TRANSLATION
    uint8_t channel = mapEntry->channel;
    if (pressed) {
      if (button != SHIFT_BUTTON_INDEX && buttonPressed[SHIFT_BUTTON_INDEX]) {
        channel = SHIFTED_BUTTON_MIDI_CHANNEL;
      }
      buttonActiveMidiChannel[button] = channel;
    } else if (buttonActiveMidiChannel[button] != 0) {
      // Match button releases to the channel used by the press, even if Shift
      // was released before the held button.
      channel = buttonActiveMidiChannel[button];
      buttonActiveMidiChannel[button] = 0;
    }

    if (mapEntry->kind == MAP_CC) {
      sendControlChange(mapEntry->number, pressed ? 127 : 0, channel);
    } else if (mapEntry->kind == MAP_NOTE) {
      if (pressed) {
        sendControlNoteOn(mapEntry->number, 100, channel);
      } else {
        sendControlNoteOff(mapEntry->number, 0, channel);
      }
    }
#else
    (void)pressed;
#endif
  }

  static void processButtons(const uint8_t *data, uint32_t len) {
    if (len < 8) return;
    if (data[0] != EP1_CMD_READ_IO && data[0] != 0x04) return;

    // CABL observed byte 6 bit 0x40 as a validity marker for button reports.
    if ((data[6] & 0x40) == 0) return;

    for (uint8_t b = 0; b < BTN_COUNT; b++) {
      if (b == 8) continue; // CABL spacer, not a real button
      bool pressed = buttonBit(data, len, b);
      if (pressed == buttonPressed[b]) continue;

      buttonPressed[b] = pressed;
      if (b == SHIFT_BUTTON_INDEX) {
        // Pending encoder deltas are not channel-tagged. Drop partial movement
        // when switching value banks so channel 1 and 2 values stay separate.
        memset(encoderPositionRemainder, 0, sizeof(encoderPositionRemainder));
        memset(encoderRemainderDirection, 0, sizeof(encoderRemainderDirection));
        memset(encoderMidiDeltaPending, 0, sizeof(encoderMidiDeltaPending));
        encoderMeterDirty = true;
      }
      setButtonLocalFeedback(b, pressed);
      if (handlePadBankButton(b, pressed)) continue;
      sendButtonMidi(b, pressed);

#if LOG_BUTTONS
      Serial.print("[BUTTON] index=");
      Serial.print(b);
      Serial.print(" state=");
      Serial.println(pressed ? "down" : "up");
#endif
    }
  }

  static void processEncoders(const uint8_t *data, uint32_t len) {
    if (len < 23) return;
    if (data[0] != EP1_CMD_READ_ERP && data[0] != 0x02) return;

#if LOG_ENCODER_PACKET_BYTES
    Serial.print("[ENCODER PACKET] len=");
    Serial.println(len);
    dumpBytes("      erp", data, len, 32);
#endif

    for (uint8_t raw = 0; raw < 11; raw++) {
      uint8_t pair0 = data[1 + (2 * raw)];
      uint8_t pair1 = data[2 + (2 * raw)];
      uint16_t currentRawPair = ((uint16_t)pair0 << 8) | pair1;
      uint16_t currentPosition = decodeMaschineErp(pair1, pair0);
      uint8_t logical = RAW_ENCODER_TO_LOGICAL[raw];
      uint32_t now = millis();

      if (!encodersInitialized) {
        encoderValue[raw] = currentRawPair;
        encoderPosition[raw] = currentPosition;
        encoderLastMoveMs[raw] = now;
        if (logical < countof(encoderLastOutputMs)) {
          encoderLastOutputMs[logical] = now;
        }
        continue;
      }

      uint16_t previousRawPair = encoderValue[raw];
      uint16_t previousPosition = encoderPosition[raw];
      if (currentRawPair == previousRawPair && currentPosition == previousPosition) continue;

      uint32_t elapsedMs = encoderLastMoveMs[raw] ?
                           (uint32_t)(now - encoderLastMoveMs[raw]) : 1;
      if (elapsedMs == 0) elapsedMs = 1;

      int16_t rawPositionDelta = encoderCircularDelta(currentPosition, previousPosition);
      bool aliasRecovered = false;
      int16_t recoveredPositionDelta =
        recoverEncoderAliasedDelta(raw, rawPositionDelta, &aliasRecovered);
      bool jitterSuppressed = false;
      bool glitchSuppressed = false;
      int16_t filteredPositionDelta =
        filterEncoderPositionDelta(raw, recoveredPositionDelta, &jitterSuppressed, &glitchSuppressed);
      uint8_t multiplier = encoderSensitivityMultiplier(filteredPositionDelta, elapsedMs);
      int16_t queuedMidiDelta =
        encoderPositionDeltaToMidiDelta(logical, filteredPositionDelta, multiplier);
      queueEncoderMidiDelta(logical, queuedMidiDelta);
      int16_t midiDelta = takeQueuedEncoderMidiDelta(logical, true);

      const EncoderMidiMap *mapEntry = findEncoderMap(logical);
      uint8_t midiValue = 0;
      bool valueChanged = false;
      dispatchEncoderMidiDelta(logical, midiDelta, &midiValue, &valueChanged);

#if LOG_ENCODER_DECODING
      uint32_t speed = encoderSpeedUnitsPerSecond(filteredPositionDelta, elapsedMs);
      Serial.print("[ENCODER] raw=");
      Serial.print(raw);
      Serial.print(" logical=");
      Serial.print(logical);
      if (mapEntry) {
        Serial.print(" name=");
        Serial.print(mapEntry->name);
      }
      Serial.print(" erp=");
      Serial.print(currentPosition);
      Serial.print(" prev=");
      Serial.print(previousPosition);
      Serial.print(" elapsedMs=");
      Serial.print(elapsedMs);
      Serial.print(" speed=");
      Serial.print(speed);
      Serial.print(" rawD=");
      Serial.print(rawPositionDelta);
      if (aliasRecovered) {
        Serial.print(" aliasD=");
        Serial.print(recoveredPositionDelta);
      }
      Serial.print(" filtD=");
      Serial.print(filteredPositionDelta);
      Serial.print(" x");
      Serial.print(multiplier);
      Serial.print(" queuedMidi=");
      Serial.print(queuedMidiDelta);
      Serial.print(" sentMidi=");
      Serial.print(midiDelta);
      if (jitterSuppressed) Serial.print(" jitter=1");
      if (glitchSuppressed) Serial.print(" glitch=1");
      if (valueChanged) Serial.print(" changed=1");
      if (mapEntry) {
        Serial.print(" cc=");
        Serial.print(mapEntry->cc);
        Serial.print(" value=");
        Serial.print(midiValue);
      }
      Serial.println();
#else
      (void)mapEntry;
      (void)midiValue;
      (void)valueChanged;
      (void)aliasRecovered;
      (void)jitterSuppressed;
      (void)glitchSuppressed;
#endif

      encoderValue[raw] = currentRawPair;
      encoderPosition[raw] = currentPosition;
    }
    encodersInitialized = true;
  }

  static uint8_t explicitMidiMessageLength(uint8_t status) {
    if (status >= 0x80 && status < 0xF0) {
      switch (status & 0xF0) {
        case 0xC0:
        case 0xD0:
          return 2;
        default:
          return 3;
      }
    }

    switch (status) {
      case 0xF1: return 2;
      case 0xF2: return 3;
      case 0xF3: return 2;
      case 0xF6: return 1;
      default: return 0;
    }
  }

  static bool midiDataBytesValid(const uint8_t *msg, uint8_t msgLen) {
    for (uint8_t i = 1; i < msgLen; i++) {
      if (msg[i] & 0x80) return false;
    }
    return true;
  }

  static void sendExplicitMidiToUsb(const uint8_t *msg, uint8_t msgLen) {
#if ENABLE_MIDI_TRANSLATION
    if (!msg || msgLen == 0) return;
    uint8_t status = msg[0];

    if (status >= 0x80 && status < 0xF0) {
      uint8_t channel = (status & 0x0F) + 1;
      switch (status & 0xF0) {
        case 0x80:
          if (msgLen >= 3) usbMIDI.sendNoteOff(msg[1], msg[2], channel);
          break;
        case 0x90:
          if (msgLen >= 3) usbMIDI.sendNoteOn(msg[1], msg[2], channel);
          break;
        case 0xA0:
          if (msgLen >= 3) usbMIDI.sendAfterTouchPoly(msg[1], msg[2], channel);
          break;
        case 0xB0:
          if (msgLen >= 3) usbMIDI.sendControlChange(msg[1], msg[2], channel);
          break;
        case 0xC0:
          if (msgLen >= 2) usbMIDI.sendProgramChange(msg[1], channel);
          break;
        case 0xD0:
          if (msgLen >= 2) usbMIDI.sendAfterTouch(msg[1], channel);
          break;
        case 0xE0:
          if (msgLen >= 3) {
            int bend = (((uint16_t)(msg[2] & 0x7F) << 7) | (msg[1] & 0x7F)) - 8192;
            usbMIDI.sendPitchBend(bend, channel);
          }
          break;
        default:
          break;
      }
      return;
    }

    switch (status) {
      case 0xF1:
        if (msgLen >= 2) usbMIDI.sendTimeCodeQuarterFrame((msg[1] >> 4) & 0x07, msg[1] & 0x0F);
        break;
      case 0xF2:
        if (msgLen >= 3) usbMIDI.sendSongPosition((uint16_t)(msg[1] & 0x7F) | ((uint16_t)(msg[2] & 0x7F) << 7));
        break;
      case 0xF3:
        if (msgLen >= 2) usbMIDI.sendSongSelect(msg[1] & 0x7F);
        break;
      case 0xF6:
        usbMIDI.sendTuneRequest();
        break;
      default:
        break;
    }
#else
    (void)msg;
    (void)msgLen;
#endif
  }

  static void routeExplicitMidiMessage(const uint8_t *msg,
                                       uint8_t msgLen,
                                       bool routeToUsb,
                                       bool routeToDinOut) {
    if (!msg || msgLen == 0) return;
    if (routeToUsb) {
      sendExplicitMidiToUsb(msg, msgLen);
    }
    if (routeToDinOut) {
      queueMaschineDinMidiOut(msg, msgLen);
    }
  }

  static void processExplicitMidiBytes(const uint8_t *bytes,
                                       uint8_t len,
                                       bool routeToUsb,
                                       bool routeToDinOut) {
    if (!routeToUsb && !routeToDinOut) return;

    uint8_t i = 0;
    while (i < len) {
      uint8_t status = bytes[i];
      if (status < 0x80) {
        i++;
        continue;
      }

      uint8_t msgLen = explicitMidiMessageLength(status);
      if (msgLen == 0) {
        i++;
        continue;
      }
      if (i + msgLen > len) {
        break;
      }

      if (midiDataBytesValid(bytes + i, msgLen)) {
        routeExplicitMidiMessage(bytes + i, msgLen, routeToUsb, routeToDinOut);
        i += msgLen;
      } else {
        i++;
      }
    }
  }

  void processDinMidiInReport(const uint8_t *data, uint32_t len) {
    if (!data || len < 3) return;
    if (data[0] != EP1_CMD_MIDI_READ) return;

    uint8_t port = data[1];
    uint8_t midiLen = data[2];
    if (midiLen > len - 3) {
      midiLen = len - 3;
    }

#if LOG_DIN_MIDI
    Serial.print("[DIN MIDI IN] port=");
    Serial.print(port);
    Serial.print(" len=");
    Serial.println(midiLen);
    dumpBytes("      midi", data + 3, midiLen, 48);
#else
    (void)port;
#endif

    processExplicitMidiBytes(data + 3,
                             midiLen,
                             ROUTE_DIN_MIDI_IN_TO_USB,
                             ROUTE_DIN_MIDI_IN_TO_DIN_OUT);
#if ENABLE_MIDI_TRANSLATION && ROUTE_DIN_MIDI_IN_TO_USB
    usbMIDI.send_now();
#endif
  }

  static void processEp1Command(const uint8_t *data, uint32_t len) {
    if (len == 0) return;
    switch (data[0]) {
      case EP1_CMD_READ_IO:
        processButtons(data, len);
        break;
      case EP1_CMD_READ_ERP:
        processEncoders(data, len);
        break;
      case EP1_CMD_READ_ANALOG:
        break;
      case EP1_CMD_MIDI_READ:
        processDinMidiInReport(data, len);
        break;
      default:
        break;
    }
  }

  void processPacket(uint8_t endpoint, const uint8_t *data, uint32_t len) {
    if (endpoint == EP4_IN_ADDR) {
      processPads(data, len);
    } else if (endpoint == EP1_IN_ADDR) {
      processEp1Command(data, len);
    }
  }
}

static bool topButtonCcToLedIndex(uint8_t cc, uint8_t *ledIndex) {
  if (cc < TOP_BUTTON_CC_FIRST || cc > TOP_BUTTON_CC_LAST) return false;
  if (ledIndex) {
    *ledIndex = TOP_BUTTON_1_LED_INDEX - (cc - TOP_BUTTON_CC_FIRST);
  }
  return true;
}

static bool topButtonCcToButtonIndex(uint8_t cc, uint8_t *buttonIndex) {
  if (cc < TOP_BUTTON_CC_FIRST || cc > TOP_BUTTON_CC_LAST) return false;
  if (buttonIndex) {
    *buttonIndex = BTN_DISPLAY_1 - (cc - TOP_BUTTON_CC_FIRST);
  }
  return true;
}

namespace RawCapture {
#if LOG_RAW_PACKETS
  struct EndpointCache {
    uint8_t endpoint = 0;
    uint8_t last[EP4_BUFSIZE];
    uint16_t lastLen = 0;
    bool haveLast = false;
  };

  static EndpointCache ep1Cache;
  static EndpointCache ep4Cache;

  static EndpointCache *cacheFor(uint8_t endpoint) {
    if (endpoint == EP1_IN_ADDR) return &ep1Cache;
    if (endpoint == EP4_IN_ADDR) return &ep4Cache;
    return nullptr;
  }

  static bool isDuplicate(EndpointCache *cache, const uint8_t *data, uint32_t len) {
    if (!cache || !cache->haveLast) return false;
    if (cache->lastLen != len) return false;
    if (len > sizeof(cache->last)) return false;
    return memcmp(cache->last, data, len) == 0;
  }

  static void remember(EndpointCache *cache, uint8_t endpoint, const uint8_t *data, uint32_t len) {
    if (!cache) return;
    cache->endpoint = endpoint;
    cache->haveLast = true;
    cache->lastLen = (len > sizeof(cache->last)) ? sizeof(cache->last) : len;
    memcpy(cache->last, data, cache->lastLen);
  }
#endif

  void logPacket(uint8_t endpoint, const uint8_t *data, uint32_t len) {
#if LOG_RAW_PACKETS
#if !LOG_RAW_EP1_PACKETS
    if (endpoint == EP1_IN_ADDR) return;
#endif
#if !LOG_RAW_EP4_PACKETS
    if (endpoint == EP4_IN_ADDR) return;
#endif
    EndpointCache *cache = cacheFor(endpoint);
#if SUPPRESS_DUPLICATE_RAW_LOGS
    if (isDuplicate(cache, data, len)) return;
#endif
    Serial.print("[RAW] ep=0x");
    printHex2(endpoint);
    Serial.print(" len=");
    Serial.println(len);
    dumpBytes("      data", data, len, RAW_LOG_MAX_BYTES);
    remember(cache, endpoint, data, len);
#else
    (void)endpoint;
    (void)data;
    (void)len;
#endif
  }

  void handlePacket(uint8_t endpoint, const uint8_t *data, uint32_t len) {
    logPacket(endpoint, data, len);
    MidiTranslation::processPacket(endpoint, data, len);
  }

  void reset() {
#if LOG_RAW_PACKETS
    ep1Cache = EndpointCache();
    ep4Cache = EndpointCache();
#endif
  }
}

// --------------------------------------------------------------------------
// USBHost_t36 custom Maschine Mk1 vendor-specific driver
// --------------------------------------------------------------------------

class MaschineMk1Driver : public USBDriver {
  struct DinMidiOutMessage {
    uint8_t port;
    uint8_t len;
    uint8_t data[DIN_MIDI_MAX_MESSAGE_BYTES];
  };

public:
  explicit MaschineMk1Driver(USBHost &) : initTimer(this) {
    init();
  }

  bool connected() const {
    return device != nullptr;
  }

  void update() {
    flushDinMidiOut();
    flushLedUpdates();
#if ENABLE_DISPLAY_METERS
    if (MidiTranslation::takeEncoderMeterDirty()) {
      requestDisplayMeterRefresh(false);
    }
    if (displayRefreshPending && displayJob == DISPLAY_IDLE && !displayOutBusy && !displayWaiting) {
      requestDisplayMeterRefresh(false);
    }
    advanceDisplay();
#endif
  }

  void setLed(uint8_t ledIndex, uint8_t brightness) {
    if (ledIndex >= MASCHINE_LED_COUNT) return;
    if (brightness > MASCHINE_LED_MAX_BRIGHTNESS) {
      brightness = MASCHINE_LED_MAX_BRIGHTNESS;
    }
    if (ledState[ledIndex] == brightness) return;

    ledState[ledIndex] = brightness;
    ledBankDirty[ledIndex >= MASCHINE_LED_BANK_SIZE] = true;
    flushLedUpdates();
  }

  bool queueDinMidiOut(const uint8_t *midi, uint8_t len, uint8_t port = DIN_MIDI_PORT) {
#if ENABLE_DIN_MIDI_BRIDGE
    if (!device) return false;
    if (!midi || len == 0 || len > DIN_MIDI_MAX_MESSAGE_BYTES) return false;
    if (dinMidiOutCount >= DIN_MIDI_OUT_QUEUE_SIZE) {
      Serial.println("[DIN MIDI OUT] queue full; message dropped");
      return false;
    }

    DinMidiOutMessage &slot = dinMidiOutQueue[dinMidiOutHead];
    slot.port = port;
    slot.len = len;
    memset(slot.data, 0, sizeof(slot.data));
    memcpy(slot.data, midi, len);
    dinMidiOutHead = (dinMidiOutHead + 1) % DIN_MIDI_OUT_QUEUE_SIZE;
    dinMidiOutCount++;

    flushDinMidiOut();
    return true;
#else
    (void)midi;
    (void)len;
    (void)port;
    return false;
#endif
  }

protected:
  bool claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len) override {
    if (dev->idVendor != MASCHINE_VID || dev->idProduct != MASCHINE_PID) return false;

    if (!printedDevice) {
      printedDevice = true;
      Serial.println();
      Serial.println("Native Instruments Maschine Mk1 candidate detected");
      Serial.print("  VID:PID=");
      printHex4(dev->idVendor);
      Serial.print(':');
      printHex4(dev->idProduct);
      Serial.print(" deviceClass=0x");
      printHex2(dev->bDeviceClass);
      Serial.print(" subclass=0x");
      printHex2(dev->bDeviceSubClass);
      Serial.print(" protocol=0x");
      printHex2(dev->bDeviceProtocol);
      Serial.print(" speed=");
      Serial.println(dev->speed == 2 ? "high" : (dev->speed == 1 ? "low" : "full"));
      Serial.println("  Treating host side as Native Instruments vendor-specific USB, not USB-MIDI.");
    }

    Serial.print("USBHost_t36 claim callback type=");
    Serial.print(type);
    Serial.print(" len=");
    Serial.println(len);
    printDescriptorChain(descriptors, len);

    // Claim only at interface level. The Linux driver then sets interface 0 to
    // alternate setting 1 before using the bulk endpoints.
    if (type != 1) return false;
    if (len < 9 || descriptors[0] != 9 || descriptors[1] != 4) return false;

    uint8_t interfaceNumber = descriptors[2];
    static constexpr uint8_t MASCHINE_INTERFACE_ALT = 1;

    uint8_t foundEp1In = 0;
    uint8_t foundEp1Out = 0;
    uint8_t foundEp4In = 0;
    uint8_t foundDisplayOut = 0;
    uint16_t ep1InSize = EP1_BUFSIZE;
    uint16_t ep1OutSize = EP1_BUFSIZE;
    uint16_t ep4InSize = EP4_BUFSIZE;
    uint16_t displayOutSize = DISPLAY_OUT_BUFSIZE;
    bool sawHidReportDescriptor = false;
    uint8_t currentInterface = interfaceNumber;
    uint8_t currentAlt = 0;

    for (uint32_t i = 0; i + 2 <= len;) {
      uint8_t bLength = descriptors[i];
      uint8_t bType = descriptors[i + 1];
      if (bLength == 0 || i + bLength > len) break;

      if (bType == 0x04 && bLength >= 9) {
        currentInterface = descriptors[i + 2];
        currentAlt = descriptors[i + 3];
      } else if (bType == 0x05 && bLength >= 7 &&
                 currentInterface == interfaceNumber &&
                 currentAlt == MASCHINE_INTERFACE_ALT) {
        uint8_t ep = descriptors[i + 2];
        uint8_t attr = descriptors[i + 3];
        uint16_t mps = descriptors[i + 4] | ((uint16_t)descriptors[i + 5] << 8);
        if ((attr & 0x03) == 2) {
          if (ep == EP1_IN_ADDR) {
            foundEp1In = ep;
            ep1InSize = mps;
          } else if (ep == EP1_OUT_ADDR) {
            foundEp1Out = ep;
            ep1OutSize = mps;
          } else if (ep == EP4_IN_ADDR) {
            foundEp4In = ep;
            ep4InSize = mps;
          } else if (ep == EP_DISPLAY_OUT_ADDR) {
            foundDisplayOut = ep;
            displayOutSize = mps;
          }
        }
      } else if (bType == 0x21 && bLength >= 9) {
        sawHidReportDescriptor = true;
        hidReportInterface = interfaceNumber;
        hidReportLength = descriptors[i + 7] | ((uint16_t)descriptors[i + 8] << 8);
      }
      i += bLength;
    }

    if (!foundEp1In || !foundEp1Out) {
      Serial.println("  Not claiming: EP1 IN/OUT bulk command pair not found in this interface chain.");
      return false;
    }

    if (!foundEp4In) {
      Serial.println("  EP4 IN not found in this chain; will try documented EP4 IN 0x84.");
      foundEp4In = EP4_IN_ADDR;
      ep4InSize = EP4_BUFSIZE;
    }

    if (!foundDisplayOut) {
      Serial.println("  EP8/display OUT 0x08 not found; LCD meters will be disabled.");
    }

    Serial.println("  Claiming Maschine Mk1 interface");
    Serial.print("  interface=");
    Serial.print(interfaceNumber);
    Serial.print(" target alt=");
    Serial.println(MASCHINE_INTERFACE_ALT);

    device = dev;
    bInterfaceNumber = interfaceNumber;
    bAlternateSetting = MASCHINE_INTERFACE_ALT;
    ep1InPacketSize = ep1InSize;
    ep1OutPacketSize = ep1OutSize;
    ep4InPacketSize = ep4InSize;
    epDisplayOutPacketSize = displayOutSize;
    hasHidReportDescriptor = sawHidReportDescriptor;

    ep1InPipe = new_Pipe(dev, 2, EP1_IN_ADDR & 0x0F, 1, ep1InPacketSize);
    ep1OutPipe = new_Pipe(dev, 2, EP1_OUT_ADDR & 0x0F, 0, ep1OutPacketSize);
    ep4InPipe = new_Pipe(dev, 2, EP4_IN_ADDR & 0x0F, 1, ep4InPacketSize);
    epDisplayOutPipe = foundDisplayOut ? new_Pipe(dev, 2, EP_DISPLAY_OUT_ADDR & 0x0F, 0, epDisplayOutPacketSize) : nullptr;

    if (!ep1InPipe || !ep1OutPipe || !ep4InPipe || (foundDisplayOut && !epDisplayOutPipe)) {
      Serial.println("  ERROR: failed to allocate USBHost_t36 pipes");
      device = nullptr;
      return false;
    }

    ep1InPipe->callback_function = ep1InCallback;
    ep1InPipe->error_callback_function = ep1InCallback;
    ep1OutPipe->callback_function = ep1OutCallback;
    ep1OutPipe->error_callback_function = ep1OutCallback;
    ep4InPipe->callback_function = ep4InCallback;
    ep4InPipe->error_callback_function = ep4InCallback;
#if ENABLE_DISPLAY_METERS
    if (epDisplayOutPipe) {
      epDisplayOutPipe->callback_function = displayOutCallback;
      epDisplayOutPipe->error_callback_function = displayOutCallback;
    }
#endif

    pendingControl = CTRL_SET_INTERFACE;
    mk_setup(setup, 0x01, 11, bAlternateSetting, bInterfaceNumber, 0);
    if (!queue_Control_Transfer(dev, &setup, nullptr, this)) {
      Serial.println("  ERROR: failed to queue SET_INTERFACE alt 1");
      pendingControl = CTRL_NONE;
      return false;
    }

    state = STATE_SETTING_INTERFACE;
    return true;
  }

  void control(const Transfer_t *transfer) override {
    (void)transfer;
    if (pendingControl == CTRL_SET_INTERFACE) {
      Serial.println("SET_INTERFACE alt 1 complete");
      pendingControl = CTRL_NONE;

      if (hasHidReportDescriptor && hidReportLength > 0) {
        uint16_t requestLen = hidReportLength;
        if (requestLen > sizeof(hidReportBuffer)) requestLen = sizeof(hidReportBuffer);
        hidReportBytesRequested = requestLen;
        Serial.print("Requesting HID report descriptor len=");
        Serial.println(requestLen);
        pendingControl = CTRL_GET_HID_REPORT;
        mk_setup(setup, 0x81, 0x06, 0x2200, hidReportInterface, requestLen);
        if (queue_Control_Transfer(device, &setup, hidReportBuffer, this)) {
          return;
        }
        Serial.println("  HID report descriptor request could not be queued");
        pendingControl = CTRL_NONE;
      }

      startCommandEndpoint();
      return;
    }

    if (pendingControl == CTRL_GET_HID_REPORT) {
      Serial.println("HID report descriptor:");
      dumpBytes("      report", hidReportBuffer, hidReportBytesRequested, sizeof(hidReportBuffer));
      pendingControl = CTRL_NONE;
      startCommandEndpoint();
      return;
    }
  }

  void timer_event(USBDriverTimer *whichTimer) override {
    if (whichTimer == &initTimer && state == STATE_WAITING_DEVICE_INFO) {
      Serial.println("GET_DEVICE_INFO timed out; sending AUTO_MSG anyway");
      continueAfterDeviceInfo();
    }
  }

  void disconnect() override {
    Serial.println("Maschine Mk1 disconnected");
    device = nullptr;
    ep1InPipe = nullptr;
    ep1OutPipe = nullptr;
    ep4InPipe = nullptr;
    epDisplayOutPipe = nullptr;
    ep1OutBusy = false;
    ep1OutCommand = 0;
    ep1OutExpectedLen = 0;
#if ENABLE_DISPLAY_METERS
    resetDisplayState();
#endif
    dinMidiOutHead = 0;
    dinMidiOutTail = 0;
    dinMidiOutCount = 0;
    dinMidiOutInFlight = false;
    dinMidiOutRetryCount = 0;
    pendingAutoStart = false;
    printedDevice = false;
    pendingControl = CTRL_NONE;
    state = STATE_DISCONNECTED;
    hasHidReportDescriptor = false;
    hidReportInterface = 0;
    hidReportLength = 0;
    hidReportBytesRequested = 0;
    bInterfaceNumber = 0;
    bAlternateSetting = 0;
    ep1InPacketSize = EP1_BUFSIZE;
    ep1OutPacketSize = EP1_BUFSIZE;
    ep4InPacketSize = EP4_BUFSIZE;
    epDisplayOutPacketSize = DISPLAY_OUT_BUFSIZE;
    resetLedState();
    RawCapture::reset();
    MidiTranslation::reset();
  }

private:
  enum DriverState : uint8_t {
    STATE_DISCONNECTED,
    STATE_SETTING_INTERFACE,
    STATE_WAITING_DEVICE_INFO,
    STATE_DISPLAYING_TEST,
    STATE_RUNNING
  };

  enum PendingControl : uint8_t {
    CTRL_NONE,
    CTRL_SET_INTERFACE,
    CTRL_GET_HID_REPORT
  };

#if ENABLE_DISPLAY_METERS
  enum DisplayJob : uint8_t {
    DISPLAY_IDLE,
    DISPLAY_INIT,
    DISPLAY_FRAME_BEGIN,
    DISPLAY_FRAME_PRE1,
    DISPLAY_FRAME_PRE2,
    DISPLAY_FRAME_SENDING
  };
#endif

  void init() {
    contribute_Pipes(pipes, countof(pipes));
    contribute_Transfers(transfers, countof(transfers));
    contribute_String_Buffers(stringBuffers, countof(stringBuffers));
    driver_ready_for_device(this);
  }

  static uint32_t actualLength(const Transfer_t *transfer) {
    uint32_t remaining = (transfer->qtd.token >> 16) & 0x7FFF;
    if (remaining > transfer->length) return 0;
    return transfer->length - remaining;
  }

  static void ep1InCallback(const Transfer_t *transfer) {
    if (!transfer || !transfer->driver) return;
    static_cast<MaschineMk1Driver *>(transfer->driver)->handleEp1In(transfer);
  }

  static void ep1OutCallback(const Transfer_t *transfer) {
    if (!transfer || !transfer->driver) return;
    static_cast<MaschineMk1Driver *>(transfer->driver)->handleEp1Out(transfer);
  }

  static void ep4InCallback(const Transfer_t *transfer) {
    if (!transfer || !transfer->driver) return;
    static_cast<MaschineMk1Driver *>(transfer->driver)->handleEp4In(transfer);
  }

#if ENABLE_DISPLAY_METERS
  static void displayOutCallback(const Transfer_t *transfer) {
    if (!transfer || !transfer->driver) return;
    static_cast<MaschineMk1Driver *>(transfer->driver)->handleDisplayOut(transfer);
  }
#endif

  void startCommandEndpoint() {
    if (!device || !ep1InPipe || !ep1OutPipe) return;
    Serial.println("Starting EP1 command IN loop");
    queueEp1In();
    state = STATE_WAITING_DEVICE_INFO;
    sendEp1Command(EP1_CMD_GET_DEVICE_INFO, nullptr, 0);
    initTimer.start(1500000);
  }

  void queueEp1In() {
    if (!device || !ep1InPipe) return;
    if (!queue_Data_Transfer(ep1InPipe, ep1InBuffer, EP1_BUFSIZE, this)) {
      Serial.println("ERROR: queue EP1 IN failed");
    }
  }

  void queueEp4In() {
    if (!device || !ep4InPipe) return;
    if (!queue_Data_Transfer(ep4InPipe, ep4InBuffer, EP4_BUFSIZE, this)) {
      Serial.println("ERROR: queue EP4 IN failed");
    }
  }

  bool sendEp1Command(uint8_t command, const uint8_t *payload, uint16_t payloadLen) {
    if (!device || !ep1OutPipe) return false;
    if (ep1OutBusy) {
      Serial.println("ERROR: EP1 OUT busy; command dropped");
      return false;
    }
    if (payloadLen > EP1_BUFSIZE - 1) payloadLen = EP1_BUFSIZE - 1;

    memset(ep1OutBuffer, 0, sizeof(ep1OutBuffer));
    ep1OutBuffer[0] = command;
    if (payload && payloadLen) {
      memcpy(ep1OutBuffer + 1, payload, payloadLen);
    }

#if SEND_EP1_COMMANDS_AS_64_BYTES
    uint16_t transferLen = EP1_BUFSIZE;
#else
    uint16_t transferLen = payloadLen + 1;
#endif

    bool logEp1Out = true;
#if !LOG_DIN_MIDI
    if (command == EP1_CMD_MIDI_WRITE) {
      logEp1Out = false;
    }
#endif
    if (logEp1Out) {
      Serial.print("[EP1 OUT] cmd=0x");
      printHex2(command);
      Serial.print(" len=");
      Serial.println(transferLen);
      dumpBytes("      out", ep1OutBuffer, transferLen, transferLen);
    }

    ep1OutCommand = command;
    ep1OutExpectedLen = transferLen;
    ep1OutBusy = true;
    if (!queue_Data_Transfer(ep1OutPipe, ep1OutBuffer, transferLen, this)) {
      ep1OutBusy = false;
      ep1OutCommand = 0;
      ep1OutExpectedLen = 0;
      Serial.println("ERROR: queue EP1 OUT failed");
      return false;
    }
    return true;
  }

  void requestAutoMessageAndStartPads() {
    if (ep1OutBusy) {
      pendingAutoStart = true;
      Serial.println("EP1 OUT busy; AUTO_MSG will be sent after current command completes");
      return;
    }

    uint8_t autoPayload[3] = {AUTO_MSG_DIGITAL, AUTO_MSG_ANALOG, AUTO_MSG_ERP};
    if (!sendEp1Command(EP1_CMD_AUTO_MSG, autoPayload, sizeof(autoPayload))) {
      return;
    }
    Serial.println("Starting EP4 pad IN loop");
    queueEp4In();
    state = STATE_RUNNING;
    MidiTranslation::refreshPadBankLeds();
  }

  void continueAfterDeviceInfo() {
#if ENABLE_DISPLAY_METERS
    if (startDisplayMetersBeforeAuto()) {
      return;
    }
#endif
    requestAutoMessageAndStartPads();
  }

#if ENABLE_DISPLAY_METERS
  bool requestDisplayMeterRefresh(bool includeInit) {
    if (!epDisplayOutPipe) return false;
    if (!displayHardwareInitialized) {
      includeInit = true;
    }

    if (displayJob != DISPLAY_IDLE || displayOutBusy || displayWaiting) {
      displayRefreshPending = true;
      return true;
    }

    uint32_t now = millis();
    if (!includeInit && (int32_t)(now - displayNextRefreshMs) < 0) {
      displayRefreshPending = true;
      return true;
    }

    displayRefreshPending = false;
    displayNextRefreshMs = now + DISPLAY_METER_REFRESH_MIN_MS;
    displayIndex = 0;
    displayInitStep = 0;
    displayFrameChunk = 0;
    displayJob = includeInit ? DISPLAY_INIT : DISPLAY_FRAME_BEGIN;
    return true;
  }

  bool startDisplayMetersBeforeAuto() {
    if (displayStarted) return true;
    displayStarted = true;

    if (!epDisplayOutPipe) {
      Serial.println("[DISPLAY] EP8/display OUT pipe missing; LCD meters not shown");
      return false;
    }

    Serial.println("[DISPLAY] initializing encoder value meters");
    setLed(MASCHINE_DISPLAY_BACKLIGHT_LED_INDEX, MASCHINE_DISPLAY_BACKLIGHT_BRIGHTNESS);
    state = STATE_DISPLAYING_TEST;
    return requestDisplayMeterRefresh(true);
  }

  void resetDisplayState() {
    displayJob = DISPLAY_IDLE;
    displayOutBusy = false;
    displayWaiting = false;
    displayStarted = false;
    displayRefreshPending = false;
    displayHardwareInitialized = false;
    displayPostWriteDelayMs = 0;
    displayDelayUntilMs = 0;
    displayNextRefreshMs = 0;
    displayOutDeadlineMs = 0;
    displayExpectedTransferLen = 0;
    displayIndex = 0;
    displayInitStep = 0;
    displayFrameChunk = 0;
  }

  void failDisplay(const char *reason) {
    Serial.print("[DISPLAY] ERROR: ");
    Serial.println(reason);
    displayJob = DISPLAY_IDLE;
    displayOutBusy = false;
    displayWaiting = false;
    displayRefreshPending = false;
    displayHardwareInitialized = false;
    displayPostWriteDelayMs = 0;
    displayDelayUntilMs = 0;
    displayNextRefreshMs = 0;
    displayOutDeadlineMs = 0;
    displayExpectedTransferLen = 0;
    if (state == STATE_DISPLAYING_TEST) {
      requestAutoMessageAndStartPads();
    }
  }

  bool queueDisplayPacket(const uint8_t *header,
                          uint8_t headerLen,
                          const uint8_t *payload = nullptr,
                          uint16_t payloadLen = 0,
                          uint16_t delayAfterMs = 0) {
    if (!device || !epDisplayOutPipe) return false;
    if (displayOutBusy || displayWaiting) return false;

    uint16_t transferLen = headerLen + payloadLen;
    if (transferLen > sizeof(displayOutBuffer)) {
      Serial.print("[DISPLAY] packet too large: ");
      Serial.println(transferLen);
      return false;
    }

    memcpy(displayOutBuffer, header, headerLen);
    if (payload && payloadLen) {
      memcpy(displayOutBuffer + headerLen, payload, payloadLen);
    }

#if LOG_DISPLAY_WRITES
    Serial.print("[DISPLAY OUT] len=");
    Serial.println(transferLen);
    dumpBytes("      out", displayOutBuffer, transferLen, 32);
#endif

#if defined(__IMXRT1062__)
    if ((uint32_t)displayOutBuffer >= 0x20200000u) {
      arm_dcache_flush_delete(displayOutBuffer, transferLen);
    }
#endif

    displayPostWriteDelayMs = delayAfterMs;
    displayExpectedTransferLen = transferLen;
    displayOutDeadlineMs = millis() + DISPLAY_OUT_TIMEOUT_MS;
    displayOutBusy = true;
    if (!queue_Data_Transfer(epDisplayOutPipe, displayOutBuffer, transferLen, this)) {
      displayOutBusy = false;
      displayPostWriteDelayMs = 0;
      displayDelayUntilMs = 0;
      displayOutDeadlineMs = 0;
      displayExpectedTransferLen = 0;
      Serial.println("[DISPLAY] queue EP8 OUT failed");
      return false;
    }
    return true;
  }

  bool queueDisplayInitCommand() {
    struct DisplayInitStep {
      uint8_t len;
      uint8_t delayAfterMs;
      uint8_t bytes[7];
    };

    static const DisplayInitStep initSteps[DISPLAY_INIT_STEPS] = {
      {4, 0,  {0, 0x00, 0x01, 0x30, 0, 0, 0}},
      {7, 20, {0, 0x00, 0x04, 0xCA, 0x04, 0x0F, 0x00}},
      {5, 0,  {0, 0x00, 0x02, 0xBB, 0x00, 0, 0}},
      {4, 0,  {0, 0x00, 0x01, 0xD1, 0, 0, 0}},
      {4, 0,  {0, 0x00, 0x01, 0x94, 0, 0, 0}},
      {6, 20, {0, 0x00, 0x03, 0x81, 0x1E, 0x02, 0}},
      {5, 20, {0, 0x00, 0x02, 0x20, 0x08, 0, 0}},
      {5, 20, {0, 0x00, 0x02, 0x20, 0x0B, 0, 0}},
      {4, 0,  {0, 0x00, 0x01, 0xA6, 0, 0, 0}},
      {4, 0,  {0, 0x00, 0x01, 0x31, 0, 0, 0}},
      {7, 0,  {0, 0x00, 0x04, 0x32, 0x00, 0x00, 0x05}},
      {4, 0,  {0, 0x00, 0x01, 0x34, 0, 0, 0}},
      {4, 0,  {0, 0x00, 0x01, 0x30, 0, 0, 0}},
      {7, 0,  {0, 0x00, 0x04, 0xBC, 0x00, 0x01, 0x02}},
      {6, 0,  {0, 0x00, 0x03, 0x75, 0x00, 0x3F, 0}},
      {6, 0,  {0, 0x00, 0x03, 0x15, 0x00, 0x54, 0}},
      {4, 0,  {0, 0x00, 0x01, 0x5C, 0, 0, 0}},
      {4, 20, {0, 0x00, 0x01, 0x25, 0, 0, 0}},
      {4, 20, {0, 0x00, 0x01, 0xAF, 0, 0, 0}},
      {7, 0,  {0, 0x00, 0x04, 0xBC, 0x02, 0x01, 0x01}},
      {4, 0,  {0, 0x00, 0x01, 0xA6, 0, 0, 0}},
      {6, 0,  {0, 0x00, 0x03, 0x81, 0x25, 0x02, 0}},
    };

    if (displayInitStep >= DISPLAY_INIT_STEPS) return false;

    uint8_t command[7];
    const DisplayInitStep &step = initSteps[displayInitStep++];
    memcpy(command, step.bytes, step.len);
    command[0] = displayIndex << 1;
    return queueDisplayPacket(command, step.len, nullptr, 0, step.delayAfterMs);
  }

  bool queueDisplayFrameChunk() {
    uint8_t d0 = displayIndex << 1;
    uint16_t offset = displayFrameChunk * DISPLAY_CHUNK_DATA_BYTES;
    uint16_t payloadLen = DISPLAY_CHUNK_DATA_BYTES;
    uint8_t header[4];
    uint8_t headerLen = 3;

    if (displayFrameChunk == 0) {
      header[0] = d0;
      header[1] = 0x01;
      header[2] = 0xF7;
      header[3] = 0x5C;
      headerLen = 4;
    } else if (displayFrameChunk == DISPLAY_FRAME_CHUNK_COUNT - 1) {
      header[0] = d0 + 1;
      header[1] = 0x01;
      header[2] = 0x52;
      payloadLen = DISPLAY_LAST_CHUNK_BYTES;
    } else {
      header[0] = d0 + 1;
      header[1] = 0x01;
      header[2] = 0xF6;
    }

    if (offset + payloadLen > DISPLAY_FRAME_BYTES) {
      return false;
    }

#if LOG_DISPLAY_PROGRESS
    Serial.print("[DISPLAY] queue frame display=");
    Serial.print(displayIndex);
    Serial.print(" chunk=");
    Serial.print(displayFrameChunk);
    Serial.print(" offset=");
    Serial.print(offset);
    Serial.print(" payload=");
    Serial.println(payloadLen);
#endif

    if (!queueDisplayPacket(header, headerLen, displayFrame + offset, payloadLen, DISPLAY_FRAME_PACKET_DELAY_MS)) {
      return false;
    }

    displayFrameChunk++;
    return true;
  }

  void advanceDisplay() {
    if (!device || !epDisplayOutPipe) return;
    if (!(state == STATE_DISPLAYING_TEST || state == STATE_RUNNING)) return;
    if (displayOutBusy) {
      if (displayOutDeadlineMs && (int32_t)(millis() - displayOutDeadlineMs) >= 0) {
        Serial.print("[DISPLAY] EP8 OUT timeout display=");
        Serial.print(displayIndex);
        Serial.print(" job=");
        Serial.print((uint8_t)displayJob);
        Serial.print(" chunk=");
        Serial.print(displayFrameChunk);
        Serial.print(" expected=");
        Serial.println(displayExpectedTransferLen);
        failDisplay("EP8 OUT timeout");
      }
      return;
    }
    if (displayWaiting) {
      if ((int32_t)(millis() - displayDelayUntilMs) < 0) return;
      displayWaiting = false;
      displayDelayUntilMs = 0;
    }

    while (true) {
      switch (displayJob) {
        case DISPLAY_IDLE:
          return;

        case DISPLAY_INIT:
          if (displayInitStep >= DISPLAY_INIT_STEPS) {
            displayJob = DISPLAY_FRAME_BEGIN;
            continue;
          }
          if (!queueDisplayInitCommand()) failDisplay("init command queue failed");
          return;

        case DISPLAY_FRAME_BEGIN:
          buildDisplayMeterFrame(displayFrame, displayIndex);
          displayFrameChunk = 0;
          displayJob = DISPLAY_FRAME_PRE1;
          continue;

        case DISPLAY_FRAME_PRE1: {
          uint8_t d = displayIndex << 1;
          uint8_t command[6] = {d, 0x00, 0x03, 0x75, 0x00, 0x3F};
          displayJob = DISPLAY_FRAME_PRE2;
          if (!queueDisplayPacket(command, sizeof(command), nullptr, 0, DISPLAY_FRAME_PACKET_DELAY_MS)) {
            failDisplay("frame pre-command 1 failed");
          }
          return;
        }

        case DISPLAY_FRAME_PRE2: {
          uint8_t d = displayIndex << 1;
          uint8_t command[6] = {d, 0x00, 0x03, 0x15, 0x00, 0x54};
          displayJob = DISPLAY_FRAME_SENDING;
          if (!queueDisplayPacket(command, sizeof(command), nullptr, 0, DISPLAY_FRAME_PACKET_DELAY_MS)) {
            failDisplay("frame pre-command 2 failed");
          }
          return;
        }

        case DISPLAY_FRAME_SENDING:
          if (displayFrameChunk < DISPLAY_FRAME_CHUNK_COUNT) {
            if (!queueDisplayFrameChunk()) failDisplay("frame chunk queue failed");
            return;
          }

          displayIndex++;
          if (displayIndex < MASCHINE_DISPLAYS) {
            displayInitStep = 0;
            displayJob = displayHardwareInitialized ? DISPLAY_FRAME_BEGIN : DISPLAY_INIT;
            continue;
          }

          displayJob = DISPLAY_IDLE;
          if (!displayHardwareInitialized) {
            displayHardwareInitialized = true;
            Serial.println("[DISPLAY] encoder meters ready");
          }
          if (state == STATE_DISPLAYING_TEST) {
            requestAutoMessageAndStartPads();
          }
          if (displayRefreshPending) {
            requestDisplayMeterRefresh(false);
          }
          return;
      }
    }
  }

  void handleDisplayOut(const Transfer_t *transfer) {
    displayOutBusy = false;
    displayOutDeadlineMs = 0;
    uint16_t expectedLen = displayExpectedTransferLen;
    displayExpectedTransferLen = 0;

#if LOG_DISPLAY_WRITES
    Serial.print("[DISPLAY OUT] complete token=0x");
    Serial.println(transfer->qtd.token, HEX);
#endif

#if DISPLAY_STRICT_TRANSFER_ERRORS
    uint32_t status = transfer ? (transfer->qtd.token & 0x78) : 0x40;
    uint32_t transferredLen = transfer ? actualLength(transfer) : 0;
    if (status || transferredLen != expectedLen) {
      Serial.print("[DISPLAY] EP8 OUT transfer failed token=0x");
      Serial.print(transfer ? transfer->qtd.token : 0, HEX);
      Serial.print(" len=");
      Serial.print(transferredLen);
      Serial.print(" expected=");
      Serial.println(expectedLen);
      failDisplay("EP8 OUT transfer error");
      return;
    }
#endif

    if (displayPostWriteDelayMs > 0) {
      uint16_t delayMs = displayPostWriteDelayMs;
      displayPostWriteDelayMs = 0;
      displayWaiting = true;
      displayDelayUntilMs = millis() + delayMs;
      return;
    }

    displayDelayUntilMs = 0;
  }
#endif

  void flushDinMidiOut() {
#if ENABLE_DIN_MIDI_BRIDGE
    if (!device || !ep1OutPipe || ep1OutBusy) return;
    if (state != STATE_RUNNING) return;
    if (dinMidiOutCount == 0) return;
    if (dinMidiOutInFlight) return;

    DinMidiOutMessage &msg = dinMidiOutQueue[dinMidiOutTail];
    uint8_t payload[2 + DIN_MIDI_MAX_MESSAGE_BYTES];
    payload[0] = msg.port;
    payload[1] = msg.len;
    memcpy(payload + 2, msg.data, msg.len);

#if LOG_DIN_MIDI
    Serial.print("[DIN MIDI OUT] port=");
    Serial.print(msg.port);
    Serial.print(" len=");
    Serial.println(msg.len);
    dumpBytes("      midi", msg.data, msg.len, DIN_MIDI_MAX_MESSAGE_BYTES);
#endif

    if (sendEp1Command(EP1_CMD_MIDI_WRITE, payload, msg.len + 2)) {
      dinMidiOutInFlight = true;
    }
#endif
  }

  void completeDinMidiOut(bool ok, uint32_t token, uint32_t transferredLen, uint16_t expectedLen) {
#if ENABLE_DIN_MIDI_BRIDGE
    if (!dinMidiOutInFlight) return;

    if (ok && dinMidiOutCount > 0) {
      dinMidiOutTail = (dinMidiOutTail + 1) % DIN_MIDI_OUT_QUEUE_SIZE;
      dinMidiOutCount--;
      dinMidiOutRetryCount = 0;
#if LOG_DIN_MIDI
      Serial.println("[DIN MIDI OUT] transfer complete");
#endif
    } else {
      Serial.print("[DIN MIDI OUT] EP1 transfer failed token=0x");
      Serial.print(token, HEX);
      Serial.print(" len=");
      Serial.print(transferredLen);
      Serial.print(" expected=");
      Serial.println(expectedLen);

      if (++dinMidiOutRetryCount > DIN_MIDI_OUT_MAX_RETRIES) {
        Serial.println("[DIN MIDI OUT] retry limit reached; dropping message");
        if (dinMidiOutCount > 0) {
          dinMidiOutTail = (dinMidiOutTail + 1) % DIN_MIDI_OUT_QUEUE_SIZE;
          dinMidiOutCount--;
        }
        dinMidiOutRetryCount = 0;
      }
    }

    dinMidiOutInFlight = false;
#else
    (void)ok;
    (void)token;
    (void)transferredLen;
    (void)expectedLen;
#endif
  }

  void resetLedState() {
    memset(ledState, 0, sizeof(ledState));
    ledBankDirty[0] = false;
    ledBankDirty[1] = false;
  }

  bool sendLedBank(uint8_t bankIndex) {
    if (bankIndex > 1) return false;

    uint8_t payload[1 + MASCHINE_LED_BANK_SIZE];
    payload[0] = (bankIndex == 0) ? MASCHINE_LED_BANK0_ID : MASCHINE_LED_BANK1_ID;
    memcpy(payload + 1,
           ledState + (bankIndex * MASCHINE_LED_BANK_SIZE),
           MASCHINE_LED_BANK_SIZE);

    return sendEp1Command(EP1_CMD_DIMM_LEDS, payload, sizeof(payload));
  }

  void flushLedUpdates() {
    if (!device || !ep1OutPipe || ep1OutBusy) return;
    if (state != STATE_RUNNING) return;

    if (ledBankDirty[0]) {
      if (sendLedBank(0)) {
        ledBankDirty[0] = false;
      }
      return;
    }

    if (ledBankDirty[1]) {
      if (sendLedBank(1)) {
        ledBankDirty[1] = false;
      }
    }
  }

  void parseDeviceInfo(const uint8_t *data, uint32_t len) {
    if (len < 15) {
      Serial.println("GET_DEVICE_INFO reply shorter than Linux caiaq_device_spec");
      return;
    }

    uint16_t fw = data[1] | ((uint16_t)data[2] << 8);
    Serial.println("Maschine device info:");
    Serial.print("  firmware=");
    Serial.println(fw);
    Serial.print("  hw_subtype=");
    Serial.println(data[3]);
    Serial.print("  erp=");
    Serial.print(data[4]);
    Serial.print(" analog_in=");
    Serial.print(data[5]);
    Serial.print(" digital_in=");
    Serial.print(data[6]);
    Serial.print(" digital_out=");
    Serial.println(data[7]);
    Serial.print("  midi_out=");
    Serial.print(data[12]);
    Serial.print(" midi_in=");
    Serial.print(data[13]);
    Serial.print(" data_alignment=");
    Serial.println(data[14]);
  }

  void handleEp1In(const Transfer_t *transfer) {
    uint32_t len = actualLength(transfer);
    if (len > EP1_BUFSIZE) len = EP1_BUFSIZE;

    if (len > 0) {
      RawCapture::handlePacket(EP1_IN_ADDR, ep1InBuffer, len);
      if (state == STATE_WAITING_DEVICE_INFO && ep1InBuffer[0] == EP1_CMD_GET_DEVICE_INFO) {
        initTimer.stop();
        parseDeviceInfo(ep1InBuffer, len);
        continueAfterDeviceInfo();
      }
    }
    queueEp1In();
  }

  void handleEp1Out(const Transfer_t *transfer) {
    uint8_t completedCommand = ep1OutCommand;
    uint16_t completedExpectedLen = ep1OutExpectedLen;
    uint32_t token = transfer ? transfer->qtd.token : 0;
    uint32_t status = transfer ? (transfer->qtd.token & 0x78) : 0x40;
    uint32_t transferredLen = transfer ? actualLength(transfer) : 0;
    ep1OutCommand = 0;
    ep1OutExpectedLen = 0;
    ep1OutBusy = false;
    bool logEp1Out = true;
#if !LOG_DIN_MIDI
    if (completedCommand == EP1_CMD_MIDI_WRITE) {
      logEp1Out = false;
    }
#endif
    if (logEp1Out) {
      Serial.print("[EP1 OUT] complete token=0x");
      Serial.println(token, HEX);
    }
    if (completedCommand == EP1_CMD_MIDI_WRITE) {
      completeDinMidiOut(status == 0 && transferredLen == completedExpectedLen,
                         token,
                         transferredLen,
                         completedExpectedLen);
    }
    if (pendingAutoStart && device != nullptr) {
      pendingAutoStart = false;
      requestAutoMessageAndStartPads();
      return;
    }
    flushDinMidiOut();
    flushLedUpdates();
  }

  void handleEp4In(const Transfer_t *transfer) {
    uint32_t len = actualLength(transfer);
    if (len > EP4_BUFSIZE) len = EP4_BUFSIZE;
    if (len > 0) {
      RawCapture::handlePacket(EP4_IN_ADDR, ep4InBuffer, len);
    }
    queueEp4In();
  }

  Pipe_t *ep1InPipe = nullptr;
  Pipe_t *ep1OutPipe = nullptr;
  Pipe_t *ep4InPipe = nullptr;
  Pipe_t *epDisplayOutPipe = nullptr;

  Pipe_t pipes[4] __attribute__((aligned(32)));
  Transfer_t transfers[8] __attribute__((aligned(32)));
  strbuf_t stringBuffers[1];

  uint8_t ep1InBuffer[EP1_BUFSIZE] __attribute__((aligned(32)));
  uint8_t ep1OutBuffer[EP1_BUFSIZE] __attribute__((aligned(32)));
  uint8_t ep4InBuffer[EP4_BUFSIZE] __attribute__((aligned(32)));
  uint8_t hidReportBuffer[512] __attribute__((aligned(32)));
#if ENABLE_DISPLAY_METERS
  uint8_t displayOutBuffer[DISPLAY_OUT_BUFSIZE] __attribute__((aligned(32)));
  uint8_t displayFrame[DISPLAY_FRAME_BYTES];
#endif
  DinMidiOutMessage dinMidiOutQueue[DIN_MIDI_OUT_QUEUE_SIZE];
  uint8_t ledState[MASCHINE_LED_COUNT] = {0};

  setup_t setup;
  USBDriverTimer initTimer;

  DriverState state = STATE_DISCONNECTED;
  PendingControl pendingControl = CTRL_NONE;
  bool ep1OutBusy = false;
  uint8_t ep1OutCommand = 0;
  uint16_t ep1OutExpectedLen = 0;
  bool pendingAutoStart = false;
  uint8_t dinMidiOutHead = 0;
  uint8_t dinMidiOutTail = 0;
  uint8_t dinMidiOutCount = 0;
  bool dinMidiOutInFlight = false;
  uint8_t dinMidiOutRetryCount = 0;
#if ENABLE_DISPLAY_METERS
  DisplayJob displayJob = DISPLAY_IDLE;
  bool displayOutBusy = false;
  bool displayWaiting = false;
  bool displayStarted = false;
  bool displayRefreshPending = false;
  bool displayHardwareInitialized = false;
  uint16_t displayPostWriteDelayMs = 0;
  uint32_t displayDelayUntilMs = 0;
  uint32_t displayNextRefreshMs = 0;
  uint32_t displayOutDeadlineMs = 0;
  uint16_t displayExpectedTransferLen = 0;
  uint8_t displayIndex = 0;
  uint8_t displayInitStep = 0;
  uint8_t displayFrameChunk = 0;
#endif
  bool ledBankDirty[2] = {false, false};
  bool printedDevice = false;
  bool hasHidReportDescriptor = false;
  uint8_t hidReportInterface = 0;
  uint16_t hidReportLength = 0;
  uint16_t hidReportBytesRequested = 0;
  uint8_t bInterfaceNumber = 0;
  uint8_t bAlternateSetting = 0;
  uint16_t ep1InPacketSize = EP1_BUFSIZE;
  uint16_t ep1OutPacketSize = EP1_BUFSIZE;
  uint16_t ep4InPacketSize = EP4_BUFSIZE;
  uint16_t epDisplayOutPacketSize = DISPLAY_OUT_BUFSIZE;
};

// --------------------------------------------------------------------------
// Global USB host objects
// --------------------------------------------------------------------------

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBSerial userial(myusb);
MaschineMk1Driver maschine(myusb);

static void setMaschineLed(uint8_t ledIndex, uint8_t brightness) {
  maschine.setLed(ledIndex, brightness);
}

static bool queueMaschineDinMidiOut(const uint8_t *midi, uint8_t len) {
#if ENABLE_DIN_MIDI_BRIDGE
  return maschine.queueDinMidiOut(midi, len);
#else
  (void)midi;
  (void)len;
  return false;
#endif
}

#if ENABLE_MIDI_TRANSLATION
static uint8_t midiValueToLedBrightness(uint8_t value) {
  if (value == 0) return 0;
  return 1 + (((uint16_t)(value - 1) * (MASCHINE_LED_MAX_BRIGHTNESS - 1)) / 126);
}

#if ROUTE_USB_MIDI_IN_TO_DIN_OUT
static bool buildExplicitMidiFromUsbMidi(uint8_t type,
                                         uint8_t channel,
                                         uint8_t data1,
                                         uint8_t data2,
                                         uint8_t *out,
                                         uint8_t *outLen) {
  if (!out || !outLen) return false;
  *outLen = 0;

  switch (type) {
    case usbMIDI.NoteOff:
    case usbMIDI.NoteOn:
    case usbMIDI.AfterTouchPoly:
    case usbMIDI.ControlChange:
    case usbMIDI.PitchBend:
      if (channel < 1 || channel > 16) return false;
      out[0] = (type & 0xF0) | ((channel - 1) & 0x0F);
      out[1] = data1 & 0x7F;
      out[2] = data2 & 0x7F;
      *outLen = 3;
      return true;

    case usbMIDI.ProgramChange:
    case usbMIDI.AfterTouchChannel:
      if (channel < 1 || channel > 16) return false;
      out[0] = (type & 0xF0) | ((channel - 1) & 0x0F);
      out[1] = data1 & 0x7F;
      *outLen = 2;
      return true;

    case usbMIDI.TimeCodeQuarterFrame:
      out[0] = 0xF1;
      out[1] = data1 & 0x7F;
      *outLen = 2;
      return true;

    case usbMIDI.SongPosition:
      out[0] = 0xF2;
      out[1] = data1 & 0x7F;
      out[2] = data2 & 0x7F;
      *outLen = 3;
      return true;

    case usbMIDI.SongSelect:
      out[0] = 0xF3;
      out[1] = data1 & 0x7F;
      *outLen = 2;
      return true;

    case usbMIDI.TuneRequest:
      out[0] = 0xF6;
      *outLen = 1;
      return true;

    default:
      return false;
  }
}
#endif

static void handleIncomingUsbMidi() {
  while (usbMIDI.read()) {
    uint8_t type = usbMIDI.getType();
    uint8_t channel = usbMIDI.getChannel();
    uint8_t cc = usbMIDI.getData1();
    uint8_t value = usbMIDI.getData2();

    if (type == usbMIDI.ControlChange) {
      uint8_t ledIndex = 0;
      uint8_t buttonIndex = 0;
      if ((TOP_BUTTON_LED_MIDI_CHANNEL == 0 || channel == TOP_BUTTON_LED_MIDI_CHANNEL) &&
          topButtonCcToLedIndex(cc, &ledIndex) &&
          topButtonCcToButtonIndex(cc, &buttonIndex)) {
        uint8_t brightness = midiValueToLedBrightness(value);
        MidiTranslation::setButtonLedBaseFromMidi(buttonIndex, brightness);

#if LOG_INCOMING_LED_MIDI
        Serial.print("[MIDI LED] ch=");
        Serial.print(channel);
        Serial.print(" cc=");
        Serial.print(cc);
        Serial.print(" value=");
        Serial.print(value);
        Serial.print(" led=");
        Serial.print(ledIndex);
        Serial.print(" brightness=");
        Serial.println(brightness);
#else
        (void)ledIndex;
#endif
      }

      if (MidiTranslation::setMappedEncoderValueFromUsbMidi(channel, cc, value)) {
#if LOG_INCOMING_ENCODER_MIDI
        Serial.print("[MIDI ENCODER] ch=");
        Serial.print(channel);
        Serial.print(" cc=");
        Serial.print(cc);
        Serial.print(" value=");
        Serial.println(value);
#endif
      }
    }

#if ROUTE_USB_MIDI_IN_TO_DIN_OUT
    uint8_t rawMidi[DIN_MIDI_MAX_MESSAGE_BYTES] = {0};
    uint8_t rawLen = 0;
    if (buildExplicitMidiFromUsbMidi(type, channel, usbMIDI.getData1(), usbMIDI.getData2(), rawMidi, &rawLen)) {
      queueMaschineDinMidiOut(rawMidi, rawLen);
    }
#endif
  }
}
#endif

static void printStartupBanner() {
  Serial.println();
  Serial.println("Maschine Mk1 Teensy 4.1 USB host-to-USB-MIDI bridge");
  Serial.println("Tools > USB Type must include MIDI.");
  Serial.print("ENABLE_MIDI_TRANSLATION=");
  Serial.println(ENABLE_MIDI_TRANSLATION);
  Serial.print("ENABLE_DIN_MIDI_BRIDGE=");
  Serial.println(ENABLE_DIN_MIDI_BRIDGE);
  Serial.print("ROUTE_CONTROL_MIDI_TO_USB=");
  Serial.println(ROUTE_CONTROL_MIDI_TO_USB);
  Serial.print("ROUTE_CONTROL_MIDI_TO_DIN_OUT=");
  Serial.println(ROUTE_CONTROL_MIDI_TO_DIN_OUT);
  Serial.print("ROUTE_USB_MIDI_IN_TO_DIN_OUT=");
  Serial.println(ROUTE_USB_MIDI_IN_TO_DIN_OUT);
  Serial.print("ROUTE_DIN_MIDI_IN_TO_USB=");
  Serial.println(ROUTE_DIN_MIDI_IN_TO_USB);
  Serial.print("ROUTE_DIN_MIDI_IN_TO_DIN_OUT=");
  Serial.println(ROUTE_DIN_MIDI_IN_TO_DIN_OUT);
  Serial.println("Connect Maschine Mk1 to the Teensy 4.1 USB host port, then connect the device port to the computer.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    delay(10);
  }

  printStartupBanner();

  RawCapture::reset();
  MidiTranslation::reset();

  myusb.begin();
}

void loop() {
  myusb.Task();
  MidiTranslation::update();

#if ENABLE_MIDI_TRANSLATION
  handleIncomingUsbMidi();
#endif
  maschine.update();
}
