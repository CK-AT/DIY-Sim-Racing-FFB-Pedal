#pragma once

#include <Arduino.h>

// Hardware mapping for the grip. Everything that depends on how YOUR grip is
// wired lives here — Main.cpp is generic. Pins below are placeholders; set them
// to match your harness (the values chosen avoid the S3 strapping pins
// GPIO0/45/46 and the native-USB pins GPIO19/20).

namespace grip {

// ---- Push buttons -----------------------------------------------------------
// Direct-wired, active-low: wire each button between its GPIO and GND (the
// internal pull-up holds it high when released). The order here is the HID
// button index (first entry = joystick button 0).
static constexpr uint8_t BUTTON_PINS[] = {8, 9, 10, 11};
static constexpr uint8_t BUTTON_COUNT = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 5;

// ---- Axis: AS5600 magnetic rotary encoder (I2C) -----------------------------
// 12-bit angle (0..4095), read from the RAW ANGLE register over I2C. The axis
// auto-calibrates: it tracks the observed angle min/max and scales the HID
// output to that span, self-fitting to the mechanical travel as you move it.
// The reading is unwrapped first (see Main.cpp), so the travel MAY cross the
// 0/4095 seam — the continuous angle is what gets calibrated and scaled.
// On the ESP32-S3-Zero, GP1..GP13 are the easy edge pins; GP35/36 are underside
// pads. Defaults use edge pins — set to your actual wiring.
static constexpr uint8_t I2C_SDA_PIN = 1;
static constexpr uint8_t I2C_SCL_PIN = 2;
// 100 kHz for margin: this build runs without external I2C pull-ups (relying on
// the ESP32's weak internal ones), where 400 kHz risks glitchy reads.
static constexpr uint32_t I2C_FREQ_HZ = 100000;
static constexpr uint8_t AS5600_ADDRESS = 0x36;   // fixed
static constexpr bool AXIS_INVERT = false;

// AS5600 12-bit angle scale. ENCODER_COUNTS is the wrap modulus; a sample-to-
// sample jump larger than half of it is treated as a 0/4095 wrap (see Main.cpp).
static constexpr uint16_t ENCODER_RAW_MAX = 4095;
static constexpr int32_t ENCODER_COUNTS = 4096;

// Max plausible angle change per sample (counts) after wrap correction. A hand
// can't move the axis more than this in one ~4 ms tick, and a true wrap leaves
// only a small residual — so anything larger is a glitch (e.g. a corrupted I2C
// read) and is rejected, so it can't poison the auto-calibration range.
static constexpr int32_t MAX_ANGLE_STEP = 512;

// ---- USB identity -----------------------------------------------------------
// Set at COMPILE TIME in platformio.ini (USB_VID/PID/PRODUCT/MANUFACTURER build
// flags), not here: with CDC-on-boot the core begins USB before setup() runs,
// so runtime USB.* calls are too late. VID/PID = 0x303b/0x8230 (FFB PID scheme),
// made possible by the project-local variants/grip_s3_mini variant.

// ---- HID --------------------------------------------------------------------
// 16-bit axis range, matching the FFB firmware (Joystick_ESP32S2).
static constexpr uint16_t HID_AXIS_MIN = 0;
static constexpr uint16_t HID_AXIS_MAX = 65535;

// Report cadence. ~250 Hz is plenty for buttons + pots and easy on USB.
static constexpr uint32_t REPORT_INTERVAL_MS = 4;

// ---- Calibration / logging ("cal") mode -------------------------------------
// Held at boot, this button brings up a USB CDC serial port for magnet
// diagnostics + calibration commands, and re-enables 1200bps-touch flashing.
// Without it the device stays HID-only, so games/SimHub show the product name
// (a composite HID+CDC device would show the HID interface string instead).
static constexpr uint8_t CAL_MODE_BUTTON_PIN = BUTTON_PINS[0];

// When true, cal mode periodically prints AS5600 magnet health over the CDC
// serial — magnet detected? too weak (gap too large) / too strong (too close)?
// plus AGC. Handy for dialing in the magnet air gap during assembly.
static constexpr bool MAGNET_DEBUG = true;
static constexpr uint32_t MAGNET_DEBUG_INTERVAL_MS = 500;

}  // namespace grip
