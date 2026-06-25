// GripController — standalone USB HID joystick: push buttons + one AS5600
// rotary axis (I2C, 12-bit), reported over native USB with Joystick_ESP32S2.
// No link to the FFB board (see ESP32/) — this enumerates on its own.

#include <Arduino.h>
#include <Wire.h>
#include <Joystick_ESP32S2.h>

#include "GripConfig.h"

// USB identity (product name, VID/PID) is set at compile time via build flags
// in platformio.ini — with CDC-on-boot the core begins USB before setup(), so
// runtime USB.* calls would be too late.

// One axis (X) for now. To add more, enable the relevant axes here and extend
// the read path. Hats / rudder / throttle stay disabled.
static Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                          grip::BUTTON_COUNT, 0,    // button count, hat switch count
                          true, false, false,       // X, Y, Z
                          false, false, false,      // Rx, Ry, Rz
                          false, false,             // rudder, throttle
                          false, false, false);     // accelerator, brake, steering

namespace {

struct ButtonState {
    bool stable = false;
    bool last_raw = false;
    uint32_t last_change_ms = 0;
};
ButtonState button_state[grip::BUTTON_COUNT];

// Auto-calibration runs on the UNWRAPPED (continuous) angle so the travel may
// cross the AS5600 0/4095 seam. observed_* and accumulated are in that frame.
int32_t observed_min = INT32_MAX;
int32_t observed_max = INT32_MIN;
int32_t accumulated = 0;       // continuous angle, relative to the first sample
uint16_t last_raw = 0;         // previous raw angle, for wrap detection
bool have_sample = false;
uint16_t last_angle = 0;       // last good raw, held on I2C read error

// Read the AS5600 12-bit RAW ANGLE register (0x0C high byte, 0x0D low byte).
// Returns the last good value if the I2C transaction fails (sensor unplugged
// / bus glitch), so the axis holds rather than snapping to zero.
uint16_t read_as5600_angle() {
    Wire.beginTransmission(grip::AS5600_ADDRESS);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) return last_angle;  // repeated start
    if (Wire.requestFrom((uint8_t)grip::AS5600_ADDRESS, (uint8_t)2) != 2) return last_angle;
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    last_angle = ((hi << 8) | lo) & 0x0FFF;
    return last_angle;
}

// Read one 8-bit AS5600 register (returns 0 on I2C failure).
uint8_t read_as5600_reg8(uint8_t reg) {
    Wire.beginTransmission(grip::AS5600_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom((uint8_t)grip::AS5600_ADDRESS, (uint8_t)1) != 1) return 0;
    return Wire.read();
}

// Print AS5600 magnet health over USB serial — a bring-up aid for setting the
// air gap. STATUS (0x0B) bits: MD=detected(0x20), MH=too strong/close(0x08),
// ML=too weak/far(0x10). AGC (0x1A) is the gain (0..128 in 3.3V mode; aim
// for mid-range): high = field too weak, low = field too strong.
void report_magnet_status() {
    uint8_t status = read_as5600_reg8(0x0B);
    uint8_t agc = read_as5600_reg8(0x1A);
    uint16_t raw = read_as5600_angle();
    if (!(status & 0x20)) {
        Serial.printf("AS5600: NO MAGNET detected (check wiring / magnet present) (raw=%u)\n", raw);
    } else if (status & 0x08) {
        Serial.printf("AS5600: magnet too STRONG - increase air gap (AGC=%u, raw=%u)\n", agc, raw);
    } else if (status & 0x10) {
        Serial.printf("AS5600: magnet too WEAK - reduce air gap (AGC=%u, raw=%u)\n", agc, raw);
    } else {
        Serial.printf("AS5600: magnet OK (AGC=%u, raw=%u)\n", agc, raw);
    }
}

uint16_t read_axis() {
    uint16_t raw = read_as5600_angle();

    // Unwrap the 0/4095 seam: a jump over half-scale between samples is a wrap,
    // not real motion (the lever can't slew >180° within one ~4 ms tick).
    if (!have_sample) {
        accumulated = raw;
        have_sample = true;
    } else {
        int32_t delta = (int32_t)raw - (int32_t)last_raw;
        if (delta > grip::ENCODER_COUNTS / 2) {
            delta -= grip::ENCODER_COUNTS;  // wrapped down through 0
        } else if (delta < -grip::ENCODER_COUNTS / 2) {
            delta += grip::ENCODER_COUNTS;  // wrapped up through 0
        }
        accumulated += delta;
    }
    last_raw = raw;

    // Auto-calibration: grow the observed range to whatever the axis reaches.
    if (accumulated < observed_min) observed_min = accumulated;
    if (accumulated > observed_max) observed_max = accumulated;

    // Sit at centre until a usable range has been observed (avoids div-by-zero
    // and a stuck-at-extreme output before the axis has been moved).
    float t = 0.5f;
    if (observed_max > observed_min) {
        t = float(accumulated - observed_min) / float(observed_max - observed_min);
    }
    if (grip::AXIS_INVERT) t = 1.0f - t;
    return (uint16_t)lroundf(grip::HID_AXIS_MIN + t * (grip::HID_AXIS_MAX - grip::HID_AXIS_MIN));
}

}  // namespace

void setup() {
    for (uint8_t i = 0; i < grip::BUTTON_COUNT; ++i) {
        pinMode(grip::BUTTON_PINS[i], INPUT_PULLUP);
    }
    Wire.begin(grip::I2C_SDA_PIN, grip::I2C_SCL_PIN, grip::I2C_FREQ_HZ);
    if (grip::MAGNET_DEBUG) Serial.begin(115200);  // USB CDC; baud is ignored

    joystick.setXAxisRange(grip::HID_AXIS_MIN, grip::HID_AXIS_MAX);
    joystick.begin(false);  // false = report manually via sendState()

    // Centre the axis before the first report.
    joystick.setXAxis((grip::HID_AXIS_MIN + grip::HID_AXIS_MAX) / 2);
}

void loop() {
    static uint32_t last_report_ms = 0;
    uint32_t now = millis();
    if (now - last_report_ms < grip::REPORT_INTERVAL_MS) return;
    last_report_ms = now;

    // Buttons: active-low, debounced (commit a new level only once it has been
    // stable for BUTTON_DEBOUNCE_MS).
    for (uint8_t i = 0; i < grip::BUTTON_COUNT; ++i) {
        bool raw = (digitalRead(grip::BUTTON_PINS[i]) == LOW);
        ButtonState &b = button_state[i];
        if (raw != b.last_raw) {
            b.last_raw = raw;
            b.last_change_ms = now;
        } else if (b.stable != raw && (now - b.last_change_ms) >= grip::BUTTON_DEBOUNCE_MS) {
            b.stable = raw;
        }
        joystick.setButton(i, b.stable ? 1 : 0);
    }

    // AS5600 rotary axis -> X.
    joystick.setXAxis(read_axis());

    joystick.sendState();

    // Periodic magnet-health readout for air-gap setup (bring-up only).
    if (grip::MAGNET_DEBUG) {
        static uint32_t last_magnet_ms = 0;
        if (now - last_magnet_ms >= grip::MAGNET_DEBUG_INTERVAL_MS) {
            last_magnet_ms = now;
            report_magnet_status();
        }
    }
}
