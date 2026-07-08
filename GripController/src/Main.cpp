// GripController — standalone USB HID joystick: push buttons + one AS5600
// rotary axis (I2C, 12-bit), reported over native USB with Joystick_ESP32S2.
// No link to the FFB board (see ESP32/) — this enumerates on its own.

#include <Arduino.h>
#include <Wire.h>
#include <Joystick_ESP32S2.h>
#include <USB.h>
#include <USBCDC.h>
#include <Preferences.h>

#include "GripConfig.h"

// USB identity (product name, VID/PID) is set at compile time via build flags
// in platformio.ini and applied when we call USB.begin() in setup().
//
// Default: HID-only AND declared as a non-composite device, so games/SimHub
// name the controller from the product string. Holding CAL_MODE_BUTTON_PIN at
// boot instead brings up a CDC serial port (-> composite device) for
// logging/calibration + 1200bps-touch flashing; that mode shows the HID
// interface string as the name, which only matters while calibrating.
//
// The CDC instance is created ONLY in cal mode: USBCDC's constructor enables
// the CDC interface, so a global instance would force composite every boot.
static USBCDC *g_cdc = nullptr;
static bool g_cal_mode = false;

// One axis (X) for now. To add more, enable the relevant axes here and extend
// the read path. Hats / rudder / throttle stay disabled.
static Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                          grip::TOTAL_BUTTON_COUNT, 0,  // button count, hat switch count
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
uint16_t last_output = (grip::HID_AXIS_MIN + grip::HID_AXIS_MAX) / 2;  // held on error
float last_travel = 0.5f;      // last normalized travel 0..1 (post-invert), for the axis button
bool axis_button = false;      // virtual axis-travel button state (Schmitt-latched)

// Stored calibration (NVS), in a boot-stable form: the absolute encoder angle
// of the min end (0..4095) + the span in counts. Loaded on boot; the live
// continuous frame is re-anchored to it on the first good sample.
const char *kNvsNamespace = "grip";
bool g_have_stored = false;
int32_t g_stored_min_abs = 0;
int32_t g_stored_span = 0;

// Read the AS5600 12-bit RAW ANGLE register (0x0C high byte, 0x0D low byte).
// Sets *ok=false and returns the last good value if the I2C transaction fails
// (sensor unplugged / bus glitch), so callers can hold rather than act on it.
uint16_t read_as5600_angle(bool *ok = nullptr) {
    Wire.beginTransmission(grip::AS5600_ADDRESS);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) { if (ok) *ok = false; return last_angle; }
    if (Wire.requestFrom((uint8_t)grip::AS5600_ADDRESS, (uint8_t)2) != 2) { if (ok) *ok = false; return last_angle; }
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    last_angle = ((hi << 8) | lo) & 0x0FFF;
    if (ok) *ok = true;
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
    if (g_cdc == nullptr) return;
    if (!(status & 0x20)) {
        g_cdc->printf("AS5600: NO MAGNET detected (check wiring / magnet present) (raw=%u)\n", raw);
    } else if (status & 0x08) {
        g_cdc->printf("AS5600: magnet too STRONG - increase air gap (AGC=%u, raw=%u)\n", agc, raw);
    } else if (status & 0x10) {
        g_cdc->printf("AS5600: magnet too WEAK - reduce air gap (AGC=%u, raw=%u)\n", agc, raw);
    } else {
        g_cdc->printf("AS5600: magnet OK (AGC=%u, raw=%u)\n", agc, raw);
    }
}

// Reset the axis auto-calibration: the observed range regrows from the next
// sample (re-zeros the continuous frame too). Exposed as a cal-mode command.
void reset_calibration() {
    observed_min = INT32_MAX;
    observed_max = INT32_MIN;
    have_sample = false;
}

// Load stored calibration (NVS) into the g_stored_* statics. Applied to the
// live frame on the first good sample in read_axis(). Read-only begin() returns
// false if the namespace doesn't exist yet (first run) — then there's none.
void load_calibration() {
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, true)) return;
    if (prefs.isKey("span")) {
        g_stored_span = prefs.getInt("span", 0);
        g_stored_min_abs = prefs.getInt("min", 0);
        g_have_stored = (g_stored_span > 0);
    }
    prefs.end();
}

// Save the current auto-calibrated range to NVS in boot-stable form.
void save_calibration() {
    if (!have_sample || observed_max <= observed_min) {
        if (g_cdc) g_cdc->println("no calibration to save (sweep the axis first)");
        return;
    }
    int32_t min_abs = ((observed_min % grip::ENCODER_COUNTS) + grip::ENCODER_COUNTS) % grip::ENCODER_COUNTS;
    int32_t span = observed_max - observed_min;
    Preferences prefs;
    prefs.begin(kNvsNamespace, false);
    prefs.putInt("min", min_abs);
    prefs.putInt("span", span);
    prefs.end();
    g_stored_min_abs = min_abs;
    g_stored_span = span;
    g_have_stored = true;
    if (g_cdc) g_cdc->printf("calibration saved (min_abs=%d span=%d)\n", (int)min_abs, (int)span);
}

// Erase stored calibration; next boot starts a fresh auto-calibration.
void clear_calibration() {
    Preferences prefs;
    prefs.begin(kNvsNamespace, false);
    prefs.clear();
    prefs.end();
    g_have_stored = false;
    if (g_cdc) g_cdc->println("stored calibration cleared");
}

uint16_t read_axis() {
    bool ok = false;
    uint16_t raw = read_as5600_angle(&ok);
    if (!ok) return last_output;  // I2C error: hold last output, don't touch cal

    // Seed the continuous frame on the first good sample.
    if (!have_sample) {
        if (g_have_stored) {
            // Re-anchor to stored calibration: place the current absolute angle
            // into the [min_abs, min_abs+span] window (mod 4096 handles the seam).
            int32_t rel = ((int32_t)raw - g_stored_min_abs) % grip::ENCODER_COUNTS;
            if (rel < 0) rel += grip::ENCODER_COUNTS;
            accumulated = g_stored_min_abs + rel;
            observed_min = g_stored_min_abs;
            observed_max = g_stored_min_abs + g_stored_span;
        } else {
            accumulated = raw;
        }
        last_raw = raw;
        have_sample = true;
    } else {
        // Unwrap the 0/4095 seam: a jump over half-scale is a wrap, not motion.
        int32_t delta = (int32_t)raw - (int32_t)last_raw;
        if (delta > grip::ENCODER_COUNTS / 2) {
            delta -= grip::ENCODER_COUNTS;  // wrapped down through 0
        } else if (delta < -grip::ENCODER_COUNTS / 2) {
            delta += grip::ENCODER_COUNTS;  // wrapped up through 0
        }
        // Reject implausible jumps (corrupted read / wrap-misdetect): ignore the
        // sample so a glitch can't shift the frame or poison observed min/max.
        // last_raw is left unchanged so a one-off spike resyncs next tick.
        if (abs(delta) <= grip::MAX_ANGLE_STEP) {
            accumulated += delta;
            last_raw = raw;
            // Auto-calibration: grow the observed range to whatever it reaches.
            // Skipped when a stored calibration is in use — that range is fixed,
            // so it defines the travel exactly rather than being a seed to grow.
            if (!g_have_stored) {
                if (accumulated < observed_min) observed_min = accumulated;
                if (accumulated > observed_max) observed_max = accumulated;
            }
        }
    }

    // Sit at centre until a usable range has been observed (avoids div-by-zero
    // and a stuck-at-extreme output before the axis has been moved).
    float t = 0.5f;
    if (observed_max > observed_min) {
        t = float(accumulated - observed_min) / float(observed_max - observed_min);
        // A fixed (stored) range can be over-travelled; clamp so the output
        // saturates at the ends instead of wrapping the uint16_t cast below.
        if (t < 0.0f) t = 0.0f;
        else if (t > 1.0f) t = 1.0f;
    }
    if (grip::AXIS_INVERT) t = 1.0f - t;
    last_travel = t;
    last_output = (uint16_t)lroundf(grip::HID_AXIS_MIN + t * (grip::HID_AXIS_MAX - grip::HID_AXIS_MIN));
    return last_output;
}

// Virtual button: ON above AXIS_BUTTON_THRESHOLD of travel, latched off only
// once travel drops AXIS_BUTTON_HYSTERESIS below it (Schmitt trigger / deadband).
bool axis_button_state(float travel) {
    if (axis_button) {
        if (travel < grip::AXIS_BUTTON_THRESHOLD - grip::AXIS_BUTTON_HYSTERESIS) axis_button = false;
    } else {
        if (travel >= grip::AXIS_BUTTON_THRESHOLD) axis_button = true;
    }
    return axis_button;
}

}  // namespace

void setup() {
    for (uint8_t i = 0; i < grip::BUTTON_COUNT; ++i) {
        pinMode(grip::BUTTON_PINS[i], INPUT_PULLUP);
    }
    Wire.begin(grip::I2C_SDA_PIN, grip::I2C_SCL_PIN, grip::I2C_FREQ_HZ);
    load_calibration();  // applied to the live frame on the first good sample

    // Cal mode: button held at boot. Sampled before joystick.begin() so the
    // descriptor is decided once, here, for this enumeration.
    g_cal_mode = (digitalRead(grip::CAL_MODE_BUTTON_PIN) == LOW);

    joystick.setXAxisRange(grip::HID_AXIS_MIN, grip::HID_AXIS_MAX);
    joystick.begin(false);  // registers the HID interface; report via sendState()

    if (g_cal_mode) {
        // Constructing USBCDC enables the CDC interface (-> composite device);
        // keep the default IAD/composite device class so HID+CDC enumerate.
        static USBCDC cdc;
        g_cdc = &cdc;
        g_cdc->begin();
        if (g_have_stored) {
            g_cdc->printf("using stored calibration (min_abs=%d span=%d)\n",
                          (int)g_stored_min_abs, (int)g_stored_span);
        }
    } else {
        // Single HID interface: declare a standard, non-composite device
        // (bDeviceClass=0) so the OS names the controller from the product
        // string instead of the HID interface string "TinyUSB HID". The core
        // otherwise defaults to the IAD/composite class (0xEF), which forces
        // interface-string naming even with one function.
        USB.usbClass(0);
        USB.usbSubClass(0);
        USB.usbProtocol(0);
    }
    USB.begin();  // finalizes the descriptor + applies the compile-time identity

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

    // Virtual button: ON above 75% of axis travel (with hysteresis), reported
    // immediately after the physical buttons.
    joystick.setButton(grip::BUTTON_COUNT, axis_button_state(last_travel) ? 1 : 0);

    joystick.sendState();

    // Cal mode only: serial commands + periodic magnet readout (CDC is up).
    if (g_cal_mode && g_cdc != nullptr) {
        while (g_cdc->available()) {
            char c = g_cdc->read();
            if (c == 'r') {
                reset_calibration();
                g_cdc->println("axis auto-calibration reset");
            } else if (c == 's') {
                save_calibration();
            } else if (c == 'c') {
                clear_calibration();
            }
        }
        if (grip::MAGNET_DEBUG) {
            static uint32_t last_magnet_ms = 0;
            if (now - last_magnet_ms >= grip::MAGNET_DEBUG_INTERVAL_MS) {
                last_magnet_ms = now;
                report_magnet_status();
            }
        }
    }
}
