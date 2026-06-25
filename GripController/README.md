# GripController

Standalone USB HID joystick firmware for a stick grip — **push buttons + one
AS5600 rotary axis** — running on a *Waveshare ESP32-S3-Zero* board. It
enumerates on the PC as its own gamepad (`DIY-FFB-Grip-MD500-Collective`); it
does **not** talk to the FFB / CAN-gateway firmware in [`../ESP32`](../ESP32).

## Hardware

- Waveshare ESP32-S3-Zero (ESP32-S3FH4R2: 4 MB flash, 2 MB PSRAM, native USB-C).
  Built against the `lolin_s3_mini` profile — the electrical match.
- Buttons wired between a GPIO and GND (active-low, internal pull-up).
- AS5600 magnetic rotary encoder on the I²C bus (addr `0x36`, SDA/SCL default to
  GP1/GP2). 12-bit angle read from the RAW ANGLE register.
- Note: on the S3-Zero, GP1–GP13 are the easy edge pins; GP35/36 are underside
  pads — keep I²C and buttons on edge pins.

All pin assignments, button count, and the I²C/axis config live in
[`include/GripConfig.h`](include/GripConfig.h) — that's the only file you edit to
match your wiring. `src/Main.cpp` is generic.

The axis **auto-calibrates**: it tracks the observed angle min/max and scales the
HID output to that span, so sweep the grip end-to-end once after power-up. The
raw angle is **unwrapped** first, so the travel may cross the encoder's 0/4095
seam (consecutive samples that jump more than half-scale are treated as a wrap).

## Build & flash

```sh
pio run -e grip-s3-mini            # build
pio run -e grip-s3-mini -t upload  # flash
```

No BOOT button needed: the firmware enumerates as a HID **+ CDC** composite
(`ARDUINO_USB_CDC_ON_BOOT=1`), and `board_upload.use_1200bps_touch` drops the
chip into the ROM bootloader automatically on upload. (The extra virtual COM
port is the CDC side — harmless.)

## Notes

- USB descriptor is **compile-time** (build flags in `platformio.ini`), because
  CDC-on-boot starts USB before `setup()` runs. `VID 0x303b / PID 0x8230`,
  product `DIY-FFB-Grip-MD500`, following the FFB firmware's PID scheme
  (gateways `0x8210+N`, axes `0x8220+N`, grip `0x8230`) so every DIY-FFB device
  is distinct on one PC. The stock `lolin_s3_mini` variant hard-defines
  `USB_VID/PID` unguarded, so we use a project-local variant
  ([`variants/grip_s3_mini`](variants/grip_s3_mini/pins_arduino.h)) — a copy
  that `#ifndef`-guards them — selected via `board_build.variant`.
- Up to 6 analog axes map to X, Y, Z, Rx, Ry, Rz. Hats, rudder, and throttle are
  disabled in the `Joystick_` constructor — enable them there and extend
  `apply_axis()` if you add more inputs later.
