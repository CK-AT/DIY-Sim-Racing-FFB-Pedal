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

If a calibration has been **saved to NVS** (cal-mode `s`), it takes over on boot
as a **fixed** range — auto-calibration no longer grows it, and over-travel
saturates at the ends. Clear it (`c`) to return to live auto-calibration.

A **virtual button** is reported after the physical ones (at HID index
`BUTTON_COUNT`): it turns **on above 75 % of axis travel** and stays on until
travel drops below 73 % (a 2 % hysteresis band, so it doesn't chatter near the
threshold). Threshold and hysteresis are in `GripConfig.h`.

## Cal mode (logging / calibration / flashing)

By default the device is **HID-only** so games/SimHub show the product name.
Hold `CAL_MODE_BUTTON_PIN` (default = first button) **at boot** to enter **cal
mode**, which adds a USB CDC serial port. This is decided once per boot. In cal
mode you get:

- **Magnet air-gap readout** (when `MAGNET_DEBUG`): every 500 ms it prints
  `magnet OK` / `too WEAK` / `too STRONG` / `NO MAGNET` + AGC. Open the serial
  monitor (115200) and adjust the gap until OK with AGC mid-range.
- **Commands** (over the CDC serial):
  - `s` — save the current axis calibration to NVS (persists across reboots).
  - `r` — reset the live auto-calibration range (regrows from the next sweep).
  - `c` — erase the stored calibration; next boot auto-calibrates fresh.
- **Button-free flashing:** the CDC port makes `1200bps-touch` work, so uploads
  enter the bootloader automatically.

Trade-off: in cal mode the device is a HID+CDC composite, so the controller
name shows as the HID interface string (`TinyUSB HID`) rather than the product
name — that only matters while calibrating, not while gaming.

## Build & flash

```sh
pio run -e grip-s3-mini            # build
pio run -e grip-s3-mini -t upload  # flash (hold the cal-mode button at boot)
```

To flash: **boot in cal mode** (hold the cal-mode button) so the CDC port
appears, then upload — `1200bps-touch` resets into the bootloader, no BOOT
button. (Normal HID-only boots have no serial port, so flash from cal mode.)

## Notes

- USB descriptor is **compile-time** (build flags in `platformio.ini`), applied
  when `setup()` calls `USB.begin()`. `VID 0x303b / PID 0x8230`, product
  `DIY-FFB-Grip-MD500-Collective`, following the FFB firmware's PID scheme
  (gateways `0x8210+N`, axes `0x8220+N`, grip `0x8230`) so every device is
  distinct on one PC. The stock `lolin_s3_mini` variant hard-defines `USB_VID/PID`
  unguarded, so we use a project-local variant
  ([`variants/grip_s3_mini`](variants/grip_s3_mini/pins_arduino.h)) — a copy that
  `#ifndef`-guards them — selected via `board_build.variant`.
- One axis (X) for now, fed by the AS5600. To add axes, enable them in the
  `Joystick_` constructor and extend the read path. Hats / rudder / throttle are
  disabled.
