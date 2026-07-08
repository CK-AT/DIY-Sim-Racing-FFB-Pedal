# ESP32 Build Guide

## Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Python 3.x (for build scripts)
- Git

## Quick Start

```bash
# Clone and enter project
cd ESP32

# Build default environment (a6-ffb-v10-ck-at)
pio run

# Upload to connected device
pio run -t upload

# Run native tests
pio test -e native
```

## Build Environments

| Environment | Target | Use Case |
|-------------|--------|----------|
| `a6-ffb-v10-ck-at` | ESP32-S3, A6 servo | **Default** - Production CK-AT hardware |
| `native` | Host machine | Unit testing (no hardware) |
| `esp32s3usbotg` | ESP32-S3 | Generic S3 with USB HID |
| `esp32` | ESP32 | Legacy devkit |
| `esp32-speedcrafter` | ESP-WROVER | Speedcrafter PCB |

### Selecting an Environment

```bash
# Build specific environment
pio run -e esp32s3usbotg

# Upload specific environment
pio run -e esp32s3usbotg -t upload

# Monitor serial output (firmware emits at 3,000,000 baud)
pio device monitor -e a6-ffb-v10-ck-at -b 3000000
```

## Versioning System

The build system automatically manages firmware versions.

### Files

| File | Purpose |
|------|---------|
| `version` | Current version (MAJOR.MINOR.PATCH) |
| `.version_state.json` | Build state tracking (auto-generated) |
| `include/Version.h` | Generated header with `VERSION` and `BUILD_TIMESTAMP` |
| `board_versions.json` | PCB version → board name mapping |
| `include/Version_Board.h` | Generated `CONTROL_BOARD` define |

### Version Behavior

1. **Auto-increment**: After a successful build, PATCH is incremented on next build
2. **Manual version**: Edit `version` file to set a specific version
3. **Clean build**: Delete `.version_state.json` to reset state

### Build Scripts (scripts/)

| Script | Phase | Purpose |
|--------|-------|---------|
| `generate_version_board.py` | PRE | Generates `Version_Board.h` from `board_versions.json` |
| `versioning_pre.py` | PRE | Auto-increments version, generates `Version.h` |
| `versioning_post.py` | POST | Marks build as successful in state file |
| `ota_package_post.py` | POST | Creates OTA package (`.ffbota` zip) |

## OTA Updates

After a successful build, the OTA system creates:

```
../OTA/
├── firmware_<env>.ffbota    # ZIP with manifest + firmware
├── firmware_<env>.bin       # Raw firmware binary
└── update_info.json         # Update manifest for all boards
```

### OTA Package Contents

```json
{
  "version": "1.2.3",
  "board": "CK-AT_A6_V1.0",
  "build_env": "a6-ffb-v10-ck-at",
  "build_timestamp": "2025-01-15T10:30:00",
  "md5": "abc123...",
  "size": 1234567
}
```

## Testing

### Native Unit Tests

Tests run on the host machine without hardware:

```bash
# Run all native tests
pio test -e native

# Run specific test
pio test -e native -f test_physics_native

# Verbose output
pio test -e native -v
```

### Test Structure

```
test/
├── stubs/                          # Arduino/ESP32 stubs for native builds
│   ├── Arduino.h
│   ├── Preferences.h
│   ├── ConfigManager.h
│   └── CommManager.h
├── test_physics_native/            # Physics simulation tests
│   └── test_physics.cpp
├── test_shifter_native/            # Shifter logic tests
│   └── test_shifter.cpp
└── test_comm_manager_native/       # Communication tests
    └── test_comm_manager.cpp
```

### Test Framework

Tests use [Unity](http://www.throwtheswitch.org/unity) framework:

```cpp
#include <unity.h>

void test_example(void) {
    TEST_ASSERT_EQUAL(expected, actual);
    TEST_ASSERT_TRUE(condition);
    TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    return UNITY_END();
}
```

## Configuration Defines

Only `PCB_VERSION` (plus the USB/`ARDUINO_*` flags) is set in `platformio.ini`.
The remaining flags below are derived per-board in `include/Main.h`, selected by `PCB_VERSION`:

| Define | Description | Set in |
|--------|-------------|--------|
| `PCB_VERSION` | Hardware revision (see `board_versions.json`) | `platformio.ini` |
| `A6SERVO` | Enable A6-series servo driver | `include/Main.h` |
| `HAS_CAN` | Enable CAN bus support | `include/Main.h` |
| `USB_JOYSTICK` | Enable USB HID gamepad | `include/Main.h` |
| `RGB_LED` | Enable WS2812 status LED | `include/Main.h` |

## Troubleshooting

### Build Fails with "Version.h not found"

Delete `.version_state.json` and rebuild:
```bash
rm .version_state.json
pio run
```

### Upload Fails on ESP32-S3

1. Put device in download mode (hold BOOT, press RESET)
2. Check USB cable supports data (not charge-only)
3. Try `pio run -t upload --upload-port COMx`

### Tests Fail to Compile

Ensure stubs are up to date with source headers:
```bash
# Check for missing stub functions
pio test -e native -v 2>&1 | grep "undefined reference"
```

### Protobuf Changes Not Reflected

Regenerate nanopb files:
```bash
pio run -t clean
pio run
```

## Continuous Integration

For CI/CD, use:

```bash
# Full build + test
pio run -e a6-ffb-v10-ck-at && pio test -e native

# Check compilation for all environments
pio run -e a6-ffb-v10-ck-at -e esp32s3usbotg -e native
```
