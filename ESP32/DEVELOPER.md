# ESP32 Developer Reference

Quick reference for common development tasks.

## Quick Commands

```bash
# Build
pio run                              # Build default env
pio run -e native                    # Build for testing

# Test
pio test -e native                   # Run all tests
pio test -e native -f test_physics   # Run physics tests only

# Upload
pio run -t upload                    # Upload to device
pio device monitor                   # Serial monitor (115200 baud)

# Clean
pio run -t clean                     # Clean build
rm .version_state.json               # Reset version state
```

## Adding a New Feature

### 1. New Physics Element

Create a new force/damping contributor:

```cpp
// include/MyElement.h
#pragma once
#include "Physics.h"

class MyElement : public SimElement {
public:
    MyElement(float param) : _param(param) {}
    void update(const SimState &state, SimAccumulators &accum) override;
    void set_param(float val) { _param = val; }
private:
    float _param;
};

// src/MyElement.cpp
#include "MyElement.h"

void MyElement::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;

    // Add force contribution
    accum.f_sum += some_force;

    // Or add damping
    accum.k_damp_sum += some_damping;

    // Or add friction
    accum.f_static_sum += static_friction;
    accum.f_kin_sum += kinetic_friction;
}
```

### 2. New Configuration Field

1. **Edit protocol** (`proto/diy_ffb_protocol.proto`):
```protobuf
message MyConfig {
    float my_new_field = 1;
}
```

2. **Rebuild** to regenerate nanopb files:
```bash
pio run -t clean && pio run
```

3. **Use in code**:
```cpp
void update_config(const MyConfig &config) {
    _my_element.set_param(config.my_new_field);
}
```

### 3. New Function Type

**Step 1.** Add proto message (`proto/diy_ffb_protocol.proto`):

```protobuf
message MyFunctionConfig {
    int32 pos_min = 1;
    int32 pos_max = 2;
    float damping = 3;
}

message FunctionConfig {
    // ... existing fields ...
    oneof specific {
        // ... existing options ...
        MyFunctionConfig my_function = 12;  // use next available tag
    }
}
```

**Step 2.** Create header (`include/MyFunction.h`):

```cpp
#pragma once
#include "IFunction.h"
#include "Physics.h"  // for Damper, Spring, etc.

class MyFunction : public IFunction {
public:
    MyFunction();
    void update_config(const MyFunctionConfig &config);
    float get_x_contact_point_min() override { return _config.pos_min; }
    float get_x_contact_point_max() override { return _config.pos_max; }
    void on_ffb_action(const FFBAction &action) override;
private:
    Damper _damper = Damper(0.1f);
    // Add other physics elements as needed (Spring, ForceCurve, etc.)
    MyFunctionConfig _config = MyFunctionConfig_init_default;
};
```

**Step 3.** Create implementation (`src/MyFunction.cpp`):

```cpp
#include "MyFunction.h"

MyFunction::MyFunction() {
    disable();
    add_element(&_damper);
    // Register all physics elements with add_element()
}

void MyFunction::update_config(const MyFunctionConfig &config) {
    _config = config;
    _damper.set_k(config.damping);
    // Configure other elements...
}

void MyFunction::on_ffb_action(const FFBAction &action) {
    // Handle game telemetry (e.g., trigger effects)
}
```

**Step 4.** Register in Main.cpp:

```cpp
// At top with other includes:
#include "MyFunction.h"

// With other global instances:
MyFunction my_function = {};

// In setup(), add to function_elements:
function_elements.add_element(&my_function);

// In on_config_update() switch:
case FunctionConfig_my_function_tag:
    my_function.update_config(function_cfg->specific.my_function);
    active_function = &my_function;
    break;
```

**Step 5.** Rebuild to regenerate nanopb files:

```bash
pio run -t clean && pio run
```

## Testing

### Writing a New Test

```cpp
// test/test_my_feature_native/test_my_feature.cpp
#include <unity.h>
#include "MyFeature.h"

void setUp(void) {}
void tearDown(void) {}

void test_basic_functionality(void) {
    MyFeature feature(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, feature.compute());
}

void test_edge_case(void) {
    MyFeature feature(0.0f);
    TEST_ASSERT_TRUE(feature.is_valid());
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_basic_functionality);
    RUN_TEST(test_edge_case);
    return UNITY_END();
}
```

### Arduino Stubs

For native tests, stub ESP32/Arduino APIs in `test/stubs/`:

```cpp
// test/stubs/Arduino.h
#pragma once
#include <cstdint>
#include <algorithm>

inline uint32_t micros() { return 0; }
inline uint32_t millis() { return 0; }
inline void delay(uint32_t) {}

using std::min;
using std::max;
```

## Debugging

### Serial Debug Output

```cpp
#include "LogOutput.h"

LogOutput::printf("Value: %.2f", some_value);
```

### Real-Time Debug

Enable debug flags via SimHub or directly:

```cpp
// In Main.cpp
debug_flags |= DEBUG_INFO_0_LOADCELL_READING;  // Load cell data
debug_flags |= DEBUG_INFO_0_CYCLE_TIMER;       // Task timing
debug_flags |= DEBUG_INFO_0_SERVO_READINGS;    // Servo status
```

### RTDebugOutput (CSV-style logging)

```cpp
#include "RTDebugOutput.h"

static RTDebugOutput<4> debug({"x", "v", "f_in", "f_out"});
debug.offer_data({x, v, f_in, f_out});  // Prints CSV line
```

## Memory Layout

### FreeRTOS Tasks

| Task | Core | Priority | Stack | Purpose |
|------|------|----------|-------|---------|
| `loop` | 0 | 1 | 8KB | Arduino main, LED, calibration results |
| `PhysicsTask` | 1 | 10 | 10KB | ADC + physics @ 1kHz |
| `CommManager` | 0 | 5 | 4KB | Serial/CAN periodic |
| `A6Servo` | 0 | 5 | 4KB | Modbus polling |

### Critical Sections

```cpp
// Config mutex - prevents config updates during physics
if (config_manager.try_take_config_semaphore()) {
    // Safe to read config
    config_manager.release_config_semaphore();
}
```

## Pin Mappings (PCB v13)

| Function | GPIO | Notes |
|----------|------|-------|
| ADC DRDY | 4 | Interrupt trigger |
| ADC SCK | 1 | SPI clock |
| ADC MISO | 2 | SPI data out |
| ADC MOSI | 3 | SPI data in |
| ADC CS | 5 | SPI chip select |
| Step | 7 | Stepper pulse |
| Dir | 6 | Stepper direction |
| Modbus TX | 16 | RS485 transmit |
| Modbus RX | 18 | RS485 receive |
| Modbus DE | 17 | RS485 direction |
| CAN TX | 34 | TWAI transmit |
| CAN RX | 33 | TWAI receive |
| RGB LED | 38 | WS2812 data |
| CFG1-4 | 48,47,21,15 | Axis ID DIP switches |

## Protocol Quick Reference

### Message Flow

```
SimHub Plugin                    Gateway                      Axis
     |                              |                           |
     |--- AxisConfig -------------->|--- (CAN) --------------->|
     |--- FunctionConfig ---------->|--- (CAN) --------------->|
     |--- FFBAction --------------->|--- (CAN) --------------->|
     |                              |                           |
     |<-- GatewayState -------------|<-- AxisState ------------|
     |<-- AxisLogMessage -----------|                          |
```

### Key Message Types

| Message | Sender | Purpose |
|---------|--------|---------|
| `AxisConfig` | Host | Hardware settings (motor, loadcell) |
| `FunctionConfig` | Host | Function params (force curve, limits) |
| `FFBAction` | Host | Game telemetry (ABS, G-force) |
| `AxisState` | Axis | Position/force feedback |
| `GatewayState` | Gateway | Axis presence bitmap |
| `AxisAction` | Host | Commands (restart, homing) |

## Common Pitfalls

### 1. Forgetting Semaphore

Always protect config access in physics task:
```cpp
if (!config_manager.try_take_config_semaphore()) {
    continue;  // Skip this iteration
}
// ... use config ...
config_manager.release_config_semaphore();
```

### 2. Division by Zero in Physics

Always guard divisions:
```cpp
float val_range = max_val - min_val;
if (fabsf(val_range) < 0.01f) return 0.0f;
return (value - min_val) / val_range;
```

### 3. Damping Stability

High damping can cause instability. The simulator clamps it:
```cpp
float k_limit = 1.9f * m / dt;  // Stability limit
k_damp = min(k_damp, k_limit);
```

### 4. Protobuf Field Changes

When changing `.proto` fields:
1. Clean build: `pio run -t clean`
2. Update SimHub plugin proto copy
3. Bump protocol version if breaking

## Performance Tips

- Physics runs at 1kHz - keep `SimElement::update()` fast
- Use `IRAM_ATTR` for ISR functions
- Prefer fixed-point math where possible (see `fastmath::`)
- Avoid allocations in physics task
