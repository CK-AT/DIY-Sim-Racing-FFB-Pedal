# Progressive Spring Feature

**Status**: NOT IMPLEMENTED (plan only)

## Goal

Add a configurable spring exponent to enable progressive/degressive centering spring behavior for flight controls.

```
F = -k * sign(x) * |x|^n
```

Where:
- `n = 1.0` → linear (default, current behavior)
- `n > 1.0` → progressive (stiffer at extremes)
- `n < 1.0` → degressive (softer at extremes)

## Data Flow

```
Graph Output → Plugin → Protobuf → CAN Transport → ESP32 → Physics
```

## Changes by Layer

### 1. SimHubPlugin: Graph Signal Catalog

**File:** `SimHubPlugin/GraphSignalCatalogData.cs`

Add new output signals:
```csharp
"FlightStickPitch.SpringExponent",
"FlightStickRoll.SpringExponent",
"FlightPedals.SpringExponent",
"FlightStickCollective.SpringExponent",
```

**File:** `SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md`

Document the new outputs.

### 2. SimHubPlugin: FFB Frame Building

**File:** `SimHubPlugin/DiyFfbPlugin.cs`

Update `TryGetGraphFlightOutputs()` to read `SpringExponent` output:
```csharp
if (TryGetGraphOutput($"{prefix}.SpringExponent", out value))
{
    springExponent = value;
    hasOutput = true;
}
```

Update `SendFlightFfb()` signature and body to include `springExponent`.

### 3. Protobuf: Message Definition

**File:** `proto/diy_ffb_protocol.proto`

Add field to `FlightFfbAction`:
```proto
message FlightFfbAction {
  float k_spring = 1;
  float k_damper = 2;
  float k_friction = 6;
  float trim_offset = 3;
  float buffet_amp = 4;
  float load_force = 5;
  float spring_exponent = 7;  // NEW: 1.0 = linear (default)
}
```

Regenerate C# and nanopb bindings.

### 4. ESP32: CAN Transport

**File:** `ESP32/src/CANManager.cpp`

Rename payload structs for clarity:
- `FlightFfbPayload` → `FlightFfbPayload1`
- `FlightFfbLoadPayload` → `FlightFfbPayload2`

Current layout:
- `FlightFfbPayload1` (8 bytes): k_spring, k_damper, trim_offset, buffet_amp — **full**
- `FlightFfbPayload2` (4 bytes): load_force, k_friction — **has room**

Add `spring_exponent` to `FlightFfbPayload2`:
```cpp
struct FlightFfbPayload2 {
    int16_t load_force;
    uint16_t k_friction;
    uint16_t spring_exponent;  // NEW: scaled, 1.0 = 100
};

constexpr float kFfbScaleExponent = 0.01f;  // 0.01 resolution, range 0-655
```

Update pack/unpack functions and receive handler accordingly.

### 5. ESP32: Physics Engine

**File:** `ESP32/include/Physics.h`

Add to `Spring` class:
```cpp
void set_exponent(float n) { _n = n; }
// ...
float _n = 1.0f;
```

Add fast power approximation to `fastmath` namespace:
```cpp
// ~10x faster than powf, ±2% accuracy for n in [0.5, 3.0]
static inline float fast_powf(float x, float n) {
    union { float f; int32_t i; } v = {x};
    v.i = (int32_t)(n * (v.i - 0x3f800000)) + 0x3f800000;
    return v.f;
}
```

**File:** `ESP32/src/Physics.cpp`

Update `Spring::update()`:
```cpp
void Spring::update(const SimState &state, SimAccumulators &accum) {
    if (!_enabled) return;
    float x = state.x - _offset;
    if (_n == 1.0f) {
        accum.f_sum -= x * _k;
    } else {
        float sign = x >= 0.0f ? 1.0f : -1.0f;
        float x_abs = fabsf(x);
        accum.f_sum -= _k * sign * fastmath::fast_powf(x_abs, _n);
    }
}
```

### 6. ESP32: Flight Stick Function

**File:** `ESP32/src/FlightStickFunction.cpp`

Update `on_ffb_action()`:
```cpp
centering_spring.set_exponent(ffb_action.function.flight_ffb.spring_exponent > 0.0f
    ? ffb_action.function.flight_ffb.spring_exponent
    : 1.0f);
```

Reset to default in timeout block:
```cpp
centering_spring.set_exponent(1.0f);
```

## Performance

| Factor | Impact |
|--------|--------|
| Loop rate | 1kHz physics × up to 16 iterations = up to 16kHz |
| `powf()` cost | ~50-100 cycles on ESP32 (vs ~5 for multiply) |
| `fast_powf()` cost | ~10 cycles, ±2% accuracy |
| Fast path | `n == 1.0f` check avoids power function entirely |
| Memory | +4 bytes per Spring instance (negligible) |

The `fast_powf` approximation uses IEEE 754 bit manipulation (`exp2(n * log2(x))`) and is sufficient for haptic feedback where ±2% error is imperceptible.

## Files Modified

| Layer | File |
|-------|------|
| Plugin | `SimHubPlugin/GraphSignalCatalogData.cs` |
| Plugin | `SimHubPlugin/DiyFfbPlugin.cs` |
| Plugin | `SimHubPlugin/Docs/FFB_Graph_Signal_Catalog.md` |
| Proto | `proto/diy_ffb_protocol.proto` |
| ESP32 | `ESP32/src/CANManager.cpp` |
| ESP32 | `ESP32/include/Physics.h` |
| ESP32 | `ESP32/src/Physics.cpp` |
| ESP32 | `ESP32/src/FlightStickFunction.cpp` |

## Testing

1. **Unit test:** Verify `Spring` output at `n=0.5`, `n=1.0`, `n=2.0`
2. **Unit test:** Verify `fast_powf` accuracy against `powf` for n in [0.5, 3.0]
3. **Integration:** Graph with constant `SpringExponent` output → verify ESP32 receives value
4. **Feel test:** Compare linear vs progressive on flight stick
