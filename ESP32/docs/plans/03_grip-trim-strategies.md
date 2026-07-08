# Grip Trim Strategies Plan

Date: 2026-03-30
Status: Planning

## Goal

Read Warthog grip buttons via SPI on the ESP32 and implement two trim strategies
as graph-native logic: hat-switch incremental trim and force-trim-release (FTR).

## Problem Statement

- Custom FFB flight stick uses a Thrustmaster Warthog grip connected via SPI
- Different aircraft require different trim behaviors:
  - **Hat trim** (GA/WW2): incremental pitch/roll offset per hat press
  - **Force trim release** (helicopter/FBW): hold button to unlock, release to lock new center
- Currently no grip input reading exists on the ESP32 or in the plugin
- Trim logic should live in the graph system so it's per-aircraft customizable via graph templates

## Scope

### In Scope
- ESP32: Read Warthog grip shift registers (74HC165) via second SPI bus
- ESP32: Expose grip buttons as USB HID joystick buttons via existing CommManager
- Plugin: Read grip buttons via DirectInput, expose as graph input signals
- Plugin: Expose `AxisState.position` as graph input signal
- Graph engine: Add stateful node types (Accumulator, SampleAndHold, EdgeDetect)
- Graph templates: Hat trim and FTR wiring as reusable sub-graphs

### Out of Scope
- Grip analog axes (Warthog grip has none)
- Trim save/restore across sessions
- Custom trim curves or non-linear trim rates (can be added later)
- Plugin UI for trim (graphs handle everything)

## Existing Infrastructure

### Protobuf (already exists)

```protobuf
message FlightFfbAction {
  float k_spring = 1;      // Centering spring gain
  float k_damper = 2;      // Damping gain
  float k_friction = 6;    // Friction gain
  float trim_offset = 3;   // Trim offset in mm — TRIM OUTPUT CHANNEL
  float buffet_amp = 4;    // Buffet amplitude
  float load_force = 5;    // Constant load force in N
}

message AxisState {
  AxisID axis_id = 1;
  float position = 2;      // Current position (0.0–1.0) — NEEDED FOR FTR
  float force = 3;
}
```

### Graph evaluation path (already exists)

```
BuildGraphInputs(data)           → graphInputs dict
  └─ GraphSignalCatalog.BuildXPlaneInputs()
BuildGraphParams()               → graphParams dict
activeGraphEvaluator.EvaluateWithTrace(graphInputs, graphParams)
  └─ returns outputs: {FlightStickPitch.TrimOffset, .SpringGain, ...}
TryGetGraphFlightOutputs()       → reads outputs
SendFlightFfb()                  → packs FlightFfbAction protobuf → ESP32
```

### Graph engine (current capabilities)

- **Node types**: Input, Param, Const, Op, Func, Include, Output
- **Ops**: Add, Sub, Mul, Div, Min, Max, Abs, Neg, Clamp, Lerp
- **Funcs**: qhat_eff, torque_norm, rpm_norm, assist_loss, buffet
- **State**: NONE — `_values` array is cleared every evaluation cycle
- **Inputs**: XPlane telemetry only (16 signals)

### ESP32 button infrastructure (already exists)

- `CommManager::set_controller_button_value(index, pressed)` — 32-button joystick HID
- Joystick HID updates sent at ~100 Hz in CommManager periodic task

## Design

### Part 1: ESP32 — Grip SPI Reading

The Warthog grip uses **74HC165** parallel-in/serial-out shift registers (3.3V compatible).
The ESP32-S3 reads them as SPI master on a second SPI bus (SPI3/HSPI), separate from the
ADS1256 ADC on SPI2.

#### Pin Assignment (TBD — depends on PCB revision)

```
GRIP_CS   → GPIOxx
GRIP_SCK  → GPIOxx
GRIP_MISO → GPIOxx  (serial data out from shift registers)
GRIP_MOSI → not connected (shift registers are read-only)
```

#### Read Cycle

```
1. Assert CS low
2. Clock out N bytes (one per 74HC165 in the chain)
3. Deassert CS
4. Decode button bitfield
5. Map to CommManager button indices
```

Poll rate: 100 Hz (in CommManager periodic task, alongside existing joystick update).

#### Button Mapping

The Warthog grip has ~19 discrete buttons + 1 hat switch (4 directions) + trim hat
(4 directions). Exact bit positions from the shift register chain need to be verified
with a logic analyzer or reference documentation. Map to CommManager buttons 0–31.

### Part 2: New Graph Input Signals

Add two new input groups alongside the existing `XPlane` group.

#### Grip button inputs

New input group: `Grip`

```
Grip.TrimHat.Up        (0.0 or 1.0)
Grip.TrimHat.Down      (0.0 or 1.0)
Grip.TrimHat.Left      (0.0 or 1.0)
Grip.TrimHat.Right     (0.0 or 1.0)
Grip.ForceTrimRelease  (0.0 or 1.0)
Grip.TrimReset         (0.0 or 1.0)
```

#### Axis position inputs

New input group: `Axis`

```
Axis.Pitch.Position    (mm, contact point position from AxisState)
Axis.Roll.Position     (mm, contact point position from AxisState)
```

#### Implementation

**GraphSignalCatalogData.cs** — add to `InputGroups` and `InputNames`:

```csharp
public static readonly IReadOnlyList<string> InputGroups = new[] { "XPlane", "Grip", "Axis" };

// Add to InputNames:
"Grip.TrimHat.Up",
"Grip.TrimHat.Down",
"Grip.TrimHat.Left",
"Grip.TrimHat.Right",
"Grip.ForceTrimRelease",
"Grip.TrimReset",
"Axis.Pitch.Position",
"Axis.Roll.Position",
```

**GraphSignals.cs** — add `BuildGripInputs()` and `BuildAxisInputs()`:

```csharp
public static void BuildGripInputs(GripInputReader grip, IDictionary<string, double> inputs)
{
    if (grip == null) { /* zero-fill all Grip.* keys */ return; }
    inputs["Grip.TrimHat.Up"]       = grip.IsPressed(GripButton.TrimHatUp) ? 1.0 : 0.0;
    inputs["Grip.TrimHat.Down"]     = grip.IsPressed(GripButton.TrimHatDown) ? 1.0 : 0.0;
    // ... etc
}

public static void BuildAxisInputs(DiyFfbPlugin plugin, IDictionary<string, double> inputs)
{
    inputs["Axis.Pitch.Position"] = plugin.GetLastAxisPosition(AxisID._1);
    inputs["Axis.Roll.Position"]  = plugin.GetLastAxisPosition(AxisID._2);
}
```

**DiyFfbPlugin.BuildGraphInputs()** — call all three builders:

```csharp
private void BuildGraphInputs(GameData data)
{
    graphInputs.Clear();
    GraphSignalCatalog.BuildXPlaneInputs(this, data, graphInputs);
    GraphSignalCatalog.BuildGripInputs(gripInputReader, graphInputs);
    GraphSignalCatalog.BuildAxisInputs(this, graphInputs);
}
```

### Part 3: Plugin — Joystick Button Reading & Binding

The grip buttons appear as a standard USB HID joystick (via the ESP32's existing
joystick descriptor). The plugin reads them using **SharpDX.DirectInput** and maps
physical buttons to logical `Grip.*` signals via user-configurable bindings.

```
┌──────────┐   SPI    ┌──────────┐   USB HID   ┌──────────────┐  Bindings  ┌───────────┐
│  Warthog │ ──────── │  ESP32   │ ──────────── │  Plugin      │ ─────────▶ │ Grip.*    │
│  Grip    │  74HC165 │  S3      │  Joystick    │  ButtonInput │            │ graph     │
│ (buttons)│          │          │  32 buttons  │  Reader      │            │ inputs    │
└──────────┘          └──────────┘              └──────────────┘            └───────────┘
```

Any DirectInput device works — not just the ESP32 joystick. Users with a stock Warthog
base or any other HOTAS can bind buttons from their existing device. **Keyboard keys**
are also supported for testing without hardware.

#### ButtonInputReader class

- On init: enumerate all DirectInput joystick/HOTAS devices + acquire keyboard
- `Poll()`: called each graph evaluation cycle, reads button/key states from bound device(s)
- Supports reading from **multiple devices** simultaneously (e.g., grip on one, throttle on another)
- Returns `bool IsPressed(ButtonBinding binding)` based on binding type
- SharpDX.DirectInput `Keyboard` device provides `KeyboardState` with all key states

#### Button Binding Model

Each `Grip.*` signal is bound to either a joystick button or a keyboard key:

```csharp
public enum BindingType { None, JoystickButton, KeyboardKey }

/// <summary>
/// Persisted in DiyFfbPluginSettings. Maps a logical grip signal
/// to a physical joystick button or keyboard key.
/// </summary>
public class ButtonBinding
{
    public BindingType Type = BindingType.None;

    // Joystick fields (used when Type == JoystickButton)
    /// <summary>DirectInput device instance GUID (survives reconnect).</summary>
    public string DeviceInstanceGuid = "";
    /// <summary>Human-readable device name (for UI display).</summary>
    public string DeviceName = "";
    /// <summary>Zero-based button index in JoystickState.Buttons[].</summary>
    public int ButtonIndex = -1;

    // Keyboard field (used when Type == KeyboardKey)
    /// <summary>DirectInput Key enum value.</summary>
    public int KeyCode = -1;

    /// <summary>Display string, e.g. "Warthog Grip: Btn 2" or "Keyboard: T".</summary>
    public string DisplayName =>
        Type == BindingType.JoystickButton ? $"{DeviceName}: Btn {ButtonIndex}" :
        Type == BindingType.KeyboardKey    ? $"Keyboard: {(Key)KeyCode}" :
        "(not bound)";
}
```

Settings storage in `DiyFfbPluginSettings`:

```csharp
/// <summary>
/// Maps logical grip signal names (e.g., "Grip.TrimHat.Up") to physical
/// joystick button or keyboard key bindings.
/// </summary>
public Dictionary<string, ButtonBinding> GripButtonBindings
    = new Dictionary<string, ButtonBinding>();
```

#### Binding UI

A dedicated section in the plugin settings tab:

```
┌─────────────────────────────────────────────────────────────────────┐
│  Grip Button Bindings                                               │
│                                                                     │
│  Signal              Binding                        [Action]        │
│  ──────────────────  ──────────────────────────────  ────────────── │
│  Trim Hat Up         (not bound)                     [Bind...]      │
│  Trim Hat Down       Keyboard: G                     [Bind...] [X]  │
│  Trim Hat Left       (not bound)                     [Bind...]      │
│  Trim Hat Right      (not bound)                     [Bind...]      │
│  Force Trim Release  Warthog Grip: Btn 2             [Bind...] [X]  │
│  Trim Reset          (not bound)                     [Bind...]      │
│                                                                     │
│                                             [Clear All Bindings]    │
└─────────────────────────────────────────────────────────────────────┘
```

**Bind flow** (press-to-bind pattern, like most sim software):

1. User clicks [Bind...] for a signal row
2. Button text changes to "Press a button or key..." and starts listening
3. Plugin polls all DirectInput joysticks AND the keyboard for any new press
4. First detected input captures the binding:
   - Joystick button → `(JoystickButton, DeviceGuid, DeviceName, ButtonIndex)`
   - Keyboard key → `(KeyboardKey, KeyCode)`
5. Binding saved to settings, UI updates to show `DisplayName`
6. Timeout after ~5 seconds if no input detected, binding unchanged
7. Escape key cancels without changing the binding

**Clear**: per-row [X] button (shown only when bound) and bulk [Clear All Bindings].

#### BuildGripInputs with bindings

```csharp
public static void BuildGripInputs(
    ButtonInputReader reader,
    Dictionary<string, ButtonBinding> bindings,
    IDictionary<string, double> inputs)
{
    foreach (var signalName in GripSignalNames)
    {
        double value = 0.0;
        if (reader != null
            && bindings.TryGetValue(signalName, out var binding)
            && binding.Type != BindingType.None)
        {
            value = reader.IsPressed(binding) ? 1.0 : 0.0;
        }
        inputs[signalName] = value;
    }
}
```

`ButtonInputReader.IsPressed(ButtonBinding)` dispatches internally:

```csharp
public bool IsPressed(ButtonBinding binding)
{
    switch (binding.Type)
    {
        case BindingType.JoystickButton:
            return GetJoystickButton(binding.DeviceInstanceGuid, binding.ButtonIndex);
        case BindingType.KeyboardKey:
            return _keyboardState.IsPressed((Key)binding.KeyCode);
        default:
            return false;
    }
}
```

This decouples graph inputs from any specific hardware — the graph just sees
`Grip.TrimHat.Up = 0 or 1`, regardless of whether it comes from a Warthog grip
on a custom ESP32 base, a stock Thrustmaster, a Virpil throttle button, or a
keyboard key pressed for testing.

### Part 4: Stateful Graph Nodes

The graph engine is currently stateless. To support trim, add three new `Func` types
that maintain state across evaluation cycles.

#### Engine change: persistent state array

Add a `_state` array to `GraphCompiledEvaluator` that is **NOT** cleared between
evaluations (unlike `_values` which is cleared every cycle):

```csharp
public sealed class GraphCompiledEvaluator
{
    private readonly double[] _values;       // cleared every eval (existing)
    private readonly double[] _extraValues;  // cleared every eval (existing)
    private readonly double[] _state;        // PERSISTS across evals (NEW)
    private readonly int[] _stateIndices;    // maps stateful node → _state slot

    // In EvaluateWithTrace():
    Array.Clear(_values, 0, _values.Length);
    Array.Clear(_extraValues, 0, _extraValues.Length);
    // NOTE: _state is NOT cleared
}
```

During compilation, each stateful Func node (`accumulator`, `sample_hold`, `edge_detect`)
gets assigned a slot in `_state`. The slot count is determined at compile time.

#### New Func: `accumulator`

Integrates a trigger signal over time with configurable step and clamp.

```
Args: [trigger, step, min, max]
  trigger: 1.0 = active, 0.0 = inactive
  step:    increment per evaluation tick while trigger is active
  min/max: clamp range

Behavior:
  if trigger > 0.5:
      state[slot] += step
      state[slot] = clamp(state[slot], min, max)
  return state[slot]
```

For hat trim, a **reset** can be wired as a separate accumulator that multiplies
the output, or we add a 5th arg `reset` that zeros the state:

```
Args: [trigger, step, min, max, reset]
  if reset > 0.5: state[slot] = 0
```

#### New Func: `sample_hold`

Captures the input value on a trigger edge and holds it.

```
Args: [input, trigger]
  input:   value to sample (e.g., Axis.Pitch.Position)
  trigger: samples on falling edge (1→0 transition)

State: [last_trigger, held_value]
  if previous_trigger > 0.5 AND current_trigger <= 0.5:
      held_value = input
  return held_value
```

This gives FTR the "capture position on button release" behavior.

#### New Func: `edge_detect`

Outputs 1.0 for one tick on rising edge, 0.0 otherwise.

```
Args: [input]

State: [previous_value]
  rising = (input > 0.5) AND (previous_value <= 0.5)
  previous_value = input
  return rising ? 1.0 : 0.0
```

Useful for debouncing hat presses (one trim step per press, not continuous).
For auto-repeat, the `accumulator` with raw trigger (no edge detect) provides
continuous stepping while held.

### Part 5: Graph Templates for Trim Strategies

#### Hat Trim Sub-Graph (`trim_hat.json`)

Reusable Include graph for one axis of hat trim:

```
Inputs:  hat_pos, hat_neg, reset
Params:  TrimStep (default: 0.5), TrimMin (default: -20), TrimMax (default: 20)
Outputs: TrimOffset

┌──────────┐     ┌─────────────────────┐
│ hat_pos  │────▶│ accumulator         │
│          │     │ (+TrimStep, clamp)  │──────▶ TrimOffset
│ hat_neg  │────▶│ accumulator         │
│          │     │ (-TrimStep, clamp)  │──┘ (Add)
└──────────┘     └─────────────────────┘
                         ▲ reset
```

Actually simpler: use a single accumulator with `Add(hat_pos, Neg(hat_neg))` as
the trigger and step as a signed value:

```json
{
  "nodes": {
    "hat_pos":    { "type": "Input", "name": "hat_pos" },
    "hat_neg":    { "type": "Input", "name": "hat_neg" },
    "reset":      { "type": "Input", "name": "reset" },
    "step":       { "type": "Param", "name": "TrimStep", "const": 0.5 },
    "min":        { "type": "Param", "name": "TrimMin", "const": -20.0 },
    "max":        { "type": "Param", "name": "TrimMax", "const": 20.0 },
    "pos_step":   { "type": "Op", "op": "Mul", "args": ["hat_pos", "step"] },
    "neg_step":   { "type": "Op", "op": "Mul", "args": ["hat_neg", "step"] },
    "net_step":   { "type": "Op", "op": "Sub", "args": ["pos_step", "neg_step"] },
    "trigger":    { "type": "Op", "op": "Add", "args": ["hat_pos", "hat_neg"] },
    "accum":      { "type": "Func", "func": "accumulator",
                    "args": ["trigger", "net_step", "min", "max", "reset"] },
    "out":        { "type": "Output", "name": "TrimOffset", "src": "accum" }
  }
}
```

#### FTR Sub-Graph (`trim_ftr.json`)

```
Inputs:  ftr_button, axis_position, spring_in
Params:  (none needed)
Outputs: TrimOffset, SpringGain

┌────────────┐     ┌──────────────┐
│ ftr_button │────▶│ sample_hold  │──▶ scale ──▶ TrimOffset
│            │     │ (axis_pos,   │
│            │     │  falling)    │
│ axis_pos   │────▶│              │
└────────────┘     └──────────────┘

┌────────────┐     ┌──────────────┐
│ ftr_button │────▶│ Neg          │
│            │     │              │──▶ Add(1.0) ──▶ Mul(spring_in) ──▶ SpringGain
└────────────┘     └──────────────┘
  (1=pressed → spring=0, 0=released → spring=spring_in)
```

```json
{
  "nodes": {
    "ftr":         { "type": "Input", "name": "ftr_button" },
    "pos":         { "type": "Input", "name": "axis_position" },
    "spring_in":   { "type": "Input", "name": "spring_in" },
    "one":         { "type": "Const", "const": 1.0 },
    "pos_mm_min":  { "type": "Param", "name": "PosMin", "const": 0.0 },
    "pos_mm_max":  { "type": "Param", "name": "PosMax", "const": 40.0 },
    "pos_range":   { "type": "Op", "op": "Sub", "args": ["pos_mm_max", "pos_mm_min"] },
    "pos_mm":      { "type": "Op", "op": "Add",
                     "args": ["pos_mm_min", { "op": "Mul", "args": ["pos", "pos_range"] }] },
    "captured":    { "type": "Func", "func": "sample_hold", "args": ["pos_mm", "ftr"] },
    "not_ftr":     { "type": "Op", "op": "Sub", "args": ["one", "ftr"] },
    "spring_out":  { "type": "Op", "op": "Mul", "args": ["spring_in", "not_ftr"] },
    "trim_out":    { "type": "Output", "name": "TrimOffset", "src": "captured" },
    "spring":      { "type": "Output", "name": "SpringGain", "src": "spring_out" }
  }
}
```

#### Top-Level Graph Wiring

In a helicopter graph template, the FTR sub-graph is wired as an Include node:

```json
"ftr_trim": {
  "type": "Include",
  "path": "trim_ftr.json",
  "inputMap": {
    "ftr_button": "grip_ftr",
    "axis_position": "axis_pitch_pos",
    "spring_in": "base_spring"
  },
  "outputMap": {
    "TrimOffset": "ftr_trim_offset",
    "SpringGain": "ftr_spring"
  }
}
```

The outputs replace the default spring/trim outputs for that axis.

### Part 6: State Reset Semantics

Stateful nodes need defined reset behavior:

| Event                         | Behavior                                  |
| ----------------------------- | ----------------------------------------- |
| Graph switch (vehicle change) | Reset all `_state` to 0.0                 |
| Profile switch                | Reset all `_state` to 0.0                 |
| Graph re-compile              | Fresh `_state` array (naturally zeroed)   |
| Explicit reset input          | Per-node via `reset` arg (accumulator)    |

The graph evaluator should expose a `ResetState()` method called on profile/vehicle change.

## Implementation Steps

### Phase 1: ESP32 Grip Reading (DEFERRED)

Deferred until hardware is ready. The button binding system supports any DirectInput
device or keyboard, so all plugin-side work can be developed and tested without the
ESP32 grip SPI integration.

1. Add SPI3 initialization for grip in a new `GripReader` component
2. Implement shift register read cycle (74HC165 chain)
3. Map bit positions to CommManager button indices
4. Verify button states with USB HID joystick viewer tool
5. Define pin assignments for target PCB

### Phase 2: Graph Engine — Stateful Nodes

1. Add `_state` array to `GraphCompiledEvaluator` (persists across evals)
2. Assign state slots during compilation for stateful Func nodes
3. Implement `accumulator` func with trigger/step/clamp/reset
4. Implement `sample_hold` func with falling-edge capture
5. Implement `edge_detect` func with rising-edge detection
6. Add `ResetState()` method, call on profile/vehicle switch
7. Unit tests for all three stateful nodes

### Phase 3: New Graph Inputs & Button Binding

1. Add `ButtonInputReader` class (SharpDX.DirectInput, multi-device enumeration, polling)
2. Add `ButtonBinding` class and `GripButtonBindings` dict to `DiyFfbPluginSettings`
3. Add `Grip.*` and `Axis.*` signals to `GraphSignalCatalogData`
4. Add `BuildGripInputs()` and `BuildAxisInputs()` to `GraphSignalCatalog`
5. Wire into `BuildGraphInputs()` in `DiyFfbPlugin`
6. Store last `AxisState.position` per axis in plugin for `Axis.*` inputs
7. Implement press-to-bind UI in plugin settings (device scan + button capture)
8. Add per-row clear and bulk clear-all for bindings

### Phase 4: Graph Templates

1. Create `trim_hat.json` sub-graph (accumulator-based hat trim)
2. Create `trim_ftr.json` sub-graph (sample-and-hold FTR)
3. Wire into helicopter and GA graph templates
4. Verify trim outputs appear in graph debug/preview UI

### Phase 5: Testing & Tuning

1. End-to-end: grip button → graph input → accumulator → TrimOffset → ESP32
2. Test FTR capture accuracy (AxisState read-back latency)
3. Test state reset on vehicle/profile switch
4. Test coexistence: hat trim + graph-computed X-Plane trim (additive)
5. Tune default TrimStep, TrimMin, TrimMax params per aircraft type

## Open Questions

1. **Accumulator rate**: Should step be per-tick (rate-dependent) or time-normalized (mm/sec)?
   Time-normalized is better for consistent feel across different eval rates.
2. **FTR damping**: Should damping also go to zero during FTR, or stay active?
   Graph can handle either — wire `not_ftr` into DamperGain Mul as well.
3. **State in sub-graphs**: When a sub-graph is used via Include, does its `_state` live
   in the sub-evaluator's instance? (Yes — `_includeCache` preserves compiled evaluators
   across evals, so their `_state` arrays persist naturally.)
4. **Trim persistence**: Should trim state survive a profile switch? Current design resets.

### Deferred (Phase 1: ESP32 Grip SPI)

1. **Pin assignment**: Which GPIOs are free on the current PCB for the second SPI bus?
2. **Shift register chain length**: How many 74HC165s are in the Warthog grip? (Believed to be 3 → 24 bits)
3. **Button mapping reference**: Verified bit-to-button mapping, or logic analyzer needed?
