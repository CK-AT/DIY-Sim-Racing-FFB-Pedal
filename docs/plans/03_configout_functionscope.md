# ConfigOut & FunctionScope — Graph Infrastructure

Two new graph features that simplify multi-function templates and enable
graph-computed config fields.

**FunctionScope** on Include nodes eliminates output wiring — sub-graphs
declare their outputs with `SignalSuffix` and the runtime auto-routes them
to the correct function. Cuts link count in half for multi-function
templates and removes parent Output nodes entirely.

**ConfigOut** is a new terminal node type (like Output) that writes
graph-computed values to `FlightStickConfig` proto fields. Keeps config
and graph logic in sync — the graph computes both amplitudes AND the
config that controls how those amplitudes are interpreted.

**Prerequisites:** None (pure plugin/graph infrastructure)


---

## 1. Motivation

### Existing graph simplification

The current heli template has ~14 links per cyclic Include (7 input +
7 output). Each function axis needs a parent Output node that gathers
output ports from the sub-graph and routes them to the signal catalog.
With 4 function axes, that's 4 Output nodes + 28 output links in the
parent template — pure boilerplate.

With FunctionScope, the sub-graph's Output node declares
`SignalSuffix` on its ports, and the Include's `FunctionScope` tells
the runtime which function to route to. Result: zero output links,
zero parent Output nodes.

| | Before (generic) | After (scoped) |
| --- | --- | --- |
| Links per cyclic include | ~14 (7 in + 7 out) | ~7 (7 in, 0 out) |
| Parent Output nodes | 4 (one per function) | 0 |

This value is immediate — existing templates can be migrated without
any other feature work.

### Graph-computed config

ConfigOut extends the benefit to static config fields. The first use
case is vibration harmonic ratios (plan 07 DDS subsystem), where the
config that controls which frequency each DDS slot runs at must match
the graph template that drives each slot's amplitude. Without ConfigOut,
these are maintained separately and can diverge.

But ConfigOut is general — any static config field that should derive
from graph Params can use it. Future candidates include per-aircraft
damping and spring constants.


---

## 2. FunctionScope on Include Nodes

A new `FunctionScope` property on Include nodes tells the runtime which
function the sub-graph's scoped outputs target.

**Include node with FunctionScope:**

```json
{
  "Id": "cyclic_pitch_include",
  "Title": "Cyclic Pitch",
  "Kind": "Include",
  "IncludePath": "..\\_embedded\\heli_cyclic.json",
  "FunctionScope": "FlightStickPitch"
}
```

**Editor UI:** `FunctionScope` appears as a dropdown on the Include
node inspector (below IncludePath, above the port lists). Options:

```
(none)                      ← default, generic mode
FlightStickPitch
FlightStickRoll
FlightPedals
FlightStickCollective
```

The dropdown values come from `GraphSignalCatalogData`. `(none)` means
all outputs use the existing generic-port pattern (no scoping).

When an Include has `FunctionScope` set, scoped Output and ConfigOut
nodes inside the sub-graph inherit it as their function group. The
parent only wires inputs — all output and config routing is implicit.


---

## 3. Scoped Output Nodes

**Scoped Output node in sub-graph** (replaces generic output ports):

```json
{
  "Id": "out_ffb",
  "Title": "FFB Outputs",
  "Kind": "Output",
  "Ports": [
    { "Name": "spring", "Kind": "Input", "SignalSuffix": "SpringGain" },
    { "Name": "damper", "Kind": "Input", "SignalSuffix": "DamperGain" },
    { "Name": "friction", "Kind": "Input", "SignalSuffix": "Friction" },
    { "Name": "load", "Kind": "Input", "SignalSuffix": "LoadForce" },
    { "Name": "trim", "Kind": "Input", "SignalSuffix": "TrimOffset" }
  ]
}
```

An Output node is scoped when its ports have `SignalSuffix` AND the
Include that contains it has a `FunctionScope`. The runtime converter
registers `FlightStickPitch.SpringGain` etc. — identical to what the
parent's explicit Output node produced before.

Without `FunctionScope` on the Include, these ports are treated as
generic output ports (by `Name`), same as today. So an Output node
with `SignalSuffix` on its ports works in both modes.


---

## 4. ConfigOut Node Type

A new terminal node (like Output) whose ports write to proto config
fields instead of streaming outputs. Each port's `ConfigField` is a
**dropdown** populated from the `OverrideFieldRegistry`, using the same
`FieldPath` naming and `DisplayName` labels that appear in the function
config UIs.

### ConfigField dropdown

The editor presents `ConfigField` as a dropdown on each ConfigOut port.
The dropdown shows `DisplayName` (what the user sees), stores `FieldPath`
(what the runtime uses). Options are filtered by the function type
implied by the parent Include's `FunctionScope`:

```text
FunctionScope = FlightStickPitch  →  shows FlightStick fields:
  "Motion Range"              (flight_stick.motion_range)
  "Damping"                   (flight_stick.damping)
  "Centering Spring Constant" (flight_stick.centering_spring_const)
  "Rotation Sign"             (flight_stick.rotation_sign)         ← NEW
  "DDS 1 Harmonic Ratio 1"   (flight_stick.vib_harmonic_ratios.0) ← NEW
  "DDS 1 Harmonic Ratio 2"   (flight_stick.vib_harmonic_ratios.1) ← NEW
  ...
```

New vibration config fields must be registered in the
`OverrideFieldRegistry` (with the `FlightStick` group) before ConfigOut
can target them. This keeps a single source of truth for what config
fields exist and how they're named.

### Example

```json
{
  "Id": "cfg_out",
  "Title": "Vib Config",
  "Kind": "ConfigOut",
  "Ports": [
    { "Name": "rotation_sign", "Kind": "Input",
      "ConfigField": "flight_stick.rotation_sign" },
    { "Name": "ratio_0", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.0" },
    { "Name": "ratio_1", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.1" },
    { "Name": "ratio_2", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.2" },
    { "Name": "ratio_3", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.3" },
    { "Name": "ratio_4", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.4" }
  ]
}
```

ConfigOut always requires `FunctionScope` on its Include — without it,
ConfigOut nodes are ignored (no function → don't know which config to
write to). The editor can warn when a sub-graph with ConfigOut nodes is
included without a FunctionScope.


---

## 5. SignalGroup Usage Rules

| Node type | SignalGroup | Constraint |
| --- | --- | --- |
| Param | freeform string | Any group name (e.g., `Aircraft`, `Vib`) |
| Input (unscoped) | freeform string | Any group (e.g., `XPlane`, `Grip`) |
| Output (unscoped) | function dropdown | Must be a valid function group |
| Output (scoped, in sub-graph) | not set | Inherited from Include's `FunctionScope` |
| ConfigOut (in sub-graph) | not set | Inherited from Include's `FunctionScope` |
| Include | `FunctionScope` dropdown | `(none)` or valid function group |

Unscoped Output nodes in the parent template still use the existing
`SignalGroup` dropdown (shown as the function group selector) — this
is unchanged. Scoped Output/ConfigOut nodes inside sub-graphs have no
`SignalGroup` of their own; they inherit from their Include site.


---

## 6. Runtime Conversion

`GraphRuntimeConverter` already builds Output nodes with
`BuildFullSignalName(node.SignalGroup, port.SignalSuffix, port.Name)`.
The change: when recursing into an included graph, if the Include node
has `FunctionScope`, pass it as the `SignalGroup` for any Output node
that has `SignalSuffix` on its ports, and for any ConfigOut node.

```
Existing (unscoped):
  sub-graph Output port "spring" → Include output port → parent link →
  parent Output(SignalGroup="FlightStickPitch", SignalSuffix="SpringGain")
  → runtime: FlightStickPitch.SpringGain

Scoped:
  sub-graph Output(port SignalSuffix="SpringGain") →
  Include(FunctionScope="FlightStickPitch") →
  runtime: FlightStickPitch.SpringGain

Same runtime result, no parent Output node, no link.
```

This mirrors the existing Output registration at
`GraphRuntimeConverter.cs:100-117` — same `BuildFullSignalName`, just
with the scope inherited from the Include node instead of from the
Output node's own `SignalGroup`.

**Backward compatibility:** `FunctionScope` defaults to `(none)`.
Existing sub-graphs with generic ports continue to work unchanged.
Both modes can coexist in the same parent template.


---

## 7. Plugin Config Change Detection

After each graph evaluation cycle (20 Hz), the plugin reads all ConfigOut
values per function and compares to the last-sent config. Upload only on
change:

```csharp
void CheckConfigOutChanges()
{
    foreach (var functionId in flightFunctionIds)
    {
        string prefix = GetGraphFunctionPrefix(functionId);
        if (prefix == null) continue;

        var config = currentConfigs[functionId];
        bool changed = false;

        // ConfigOut nodes use OverrideFieldRegistry FieldPaths like
        // "flight_stick.vib_harmonic_ratios.3"
        foreach (var (field, value) in GetConfigOutputs(prefix))
        {
            changed |= SetConfigField(config, field, value);
        }

        if (changed)
        {
            EnqueueConfigUpload(functionId, config);
        }
    }
}
```

In steady state (no Param edits), ConfigOut values are constant and
no uploads occur — this is a cheap comparison, not a real upload.

**Startup**: on profile load, ConfigOut values are evaluated and
uploaded as part of the initial config send. No special path needed.

**User edits Param slider**: next graph evaluation cycle picks up the
new Param value → flows through computation nodes → ConfigOut value
changes → plugin detects → config upload. Latency: one eval cycle
(50 ms at 20 Hz). Acceptable for static config changes.


---

## 8. Safe Defaults

* Graph without ConfigOut nodes → no config fields modified →
  existing graphs completely unaffected.
* ConfigOut with computed value 0 for ratios → DDS slot disabled
  (zero ratio = skip). Safe fallback.
* Disconnected ConfigOut input port → value = 0 → safe default.


---

## 9. Migrating Existing Graphs

Scoped Output is optional — existing graphs with generic Output nodes
and explicit parent wiring continue to work. But scoped mode can
simplify existing templates too. Migration per sub-graph:

1. Add `SignalSuffix` to Output node ports in the sub-graph
2. Add `FunctionScope` to Include nodes in parent template
3. Remove Output nodes and output links from parent template

This is backward-compatible: new sub-graphs default to scoped,
existing ones stay generic until migrated. Both modes can coexist
in the same parent template (some includes scoped, others generic).

### Sharing across functions

Aircraft-level values like blade count are regular `Aircraft.*` Params,
shared across all functions by the existing Param system. They flow
into each sub-graph include as Input wiring (this already exists — no
new links needed). Inside the sub-graph, computation nodes derive
ratios and wire them to the scoped ConfigOut terminal:

```
heli_default.json (parent template)

  Aircraft.BladeCount   [Param = 5]     ← shared, one slider
  Aircraft.RotationSign [Param = 1]     ← shared
  Aircraft.TRGearRatio  [Param = 4.62]  ← shared

  cyclic_pitch_include
    Kind: Include, FunctionScope: "FlightStickPitch"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign
    (no output links — scoped nodes handle it)

  cyclic_roll_include
    Kind: Include, FunctionScope: "FlightStickRoll"
    Inputs wired: torque, rpm, aero_trq, blade_count, rotation_sign

  pedals_include
    Kind: Include, FunctionScope: "FlightPedals"
    Inputs wired: torque, rpm, aero_trq, tr_gear_ratio
```

Same `Aircraft.BladeCount` Param feeds both cyclic includes. Changing
blade count → graph re-evaluates → scoped ConfigOut values change for
pitch and roll → plugin detects → config upload for both functions.


---

## 10. Implementation

1. Register new vibration config fields in `OverrideFieldRegistry`
   (`flight_stick.rotation_sign`, `flight_stick.vib_harmonic_ratios.0`..`.4`,
   `flight_stick.vib2_harmonic_ratios.0`..`.1`, same for `flight_pedals.*`)
2. Add `FunctionScope` property to Include node JSON schema
3. Add `SignalSuffix` property to port JSON schema
4. Add `FunctionScope` dropdown to Include node inspector in editor UI
5. Add `ConfigOut` node type to graph node schema (`NodeType.ConfigOut`)
6. Add `ConfigField` dropdown to ConfigOut port inspector, populated from
   `OverrideFieldRegistry` filtered by the config type implied by `FunctionScope`
7. Implement scoped output registration in `GraphRuntimeConverter`
   (inherit `FunctionScope` as `SignalGroup` for Output nodes with `SignalSuffix`)
8. Implement ConfigOut evaluation in `GraphRuntimeConverter`
   (register as `NodeType.ConfigOut` with function scope and `ConfigField`)
9. Implement `CheckConfigOutChanges()` in plugin — config change detection
   and upload on change
10. Migrate existing heli template to use FunctionScope (eliminate output links)
11. Test backward compatibility (existing graphs without FunctionScope unchanged)
