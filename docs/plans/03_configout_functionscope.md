# ConfigOut & FunctionScope — Graph Infrastructure

Two new graph features that simplify multi-function templates and enable
graph-computed config fields.

**FunctionScope** on Include nodes eliminates output wiring — sub-graphs
with Scoped Output nodes declare their outputs with `SignalSuffix` and
the runtime auto-routes them to the correct function. Cuts link count
in half for multi-function templates and removes parent Output nodes
entirely.

**ConfigOut** is a new terminal node type (like Output) that writes
graph-computed values to proto config fields. Each ConfigOut node has a
`ConfigType` that determines which fields are available. Keeps config
and graph logic in sync — the graph computes both amplitudes AND the
config that controls how those amplitudes are interpreted.

**Prerequisites:** None (pure plugin/graph infrastructure)

**Status:** Implemented.


---

## 1. Motivation

### Existing graph simplification

The current heli template has ~14 links per cyclic Include (7 input +
7 output). Each function axis needs a parent Output node that gathers
output ports from the sub-graph and routes them to the signal catalog.
With 4 function axes, that's 4 Output nodes + 28 output links in the
parent template — pure boilerplate.

With FunctionScope + Scoped Output nodes, the sub-graph's Output node
declares `SignalSuffix` on its ports via a dropdown, and the Include's
`FunctionScope` tells the runtime which function to route to. Result:
zero output links, zero parent Output nodes.

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

```text
(none)                      ← default, generic mode
FlightStickPitch
FlightStickRoll
FlightPedals
FlightStickCollective
```

The dropdown values come from `GraphSignalCatalogData.FunctionScopeOptions`.
`(none)` means all outputs use the existing generic-port pattern (no
scoping).

When an Include has `FunctionScope` set, Scoped Output and ConfigOut
nodes inside the sub-graph inherit it as their function group. The
parent only wires inputs — all output and config routing is implicit.

**ConfigType mismatch warning:** If the included sub-graph has ConfigOut
nodes whose `ConfigType` doesn't match the function type implied by the
`FunctionScope` (e.g., ConfigType "FlightPedals" on an Include with
FunctionScope "FlightStickPitch"), the Include inspector shows an orange
warning. The mapping is:

| FunctionScope | Expected ConfigType |
| --- | --- |
| FlightStickPitch | FlightStick |
| FlightStickRoll | FlightStick |
| FlightStickCollective | FlightStick |
| FlightPedals | FlightPedals |


---

## 3. Scoped Output Nodes

Output nodes in library sub-graphs have a **Scoped checkbox**. When
checked, the node switches from freeform port names to signal-suffix
dropdowns, and its outputs are auto-registered by the parent Include's
FunctionScope instead of appearing as ports on the Include node.

**Scoped = false (default, unscoped):**
- Ports use freeform `Name` (e.g., "spring", "my_output")
- Appear as output ports on the parent Include node
- Must be wired to a parent Output node explicitly
- Fully backward compatible

**Scoped = true:**
- Ports use a **SignalSuffix dropdown** populated from
  `GraphSignalCatalogData.OutputSuffixes` (BuffetAmplitude, DamperGain,
  Friction, LoadForce, SpringGain, TrimOffset)
- No SignalGroup dropdown — the group is determined by the parent
  Include's FunctionScope at runtime
- Do NOT appear as ports on the parent Include node
- Converter auto-registers `FunctionScope + "." + SignalSuffix`
- Node header shows "Output (Scoped, Ports: N)"

Both scoped and unscoped Output nodes can coexist in the same sub-graph.

**Example (scoped Output node in a library sub-graph):**

```json
{
  "Id": "out_ffb",
  "Title": "FFB Outputs",
  "Kind": "Output",
  "Scoped": true,
  "Ports": [
    { "SignalSuffix": "SpringGain", "Kind": "Input" },
    { "SignalSuffix": "DamperGain", "Kind": "Input" },
    { "SignalSuffix": "Friction", "Kind": "Input" },
    { "SignalSuffix": "LoadForce", "Kind": "Input" },
    { "SignalSuffix": "TrimOffset", "Kind": "Input" }
  ]
}
```


---

## 4. ConfigOut Node Type

A new terminal node (like Output) whose ports write to proto config
fields instead of streaming outputs.

### ConfigType

Each ConfigOut node has a **ConfigType dropdown** that determines which
`OverrideFieldRegistry` fields are available for its ports:

```text
(none)          ← shows all float fields
FlightStick     ← shows FlightStick fields + shared fields
FlightPedals    ← shows FlightPedals fields + shared fields
```

**Shared field groups** (available regardless of ConfigType):
OutputScaling, Physics, StaticBalanceTuning, ForceFeedback.

**Function-specific groups** (only available when ConfigType matches):
FlightStick, FlightPedals, AutomotivePedals, Shifter, Damper.

### ConfigField dropdown

Each ConfigOut port has a `ConfigField` dropdown populated from the
`OverrideFieldRegistry`, filtered by `ConfigType`. The dropdown shows
`FieldPath` values (e.g., `flight_stick.damping`, `simulated_mass`).

### Example

```json
{
  "Id": "cfg_out",
  "Title": "Vib Config",
  "Kind": "ConfigOut",
  "ConfigType": "FlightStick",
  "Ports": [
    { "Name": "rotation_sign", "Kind": "Input",
      "ConfigField": "flight_stick.rotation_sign" },
    { "Name": "ratio_0", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.0" },
    { "Name": "ratio_1", "Kind": "Input",
      "ConfigField": "flight_stick.vib_harmonic_ratios.1" }
  ]
}
```

ConfigOut always requires `FunctionScope` on its Include — without it,
ConfigOut nodes are ignored (no function → don't know which config to
write to). The Include inspector shows a warning when `FunctionScope`
implies a different config type than the sub-graph's ConfigOut nodes
declare.


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

**ConfigOut** additionally has a `ConfigType` dropdown (FlightStick /
FlightPedals) that filters its port field options. This is independent
of `SignalGroup`.

Unscoped Output nodes in the parent template still use the existing
`SignalGroup` dropdown (shown as the function group selector) — this
is unchanged. Scoped Output/ConfigOut nodes inside sub-graphs have no
`SignalGroup` of their own; they inherit from their Include site.


---

## 6. Runtime Conversion

The converter handles three cases for Include nodes with `FunctionScope`:

**Scoped Outputs:** For each `ScopedOutput` in the `CachedInterface`,
create a top-level `NodeType.Output` runtime node with
`Name = FunctionScope + "." + SignalSuffix`. Source is the Include's
OutputMap entry for that port.

**Scoped ConfigOuts:** For each `ConfigOutput` in the `CachedInterface`,
create a top-level `NodeType.ConfigOut` runtime node with
`Name = FunctionScope + ":" + ConfigField`. Source is the Include's
OutputMap entry for that port.

**Unscoped Outputs:** Continue to appear as ports on the Include node
and are wired to parent Output nodes explicitly — no change from today.

```text
Existing (unscoped):
  sub-graph Output port "spring" → Include output port → parent link →
  parent Output(SignalGroup="FlightStickPitch", SignalSuffix="SpringGain")
  → runtime: FlightStickPitch.SpringGain

Scoped:
  sub-graph Output(Scoped=true, port SignalSuffix="SpringGain") →
  Include(FunctionScope="FlightStickPitch") →
  runtime: FlightStickPitch.SpringGain

Same runtime result, no parent Output node, no link.
```

**Backward compatibility:** `FunctionScope` defaults to `(none)`,
`Scoped` defaults to `false`. Existing sub-graphs with generic ports
continue to work unchanged. Both modes can coexist in the same parent
template.


---

## 7. Plugin Config Change Detection

After each graph evaluation cycle (20 Hz), the plugin reads ConfigOut
values from `GraphEvaluationResult.ConfigOutputs` and compares to the
last-sent values. ConfigOut keys use the format
`"FunctionScope:FieldPath"` (e.g., `"FlightStickPitch:flight_stick.damping"`).

On change, the plugin:

1. Parses the key into scope name + field path
2. Resolves the function ID from the scope name
3. Validates the field belongs to the correct config type (or a shared
   group) — mismatches are silently skipped
4. Applies the new value via `OverrideFieldRegistry.SetValue()` through
   the `ConfigOrchestrator`

**Shared field groups** (OutputScaling, Physics, StaticBalanceTuning,
ForceFeedback) are valid for any function scope. Function-specific
groups (FlightStick, FlightPedals, etc.) must match the scope.

In steady state (no Param edits), ConfigOut values are constant and
no uploads occur — this is a cheap comparison, not a real upload.

**Startup**: on profile load, ConfigOut values are evaluated and
uploaded as part of the initial config send. No special path needed.

**Graph reload**: `_lastConfigOutValues` is cleared, forcing a full
re-evaluation on the next cycle.


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

1. Check `Scoped` on the Output node in the sub-graph
2. Select appropriate `SignalSuffix` for each port from the dropdown
3. Add `FunctionScope` to Include nodes in parent template
4. Remove the now-redundant parent Output nodes and output links

This is backward-compatible: new sub-graphs default to scoped,
existing ones stay generic until migrated. Both modes can coexist
in the same parent template (some includes scoped, others generic).

### Sharing across functions

Aircraft-level values like blade count are regular `Aircraft.*` Params,
shared across all functions by the existing Param system. They flow
into each sub-graph include as Input wiring (this already exists — no
new links needed). Inside the sub-graph, computation nodes derive
ratios and wire them to the scoped ConfigOut terminal:

```text
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

1. Add `ConfigOut` to `GraphNodeKind` enum, `FunctionScope` and `Scoped`
   to `GraphNode`, `ConfigField` to `GraphPort`, `ConfigType` to
   `GraphNode` — `GraphModel.cs`
2. Add `ConfigOut` to `NodeType` enum, `ConfigOutputs` dict to
   `GraphEvaluationResult` — `GraphEvaluator.cs`, `GraphCompiledEvaluator.cs`
3. Serialize `FunctionScope`, `Scoped`, `ConfigType`, `ConfigField` in
   DTOs — `GraphSerializer.cs`
4. `ExtractInterface`: split Output nodes into `Outputs` (unscoped) and
   `ScopedOutputs` (with SignalSuffix); extract `ConfigOutputs` from
   ConfigOut nodes — `GraphSerializer.cs`
5. Converter: ConfigOut node conversion, FunctionScope scoped output +
   config output registration — `GraphRuntimeConverter.cs`
6. Add `FunctionScopeOptions`, `ConfigTypeOptions`, `OutputSuffixes`,
   `GetConfigTypeForScope()` — `GraphSignalCatalogData.cs`
7. Editor UI: FunctionScope dropdown on Include, Scoped checkbox on
   Output, ConfigType dropdown on ConfigOut, ConfigField dropdown on
   ConfigOut ports, ConfigType mismatch warning on Include —
   `GraphEditorControl.xaml` + `.xaml.cs`
8. `CheckConfigOutChanges()` in plugin eval loop with ConfigType
   validation and shared field group handling — `DiyFfbPlugin.cs`
9. Register new vibration config fields in `OverrideFieldRegistry`
   (deferred to plan 07 — fields must exist before ConfigOut can
   target them)
10. Migrate existing heli template to use FunctionScope + Scoped
    (validation step)
