# Graph Node Types Design

Status: Comprehensive design document for all graph node types and planned enhancements.

## Overview

This document covers all node types in the graph editor, their current implementation, and planned improvements.

### Node Type Summary

| Kind | Purpose | Default Ports | Can Add Ports | Has Controls |
|------|---------|---------------|---------------|--------------|
| Input | Graph input signal | 1 out | Yes (outputs) | No |
| Output | Graph output signal | 1 in | Yes (inputs) | No |
| Param | Configurable parameter | 1 out | Yes (outputs) | Yes (widget) |
| Const | Fixed constant value | 1 out | No | Yes (value) |
| Op | Math operation | 2 in, 1 out | No (fixed) | Yes (op selector) |
| Func | Built-in function | varies, 1 out | No (auto) | Yes (func selector) |
| Include | Subgraph reference | 1 in, 1 out | Yes (both) | Yes (path + ports) |

### Color Coding

| Kind | RGB | Hex | Visual |
|------|-----|-----|--------|
| Input | (60, 120, 180) | #3C78B4 | Blue |
| Output | (200, 120, 40) | #C87828 | Orange |
| Param | (140, 80, 180) | #8C50B4 | Purple |
| Const | (100, 100, 100) | #646464 | Gray |
| Op | (80, 150, 80) | #509650 | Green |
| Func | (60, 140, 160) | #3C8CA0 | Teal |
| Include | (180, 80, 140) | #B4508C | Magenta |

---

## Input Node

### Purpose
Defines an input signal that flows into the graph from external sources. When the graph is used as an include, Input nodes become input ports on the Include node.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1158-1161](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1158-L1161)):
```csharp
if (kind == GraphNodeKind.Input)
{
    node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
}
```

**Visual Layout**:
```
┌─────────────────────┐
│ ▶ velocity          │  ← Blue title bar
├─────────────────────┤
│              ○ out  │  ← Output port (orange circle)
│              ○ out1 │  ← Additional outputs possible
└─────────────────────┘
```

**Inspector Panel**:
- Title field (editable)
- Port list with signal picker (GraphSignalCatalog.InputNames)
- **Add Output Port** button visible
- Each output port can select from available input signals

**Port Management**:
- Can add multiple output ports via inspector
- Each output port maps to a signal name
- Port names editable, must be unique within node

**Runtime Conversion** ([GraphRuntimeConverter.cs:83-95](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L83-L95)):
- Each output port becomes a separate runtime Input node
- Runtime node ID: `"{editorNodeId}:{portName}"`
- Runtime node Name: port name (used for input lookup)

**Evaluation** ([GraphEvaluator.cs:95-96](SimHubPlugin/GraphTest/GraphEvaluator.cs#L95-L96)):
```csharp
case NodeType.Input:
    _values[node.Id] = inputs.TryGetValue(node.Name, out var val) ? val : 0.0;
    break;
```

### Proposed Improvements

#### 1. Group Dropdown with Signal Selection

Replace freeform title with a **group dropdown**. Each node selects one group (top-level signal namespace), and ports select signals from within that group.

**Visual**:
```
┌─────────────────────────┐
│ ▶ [XPlane          ▼]   │  ← Group dropdown (blue title bar)
├─────────────────────────┤
│ [Speed.IAS     ▼] ○ out │  ← Port dropdown selects within group
│ [Angle.Alpha   ▼] ○ out │
│ [Rate.Roll     ▼] ○ out │
└─────────────────────────┘
```

**Behavior**:
- One group per node (e.g., "XPlane")
- Port dropdowns show available signals within that group
- Port display shows suffix only (e.g., "Speed.IAS" not "XPlane.Speed.IAS")
- Runtime uses full signal name: `{group}.{portSelection}` → `XPlane.Speed.IAS`

**Validation**:
- Group must be valid (from `GraphSignalCatalog.InputGroups`)
- Signal selection must exist in catalog (`GraphSignalCatalog.InputNames`)
- Unknown signals are blocked at edit time

**Data Model Change**:
```csharp
public sealed class GraphNode
{
    // ... existing ...
    public string SignalGroup { get; set; }  // e.g., "XPlane"
}

public sealed class GraphPort
{
    // ... existing ...
    public string SignalSuffix { get; set; }  // e.g., "Speed.IAS"
}
```

**Status**: [x] Implemented

#### 2. Default Value
Allow Input nodes to specify a fallback value when no external input is provided.

```csharp
public double InputDefaultValue { get; set; } = 0.0;
```

**Visual** (shown as hint on port):
```
│ [Speed.IAS ▼] (0)  ○ out │  ← Default in parentheses
```

**Status**: [ ] Not implemented

#### 3. Input Metadata
Optional units and description for documentation (in inspector).

```csharp
public string InputUnits { get; set; }        // "m/s", "kts", etc.
public string InputDescription { get; set; }  // Tooltip text
```

**Status**: [ ] Not implemented

### Edge Cases

1. **No group selected**: Show placeholder "Select group..."
2. **No signal selected**: Show placeholder "Select signal..."
3. **Duplicate port signals**: Allowed (same signal mapped to multiple outputs)
4. **Unknown signal at runtime**: Blocked at edit time via validation

---

## Output Node

### Purpose
Defines an output signal from the graph. When the graph is used as an include, Output nodes become output ports on the Include node.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1187-1190](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1187-L1190)):
```csharp
else if (kind == GraphNodeKind.Output)
{
    node.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
}
```

**Visual Layout**:
```
┌─────────────────────┐
│         force ◀     │  ← Orange title bar
├─────────────────────┤
│ in ○                │  ← Input port (blue circle)
│ in1 ○               │  ← Additional inputs possible
└─────────────────────┘
```

**Inspector Panel**:
- Title field (editable)
- Port list with signal picker (GraphSignalCatalog.OutputNames)
- **Add Input Port** button visible
- Each input port represents a graph output

**Port Management**:
- Can add multiple input ports via inspector
- Each input port = one graph output value
- Port names must be unique within node

**Runtime Conversion** ([GraphRuntimeConverter.cs:47-64](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L47-L64)):
```csharp
foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
{
    if (TryGetInputSource(nodes, graph.Links, node.Id, port.Name, out var source))
    {
        var outputNode = new GraphNode
        {
            Id = BuildPortId(node.Id, port.Name),  // "{nodeId}:{portName}"
            Name = port.Name,
            Type = NodeType.Output,
            Src = source  // Source node ID
        };
        runtime.Nodes[outputNode.Id] = outputNode;
    }
}
```

**Evaluation** ([GraphEvaluator.cs:113-115](SimHubPlugin/GraphTest/GraphEvaluator.cs#L113-L115)):
```csharp
case NodeType.Output:
    _values[node.Id] = Resolve(node.Src);
    break;
```

### Proposed Improvements

#### 1. Group Dropdown with Output Signal Selection

Same pattern as Input nodes: replace freeform title with a **group dropdown**. Each node selects one output group, and ports select signals within that group.

**Visual**:
```
┌───────────────────────────────┐
│   [FlightStickPitch   ▼] ◀   │  ← Group dropdown (orange title bar)
├───────────────────────────────┤
│ [SpringGain  ▼] ○             │  ← Port dropdown selects within group
│ [DamperGain  ▼] ○             │
│ [LoadForce   ▼] ○             │
└───────────────────────────────┘
```

**Behavior**:

- One group per node (e.g., "FlightStickPitch", "FlightPedals")
- Port dropdowns show available signals within that group
- Port display shows suffix only (e.g., "SpringGain" not "FlightStickPitch.SpringGain")
- Runtime uses full signal name: `{group}.{portSelection}` → `FlightStickPitch.SpringGain`

**Validation**:

- Group must be valid (from `GraphSignalCatalog.OutputGroups`)
- Signal selection must exist in catalog (`GraphSignalCatalog.OutputNames`)
- Unknown signals are blocked at edit time

**Data Model Change**: Same as Input nodes - uses `SignalGroup` on node and `SignalSuffix` on port.

**Status**: [x] Implemented

#### 2. Output Metadata
Optional units and description for documentation (in inspector).

```csharp
public string OutputUnits { get; set; }        // "N", "Nm", etc.
public string OutputDescription { get; set; }  // Tooltip text
```

**Status**: [ ] Not implemented

#### 3. Value Display
Show live output value on the node during preview.

**Visual**:
```
│ [SpringGain ▼] ○  [42.5]  │  ← Live value during preview
```

**Status**: [ ] Partially implemented (values shown on output ports elsewhere)

### Edge Cases

1. **Unconnected input**: Evaluates to 0.0
2. **Multiple outputs same name**: Allowed, both collected in results

---

## Param Node

### Purpose
Defines a configurable parameter with a UI widget. Parameters can be adjusted by users and are persisted with the graph.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1162-1165](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1162-L1165)):
```csharp
else if (kind == GraphNodeKind.Param)
{
    node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
}
```

**Visual Layout**:
```
┌───────────────────────────────────┐
│            damping                │  ← Purple title bar
├───────────────────────────────────┤
│ [========■====] 0.85       ○ out  │  ← Widget + output port
│ [Linear       ▼]           ○ mode │  ← Multiple params possible
└───────────────────────────────────┘
```

**Data Model** ([GraphModel.cs:61-86](SimHubPlugin/GraphEditor/GraphModel.cs#L61-L86)):
```csharp
public sealed class GraphParam
{
    public string Name { get; set; }
    public double DefaultValue { get; set; }
    public double Min { get; set; }
    public double Max { get; set; }
    public GraphParamUi Ui { get; set; }
}

public sealed class GraphParamUi
{
    public string Widget { get; set; }           // "slider", "knob", "checkbox", "enum", "text"
    public string Label { get; set; }
    public string Group { get; set; }
    public string Units { get; set; }
    public double? Step { get; set; }
    public int? Precision { get; set; }
    public bool LogScale { get; set; }
    public List<GraphParamOption> Options { get; }  // For enum widget
}
```

**Widget Types** ([GraphEditorControl.xaml.cs:452-549](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L452-L549)):

| Widget | Control | Value Range |
|--------|---------|-------------|
| slider | Slider | Min to Max, continuous |
| knob | Slider (styled) | Min to Max, continuous |
| checkbox | CheckBox | 0.0 or 1.0 |
| enum | ComboBox | From Options list |
| text | TextBox | Any numeric value |

**Inspector Panel**:
- Title field
- **Add Output Port** button (can have multiple param outputs)
- Per-port configuration:
  - Widget type selector
  - Default value
  - Min/Max range
  - Label, Group, Units
  - Step, Precision
  - LogScale checkbox
  - Enum options editor (if enum widget)

**Runtime Conversion** ([GraphRuntimeConverter.cs:83-95](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L83-L95)):
- Each output port becomes a separate runtime Param node
- Runtime node ID: `"{nodeId}:{portName}"`
- Runtime node Name: port name (used for parameter lookup)

**Evaluation** ([GraphEvaluator.cs:98-100](SimHubPlugin/GraphTest/GraphEvaluator.cs#L98-L100)):
```csharp
case NodeType.Param:
    _values[node.Id] = parameters.TryGetValue(node.Name, out var val) ? val : 0.0;
    break;
```

### Proposed Improvements

#### 1. Freeform Group Dropdown with Signal Naming

Similar pattern to Input/Output nodes: group from catalog, but **freeform signal names** within the group.

**Visual**:
```
┌───────────────────────────────────────┐
│           [Aircraft       ▼]          │  ← Group dropdown (from catalog)
├───────────────────────────────────────┤
│ [========■====] 0.85                  │
│ [Vref          ]              ○ out   │  ← Freeform signal name
│                                       │
│ [====■========] 1200                  │
│ [Rotor.SpeedNom]              ○ out   │  ← Can use dot notation
└───────────────────────────────────────┘
```

**Behavior**:

- Group must be from catalog-defined set: `Aircraft`, `System`, `Vehicle`, or `<function name>`
- Port signal names are freeform (user types the suffix)
- Runtime uses full signal name: `{group}.{portName}` → `Aircraft.Vref`

**Validation**:

- Group must be from `GraphSignalCatalog.ParamGroups`
- Signal names cannot match known **output** signals (e.g., cannot use `FlightStickPitch.SpringGain`)
- This prevents accidental parameter/output name collisions

**Data Model Change**: Same as Input/Output nodes - uses `SignalGroup` on node and `SignalSuffix` on port.

**Status**: [x] Implemented

#### 2. Inline Value Display
Show current value next to slider on node.

**Status**: [x] Already implemented - widgets show values

#### 3. Parameter Grouping on Node
Visually group related parameters when multiple ports exist.

**Status**: [ ] Not implemented - groups only used in external panels

### Edge Cases

1. **Value outside Min/Max**: Clamped on edit
2. **Missing enum option**: Shows raw value
3. **Precision mismatch**: Display rounds, internal value preserved
4. **Output name collision**: Blocked at edit time with validation error

---

## Const Node

### Purpose
Provides a fixed constant value. Simpler than Param when no UI configurability is needed.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1178-1181](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1178-L1181)):
```csharp
else if (kind == GraphNodeKind.Const)
{
    node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
}
```

**Visual Layout**:
```
┌─────────────┐
│    Const    │  ← Gray title bar
├─────────────┤
│        ○ out│
└─────────────┘
```

**Properties**:
```csharp
public double ConstValue { get; set; }  // The constant value
```

**Inspector Panel** ([GraphEditorControl.xaml.cs:1941, 1951](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1941)):
- Title field
- EditConst TextBox (numeric value input)
- Port list (read-only, single port)

**Runtime Conversion** ([GraphRuntimeConverter.cs:14-24](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L14-L24)):
- ConstValue directly copied to runtime node
- Runtime node uses same ID as editor node

**Evaluation** ([GraphEvaluator.cs:101-103](SimHubPlugin/GraphTest/GraphEvaluator.cs#L101-L103)):
```csharp
case NodeType.Const:
    _values[node.Id] = node.ConstValue;
    break;
```

### Proposed Improvements

#### 1. Named Constants
Optional display name for documentation.

```csharp
public string ConstName { get; set; }  // e.g., "PI", "GRAVITY"
```

**Visual**:
```
┌─────────────────────┐
│ PI                  │
├─────────────────────┤
│ 3.14159        ○ out│
└─────────────────────┘
```

**Status**: [ ] Not implemented

#### 2. Inline Editing
Double-click to edit value directly on node.

**Status**: [ ] Not implemented

#### 3. Common Constants Palette
Quick-add buttons in inspector: 0, 1, -1, PI, e, 2*PI

**Status**: [ ] Not implemented

### Edge Cases

1. **Non-numeric input**: Rejected, previous value kept
2. **Very large values**: Scientific notation display

---

## Op Node

### Purpose
Performs mathematical operations on input values.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1166-1172](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1166-L1172)):
```csharp
else if (kind == GraphNodeKind.Op)
{
    node.Op = "mul";  // Default operation
    node.Ports.Add(new GraphPort { Name = "a", Kind = GraphPortKind.Input });
    node.Ports.Add(new GraphPort { Name = "b", Kind = GraphPortKind.Input });
    node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
}
```

**Visual Layout**:
```
┌─────────────────┐
│   Op (mul)      │  ← Green title bar, shows operation
├─────────────────┤
│ a ○        ○ out│
│ b ○             │
└─────────────────┘
```

**Supported Operations** ([GraphEvaluator.cs:18-29](SimHubPlugin/GraphTest/GraphEvaluator.cs#L18-L29)):

| Op | Inputs | Formula | Notes |
|----|--------|---------|-------|
| add | a, b | a + b | |
| sub | a, b | a - b | |
| mul | a, b | a × b | Default |
| div | a, b | a / b | Safe: returns 0 if b < 1e-9 |
| min | a, b | min(a, b) | |
| max | a, b | max(a, b) | |
| abs | a | \|a\| | Single input |
| neg | a | -a | Single input |
| clamp | a, min, max | clamp(a, min, max) | Three inputs |
| lerp | a, b, t | a + (b-a) × t | Three inputs |

**Inspector Panel** ([GraphEditorControl.xaml.cs:1942, 1952](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1942)):
- Title field
- EditOp ComboBox (operation selector)
- Port list (read-only, fixed based on operation)

**Port Configuration**:
- Ports are **fixed** based on operation type
- Cannot add/remove ports manually
- Port names: "a", "b" (and "min", "max", "t" for clamp/lerp); neg uses output "-a"

**Runtime Conversion** ([GraphRuntimeConverter.cs:26-36](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L26-L36)):
- Op string mapped to OpType enum
- Input ports converted to Args list (ordered)

**Evaluation** ([GraphEvaluator.cs:104-166](SimHubPlugin/GraphTest/GraphEvaluator.cs#L104-L166)):
```csharp
private double EvalOp(GraphNode node)
{
    double a = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
    double b = node.Args.Count > 1 ? Resolve(node.Args[1]) : 0.0;

    switch (node.Op)
    {
        case OpType.Add: return a + b;
        case OpType.Div: return Math.Abs(b) < 1e-9 ? 0.0 : a / b;
        case OpType.Clamp:
            double min = node.Args.Count > 1 ? Resolve(node.Args[1]) : 0.0;
            double max = node.Args.Count > 2 ? Resolve(node.Args[2]) : 1.0;
            return Math.Min(max, Math.Max(min, a));
        // ... etc
    }
}
```

### Proposed Improvements

#### 1. Dynamic Ports for Clamp/Lerp
Currently clamp/lerp use the default 2-input port setup. Should auto-configure to 3 inputs.

**Status**: [ ] Not implemented - ports don't auto-adjust for operation

#### 2. Variadic Inputs
Allow add, mul, min, max to accept N inputs.

```
┌─────────────────┐
│   Op (add)      │
├─────────────────┤
│ a ○        ○ out│
│ b ○             │
│ c ○             │
│ [+ Add Input]   │
└─────────────────┘
```

**Status**: [ ] Not implemented

#### 3. Operation Preview
Show intermediate values on ports during preview.

**Status**: [ ] Partial - output value shown, input values not

### Edge Cases

1. **Division by zero**: Returns 0.0
2. **Clamp with min > max**: Uses values as-is (no swap)
3. **Missing inputs**: Defaults to 0.0

---

## Func Node

### Purpose
Calls a built-in function with specific semantics for FFB calculations.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1173-1177](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1173-L1177)):
```csharp
else if (kind == GraphNodeKind.Func)
{
    node.Func = _funcChoices[0];  // Default: "qhat_eff"
    EnsureFuncPorts(node);        // Auto-configure ports
}
```

**Visual Layout**:
```
┌─────────────────────┐
│   Func (qhat_eff)   │  ← Teal title bar, shows function
├─────────────────────┤
│ ias_kts ○      ○ out│
│ vref_kts ○          │
└─────────────────────┘
```

**Available Functions** ([GraphEditorControl.xaml.cs:2052-2067](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L2052-L2067)):

| Function | Inputs | Description |
|----------|--------|-------------|
| qhat_eff | ias_kts, vref_kts | Dynamic pressure ratio |
| torque_norm | trq, trq_ref | Normalized torque |
| rpm_norm | rpm, rpm_ref | Normalized RPM |
| assist_loss | rpm_norm | Power steering assist loss |

**Port Auto-Configuration** ([GraphEditorControl.xaml.cs:2011-2050](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L2011-L2050)):
```csharp
private void EnsureFuncPorts(GraphNode node)
{
    string func = (node.Func ?? "").Trim().ToLowerInvariant();
    string[] desiredInputs = GetFuncInputNames(func);

    EnsurePort(node, GraphPortKind.Output, "out");

    // Remove excess ports, add missing ports, rename to match signature
    // ...
}
```

**Key Behavior**:
- Changing function **automatically updates ports** via EnsureFuncPorts()
- Old ports removed, new ports added
- Links to old ports are broken if signature changes

**Inspector Panel** ([GraphEditorControl.xaml.cs:1943, 1953](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1943)):
- Title field
- EditFunc ComboBox (function selector)
- Port list (read-only, auto-configured)

**Runtime Conversion** ([GraphRuntimeConverter.cs:37-46](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L37-L46)):
- Func name directly copied
- Input ports converted to Args list

**Evaluation** ([GraphEvaluator.cs:168-202](SimHubPlugin/GraphTest/GraphEvaluator.cs#L168-L202)):
```csharp
private double EvalFunc(GraphNode node)
{
    switch (node.Func)
    {
        case "qhat_eff":
            double iasKts = Resolve(node.Args[0]);
            double vref = Resolve(node.Args[1]) ?? 60.0;
            double iasMps = iasKts * 0.514444;
            double vrefMps = vref * 0.514444;
            return vrefMps <= 0 ? 0 : (iasMps * iasMps) / (vrefMps * vrefMps);
        // ...
    }
}
```

### Proposed Improvements

#### 1. Function Documentation
Show function description, formula, and input descriptions.

**Inspector panel**:
```
┌─────────────────────────────────┐
│ Function: qhat_eff              │
├─────────────────────────────────┤
│ Description:                    │
│ Dynamic pressure ratio for      │
│ aerodynamic force scaling.      │
│                                 │
│ Formula: (IAS/Vref)²            │
│                                 │
│ Inputs:                         │
│ • ias_kts: Indicated airspeed   │
│ • vref_kts: Reference speed     │
└─────────────────────────────────┘
```

**Status**: [ ] Not implemented

#### 2. Searchable Function Picker
For larger function libraries.

**Status**: [ ] Not implemented

#### 3. Custom Functions
User-defined functions via scripting or expression.

**Status**: [ ] Not implemented - use Include for custom logic

### Edge Cases

1. **Unknown function**: Evaluates to 0.0
2. **Missing inputs**: Uses 0.0 or function-specific default
3. **Function change**: Ports auto-update, links may break

---

## Include Node

### Purpose
References another graph file, enabling modular graph composition. The included graph is evaluated as a subgraph with mapped inputs/outputs.

### Current Implementation

**Creation** ([GraphEditorControl.xaml.cs:1182-1186](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L1182-L1186)):
```csharp
else if (kind == GraphNodeKind.Include)
{
    node.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
    node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
}
```

**Visual Layout**:
```
┌─────────────────────────┐
│   damper_curve.json     │  ← Magenta title bar
├─────────────────────────┤
│ in ○               ○ out│  ← Manually configured ports
│ velocity ○      ○ force │
└─────────────────────────┘
```

**Properties**:
```csharp
public string IncludePath { get; set; }  // Path to included graph
public List<GraphPort> Ports { get; }    // Input/Output port mappings
```

**Inspector Panel** ([GraphEditorControl.xaml.cs:2988-3063](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L2988-L3063)):
- Title field
- Include path field with Browse button
- Open Include button (opens in new tab)
- **Input Ports** section:
  - Add Input button
  - Per-port: name editor + delete button
- **Output Ports** section:
  - Add Output button
  - Per-port: name editor + delete button

**Port Management**:
- Fully editable: add, remove, rename
- Input ports map to included graph's Input nodes
- Output ports map to included graph's Output nodes
- **Manual configuration required** - ports don't auto-sync with included graph

**Runtime Conversion** ([GraphRuntimeConverter.cs:65-82](SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L65-L82)):
```csharp
// Build InputMap: port name → source node ID
foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
{
    if (TryGetInputSource(..., out var source))
        runtimeNode.InputMap[port.Name] = source;
}

// Build OutputMap: port name → synthetic output ID
foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
{
    runtimeNode.OutputMap[port.Name] = $"{node.Id}:{port.Name}";
}
```

**Evaluation** ([GraphEvaluator.cs:204-263](SimHubPlugin/GraphTest/GraphEvaluator.cs#L204-L263)):
```csharp
private double EvalInclude(GraphNode node, ...)
{
    var subGraph = _resolver.GetGraph(node.Path);

    // Map inputs
    var subInputs = new Dictionary<string, double>();
    foreach (var mapping in node.InputMap)
        subInputs[mapping.Key] = Resolve(mapping.Value);

    // Evaluate subgraph
    var subOutputs = new GraphEvaluator(subGraph, _resolver)
        .Evaluate(subInputs, parameters);

    // Store outputs as synthetic nodes
    foreach (var mapping in node.OutputMap)
        if (subOutputs.TryGetValue(mapping.Key, out var value))
            _values[mapping.Value] = value;
}
```

### Current Limitations

1. **Manual port configuration**: Must manually add ports matching included graph's interface
2. **No parameter exposure**: Included graph's parameters not accessible on Include node
3. **Silent mismatches**: Wrong port names cause silent failures (0.0 values)
4. **No interface visibility**: Must open included graph to see its inputs/outputs

### Proposed Improvements

#### 1. Auto-Surface Ports

Automatically populate Include node ports from included graph's Input/Output nodes.

**Interface Extraction**:
```csharp
public sealed class IncludedGraphInterface
{
    public List<IncludedPort> Inputs { get; } = new();   // From Input nodes
    public List<IncludedPort> Outputs { get; } = new();  // From Output nodes
    public List<IncludedParam> Params { get; } = new();  // From Param definitions
    public string GraphHash { get; set; }                // For change detection
}
```

**Extraction Logic**:
```csharp
public static IncludedGraphInterface ExtractInterface(GraphDefinition graph)
{
    var result = new IncludedGraphInterface();

    // Input nodes → input ports
    foreach (var node in graph.Nodes.Where(n => n.Kind == GraphNodeKind.Input))
        result.Inputs.Add(new IncludedPort { Name = node.Title });

    // Output nodes → output ports
    foreach (var node in graph.Nodes.Where(n => n.Kind == GraphNodeKind.Output))
        result.Outputs.Add(new IncludedPort { Name = node.Title });

    // Param definitions → param controls
    foreach (var param in graph.Params.Values)
        result.Params.Add(new IncludedParam { Name = param.Name, Definition = param });

    return result;
}
```

**Port Sync**:
```csharp
public void SyncIncludePorts(GraphNode includeNode, IncludedGraphInterface iface)
{
    var existingLinks = GetLinksForNode(includeNode.Id);
    includeNode.Ports.Clear();

    foreach (var input in iface.Inputs)
        includeNode.Ports.Add(new GraphPort { Name = input.Name, Kind = GraphPortKind.Input });

    foreach (var output in iface.Outputs)
        includeNode.Ports.Add(new GraphPort { Name = output.Name, Kind = GraphPortKind.Output });

    RestoreCompatibleLinks(includeNode, existingLinks);
}
```

**Status**: [ ] Not implemented

#### 2. Surface Parameter Controls

Display included graph's parameters as editable controls on the Include node.

**Visual**:
```
┌───────────────────────────────────────┐
│          damper_curve.json            │
├───────────────────────────────────────┤
│ velocity ○                   ○ output │  ← Auto-populated ports
│ position ○                   ○ debug  │
├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┤
│ damping     [========■==] 0.85        │  ← Parameters from included graph
│ stiffness   [====■======] 200  ↺      │  ← Overridden (bold + reset)
│ mode        [Linear      ▼]           │
└───────────────────────────────────────┘
```

**Parameter Override Storage**:
```csharp
public sealed class GraphNode
{
    // ... existing ...
    public Dictionary<string, double> IncludeParamOverrides { get; } = new();
}
```

**Override Behavior**:
- If param in IncludeParamOverrides → use override value
- Else → use included graph's default
- Reset button removes override

**Runtime Evaluation Update**:
```csharp
private double EvaluateIncludeGraph(GraphNode node, GraphDefinition subGraph, ...)
{
    var subParams = new Dictionary<string, double>();

    // Start with defaults
    foreach (var param in subGraph.Params)
        subParams[param.Key] = param.Value.DefaultValue;

    // Apply overrides
    foreach (var kvp in node.ParamOverrides)
        if (subParams.ContainsKey(kvp.Key))
            subParams[kvp.Key] = kvp.Value;

    // Evaluate with merged params
    return new GraphEvaluator(subGraph, _resolver).Evaluate(subInputs, subParams);
}
```

**Status**: [ ] Not implemented

#### 3. Interface Caching

Cache extracted interfaces for performance:

```csharp
public sealed class IncludeInterfaceCache
{
    private readonly Dictionary<string, (string Hash, IncludedGraphInterface Interface)> _cache;

    public IncludedGraphInterface GetInterface(string path, GraphDefinition graph)
    {
        string hash = ComputeGraphHash(graph);
        if (_cache.TryGetValue(path, out var cached) && cached.Hash == hash)
            return cached.Interface;

        var iface = ExtractInterface(graph);
        _cache[path] = (hash, iface);
        return iface;
    }
}
```

**Status**: [ ] Not implemented

#### 4. Change Detection Triggers

Re-sync ports when:
- Include path changed
- Included graph saved (in another tab)
- Manual refresh button clicked

**Status**: [ ] Not implemented

### Edge Cases

1. **Circular includes**: Detect and show error
2. **Missing file**: Show error state, preserve last-known ports
3. **Port name mismatch**: Broken links warning on sync
4. **No inputs/outputs**: Node renders with just parameters
5. **Many parameters (10+)**: Collapse by default

---

## Cross-Cutting Concerns

### Node Sizing

Node size calculated based on content ([GraphEditorControl.xaml.cs:2600-2655](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L2600-L2655)):

```csharp
private double ComputeNodeWidth(GraphNode node) { ... }
private double ComputeNodeHeight(GraphNode node)
{
    int inputs = node.Ports.Count(p => p.Kind == GraphPortKind.Input);
    int outputs = node.Ports.Count(p => p.Kind == GraphPortKind.Output);
    int portRows = Math.Max(inputs, outputs);
    return 42 + portRows * 25;  // Title bar + ports
}
```

**Proposed**: Add parameter row height for Include nodes with surfaced params.

### Preview Values

Output ports show live values during preview ([GraphEditorControl.xaml.cs:409-425](SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L409-L425)):

```csharp
if (port.Kind == GraphPortKind.Output)
{
    var valueLabel = new TextBlock
    {
        Text = "[--]",
        Foreground = Brushes.LimeGreen,
        // ...
    };
}
```

### Error States

Proposed visual indication for errors:
- Missing include file: Red title bar
- Invalid function: Warning icon
- Circular reference: Error overlay

**Status**: [ ] Not implemented

---

## Implementation Phases

### Phase 1: Include Node Enhancement (Priority)
- [ ] Create `IncludedGraphInterface` class
- [ ] Implement `ExtractInterface()` method
- [ ] Add `IncludeInterfaceCache` to `GraphEditorTabManager`
- [ ] Implement `SyncIncludePorts()` with link preservation
- [ ] Add `IncludeParamOverrides` to `GraphNode`
- [ ] Update serializer for new properties
- [ ] Build parameter controls on Include nodes
- [ ] Add override indication (bold + reset)
- [ ] Update runtime evaluator for param overrides

### Phase 2: Op Node Improvements
- [ ] Auto-configure ports for clamp/lerp (3 inputs)
- [ ] Consider variadic inputs for add/mul/min/max

### Phase 3: Input/Output/Param Signal Selection

- [x] Add `SignalGroup` to GraphNode, `SignalSuffix` to GraphPort
- [x] Update `GraphSignalCatalog` with group accessors (`InputGroups`, `OutputGroups`, `ParamGroups`)
- [x] Build group dropdown UI for Input/Output/Param nodes
- [x] Implement signal suffix dropdown within selected group
- [x] Add validation: block unknown input/output signals, block output names in Param
- [ ] Add `InputDefaultValue` property
- [ ] Add metadata fields (units, description)

### Phase 4: Const/Func Polish
- [ ] Named constants
- [ ] Inline const editing
- [ ] Function documentation display

### Phase 5: Error States & Polish
- [ ] Visual error indication
- [ ] Circular include detection
- [ ] Missing file handling

---

## Open Questions

1. **Should parameter groups be shown on Include nodes?**
   If included graph uses `Ui.Group`, should Include node show grouped params?

2. **Should there be a "compact" mode for Include nodes?**
   Option to hide parameters on node, edit only in inspector?

3. **Should we support partial port exposure?**
   Allow user to hide specific auto-detected ports?

4. **Op node port auto-adjustment?**
   Should changing operation auto-adjust port count (e.g., mul→clamp adds third input)?

5. **Function versioning?**
   How to handle function signature changes across versions?

---

## Pending Tweaks

Add ideas here in any format. They'll be integrated into the appropriate sections above.

```
Example format:
- [NodeType] Brief description of tweak
- [Include] Add "Refresh" button to re-sync ports from included graph
- [Op] Auto-adjust ports when switching to clamp/lerp
```

### Tweaks to Integrate

<!-- Add your tweaks below this line -->

(None pending)

### Integrated Tweaks

- ~~[Input, Output]: Group dropdown replacing freeform title~~ → Integrated into Input §1, Output §1
- ~~[Param]: Freeform group dropdown with output name prohibition~~ → Integrated into Param §1

