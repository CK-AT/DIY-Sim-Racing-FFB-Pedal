# SimHub DIY FFB Plugin Design

Status: Living design document for the DIY Force Feedback Plugin for SimHub.

## Overview

The DIY FFB Plugin is a SimHub plugin that provides advanced force feedback control for custom DIY hardware (pedals, sticks, etc.). The plugin supports multiple games and provides flexible, graph-based FFB tuning without requiring code changes.

## Goals

- Support custom DIY FFB hardware via USB HID or serial communication
- Provide game-agnostic FFB profiles with per-vehicle tuning
- Enable flexible FFB tuning through node-based graphs
- Support both flight sim and racing sim use cases
- Maintain deterministic, real-time FFB evaluation (120Hz+ update rates)
- Allow hardware configuration without recompilation

## Architecture

### Core Components

1. **DiyFfbPlugin** - Main plugin class, SimHub integration point
   - Game telemetry ingestion
   - Device discovery and communication
   - Profile management and persistence
   - FFB frame generation and transmission

2. **FFB Graph System** - Node-based FFB pipeline (see [FFB_Graph_Design.md](FFB_Graph_Design.md))
   - Graph editor UI
   - Runtime evaluator (compiled + interpreted modes)
   - Parameter resolution (three-tier: include → graph → vehicle)
   - Signal catalog and type system

3. **Kinematics System** - Physics solver for linkage-based hardware
   - General kinematics solver (pins, bars, constraints)
   - Force factor calculations
   - Position/velocity mapping

4. **Configuration UI** - WPF-based control panels
   - System settings
   - Function-specific settings (per axis)
   - Graph parameter exposure
   - Device configuration

5. **Protocol Layer** - Device communication
   - USB HID transport
   - Serial transport
   - Protobuf message encoding
   - Config transfer and validation

## FFB Graph System

The FFB Graph System is the core innovation of this plugin. It allows users to define FFB behavior as a directed acyclic graph (DAG) of operations, making it possible to tune FFB without code changes.

See [FFB_Graph_Design.md](FFB_Graph_Design.md) for detailed design and [FFB_Graph_Progress.md](FFB_Graph_Progress.md) for implementation status.

### Key Features

- **Node Types**: Input, Param, Const, Op, Func, Include, Output
- **Compiled Evaluation**: Precomputed topological order for fast runtime execution
- **Live Preview**: Editor shows live values during simulation
- **Hierarchical Parameters**: Three-tier resolution (include defaults → graph overrides → vehicle profile overrides)
- **Reusable Blocks**: Include graphs for common patterns (actuator models, filters, etc.)

### Graph Selection

Graphs are selected per vehicle using `(GameId, CarId)` with fallback to per-game default graphs:

```
Vehicle-specific:  graphs/{GameId}/{CarId}.json
Game default:      graphs/{GameId}/default.json
```

## Kinematics System

The kinematics system solves for linkage-based mechanical systems (e.g., pedals with rotating arms, triangular linkages).

### Solver Features

- General pin/bar constraint solver
- Collinearity detection and handling
- Polynomial coefficient calculation (position → force factor)
- Validation (duplicate pins, grounded contacts, zero-length bars, etc.)

### Use Cases

- **DIY Pedals**: Linkage-based pedals with load cells
- **Custom Controls**: Any mechanism with pins, bars, and constraints

## Testing Infrastructure

The plugin uses a unified test infrastructure with two test projects:

### GraphTest (FFB Graph Tests)

**Location**: `SimHubPlugin/GraphTest/`

**Scope**: All FFB graph-related tests (23 tests total)

**Runtime Tests (15 tests)**:
- Graph evaluator correctness (basic outputs, trace values)
- Compiled evaluator parity with interpreted evaluator
- JSON serialization/deserialization roundtrip
- Include graph resolution (inline, file-based, block library)
- Validation (missing outputs, unknown functions, arg counts, schema version)
- Edge cases (clamp bound order warnings, output name uniqueness)

**Editor Tests (8 tests)**:
- GraphEditor JSON roundtrip (positions, ports, links)
- Param UI schema serialization (widget types, units, log scale, options)
- Graph preview evaluation
- ParamValues serialization (graph-level overrides)
- Three-tier parameter resolution:
  - Include default
  - Graph override
  - Vehicle override
  - Full cascade

**Key Test Patterns**:
```csharp
TestRunner.RunTest("Test name", TestMethod)
TestRunner.PrintResults("Suite name", results)
```

### KinematicsTests (Physics Validation)

**Location**: `SimHubPlugin/KinematicsTests/`

**Scope**: General kinematics solver validation (18 tests)

**Test Categories**:
- **Correctness**: Centered contact position, collinear bar solutions, force coefficients
- **Validation**: Missing contacts, duplicate pins, grounded constraints, zero-length bars
- **Edge Cases**: Extra collinear pins, shared collinear bars, unknown pins
- **Finite Checks**: Coefficient arrays must not contain NaN/Infinity
- **Migration**: Legacy DIY pedal config migration

### Shared Test Infrastructure

**Location**: `SimHubPlugin/TestCommon/TestRunner.cs`

**Shared Utilities**:
- `TestRunner.RunTest()` - Executes test with exception handling
- `TestRunner.PrintResults()` - Formatted test output
- `TestResult` - Standardized result class with pass/fail/message

**Benefits**:
- Consistent test patterns across all projects
- Reduced code duplication
- Easy to add new test projects

### Running the Tests

Both test projects are console applications that can be run directly after building.

**Build and Run GraphTest** (FFB Graph Tests):

```bash
# Build using MSBuild
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "SimHubPlugin\GraphTest\GraphTest.csproj" \
  /p:Configuration=Debug \
  /v:minimal \
  /nologo

# Run the tests
./SimHubPlugin/GraphTest/bin/Debug/net48/GraphTest.exe
```

**Build and Run KinematicsTests**:

```bash
# Build using MSBuild
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "SimHubPlugin\KinematicsTests\KinematicsTests.csproj" \
  /p:Configuration=Debug \
  /v:minimal \
  /nologo

# Run the tests
./SimHubPlugin/KinematicsTests/bin/Debug/KinematicsTests.exe
```

**Expected Output Format**:

```text
FFB Graph Tests:
  [PASS] Test name 1...
  [PASS] Test name 2...
  [FAIL] Test name 3...
  22/23 tests passed.
```

**Notes**:

- Tests are self-contained - no external test runner required
- Each test project reports its own pass/fail summary
- Failed tests include error messages for debugging
- Tests can be run from any directory (they don't depend on working directory)

## Building the Plugin

### Build Requirements

- Visual Studio 2017+ or MSBuild 15.0+
- .NET Framework 4.8 SDK
- SimHub installed at `C:\Program Files (x86)\SimHub` (for plugin references)

### Building with MSBuild.exe (Recommended)

The plugin should be built using MSBuild.exe directly, which is what Visual Studio uses internally. This ensures proper XAML compilation for .NET Framework 4.8 projects.

**Location**: Use `vswhere` to find MSBuild.exe:

```bash
"/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe" \
  -latest -requires Microsoft.Component.MSBuild \
  -find "MSBuild\*\*\Bin\MSBuild.exe"
```

**Build Command** (from Git Bash):

```bash
MSYS_NO_PATHCONV=1 \
  "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" \
  "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" \
  /p:Configuration=Release \
  /v:minimal \
  /nologo
```

**CRITICAL**: The `MSYS_NO_PATHCONV=1` environment variable is required when using MSBuild.exe from Git Bash. Without it, Git Bash mangles the command line:

- Strips backslashes from Windows paths (`d:\Projects\...` → `d:Projects...`)
- Converts `/parameter` flags to Git paths (`/nologo` → `C:/Program Files/Git/nologo`)
- Causes MSB1008 "Only one project can be specified" errors

**Build Configurations**:

- **Debug**: Standard compilation, no dependency merging
- **Release**: ILRepack merges all dependencies into single `DiyFfbPlugin.dll` at `C:\Program Files (x86)\SimHub\DiyFfbPlugin.dll`

### Dependency Merging (ILRepack)

The plugin uses ILRepack to create a self-contained DLL in Release builds. All external dependencies are merged into `DiyFfbPlugin.dll` to simplify deployment.

**Currently Merged Dependencies**:

- Google.Protobuf.dll
- COBS.NET.dll
- NullFX.CRC.dll
- System.IO.Ports.dll
- System.Text.Json.dll
- System.Threading.Tasks.Dataflow.dll

**Dependencies Provided by SimHub** (NOT merged):

- Newtonsoft.Json
- MathNet.Numerics
- AvalonDock
- System.Memory, System.Buffers (via SimHub)

**IMPORTANT**: When adding new package references via NuGet, you MUST update the ILRepack merge list in the project's PostBuildEvent. See [AGENTS.md](../../AGENTS.md) rule 21:

> Ensure the SimHub plugin builds as a self-contained DLL with all dependencies merged via ILRepack (Release builds only). Any new package references must be added to the ILRepack merge list in the post-build event.

### Why Not dotnet CLI?

The `dotnet build` command has issues with .NET Framework 4.8 projects containing XAML:

- XAML resources may not compile correctly (e.g., SplineForceCurve compilation failures)
- Missing WPF build targets in dotnet CLI environment
- MSBuild.exe (used by Visual Studio) has full support for WPF/XAML in .NET Framework projects

Use MSBuild.exe directly for reliable builds.

## Configuration and Persistence

### Profile Hierarchy

1. **Plugin Settings** (`DiyFfbPluginSettings`)
   - System-level settings
   - Device configuration
   - Per-game/per-vehicle profiles

2. **Aircraft FFB Profile** (`AircraftFfbProfile`)
   - Function enable/disable
   - Function-specific gains and parameters
   - Graph parameter overrides (`GraphParamValues`)
   - Active graph path

3. **Graph Files** (JSON)
   - Graph structure (nodes, links)
   - Parameter definitions (metadata, defaults, ranges)
   - Graph-level parameter overrides (`ParamValues`)

### Save Workflow

- **In-Memory Changes**: Parameter changes update profile immediately
- **Persistence**: Explicit save required (Save Profile button)
- **Dirty Tracking**: Profile comparison detects unsaved changes
- **Graph Changes**: ActiveGraphChanged event notifies UI to refresh

## UI Components

### Main Tabs

1. **System Tab**
   - Active graph selector
   - System parameters (group = "System")
   - Device status
   - Connection controls

2. **Function Tabs** (per axis: Pitch, Roll, Yaw, Collective, Pedals, etc.)
   - Enable FFB toggle
   - FFB Parameters (group = "FlightStickPitch", etc.)
   - Function-specific settings

3. **Graph Editor Tab**
   - Canvas with pan/zoom
   - Node palette
   - Inspector panel
   - Live preview values

### Dynamic UI Generation

**GraphParamControlBuilder** generates WPF controls from param metadata:
- Slider (linear/log scale)
- Knob (rotary control)
- Checkbox (boolean)
- Enum (dropdown)
- Text (numeric input)

**Metadata**:
```json
{
  "widget": "slider",
  "label": "Spring Gain",
  "group": "FlightStickPitch",
  "units": "N",
  "min": 0.0,
  "max": 10.0,
  "step": 0.1,
  "precision": 2,
  "logScale": false
}
```

## Integration with SimHub

### Plugin Interface

The plugin implements SimHub's `IPlugin` and `IWPFSettingsV2` interfaces:

```csharp
public class DiyFfbPlugin : IPlugin, IDataPlugin, IWPFSettingsV2
{
    // Plugin lifecycle
    public void Init(PluginManager pluginManager);
    public void DataUpdate(PluginManager pluginManager, ref GameData data);
    public void End(PluginManager pluginManager);

    // Settings UI
    public Control GetWPFSettingsControl(PluginManager pluginManager);
}
```

### Game Data Flow

1. SimHub calls `DataUpdate()` with game telemetry (120Hz+)
2. Plugin extracts relevant signals (airspeed, forces, pedal positions, etc.)
3. Graph evaluator computes FFB outputs (spring, damper, friction, load)
4. FFB frame is built and transmitted to hardware
5. Hardware returns state (position, force, button presses)

### Supported Games

- X-Plane 11/12
- Microsoft Flight Simulator (MSFS 2020)
- DCS World
- Assetto Corsa / ACC
- Additional games via SimHub's game integration layer

## Data Model

### Core Types

**GraphDefinition** (Editor model):
```csharp
class GraphDefinition {
    int Version;
    List<GraphNode> Nodes;
    List<GraphLink> Links;
    Dictionary<string, GraphParam> Params;
    Dictionary<string, double> ParamValues;  // Graph-level overrides
}
```

**GraphNode** (Runtime model):
```csharp
class GraphNode {
    string Id;
    NodeType Type;
    string Name;
    OpType Op;
    string Func;
    List<string> Args;
    string Src;
    Dictionary<string, string> InputMap;   // For includes
    Dictionary<string, string> OutputMap;  // For includes
}
```

**AircraftFfbProfile**:
```csharp
class AircraftFfbProfile {
    string ActiveGraphPath;
    Dictionary<string, double> GraphParamValues;  // Vehicle-level overrides
    // ... function-specific settings ...
}
```

## Performance Considerations

### Critical Path

The FFB evaluation runs every frame (120Hz+), so performance is critical:

1. **Graph Compilation**: Precompute topological order once per graph load
2. **Cached Evaluator**: Reuse evaluator instance, avoid allocations
3. **Parameter Lookup**: Dictionary access, O(1)
4. **Node Evaluation**: Switch on node type, inline simple operations
5. **Output Collection**: Preallocated dictionary

### Optimization Strategies

- Use compiled evaluator for runtime (interpreted only for preview)
- Cache graph resolution results (includes, block library)
- Minimize allocations in hot path
- Use `Dictionary<string, double>` for fast parameter/signal lookup
- Avoid LINQ in FFB frame generation

## Open Design Questions

1. **Typed Ports**: Should we add type checking beyond numeric values?
2. **Stateful Nodes**: Allow integrators, delays, or keep pure evaluation?
3. **Per-Vehicle Graph Overrides**: How to surface graph-level edits per vehicle?
4. **Units and Validation**: Enforce unit consistency across signals?
5. **Include Versioning**: Pin include versions or use content hashing?

## Future Enhancements

### Short Term

- Graph template selector for first-seen vehicles
- Default graphs (plane_default.json, heli_default.json)
- FlightPedalsConfigControl param UI
- Graph template registry

### Medium Term

- Orthogonal edge routing
- Mini-map for large graphs
- Tooltips with mini-curves on param nodes
- Live cursors showing current value on curves
- Typed units with validation warnings

### Long Term

- Remote graph editing via web UI
- Graph profiling and performance analysis
- Conditional nodes (if/else, switch)
- Stateful nodes (integrator, delay, filter)
- Custom function blocks (user-defined functions)
- Multi-device coordination

## References

- [FFB_Graph_Design.md](FFB_Graph_Design.md) - Detailed FFB graph system design
- [FFB_Graph_Progress.md](FFB_Graph_Progress.md) - Implementation progress tracking
- [XPlane_FFB.html](XPlane_FFB.html) - Legacy X-Plane FFB documentation
