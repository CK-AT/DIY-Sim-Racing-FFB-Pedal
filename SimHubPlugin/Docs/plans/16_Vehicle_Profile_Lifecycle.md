# Vehicle Profile Lifecycle

Date: 2026-01-30
Status: Reference Document

## Overview

This document describes the complete lifecycle of vehicle profiles in the SimHub FFB plugin: what they store, where they're persisted, and when they're loaded/saved.

## Data Structures

### DiyFfbPluginSettings (Root Container)

The main settings class, serialized to JSON by SimHub.

**Profile-related fields:**

| Field | Type | Purpose |
| ----- | ---- | ------- |
| AircraftFfbProfiles | Dictionary<string, AircraftFfbProfile> | Per-vehicle profiles, keyed by CarId |
| VehicleGraphPaths | Dictionary<string, string> | Per-vehicle graph paths (gameId:carId → path) |
| GameGraphPaths | Dictionary<string, string> | Per-game fallback graph paths (gameId → path) |

### AircraftFfbProfile

Per-vehicle FFB configuration.

```csharp
public class AircraftFfbProfile
{
    public FunctionFfbSettings FlightStickPitch;
    public FunctionFfbSettings FlightStickRoll;
    public FunctionFfbSettings FlightStickCollective;
    public FunctionFfbSettings FlightPedals;
    public int XPlaneRotorIndex = -1;  // -1 = auto
    public Dictionary<string, double> GraphParamValues;  // Param overrides
}
```

**Notes:**

- FunctionFfbSettings currently a stub (reserved for future per-control settings)
- GraphParamValues holds user-customized graph parameter overrides (Tier 3)

### Vehicle Identification

- **CarId**: Primary key from game telemetry (e.g., `"A320_NEO"`, `"FBW_A32NX"`)
- **Vehicle Graph Key**: `"{gameId}:{carId}"` used for graph path lookups
- **Profile Key**: Plain CarId (profiles are keyed by vehicle, not game+vehicle)

## Storage Locations

### Primary Settings (SimHub Managed)

- **Method**: `SaveCommonSettings("GeneralSettings", Settings)`
- **Location**: SimHub plugin data folder (typically `%APPDATA%/SimHub/PluginData/`)
- **Format**: JSON via Newtonsoft.Json
- **Contains**: All AircraftFfbProfiles, graph paths, device settings, etc.

### Graph Files

- **Directory**: `{AppDomain.CurrentDomain.BaseDirectory}/graphs/`
- **Naming**: `{sanitized_vehicle_graph_key}.json`
- **Content**: GraphDefinition with nodes, connections, and graph-level ParamValues (Tier 2)

## Parameter Override Resolution

Three-tier system, highest priority wins:

| Tier | Source | Scope | Storage |
| ---- | ------ | ----- | ------- |
| 1 | ParamDef node default | All users of param | Graph definition |
| 2 | graph.ParamValues | All vehicles using graph | Graph JSON file |
| 3 | profile.GraphParamValues | Single vehicle | AircraftFfbProfiles |

Resolution: `ResolveParamValue(name, default)` checks Tier 3 → 2 → 1.

## Lifecycle Events

### Startup (Init)

```
1. ReadCommonSettings("GeneralSettings")
   └─ Load all AircraftFfbProfiles from SimHub data store
```

### Game Change (HandleGameChange)

```
1. Store activeGameId
2. ResolveActiveGraph(gameId, activeCarId)
   └─ Re-resolve graph path for current vehicle
```

### Aircraft Change (HandleAircraftChange)

```
1. Check HasUnsavedProfileChanges() for previous aircraft
   └─ Prompt save if dirty

2. Apply pending FFB profile if hasPendingFfbProfile flag set

3. ApplyAircraftProfile(carId)
   └─ Load from AircraftFfbProfiles[carId] or use defaults

4. ResolveActiveGraph(gameId, carId)
   └─ Resolve path: vehicle-specific → game fallback → prompt user
   └─ Load graph file, compile evaluator
```

### Graph Param Edit (UI)

```
1. Store value in profile.GraphParamValues[paramName]
2. Set hasDirtyGraphParams = true
3. Queued for save on aircraft change or shutdown
```

### Shutdown (End)

```
1. Check HasUnsavedProfileChanges(activeCarId)
   ├─ Compare BuildCurrentAircraftProfile() vs stored
   ├─ Check hasDirtyGraphParams flag
   └─ If dirty: prompt user to save
      ├─ Yes: SaveCurrentAircraftProfile(activeCarId)
      └─ No: discard changes

2. SaveCommonSettings("GeneralSettings", Settings)
   └─ Persist entire settings dict to disk

3. Cleanup: close serial ports, stop UDP receiver
```

## Change Detection

**HasUnsavedProfileChanges(carId):**

- Builds current profile via `BuildCurrentAircraftProfile()`
- Compares against stored `AircraftFfbProfiles[carId]`
- Also checks `hasDirtyGraphParams` flag

**AreProfilesEqual():**

- Compares all four FunctionFfbSettings
- Compares XPlaneRotorIndex
- Compares GraphParamValues dictionaries

## Lifecycle Diagram

```
┌──────────────────────────────────────────────────────────────┐
│ STARTUP                                                      │
│ ────────                                                     │
│ ReadCommonSettings() → Load all AircraftFfbProfiles          │
└──────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────┐
│ RUNNING (DataUpdate loop)                                    │
│ ───────                                                      │
│ Game change? → ResolveActiveGraph()                          │
│ Aircraft change? → Save previous if dirty                    │
│                 → ApplyAircraftProfile(carId)                │
│                 → ResolveActiveGraph(gameId, carId)          │
│ Param edit? → Store in profile.GraphParamValues              │
│            → Mark dirty                                      │
└──────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────┐
│ SHUTDOWN                                                     │
│ ────────                                                     │
│ HasUnsavedProfileChanges()? → Prompt save                    │
│ SaveCommonSettings() → Persist all profiles                  │
│ Cleanup → Close connections                                  │
└──────────────────────────────────────────────────────────────┘
```

## Key Code Locations

| Function | File | Line | Purpose |
| -------- | ---- | ---- | ------- |
| Init | DiyFfbPlugin.cs | ~2762 | Load settings on startup |
| End | DiyFfbPlugin.cs | ~786 | Save on shutdown |
| HandleAircraftChange | DiyFfbPlugin.cs | ~1849 | Profile apply/save on vehicle change |
| ApplyAircraftProfile | DiyFfbPlugin.cs | ~1975 | Load profile into runtime |
| SaveCurrentAircraftProfile | DiyFfbPlugin.cs | ~1959 | Persist profile to settings |
| ResolveActiveGraph | DiyFfbPlugin.cs | ~1610 | Graph resolution & loading |

## Related Documents

- [15_Graph_Param_Override_Migration_Plan.md](15_Graph_Param_Override_Migration_Plan.md) — Handling graph changes while preserving overrides
