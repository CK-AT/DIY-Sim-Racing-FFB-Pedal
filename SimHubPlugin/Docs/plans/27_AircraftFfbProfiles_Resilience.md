# AircraftFfbProfiles Data Loss Protection

## Problem

`AircraftFfbProfiles` in the settings JSON was found empty (`{}`). Likely cause: JSON.NET silently failed to deserialize `FunctionConfigOverrides` entries containing protobuf types with `RepeatedField<T>` collections, then the empty dict was persisted on shutdown, permanently destroying all profiles.

Three issues to fix:

## Fix 1: Protobuf types break JSON.NET round-trip

**File:** `TieredConfig/TieredConfigTypes.cs`

`FunctionConfigOverrides` has 3 properties that are raw protobuf types with `RepeatedField<T>`:
- `SplineForceCurveConfig ForceCurve` (line 48) — `RepeatedField<uint>`, `RepeatedField<float>`
- `ShifterConfig ShifterConfig` (line 65) — `RepeatedField<ShifterGateSegment>`, `RepeatedField<ShifterDetent>`
- `ShifterDetectConfig ShifterDetectConfig` (line 66) — `RepeatedField<ShifterGearSlot>`

**Fix:** Add `[JsonIgnore]` to each and add a companion `*Json` string property using `Google.Protobuf.JsonFormatter`/`JsonParser`. Same pattern as `FunctionBaselines` (DiyFfbPlugin.cs:2416-2419). Setter wraps parse in try/catch so bad data nulls silently.

**Backward compat:** Old JSON with `"ForceCurve": { ... }` (object) is silently skipped by JSON.NET (property is `[JsonIgnore]`). Graph-related fields (`GraphPath`, `GraphParamValues`, etc.) are simple types — always unaffected.

**No changes needed** in ConfigMerger, OverrideFieldRegistry, ShifterConfigControl — they use the runtime properties.

## Fix 2: `BuildCurrentAircraftProfile` drops fields on save

**File:** `DiyFfbPlugin.cs` (lines 3069-3086)

Currently copies only `XPlaneRotorIndex`, `GraphPath`, `GraphParamValues`. Drops:
- `FunctionOverrides`
- `ActiveFunctionIds`
- `LastReviewedGraphHash`
- `LastReviewedParamSnapshots`

**Fix:** Copy all fields (shallow copy of collections).

## Fix 3: Backup/restore safety net

**File:** `DiyFfbPlugin.cs`

New `BackupOrRestoreAircraftProfiles()` method:
- If profiles non-empty → `SaveCommonSettings("AircraftProfilesBackup", ...)` (creates `DiyFfbPlugin.AircraftProfilesBackup.json`)
- If profiles empty + backup exists → restore from backup + log warning

Call in `Init()` between `MigrateVehicleGraphPaths()` (line 3911) and `InitializeManagerFromSettings()` (line 3914).

Also update `ReplaceAircraftFfbProfiles()` (line 3822) to update the backup when profiles are intentionally replaced.

## Follow-up (out of scope)

`AxisParameterOverrides` (same file, line 144) has the same protobuf-in-JSON.NET risk (`KinematicParameters`, `StaticBalanceConfig`). Same fix pattern applies but stored in `FunctionAxisOverrides`, not inside AircraftFfbProfiles.
