# Per-Profile MSFS Variable Binding Overrides

## Problem Statement

MSFS SimConnect variable bindings live **only inside the graph node** today:

- **Read** (`MsfsVarDef`, Plan 23): each output port carries `SignalSuffix` (alias), `SimVar` (raw datum, e.g. `L:HELI_COLL_TRIM_TGT`), `Unit`. Scanned by `UpdateMsfsCustomVars` (`DiyFfbPlugin.cs:1230-1256`).
- **Write** (`MsfsVarOut`, Plan 24): each input port carries `SimVar` (target with `A:`/`L:`/`B:`/`K:` prefix), `Unit`, and a linear range map `InMin/InMax → OutMin/OutMax`. Scanned by `UpdateMsfsWritableVars` (`DiyFfbPlugin.cs:1264-1300`); dispatched by `CheckMsfsVarOutChanges` (`1309-1364`). The write alias key is **`port.Name`** (not `SignalSuffix`).

Port fields are on `GraphPort` (`GraphModel.cs`: `SignalSuffix:263`, `SimVar:286-291`, `Unit:293-298`, range `300-309`).

Because these bindings are graph-scoped, **two aircraft cannot share one FFB graph if their LVAR names or scaling differ.** The user has a concrete case where the graph *logic* is identical across aircraft but the MSFS variables are not.

**Precedent that this design copies:** the tiered-config system already does **Baseline → Profile → User** overrides for function configs (`ConfigLayer`, `TieredConfigTypes.cs:35-40`; `FunctionConfigOverrides`, `60-213`; `OverrideFieldRegistry`), and `GraphParamValues` already vary per profile for a shared graph (`ResolveParamValue`, 3-tier, `DiyFfbPlugin.cs:2922-2942`). `ConfigIn`/`ConfigOut` already bind graph ports to per-profile config values scoped by function. We add the analogous layer for MSFS-var bindings, **keyed by alias** (bindings are not function-scoped, so they don't fit the function-ID-keyed `FunctionOverrides`).

## Decision (from design review)

- Override the **full binding**: raw `SimVar` name **+** `Unit` **+** range map (`In/Out`).
- Edit in a **dedicated per-vehicle panel** in the plugin UI (parallel to per-vehicle tuning params).
- Resolution is two layers: **Baseline = graph node value**, **Profile = override** (first non-null wins). No separate User layer for now.

## Revision — range map & read normalization (implementation feedback)

The initial cut applied a range map to **writes only** (`MsfsVarOut`) and let the
per-vehicle panel override all four range fields. Review changed this:

- **Reads (`MsfsVarDef`) now carry a range map too.** The raw SimConnect value is
  normalized `In[min..max] → Out[min..max]` (clamped) before entering the graph as
  `MSFS.<alias>`. This lets two aircraft feed differently-scaled raw vars into one
  shared graph. Applied in `GraphSignalCatalog.BuildMsfsInputs` via
  `plugin.NormalizeMsfsReadValue(alias, raw)`; the per-alias map is published
  atomically by `UpdateMsfsCustomVars` (`_msfsVarInMaps`).
- **Identity default = passthrough.** A `0..1 → 0..1` map is treated as a no-op on
  reads (`MsfsBindingResolver.IsIdentityRange`) so existing read ports carrying raw
  units (RPM, altitude, …) are **never** clamped into `0..1`. Normalization only
  engages once a non-identity range is configured. (Writes always apply the map, as
  before — graph output is expected in `0..1`.)
- **Sim side is overridable; graph side is intrinsic.** The range map's *sim side* is
  per-aircraft (editable in the panel); the *graph side* is graph-authored (inherited,
  shown read-only). Because reads and writes put the sim value on opposite ends of the
  map, the editable pair differs by direction:
  - **Read:** sim value is the map **input** → `In*` (Raw) is overridable; `Out*` (Norm) is fixed.
  - **Write:** sim value is the map **output** → `Out*` (Sim) is overridable; `In*` (Graph) is fixed.
  So `MsfsVarBindingOverride.In*`/`Out*` are **not** both write-only (see below) —
  reads override `In*`, writes override `Out*`. `SetMsfsReadBinding` writes only `In*`;
  `SetMsfsWriteBinding` writes only `Out*`; the untouched pair stays null → inherits.
- **Graph editor:** the `MsfsVarDef` node inspector gained the same `In/Out` range-map
  grid the `MsfsVarOut` inspector already had, so graph authors set the read defaults
  (raw range + normalized target) per port.
- The linear-map math (`MapLinearClamped`, `IsIdentityRange`, `NormalizeRead`) lives in
  `MsfsBindingResolver` so it is unit-testable without SimConnect and shared by both
  the read and write paths.

## Design

### Data model (`DiyFfbPluginSettings.cs`)

Add a small override type and two dictionaries on `AircraftFfbProfile` (keep read and write separate so a read alias and a write alias that share a string never collide):

```csharp
public sealed class MsfsVarBindingOverride
{
    public string SimVar;          // null = inherit graph node
    public string Unit;            // null = inherit
    public double? InMin, InMax;   // read overrides these (raw side); null = inherit
    public double? OutMin, OutMax; // write overrides these (sim side); null = inherit
    public bool IsEmpty => SimVar == null && Unit == null
        && !InMin.HasValue && !InMax.HasValue && !OutMin.HasValue && !OutMax.HasValue;
}

// on AircraftFfbProfile:
public Dictionary<string, MsfsVarBindingOverride> MsfsReadVarOverrides;   // keyed by alias (SignalSuffix)
public Dictionary<string, MsfsVarBindingOverride> MsfsWriteVarOverrides;  // keyed by alias (port.Name)
```

Both default to empty/`null` → fully backward compatible; existing profiles deserialize unchanged. Persisted automatically as part of `AircraftFfbProfiles` in `GeneralSettings`, and carried by `ExportedProfile` since it embeds the profile.

### Resolution at scan time (overlay, not merge into the graph)

The graph object stays untouched — overlay happens where the registration lists are built.

- **`UpdateMsfsCustomVars`** (`DiyFfbPlugin.cs:1230-1256`): for each `MsfsVarDef` output port, look up `MsfsReadVarOverrides[alias]` and compute
  - `name = ov?.SimVar ?? port.SimVar`
  - `units = ov?.Unit ?? port.Unit`
  - `alias = port.SignalSuffix` (never overridden — it is the graph-facing identity)
  before building `MsfsCustomVar(alias, name, units)`.
- **`UpdateMsfsWritableVars`** (`DiyFfbPlugin.cs:1264-1300`): for each `MsfsVarOut` input port keyed by `port.Name`, look up `MsfsWriteVarOverrides[alias]` and compute effective `SimVar`, `Unit`, and range (`In/Out`) with the same `?? port.X` fallback before building the write list and the `alias→MsfsVarOutMap`.

Both scans already read exclusively from `activeVehicleGraph.Nodes` and already re-run on graph/vehicle change, so the overlay slots in cleanly.

### Re-applying after an edit

Add a public `ApplyMsfsVarOverrides()` that re-runs both scans and bumps `WriteGeneration` (so `A:`/`L:` state is force-re-pushed; actuations `B:`/`K:` are already exempt in `CheckMsfsVarOutChanges`). The per-vehicle panel calls it after any edit. Persist via the existing debounced settings saver.

### Enumerating bindings for the UI

Add a helper that walks `activeVehicleGraph.Nodes` and returns, per direction, the list of `(alias, defaultSimVar, defaultUnit, defaultRange)` from `MsfsVarDef`/`MsfsVarOut` ports. The panel uses the defaults as placeholder/greyed baseline text; the override dict holds only user-entered deltas.

## UI — dedicated per-vehicle panel

A new **"MSFS Bindings"** section in the plugin UI, built the same way per-vehicle tuning params are (see `BuildGraphParams`, called from `DiyFfbPlugin.cs:3141`, and the tuning UI in `DiyFfbPluginUI`):

- One row per alias, grouped **Read** / **Write**.
- Read row: `alias` (label), `SimVar` (text), `Unit` (text). Placeholder shows the graph default; empty field = inherit.
- Write row: adds `InMin/InMax/OutMin/OutMax` (numeric).
- Per-row **Reset** clears that alias's override (removes the dict entry).
- Editing writes into `MsfsReadVarOverrides` / `MsfsWriteVarOverrides` for the active profile, then calls `ApplyMsfsVarOverrides()`.
- Visible only when the active graph actually declares MSFS-var nodes.

## Files touched

| File | Change |
|------|--------|
| `DiyFfbPluginSettings.cs` | `MsfsVarBindingOverride` type; `MsfsReadVarOverrides`/`MsfsWriteVarOverrides` on `AircraftFfbProfile` |
| `DiyFfbPlugin.cs` | Overlay in `UpdateMsfsCustomVars` / `UpdateMsfsWritableVars`; `ApplyMsfsVarOverrides()`; binding-enumeration helper |
| `DiyFfbPluginUI.xaml(.cs)` | New per-vehicle "MSFS Bindings" panel bound to the override dicts |

## Edge cases

- **Alias renamed / removed in the graph** → orphan override entries. Ignored at scan time (like orphan `GraphParamValues`). Optional later: a cleanup/review pass akin to the param-review dialog.
- **Read vs write alias name clash** — avoided by keeping two dicts.
- **Prefix change on a write override** (e.g. `L:` → `B:`) changes the transport; `ConfigureWritables` already routes by prefix, and the `WriteGeneration` bump on re-apply re-registers. Ensure `ApplyMsfsVarOverrides()` triggers a full re-configure, not just a value push.
- **Unit override with an incompatible unit** — SimConnect may reject; surface via the existing MSFS var rejection markers (`ValidateMsfsVarEntries`) if feasible.

## Testing

- Unit: overlay resolution — baseline-only, override-name-only, full override; empty override == baseline (extend `GraphTest`). Since the overlay is a small pure function, factor it out (e.g. `ResolveMsfsBinding(port, override)`) so it is unit-testable without SimConnect.
- Unit: `IsEmpty` and dict prune-on-reset.
- Manual (MSFS 2024, per Plan 19/24 validation): one shared graph across two aircraft with different LVARs; confirm each profile drives its own SimVar and scaling for both read and write, and that switching aircraft re-registers correctly.

## Out of scope

- User-layer (personal) MSFS-var overrides — only Baseline/Profile here.
- Overriding the alias itself (it is the wiring identity).
- Bundling graph files into exported profiles (tracked with Plan 33's export limitation).
