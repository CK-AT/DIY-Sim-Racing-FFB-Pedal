# Plan 23 — Custom MSFS Variables Defined in the Graph (SimVars + LVARs, typo-tolerant)

Let a graph declare **its own** MSFS SimConnect variables — arbitrary
SimVars (`A:`) and aircraft **local variables** (`L:` / LVARs) — on top of
the fixed default set, author them **in the graph editor**, and have the
in-process SimConnect client register them dynamically. A misspelled or
aircraft-absent variable must **degrade gracefully** (that one value reads
0, everything else keeps streaming) and surface a **warning**, never break
the whole telemetry stream.

**Driving use case:** a new helicopter with autopilot and trim on the
**collective and pedals**. The relevant state (AP engaged, collective/pedal
trim target, force-trim status) is almost certainly exposed only as
aircraft **LVARs**, whose names we don't know yet and will discover by
trial. The current pipeline can only carry a hardcoded, build-time set of
36 SimVars, so today this needs a C# edit + MSBuild + restart per guess —
exactly the loop that makes LVAR discovery painful.

**Prerequisites:** [Plan 19](19_msfs_pure_csharp_simconnect.md) (in-process
pure-C# SimConnect client — the transport this plan makes dynamic).
Related: [Plan 20](20_msfs_graph_synthesised_derivations.md) (moved
derivations into the graph; established "graph is where MSFS-side authoring
happens").

This plan also **removes the dead Bridge/UDP MSFS path** (§3.5) — its sender
was deleted with Plan 19, and keeping a second positional readback in lockstep
with the new dynamic one is pure liability.

**Out of scope:** *writing* SimVars/LVARs back to the sim or triggering
events (read-only here); X-Plane custom datarefs (separate transport — see
§10 for the symmetry note); the HTML graph exporter surfacing warnings
(nice-to-have, §10).

---

## 1. Goal & Strategy

One architectural pivot — **the MSFS data definition becomes dynamic and
failure-tolerant** — with four coupled outcomes:

1. **The SimConnect data definition is built from a supplied list**, not the
   hardcoded 36-entry table. Defaults stay exactly the 36 we ship; a graph's
   declared custom variables are appended.
2. **Custom variables are declared in the graph** (a new `MsfsVars` section
   in the graph JSON) and referenced by ordinary `MSFS.<alias>` Input nodes.
   Tuning/discovery becomes a graph-editor activity, not a build-restart.
3. **A bad variable is isolated, not fatal.** Each custom is validated by a
   per-var `PERIOD_ONCE` probe *before* it joins the streaming definition; a var
   whose probe raises (a bad `A:` name → `NAME_UNRECOGNIZED`) is dropped and
   recorded as a warning while every other value keeps streaming. This also
   fixes a latent fragility: today one bad datum makes
   [`HandleSimObjectData`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L270)
   reject **all** samples (`defineCount != SampleCount → return`). (A mistyped
   `L:` LVAR reads `0` rather than raising — caught by the live readout, not the
   probe; §4.4.)
4. **Warnings are visible** — in the editor (a dedicated panel + inspector
   banner, reusing the existing warning-surface pattern), in the SimHub log,
   and via a live-value readout so a wrong *unit* (which doesn't raise an
   exception, just mis-scales) is spottable.

**Strategy:** ship **phase 0 (dynamic + tolerant client, no behavioural
change)** first behind a hard parity gate — with only the 36 defaults it
must behave byte-for-byte as today. Graph declaration, editor UI, and
warning surfaces land on top in independent phases.

---

## 2. Why

* **LVAR discovery collapses from days to minutes.** The new heli's AP/trim
  variables are unknown. Each guess today = edit
  [`MsfsSimVarTable`](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs), extend the
  enum, bump `SampleCount`, add a packet field, edit `BuildMsfsInputs`,
  MSBuild, restart SimHub, reload graph, fly (the 4-file lockstep recorded in
  the `adding-msfs-signal` procedure). After: type the name into the editor,
  reload the graph. Wrong guess → a red row, not a broken session.
* **Graceful typos are a hard requirement, not a nicety.** We *will* mistype
  LVAR names and reference vars that don't exist on every aircraft. The
  current "one bad datum drops every sample" behaviour turns a typo into
  "all FFB telemetry silently dead" — the worst possible failure mode for a
  discovery workflow.
* **The downstream is already dynamic.** The graph reads signals from a
  `Dictionary<string,double>` by name
  ([`BuildMsfsInputs`](../../SimHubPlugin/GraphSignals.cs#L82-L131),
  [`BuildGraphInputs`](../../SimHubPlugin/DiyFfbPlugin.cs#L2809-L2817)). Only
  the *transport* is rigid. We're removing a build-time bottleneck, not
  re-plumbing the evaluator.
* **Per-aircraft var sets fall out of the existing per-vehicle graph/profile
  model.** Different helis declare different LVARs in their own graph; no
  global registry to maintain.
* **One transport, not two.** Deleting the dead Bridge/UDP path (§3.5) means
  the dynamic definition has a single readback to get right, not two kept in
  positional lockstep.

Cost is a one-time transport refactor with a parity gate; the gate is
cheap because phase 0 makes no behavioural change.

---

## 3. Current State (Audit)

### 3.1 The fixed positional contract (the thing we're loosening)

Four artefacts encode "exactly 36 SimVars, by position":

| Artefact | File | Role |
| --- | --- | --- |
| `Entries[36]` | [MsfsSimVarTable.cs:20-58](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs#L20-L58) | Registration list (name, unit), in order |
| `MsfsSampleIndex` (0-35) | [MsfsSimVarTable.cs:65-103](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs#L65-L103) | Self-documenting slot indices |
| `SampleCount = 36` | [MsfsSimVarTable.cs:60](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs#L60) | Buffer size + readback gate |
| `MsfsUdpPacket` (36 fields) | [DiyFfbPlugin.cs:253-296](../../SimHubPlugin/DiyFfbPlugin.cs#L253-L296) | Typed per-field snapshot |

The client registers by looping `Entries`
([`RegisterSimVars`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L196-L211)),
subscribes at `SIM_FRAME`, and reads back **exactly** `SampleCount` doubles
in order, **rejecting the frame** if the count doesn't match
([HandleSimObjectData:270](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L270)).
[`ApplyMsfsSimConnectSample`](../../SimHubPlugin/DiyFfbPlugin.cs#L1227-L1274)
maps positionally: `packet.Field = (float)s[(int)MsfsSampleIndex.X]`.

**The `_sampleBuffer` is fixed-size** (`new double[SampleCount]`,
[MsfsSimConnectClient.cs:44](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L44)).

### 3.2 The protocol already supports what we need

* [`WriteAddToDataDefinition`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L98-L111)
  takes a **`datumId`** (currently always `Unused`) and a **unit string** —
  and `L:NAME` works as a datum name since MSFS 2020 SU12 / MSFS 2024 (see
  the LVAR discussion preceding this plan).
* [`TryReadException`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L179-L190)
  returns the failing **`sendId`** and `index`. The client already assigns a
  unique `sendId` per message via `NextSendId()`
  ([MsfsSimConnectClient.cs:322-325](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L322-L325))
  and already logs exceptions with their `sendId`
  ([:251-257](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L251-L257)).
* A `RequestDataOnSimObject` with `PERIOD_ONCE` on a data definition yields
  **exactly one data reply for a valid definition, or an exception for a bad
  one.** So each custom can be validated in isolation by probing it in its own
  throwaway definition (§4.2) and reading the binary outcome — reply = valid,
  exception = invalid — attributed directly by the probe's `requestId` (no
  `sendId → entry` correlation needed). This sidesteps the version-dependent
  question of whether a bad datum poisons a *shared* definition or is merely
  dropped, and whether its exception fires at add-time or request-time.

**This is the whole mechanism for graceful typos** and it needs no new protocol
messages, only that we (a) stop hardcoding the count and (b) validate customs by
per-var `PERIOD_ONCE` probe before adding them to the shared streaming
definition. Note it reliably catches bad `A:` SimVar **names** (they raise
`NAME_UNRECOGNIZED`); a mistyped `L:` LVAR does **not** raise — SimConnect
resolves an unknown LVAR to `0` — so LVAR correctness stays a
runtime-observation concern (live readout, §4.4), not a registration-time
guarantee.

### 3.3 The graph consumes signals by name (downstream is ready)

`BuildMsfsInputs` writes `inputs["MSFS.X"] = packet.Field` into an
`IDictionary<string,double>`
([GraphSignals.cs:82-131](../../SimHubPlugin/GraphSignals.cs#L82-L131));
`BuildGraphInputs` rebuilds that dict **every frame**
([DiyFfbPlugin.cs:2809-2817](../../SimHubPlugin/DiyFfbPlugin.cs#L2809-L2817))
and the evaluator resolves Input nodes by dictionary key
([EvaluateActiveGraph:2682-2712](../../SimHubPlugin/DiyFfbPlugin.cs#L2682-L2712)).
Adding a new `inputs["MSFS.<alias>"] = value` is transparent to everything
downstream.

### 3.4 Authoring & catalog (what needs new UI)

* Signals come from a **static** catalog
  ([GraphSignalCatalogData.cs:46-121](../../SimHubPlugin/GraphSignalCatalogData.cs#L46-L121));
  the Input-node port picker is **constrained** to
  `GetInputSignalsForGroup("MSFS")`
  ([:226-234](../../SimHubPlugin/GraphSignalCatalogData.cs#L226-L234),
  bound at
  [GraphEditorControl.xaml.cs:5160-5168](../../SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L5160-L5168)).
  **Freeform entry is not allowed** in top-level graphs.
* `GraphPort` has `Name`, `SignalSuffix`, `Negate`, `ConfigField`,
  `BusName` — **no unit or datatype field**
  ([GraphModel.cs:217-249](../../SimHubPlugin/GraphEditor/GraphModel.cs#L217-L249)).
* [`GraphUsageScanner`](../../SimHubPlugin/GraphEditor/GraphUsageScanner.cs#L207-L233)
  scans **include references, not signal references** — it can't currently
  enumerate "the MSFS vars this graph needs."
* Warning-surface precedent exists: `IncludeErrorMessage` /
  `ConfigTypeMismatchMessage` dependency properties + inspector banners
  ([GraphEditorControl.xaml.cs:137-175](../../SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs#L137-L175)).
* [`validate_graphs.py`](../../SimHubPlugin/graphs/validate_graphs.py#L95-L119)
  does structural checks only — no signal-name validation.

### 3.5 Dead Bridge/UDP path — deleted as step 0

The UDP `Bridge` mode still has code but is **dead**: its C++ sender
(`MsfsFfbDataProvider.exe`) was removed with Plan 19, so `Bridge` mode
connects to nothing. It is a second, positional copy of the exact mapping
this plan is about to make dynamic — keeping it means maintaining two
readback paths in lockstep for no benefit. **Delete it as the first step of
phase 0** rather than working around it. Removal surface:

| Item | Location |
| --- | --- |
| `enum MsfsConnectionMode` + `MsfsConnectionMode` field | [DiyFfbPluginSettings.cs:12](../../SimHubPlugin/DiyFfbPluginSettings.cs#L12), [:162](../../SimHubPlugin/DiyFfbPluginSettings.cs#L162) (and `MsfsUdpEnabled` / `MsfsUdpPort`, plus any settings-UI controls) |
| UDP fields | [DiyFfbPlugin.cs:128-130](../../SimHubPlugin/DiyFfbPlugin.cs#L128-L130) (`msfsUdpClient/Thread/Cts`) |
| `StartMsfsUdpReceiver` / stop / `MsfsUdpLoop` | [DiyFfbPlugin.cs:1342-1430](../../SimHubPlugin/DiyFfbPlugin.cs#L1342-L1430) |
| `ParseMsfsPacket` | [DiyFfbPlugin.cs:1432-1504](../../SimHubPlugin/DiyFfbPlugin.cs#L1432-L1504) |
| `StartMsfsBridgeProcess` / `StopMsfsBridgeProcess` / `_msfsBridgeProcess` | [DiyFfbPlugin.cs:1282-1340](../../SimHubPlugin/DiyFfbPlugin.cs#L1282-L1340), call site [:955](../../SimHubPlugin/DiyFfbPlugin.cs#L955) |
| Connection-mode branch → collapses to `StartMsfsClient()` | [DiyFfbPlugin.cs:4597-4604](../../SimHubPlugin/DiyFfbPlugin.cs#L4597-L4604) |
| `DeployMsfsBridge` build target (copies the deleted exe) | [DiyFfbPlugin.csproj:477-485](../../SimHubPlugin/DiyFfbPlugin.csproj#L477-L485) |
| Stale comment referencing `ParseMsfsPacket` | [SimConnectProtocol.cs:194](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L194) |

After removal there is a **single** MSFS path — `MsfsSimConnectClient` →
`ApplyMsfsSimConnectSample` → `latestMsfsPacket` — which is what the rest of
this plan builds on. This removal is independent of the custom-variable
feature and could ship as its own small commit ahead of the rest.

> The `adding-msfs-signal` memory's "do NOT touch the UDP `ParseMsfsPacket`
> path" caveat is obsoleted by this deletion — update that memory when the
> removal lands.

---

## 4. Design

### 4.1 Where custom variables are declared — a new `MsfsVarDef` node

Add a node kind `MsfsVarDef` ("MSFS Vars"), authored on the canvas. Each of
its **output ports** declares one custom variable *and* emits that variable's
live value — the node is both the declaration and the wire source, like an
MSFS Input node but with the registration metadata attached per port:

| Port field | Meaning | Example |
| --- | --- | --- |
| `SignalSuffix` (alias) | graph-facing name → signal `MSFS.<alias>`; the port label you wire from | `AP.CollectiveTrim` |
| `SimVar` *(new)* | the raw SimConnect `A:`/`L:` datum name to register | `L:HELI_COLL_TRIM_TGT` |
| `Unit` *(new)* | SimConnect unit string | `number` |
| `Type` | datatype; `Float64` only for now | `Float64` |

You wire straight from a port (e.g. `AP.CollectiveTrim → include.trim_in`);
no Input node and no catalog entry are involved.

**Why a node (vs a detached panel + top-level list):**

* It lives in the graph and reads as "these are this aircraft's extra sim
  variables," each port a wire source — nothing to cross-reference.
* **Kind-specific port metadata is an established pattern:** ConfigOut/In
  ports carry `ConfigField`, LocalSend/Receive ports carry `BusName`
  ([GraphModel.cs:217-249](../../SimHubPlugin/GraphEditor/GraphModel.cs#L217-L249)).
  `MsfsVarDef` ports carry `SimVar`+`Unit` the same way — the two new fields
  are inert on every other kind, so the "pollutes the model" objection to
  per-port metadata doesn't apply when the fields are scoped to a purpose-built
  kind.
* **Warnings attach to the exact port:** a failed registration paints that
  output port red and shows its live value inline. The editor already renders
  per-output-port value labels (`outputValueLabels` / `UpdateNodeValues`) — but
  today they're fed by the *preview evaluator* with synthetic inputs, so showing
  *actual* streaming values there is new runtime→editor plumbing (§4.4), not free
  reuse. Still a better locus than a detached panel.
* One node with many ports, or several nodes, both work; the editor flags
  duplicate aliases (they'd collide in the `inputs` dictionary).

**Model changes:** add `MsfsVarDef` to `GraphNodeKind`
([GraphModel.cs:8-51](../../SimHubPlugin/GraphEditor/GraphModel.cs#L8-L51))
and two `GraphPort` fields `SimVar`, `Unit` (used only by this kind). Output
ports only. **Adding the kind is mechanical but multi-site, not one line** —
plan on touching every place that dispatches on kind: `MapNodeType` +
the `Convert` if-else chain
([GraphRuntimeConverter.cs](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs)),
V1→V2 migration and the `isSignalNode` determination in the serializer
([GraphSerializer.cs](../../SimHubPlugin/GraphEditor/GraphSerializer.cs)),
`GraphValidator`, `GetTitleBarColor`, and the palette `AddNode` (§5, phase 2).

**Two wiring details easy to miss:**

* The node must set **`SignalGroup = "MSFS"`** — `BuildFullSignalName` returns
  `group + "." + suffix` only when both are non-empty, otherwise it falls back
  to the legacy port name and the `MSFS.` prefix never appears.
* The serializer stores port aliases in `SignalSuffix` via a **manual DTO**
  (`GraphPortDto` + `ShouldSerialize*` + `FromModel`/`ToModel`), gated on an
  `isSignalNode` flag. `MsfsVarDef` must be treated as a signal node there or
  `SignalSuffix` (and the new `SimVar`/`Unit`) won't round-trip; the two new
  fields need explicit DTO plumbing — they do **not** serialize by reflection.

**Two consumers of the same node — registration vs evaluation:**

* **Registration (plugin, at load):** scan the graph for `MsfsVarDef` nodes,
  collect `(alias, SimVar, Unit)` per output port, build the SimConnect list
  (§4.2). Read from the editor-format graph the same way includes/params are
  read at load.
* **Evaluation (runtime):** the editor→runtime converter
  ([GraphRuntimeConverter.cs](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs))
  emits each `MsfsVarDef` output port as an ordinary MSFS input signal
  `MSFS.<alias>` — identical to an Input node's port. The raw name/unit are
  registration-only and never enter the runtime graph, so the value resolves
  via `inputs["MSFS.<alias>"]` with **zero evaluator changes**.

**Declaration scope:** `MsfsVarDef` nodes live in the **top-level graph only**
(§9.1); included sub-graphs consume the values by wiring from the node's
ports. Aliases must be unique across all `MsfsVarDef` nodes (editor +
`validate_graphs.py` enforce). Grouping into several nodes by use/axis is the
intended convention (§9.3).

### 4.2 Dynamic, tolerant SimConnect client

The client is handed an **ordered registration list** = the 36 defaults
(from `MsfsSimVarTable`, unchanged) **followed by** the graph's customs.
Validation is decoupled from streaming — probe each custom in isolation, then
stream only the survivors in one shared definition:

1. **Defaults → shared definition, unprobed.** Add the 36 defaults to the
   streaming definition (`DefineId`) exactly as today. They're known-good, so
   slots 0-35 stay the 36 defaults in add-order — the positional default mapping
   is preserved (helps the parity gate).
2. **Probe each custom in its own throwaway definition.** For custom *i*, add it
   alone to a scratch `defineId` and issue `RequestDataOnSimObject` with
   `PERIOD_ONCE` on a per-probe `requestId`, attributing any exception back to
   the var by `requestId`. **Drain rule — invalid ⟺ exception, *not* "no data
   reply".** A bad name raises `NAME_UNRECOGNIZED` promptly (exceptions are
   generated when SimConnect *processes* the request), but a good var's
   `PERIOD_ONCE` data is dispatched on the next `SIM_FRAME`, which does **not**
   tick while the sim is paused or in a menu — exactly when a graph load might
   fire. So treat *no exception within the timeout* as **valid** and only an
   exception as **invalid**; waiting for the data reply would false-fail every
   good var whenever the sim isn't stepping.
   * **Sequential probing (chosen).** Probe one var at a time — add → request
     `PERIOD_ONCE` → wait → clear → reuse a *single* scratch `defineId` for the
     next. This avoids scratch-definition accumulation across re-registrations
     and keeps the logic simple. (Pipelining — fire all, distinct scratch id
     per in-flight probe, clear them all after — is a possible later
     optimisation but isn't worth the id-lifecycle bookkeeping at our scale.)
   * **Id-space:** the single scratch `defineId`/`requestId` must not collide
     with the streaming `DefineId`/`RequestId` (currently both `1`) — use a
     distinct value (e.g. `1000`).
   * **Bounded per-probe wait.** Since a bad name's exception is prompt on a
     local pipe and a good var may yield *no* reply while paused (the drain rule
     above), use a *short* per-probe timeout (~50-100 ms). Worst case is then
     N × timeout of blocking during a paused-sim registration — sub-second for
     the realistic handful of customs, ~a few seconds at the 64 cap. Probing
     runs on the worker thread with the stream stopped (§4.2 re-registration),
     so it never competes with live sampling; debouncing keeps it off the hot
     path.
3. **Append survivors to the shared definition** in declared order. Because
   every survivor already passed its probe, the shared streaming definition is
   valid-by-construction — no merged-definition survivor map or `sendId`
   correlation needed (keep a lightweight `defineCount == registeredCount` check
   as cheap defense). Invalid customs go to `FailedVars` and are simply omitted.
4. **Subscribe** at `SIM_FRAME`. Each frame, `payload[i]` ↔ `registered[i]` (36
   defaults + surviving customs, in order). Deliver a result the plugin can
   consume by name (see §4.3). Buffer is sized to the registered count, not a
   constant.

**Why per-var probing (vs one merged add + a `sendId` survivor map):** it removes
the plan's biggest unknown — whether a bad datum poisons a shared definition or
is silently dropped, and whether its exception fires at add- or request-time —
none of which we then rely on. Each probe's outcome is unambiguous and the
streaming path is validated-by-construction. Cost is N one-time `PERIOD_ONCE`
round-trips on a local pipe *at registration only* (never per-frame), negligible
even at the 64-var cap.

**LVARs are the exception to "validated."** A bad `A:` name raises
`NAME_UNRECOGNIZED`, so the probe drops it. A mistyped `L:` LVAR does **not**
raise — SimConnect resolves an unknown LVAR to `0` — so it passes the probe and
streams `0`. Probing therefore guarantees the stream is never *poisoned*, but it
does **not** catch LVAR typos; that stays a runtime-observation concern via the
live readout (§4.4). Since LVAR discovery is the driving use case, don't oversell
probing as early typo-detection.

`datumId` is set per entry (`= index` in the registered list) so a future move to
**tagged** streaming (`SIMCONNECT_DATA_REQUEST_FLAG_TAGGED`, delta updates keyed
by `datumId`) is a localized change; phase 0 uses non-tagged full-frame reads.

**Re-registration — one path, on the worker thread.** Add a
`SetRegistrationList(entries)` API that stops the active stream
(`RequestDataOnSimObject` with `PERIOD_NEVER`), clears the data definition
(`SendIdClearDataDefinition` already exists,
[SimConnectProtocol.cs:50](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L50)),
and re-runs probe + registration. All pipe writes and the shared `_txBuffer` are
owned by the read-loop worker thread, so `SetRegistrationList` must **hand the
new list to the worker** (a synchronized field the loop picks up at a safe
point) — never write to the pipe from the caller's thread. It is called whenever
the **effective definition changes** — graph load, editor edit, profile switch,
vehicle change — all just causes of the same event (§9.2). **Debounced**, and
skipped when *nothing relevant changed*: the guard compares the *(registration
list + aircraft title)*, so a swap between two aircraft that happen to share one
graph still re-registers and re-evaluates per-aircraft `L:` availability, while a
no-op event (same list, same aircraft) doesn't cause a mid-flight data gap.

### 4.3 Sample delivery & plugin mapping

Keep the 36 defaults mapped exactly as today (they never fail), and add a
custom channel:

* Client delivers, per frame, the survivor values **plus** the survivor name
  list (or a small `IReadOnlyDictionary<string,double>` keyed by raw datum
  name). Defaults keep their positional read into `MsfsUdpPacket`; a missing
  default (shouldn't happen) is logged once.
* `MsfsUdpPacket` gains `Dictionary<string,double> Custom` (keyed by
  **alias**). `ApplyMsfsSimConnectSample` fills it by looking up each declared
  custom's raw `SimVar` in the survivor values; absent (failed) customs are
  omitted → they read as 0 downstream via the default `TryGetValue`.
* `BuildMsfsInputs` appends: `foreach (kv in packet.Custom) inputs["MSFS." +
  kv.Key] = kv.Value;` — leaving the 36 hardcoded lines untouched.

### 4.4 Graceful typos & warnings

| Stage | Detects | Surface |
| --- | --- | --- |
| Edit-time (syntax) | empty alias/name, duplicate alias (across all `MsfsVarDef` nodes), alias colliding with a built-in `MSFS.*`, unknown unit preset | offending output port marked in the `MsfsVarDef` node inspector; `validate_graphs.py` error |
| Edit-time (A-vars) | unknown `A:` SimVar name vs a bundled SDK name list | warning marker (yellow) on the port. LVARs (`L:`) can't be validated offline — no warning |
| Runtime (`A:` existence) | per-var `PERIOD_ONCE` probe raises `NAME_UNRECOGNIZED` (§4.2) | client `FailedVars` list → SimHub log + editor banner (reuse `IncludeError`-style dep prop) + **red output port** on the node |
| Runtime (`L:` typo) | *no exception* — unknown LVAR passes the probe and reads `0` | **live value shown inline** on the port — a stuck `0` is the only signal (same surface as wrong-unit) |
| Runtime (wrong unit) | *no exception* — value mis-scales | **live value shown inline** on the port (editor already renders output-port value labels) so 0/implausible values are visible |

The runtime→editor channel: the client exposes `IReadOnlyList<FailedVar>`
(alias, raw name, exception code) and current per-var values; the plugin
relays them so the editor can flag the specific `MsfsVarDef` output ports and
show a status banner. The "one bad var kills the stream" fragility is gone by
construction (§4.2).

### 4.5 Units & datatypes

`Float64` only (matches the existing all-FLOAT64 definition). Provide a unit
**preset list** (`number`, `bool`, `percent`, `percent over 100`, `radians`,
`degrees`, `knots`, `feet per second`, `foot pounds`, …) with freeform
override. Note in the panel help: wrong unit for an `A:` var may mis-scale
silently (hence the live readout); `number` is the safe default for `L:` vars.

---

## 5. Phases

Parity-gated, mirroring Plan 20's discipline.

* **Phase 0 — Delete the dead Bridge/UDP path, then make the client dynamic &
  tolerant (no behavioural change).** First remove the Bridge/UDP surface
  (§3.5) so there's one MSFS path. Then refactor `MsfsSimConnectClient` to
  register from a supplied ordered list (defaults = the 36), assign `datumId`s,
  validate customs by per-var `PERIOD_ONCE` probe (§4.2), stream the survivors,
  size buffers dynamically, and add `FailedVars`. Move re-registration onto the
  worker thread. **Parity gate:** with only the 36 defaults, the *delivered
  sample stream* is byte-identical to today (the registration wire bytes do
  change — `datumId` is now `index`, not `Unused` — but non-tagged reads ignore
  `datumId`, so the samples are unaffected). **Probe validation:** confirm a bad
  `A:` test var raises `NAME_UNRECOGNIZED` on its probe and is dropped while the
  36 keep streaming, and that an unknown `L:` var reads `0` rather than erroring
  (expected — §4.2). Ships the fragility fix (and the dead-code removal) before
  any graph feature exists. *The removal can also land as its own commit first —
  it has no dependency on the rest.*

* **Phase 1 — `MsfsVarDef` model + runtime plumbing (no palette UI yet).**
  Add the `MsfsVarDef` kind + `SimVar`/`Unit` port fields to the model and
  serializer; teach the editor→runtime converter to emit its ports as
  `MSFS.<alias>` inputs; plugin scans the active graph's `MsfsVarDef` nodes and
  feeds `SetRegistrationList`; `packet.Custom` + `BuildMsfsInputs` append;
  re-register on graph/profile/vehicle change (debounced). **End-to-end by
  hand-authoring the node JSON:** declare an `L:` var, wire its port into a
  cue, watch it drive; a bad name warns in the log and leaves the rest live.

* **Phase 2 — Editor authoring for `MsfsVarDef`.** Palette entry (`AddNode`
  menu item) + node rendering (a new `GetTitleBarColor` case); an inspector port
  grid to add/edit/remove vars with columns *alias / SimVar / Unit*. Model it on
  the **LocalSend/Receive and ConfigOut/In** editable-port editors
  (`ButtonAddBusPort`/`RemoveBusPort`, "Add Port"/"Add Field") — **not** Include,
  whose ports are read-only. The existing bus grid only shows name/bus, so the
  `SimVar`/`Unit` columns are a template *extension*, not free. Unit preset
  dropdown; edit-time syntax validation (§4.4 row 1). No catalog-picker work —
  consumers wire from the node's ports directly.

* **Phase 3 — Warning surfaces.** Two distinct pieces of work, don't conflate:
  (a) **reuse** the existing warning-banner dep-props (`IncludeError`-style) and
  the per-output-port value-label rendering; (b) **new plumbing** — a
  runtime→editor channel carrying `FailedVars` and *live streaming* MSFS values
  (the labels are preview-fed today) so ports paint red and show real values.
  Plus SimHub log lines and `validate_graphs.py` checks that `MsfsVarDef` ports
  are well-formed and aliases unique. **Collision check (alias vs built-in
  `MSFS.*`) needs a name list in Python** — the script has no MSFS domain
  knowledge today, so bundle/generate the built-in name list (from the C#
  catalog) for it. Optional: bundle the SDK `A:` name list for row-2 validation.

* **Phase 4 — Optional/future.** Scoped LVars (`L:1:` — see the MSFS
  DevSupport thread); tagged delta streaming for large custom sets; expose
  customs + warnings in the HTML graph exporter (`render_graph_html.py`);
  generalize `MsfsVarDef` → `VarDef` with a `SignalGroup` for symmetric
  X-Plane custom datarefs. *(Aircraft-change re-registration is no longer here
  — it's part of Phase 1's single re-registration path per §9.2.)*

---

## 6. Risks & Mitigations

* **Probe outcome interpretation.** The drain rule is **invalid ⟺ exception**
  (§4.2): an exception marks the var bad; *no exception within the short
  per-probe timeout marks it valid* (a good var yields no `PERIOD_ONCE` reply
  while the sim is paused, so "no reply" must **not** mean failure — that was the
  point of inverting the rule). The per-probe `requestId` gives unambiguous
  attribution. Keep the lightweight `defineCount == registeredCount` check on the
  shared stream as a backstop against a var that probed clean but then errors on
  the merged add (shouldn't happen).
* **LVAR aircraft-specificity.** A var valid on heli A is absent on heli B.
  Handled by the single re-registration path (§4.2/§9.2): a vehicle change
  re-registers (the guard keys on aircraft title even when the graph is
  shared), so values and any `A:` `FailedVars` re-evaluate for the current
  aircraft rather than going stale; an absent `L:` var simply reads `0` again.
* **Late-initialising LVARs.** Aircraft LVARs come online progressively as the
  aircraft's systems initialise; a *correctly-spelled* `L:` var probed too early
  reads `0` and won't recover until the next re-registration trigger —
  surprising mid-discovery ("right name, still 0"). Mitigation: a one-shot
  re-register of the custom set a few seconds after aircraft-load settles,
  and/or an on-demand "retry vars" affordance. §9.2's re-registration path
  already exists; this just adds a delayed trigger.
* **Silent unit mis-scale.** No exception fires; only the live readout
  reveals it. Documented; unavoidable without a units database.
* **Mid-flight re-registration gap.** Clearing/rebuilding the definition
  briefly interrupts data. Mitigation: only re-register when the set actually
  changed; debounce; keep the last packet during the gap.
* **Buffer bounds.** `_rxBuffer` is 4096 B
  ([MsfsSimConnectClient.cs:42](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L42))
  → ~500 FLOAT64 slots incl. header; fine for tens of customs. Cap custom
  count (e.g. 64) and/or grow the buffer with the registered count.
* **Naming collisions.** A declared alias equal to a built-in `MSFS.*` name
  would shadow/duplicate a dictionary key. Reject at edit-time and in
  `validate_graphs.py`.

---

## 7. Testing

* **Unit (native/C#):** probe-outcome logic — feed synthetic per-probe replies
  and exceptions and assert the survivor set and the resulting offset→name
  mapping, including multiple failures and failures at the first/mid/last custom.
* **Parity:** 36-defaults-only registration vs the current build — identical
  packets over a captured session.
* **Fault injection:** append a deliberately bad var; assert the 36 defaults
  keep streaming and `FailedVars` reports the bad one.
* **Integration:** a known-good `A:` custom (e.g. `GENERAL ENG RPM:1`,
  `number`) streams a plausible value; an `L:` var on the target heli drives a
  cue; wrong-unit var shows an implausible value in the readout.
* **Regression:** `validate_graphs.py` flags a malformed `MsfsVarDef` port
  (empty alias/SimVar) and duplicate aliases across nodes.

---

## 8. Files Touched (estimate)

| File | Phase | Change |
| --- | --- | --- |
| [DiyFfbPlugin.cs](../../SimHubPlugin/DiyFfbPlugin.cs), [DiyFfbPluginSettings.cs](../../SimHubPlugin/DiyFfbPluginSettings.cs), [DiyFfbPlugin.csproj](../../SimHubPlugin/DiyFfbPlugin.csproj) | 0 | **delete** Bridge/UDP path (§3.5): mode enum + UDP fields/methods, `ParseMsfsPacket`, bridge process, deploy target |
| [MsfsSimConnectClient.cs](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs) | 0 | dynamic list, datumId, per-var `PERIOD_ONCE` probe + survivor stream, `FailedVars`, worker-thread `SetRegistrationList` |
| [MsfsSimVarTable.cs](../../SimHubPlugin/Msfs/MsfsSimVarTable.cs) | 0 | defaults become the seed of the registration list; keep enum for default mapping |
| [DiyFfbPlugin.cs](../../SimHubPlugin/DiyFfbPlugin.cs) | 0-1 | `MsfsUdpPacket.Custom`; `ApplyMsfsSimConnectSample` custom fill; scan `MsfsVarDef` nodes and feed the client on graph/profile/vehicle change |
| [GraphSignals.cs](../../SimHubPlugin/GraphSignals.cs) | 1 | append `packet.Custom` to `MSFS.*` inputs |
| [GraphModel.cs](../../SimHubPlugin/GraphEditor/GraphModel.cs) | 1 | `MsfsVarDef` kind + `SimVar`/`Unit` `GraphPort` fields |
| [GraphSerializer.cs](../../SimHubPlugin/GraphEditor/GraphSerializer.cs) | 1 | `GraphPortDto` `SimVar`/`Unit` + `ShouldSerialize*`/`FromModel`/`ToModel`; treat `MsfsVarDef` as `isSignalNode`; V1→V2 migration case |
| [GraphRuntimeConverter.cs](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs) | 1 | `MapNodeType` case + `Convert` else-if: emit `MsfsVarDef` output ports as `MSFS.<alias>` inputs (needs `SignalGroup="MSFS"`) |
| GraphValidator.cs | 1 | kind dispatch case for `MsfsVarDef` (parallel to LocalSend/Receive) |
| GraphEditor (palette `AddNode`, `GetTitleBarColor`, port-grid inspector, warning banner + red ports, live-value feed) | 2-3 | author `MsfsVarDef`; validation; **new runtime→editor value/`FailedVars` channel** (labels are preview-fed today) |
| [validate_graphs.py](../../SimHubPlugin/graphs/validate_graphs.py) | 3 | `MsfsVarDef` port well-formedness + unique aliases; **+ bundled built-in `MSFS.*` name list** for collision check (script has no MSFS knowledge today) |

---

## 9. Resolved Decisions

1. **`MsfsVarDef` is top-level only.** Included sub-graphs reference the
   node's ports via wiring; they don't declare their own vars. No alias
   namespacing across includes to design.
2. **One unified re-registration path — no separate "aircraft change" work.**
   Re-registration is triggered by *the effective registration set changing*;
   editor edits and aircraft swaps are just causes of that. Because
   per-aircraft profiles swap the active graph on vehicle change, an aircraft
   swap already flows through the same `SetRegistrationList` path. **One
   guard refinement (§4.2):** the "skip when unchanged" check keys on
   *(registration list + aircraft title)*, not the list alone — so when two
   aircraft share one graph, a swap still re-registers and re-evaluates
   per-aircraft `L:` availability. The former phase-4 "re-register on aircraft
   title" item is dissolved into Phase 1.
3. **No node-granularity constraint.** One `MsfsVarDef` or many, both work;
   the intended convention is to group by use/axis (e.g. a node per axis or
   per subsystem). Purely a template-authoring choice.

---

## 10. Notes

* **X-Plane symmetry (future).** X-Plane inputs are the same fixed-mapping
  shape ([BuildXPlaneInputs](../../SimHubPlugin/GraphSignals.cs#L30-L76)) fed
  by the X-Plane plugin's datarefs. Custom datarefs would be the analogous
  feature on that side; the `MsfsVarDef` node should generalize to a
  `VarDef` node with a `SignalGroup` (MSFS now, XPlane later) so the authoring
  UX is shared, but the X-Plane transport work is separate and out of scope.
* **Read-only.** This plan reads variables. Writing SimVars/LVARs (e.g. to
  command the sim) is a distinct, higher-risk feature — explicitly excluded.
