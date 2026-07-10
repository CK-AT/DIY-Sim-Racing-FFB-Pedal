# Plan 24 — Writing MSFS Variables & Input Events from the Graph (SimVars, LVARs, B: events; per-port range mapping)

Let a graph **write** values back into MSFS — settable SimVars (`A:`),
aircraft **local variables** (`L:` / LVARs), and **input events** (`B:`
bindings) — via a new `MsfsVarOut` output node, the write-side mirror of Plan
23's read-only `MsfsVarDef`. Each write port carries the raw target name +
unit (registration metadata, exactly like `MsfsVarDef`) **plus an integrated
linear range map** (`InMin/InMax → OutMin/OutMax`) applied at the boundary
before the value hits the sim. The node dispatches by target-name prefix —
`A:`/`L:` go over `SetDataOnSimObject`, `B:` over the Input Event API — behind
one uniform port model. A non-settable var, a bad name, or an input event
absent on the current aircraft must **degrade gracefully** (that one write
fails, logged as a warning, everything else keeps working), never break the
telemetry stream or the pipe.

**Driving use case: drive an aircraft cockpit binding — a `B:` input event —
*from the graph*.** Many cockpit interactions (presets, toggles, axis bindings
exposed as model-behavior events like `B:PEDALS_PRESET_SET`) are reachable
*only* through the Input Event API — `SetDataOnSimObject` cannot touch them.
The graph reads the relevant state (a HID axis, a derived value, whatever it
computes), maps it, and actuates the binding directly. The graph *is* the
control. These bindings are **aircraft-specific and discovered by trial**
(inspected in MSFS's dev tools / behaviors debugger), so authoring them in the
graph — not a C# edit + rebuild per guess — is the whole point. Binding a
control axis *through the graph* (leave the axis unbound in-sim, report a
corrected/trim-cancelled position back) is the same capability over the
`A:`/`L:` transport; the correction math is **all graph, no C#**.

**Prerequisites:** [Plan 19](19_msfs_pure_csharp_simconnect.md) (in-process
pure-C# SimConnect client — the transport we extend with a write path).
[Plan 23](23_msfs_custom_graph_variables.md) (`MsfsVarDef` read node, dynamic
data definitions, `SetCustomVars`/worker-thread ownership, the
`GraphPort.SimVar`/`Unit` fields — all of which this plan mirrors). Related:
[Plan 03](03_configout_functionscope.md) (`ConfigOut` change-detected output
dispatch — the runtime pattern `MsfsVarOut` copies).

**In scope — two write transports behind one node:**

* **`B:` input events via the Input Event API** (§4.5) — the driving transport.
  `EnumerateInputEvents` → hash cache → `SetInputEvent`. Aircraft bindings
  (e.g. `B:PEDALS_PRESET_SET`) are only reachable this way. It's a genuinely
  separate transport (no data-definition reuse), so it lands as its own phase
  (Phase 2) — but it is the deliverable, not an add-on.
* **`A:`/`L:` via `SetDataOnSimObject`** (§4.2) — settable SimVars and LVARs.
  Also in scope, and sequenced **first** (Phase 0/1): it validates the shared
  node, range map, and change-detected dispatch against a known-good transport
  before the unvalidated Input Event machinery is built on top.

**Out of scope:**

* **X-Plane dataref writeback** — separate transport (§10 symmetry note).

> **Status (2026-07-10, live MSFS 2024):** `A:`/`L:` (`SetDataOnSimObject`) and
> `K:` key/sim events are **validated and working**. `K:` was originally listed
> out of scope (betting `B:` would cover binding needs) — that bet was wrong;
> `B:` alone is insufficient, so `K:` was added via `MapClientEventToSimEvent`
> (`0x04`) + `TransmitClientEvent` (`0x05`) — routed by the `K:` prefix through
> the same node/range-map/dispatch as `B:`, just a different builder — and is
> the recommended actuation path. **`B:`
> input events are BLOCKED:** node-simconnect's `EnumerateInputEvents` id `0x4f`
> is faithfully reproduced but MSFS 2024 hard-closes the pipe on it (~3 ms,
> parse-time rejection). The 2024 wire id shifted and is not in the SDK header;
> a circuit-breaker disables the enumerate after 3 no-reply attempts so `B:`
> fails safe (reads + other writes unaffected). Getting `B:` working needs a
> packet capture / DLL analysis, not a node-simconnect value. See
> [[plan-24-writeback-validated]] memory.

---

## 1. Goal & Strategy

One capability — **the SimConnect client gains a write path** — surfaced as a
graph output node with built-in scaling, over **two transports**. Coupled
outcomes:

1. **`SetDataOnSimObject` (`0x10`) is added to the protocol** for `A:`/`L:`
   writes — the one new outbound builder for that path. It reuses the existing
   `AddToDataDefinition` registration and the outbound-header convention already
   proven by the read path.
2. **The Input Event API is added** for `B:` writes — `EnumerateInputEvents`
   (async, aircraft-scoped) → a `name → hash` cache → `SetInputEvent(hash,
   value)`. Separate machinery (no data-definition reuse), same worker-thread
   ownership.
3. **Writes are declared in the graph** via a new `MsfsVarOut` node whose
   **input ports** each name a target (`A:`/`L:`/`B:`) + unit. This mirrors
   `MsfsVarDef` (output ports, read). The node is transport-agnostic; the
   **client routes by name prefix**.
4. **Range mapping is per-port and integrated.** Each write port carries
   `InMin/InMax/OutMin/OutMax`; the incoming graph value is linearly mapped
   (and always clamped) at the write boundary — no separate scale node — and
   applies identically to a var write or an input-event value.
5. **A bad/non-settable/absent target is isolated, not fatal.** Writes are
   coalesced and change-detected (the `ConfigOut` pattern); a failing var
   returns an async `RecvIdException`, an unresolved `B:` hash is dropped —
   both logged against the offending alias. The read stream is never touched.

**Strategy:** the deliverable is the **`B:` Input Event transport**, but it's
the most unvalidated piece, so build the shared machinery under it first on a
known-good transport. Ship the **`SetDataOnSimObject` transport first**
(testable in isolation against a known-settable var), then the `MsfsVarOut`
node + runtime dispatch (proves the whole path end-to-end for `A:`/`L:`), then
the **`B:` Input Event transport** routed through that same node — the payoff
phase — then editor authoring, then warning surfaces. Same phase discipline
Plan 23 used; the ordering is de-risking, not deprioritisation.

---

## 2. Why

* **Aircraft bindings need `B:` input events — there is no substitute, and
  this is the point of the plan.** Many cockpit interactions (presets, toggles,
  axis bindings exposed as model-behavior events like `B:PEDALS_PRESET_SET`)
  are reachable *only* through the Input Event API. `SetDataOnSimObject` cannot
  touch them. These bindings are aircraft-specific and found by trial in MSFS's
  dev tools, so a C# edit + rebuild per guess is exactly the loop this plan
  removes — the same discovery pain Plan 23 fixed for reads.
* **Any graph-computed value can drive the sim — writeback is the missing
  half.** Reporting a graph-computed value (a corrected/trim-cancelled axis
  position, a preset selection, whatever the graph derives) requires *sending*
  it. The entire class of "process input in the graph, then feed the result to
  MSFS" is blocked without a write path, for both transports.
* **The `A:`/`L:` transport is 90% there** (and lands first as the proving
  ground). The client already builds outbound packets with the
  `0xF0000000 | type` header
  ([`WriteHeaderPlaceholder`/`FinaliseHeader`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L208-L222)),
  owns the pipe on the worker thread, assigns a unique `sendId` per message
  ([`NextSendId`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L653-L656)),
  and registers datums with `AddToDataDefinition` (which works for writes too).
  `SetDataOnSimObject` is one builder + one worker-drained queue — so it
  validates the shared node/dispatch/range-map cheaply before the `B:`
  machinery is built on top.
* **The graph is already the MSFS-side authoring surface.** Plan 23 made
  reads graph-declared and per-aircraft; writes are the symmetric other half.
  A per-aircraft binding is authored, not hardcoded.
* **The output-dispatch pattern already exists.** `ConfigOut` values come back
  from graph eval in `lastGraphEvaluation.ConfigOutputs` and are change-detected
  + dispatched by
  [`CheckConfigOutChanges`](../../SimHubPlugin/DiyFfbPlugin.cs#L1693-L1793)
  against
  [`_lastConfigOutValues`](../../SimHubPlugin/DiyFfbPlugin.cs#L151).
  `MsfsVarOut` is the same shape with a different sink (the sim, not the config
  tier).

Cost is one new protocol message, a worker-thread write queue, and a node kind
that reuses Plan 23's `SimVar`/`Unit` port fields.

---

## 3. Current State (Audit)

### 3.1 The transport supports reads only

[`MsfsSimConnectClient`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs)
connects, handshakes, registers a data definition, subscribes at `SIM_FRAME`,
and reads. It **never writes** a `SetDataOnSimObject`. All pipe writes happen
on the worker thread; callers hand work in via a synchronized field +
dirty-flag and the worker picks it up at a safe point — the
[`SetCustomVars`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L130-L144) /
`_customsDirty` /
[`ReadLoop`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L473-L507)
pattern. **This is the concurrency contract any write API must obey:** the
caller never touches the pipe or `_txBuffer`.

### 3.2 The protocol has the writers we need except one

* Outbound builders exist for Open, `AddToDataDefinition` (`0x0c`),
  `ClearDataDefinition` (`0x0d`), `RequestDataOnSimObject` (`0x0e`)
  ([SimConnectProtocol.cs:48-51](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L48-L51)).
  The write function **`SetDataOnSimObject` is `0x10`** — sequential in the
  same SDK enum, consistent with the three we already ship. **Missing; this
  plan adds it.**
* [`WriteAddToDataDefinition`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L98-L111)
  already takes a `datumId` and a unit string and works identically for a
  write-target datum — a write var is registered the same way a read var is.
* [`TryReadException`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L191-L202)
  returns the failing `sendId`; the read loop already logs exceptions with it
  ([:497-504](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L497-L504)). A
  `sendId → alias` map turns that generic log into a named-var warning.
* [`MaxOutboundPacketBytes = 1024`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L145)
  — a single-FLOAT64 `SetDataOnSimObject` is `16 (header) + 20 (body) + 8
  (value) = 44` bytes, well within the existing send buffer. No bound change.

### 3.3 The graph read node this mirrors — `MsfsVarDef`

Plan 23's [`MsfsVarDef`](../../SimHubPlugin/GraphEditor/GraphModel.cs#L61)
node: **output** ports, each carrying
[`SimVar`/`Unit`](../../SimHubPlugin/GraphEditor/GraphModel.cs#L266-L273)
registration metadata + a `SignalSuffix` alias; the port emits
`MSFS.<alias>`. The plugin scans these nodes
([`UpdateMsfsCustomVars`](../../SimHubPlugin/DiyFfbPlugin.cs#L1209-L1235)) and
hands the list to
[`SetCustomVars`](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L130). The
converter emits the ports as ordinary MSFS input signals
([GraphRuntimeConverter.cs:271-277](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L271-L277),
[MapNodeType:349](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L349)),
and the serializer round-trips it as a signal node
([GraphSerializer.cs:636-643](../../SimHubPlugin/GraphEditor/GraphSerializer.cs#L636-L643),
DTO fields [:779-781](../../SimHubPlugin/GraphEditor/GraphSerializer.cs#L779-L781)).
**`MsfsVarOut` is this node with the port direction flipped (inputs), plus
range-map fields, plus a write sink instead of a read source.**

### 3.4 The output-dispatch pattern this mirrors — `ConfigOut`

Graph eval returns `lastGraphEvaluation.ConfigOutputs` (a
`Dictionary<string,double>`).
[`CheckConfigOutChanges`](../../SimHubPlugin/DiyFfbPlugin.cs#L1693-L1793),
called every eval from
[`EvaluateActiveGraph`](../../SimHubPlugin/DiyFfbPlugin.cs#L2503-L2506),
diffs each value against
[`_lastConfigOutValues`](../../SimHubPlugin/DiyFfbPlugin.cs#L151) (1e-6
tolerance) and dispatches only changes. **`MsfsVarOut` adds a parallel
`MsfsVarOutputs` channel + a `CheckMsfsVarOutChanges` that dispatches changed
values to `client.WriteValue(...)` instead of the config tier.**

---

## 4. Design

### 4.1 A new `MsfsVarOut` node — write with integrated range mapping

Add a node kind `MsfsVarOut` ("MSFS Vars Out"). Each **input port** consumes a
graph value and writes it to one target — a settable var or an input event:

| Port field | Meaning | Example |
| --- | --- | --- |
| `SignalSuffix` (alias) | label + warning/log key for this write | `Elev.TrimCorrected` |
| `SimVar` *(reused)* | raw target name; **prefix selects the transport** | `ELEVATOR TRIM PCT` / `L:MY_AXIS` / `B:PEDALS_PRESET_SET` |
| `Unit` *(reused)* | SimConnect unit string (`A:`/`L:` only; ignored for `B:`) | `percent over 100` |
| `InMin`/`InMax` *(new)* | expected graph-value input range | `-1` / `1` |
| `OutMin`/`OutMax` *(new)* | target value range for that input span | `-100` / `100` |

You wire straight into a port (e.g. `corrected_elev → MsfsVarOut.in`); the
mapped value is written each time it changes.

**Transport routing is by prefix, transparent to the node.** The client
inspects the target name: `A:`/bare-SimVar and `L:` → `SetDataOnSimObject`
(§4.2); `B:` → the Input Event API (§4.5). The node model, the range map, and
the change-detected dispatch are identical for both — only the client's
per-target registration and the final send differ. An input event still takes a
numeric value (`SetInputEvent` carries a FLOAT64), so the range map applies
unchanged — e.g. a trigger maps `0..1 → 0..1`, a preset selector `0..1 →
0..N`.

**Range mapping lives at the write boundary, not in graph eval.** Like
`SimVar`/`Unit`, the min/max are registration/dispatch metadata that must
never enter the runtime graph (Plan 23's rule — keeps eval pure and the
converter treating the node uniformly). The map is applied in
`CheckMsfsVarOutChanges` (§4.3):

```
t   = (in - InMin) / (InMax - InMin)     // guard InMax == InMin → t = 0
out = OutMin + t * (OutMax - OutMin)
out = clamp(out, min(OutMin,OutMax), max(OutMin,OutMax))   // default: clamp
```

**Always clamp — no extrapolation.** Sending an out-of-range value into the
sim (a value past a control axis's travel) is never wanted, so the output is
unconditionally saturated to `[min(OutMin,OutMax), max(OutMin,OutMax)]`; there
is no opt-out. Identity default (`0..1 → 0..1`) means an unconfigured port is a
passthrough.

**Model changes:** add `MsfsVarOut` to
[`GraphNodeKind`](../../SimHubPlugin/GraphEditor/GraphModel.cs#L8-L62); add four
`double` fields to `GraphPort` (`InMin=0, InMax=1, OutMin=0, OutMax=1`). Reuse
the existing `SimVar`/`Unit`. Ports are **input-only**. As Plan 23 warned,
adding a kind is mechanical but multi-site — touch every kind-dispatch:
`MapNodeType` + the `Convert` chain
([GraphRuntimeConverter.cs](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs)),
the serializer's DTO + `isSignalNode` determination
([GraphSerializer.cs](../../SimHubPlugin/GraphEditor/GraphSerializer.cs)),
`GraphValidator`, `GetTitleBarColor`, and the palette `AddNode` (phase 3).

**Serialization:** unlike `MsfsVarDef`, `MsfsVarOut` ports are **input sinks**,
not signal-bound ports — they resolve their input by wiring, and their alias is
a plain label. Treat it like `ConfigOut` in the serializer (freeform `Name`,
not signal binding). The four map fields + `SimVar`/`Unit` need explicit
`GraphPortDto` plumbing with `ShouldSerialize*`
([GraphSerializer.cs:779-790](../../SimHubPlugin/GraphEditor/GraphSerializer.cs#L779-L790)) —
they do **not** round-trip by reflection.

**Runtime converter:** `MsfsVarOut` is an output *sink*. The converter must
emit each input port's incoming value into the eval result's `MsfsVarOutputs`
channel keyed by alias — the write-side analogue of how `ConfigOut` ports feed
`ConfigOutputs`. (Mechanically: the node evaluates its inputs and records them;
it has no output port feeding other nodes.)

**Declaration scope:** top-level graph only, matching `MsfsVarDef` (§9.1). One
node or several; group by axis/subsystem by convention. Aliases unique across
all `MsfsVarOut` nodes (editor + `validate_graphs.py`).

### 4.2 Write transport — `SetDataOnSimObject`

**Protocol** ([SimConnectProtocol.cs](../../SimHubPlugin/Msfs/SimConnectProtocol.cs)):

* Add `SendIdSetDataOnSimObject = 0x10` and `DataSetFlagDefault = 0`.
* Add one builder `WriteSetDataOnSimObjectFloat64(buf, protocol, defineId,
  objectId, value, sendId)`. Body after the 16-byte header:

  ```
  DefineID(u32) ObjectID=ObjectIdUser(u32) Flags=0(u32) ArrayCount=1(u32) cbUnitSize=8(u32) value(f64)
  ```

  `ObjectID` is always `ObjectIdUser` (`0`) — the user aircraft — the same
  constant the read path passes to `RequestDataOnSimObject`
  ([MsfsSimConnectClient.cs:464](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L464)).

  Reuse `WriteUInt32`; add a `WriteFloat64` primitive (mirror of
  [`WriteFloat32`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L233-L239),
  `BitConverter.GetBytes(double)` → 8 LE bytes).

> **Clean-room caution.** The `0x10` id and the body layout are new and
> **unvalidated** — Plan 19's known-good set covers only the read path.
> Cross-check the id and field order against node-simconnect
> (`setDataOnSimObject`) per the Plan 19 §9 clean-room rule (reproduce
> constants, copy no source), and **live-validate** against MSFS 2024 before
> trusting the trim-cancel path. **Pay particular attention to `ArrayCount`**:
> SDK versions differ on whether `0` means "one element" or the count is
> literal, and a wrong value here corrupts the write without a clean exception
> (garbage into the sim, per §6) rather than failing loudly. Confirm it and the
> `cbUnitSize`/`Flags` ordering against the reference before the read-back test.
> Add it to the Plan 19 validated-constants memory once confirmed.

**Client** ([MsfsSimConnectClient.cs](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs)),
mirroring the `SetCustomVars` discipline:

* **Write define id-space.** A dedicated band (e.g. base `2000`), **one define
  per writable var**, each holding a single FLOAT64. Independent of the
  streaming define (`1`) and the probe define (`1000`) — a var can be both read
  (`MsfsVarDef`) and written (`MsfsVarOut`) with no collision.
* **Clear each write define before re-adding to it.** A reconfigure reassigns
  defineIds to whatever the new write set is, so define `2000` may hold a
  *different* var than last pass. `AddToDataDefinition` **appends** — re-adding
  to a define that still holds its previous datum leaves two datums in it, and
  the single-FLOAT64 assumption baked into `WriteSetDataOnSimObjectFloat64`
  (`ArrayCount=1`, `cbUnitSize=8`) then writes to the wrong/first datum with no
  exception. So on every rebuild, `ClearDataDefinition`
  ([`WriteClearDataDefinition`](../../SimHubPlugin/Msfs/SimConnectProtocol.cs#L116)
  already exists) each write define before re-`AddToDataDefinition`, exactly as
  the read path resets define `1`. Cheapest correct policy: clear the whole
  used write band at the top of `Configure`, then rebuild.
* **`SetWritableVars(IReadOnlyList<MsfsCustomVar>)`** — declares the full write
  set (reuses the `MsfsCustomVar` alias/name/unit record). The worker
  **partitions by prefix**: `A:`/`L:` entries register into their own define
  inside `Configure` (building an `alias → (defineId, name)` map); `B:` entries
  go to the Input Event resolver (§4.5, `alias → hash` map). Re-registration on
  reconfigure rebuilds both for free — same lifecycle as the read customs.
  **The write defines are built only inside `Configure`, so `SetWritableVars`
  must wake the worker to re-enter it** — set a dedicated `_writablesDirty`
  flag, OR'd into the `_customsDirty` check at the top of `ReadLoop`
  ([:478-482](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L478-L482)) so a
  changed write set forces a `LoopResult.Reconfigure`. Do **not** rely on the
  read scan's `SetCustomVars` (which always dirties) to fire the reconfigure:
  a graph with `MsfsVarOut` writes but no `MsfsVarDef` reads would change the
  write set without any read-side dirty, and the write defines / `B:` enumerate
  would never build. The two dirty flags are independent inputs to the same
  reconfigure gate.
* **`WriteValue(string alias, double value)`** — thread-safe, **coalesced**: a
  `Dictionary<string,double>` keyed by alias, latest-value-wins, + a dirty
  flag. The worker drains and **routes each alias by how it registered**
  (define → `SetDataOnSimObject`; hash → `SetInputEvent`). An alias with no
  route yet — a var whose define isn't built or a `B:` hash that never resolved
  (§4.5) — is **dropped-and-logged**, symmetric across both transports; it is
  not an error. A live axis is produced per-sample; only the freshest value
  matters when the worker next drains. The caller never touches the pipe.
  Coalescing (latest-wins between drains) is the *only* client-side dedup —
  change-detection already happens plugin-side in `CheckMsfsVarOutChanges`
  (§4.3), so the drain does not re-diff.
* **Drain in `ReadLoop`.** At the top of each iteration, alongside the existing
  `_customsDirty` check
  ([:478-482](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L478-L482)),
  snapshot + clear the pending-writes dict and send one write per entry via its
  routed transport. Latency = one loop iteration ≈ one sim frame while data
  flows — fine for a control-axis binding, which is only relevant in-flight.
  **Known limitation:** the loop blocks in `ReadOnePacket`, so a pending write
  is not sent until the *next* inbound packet arrives. During a pause / menu /
  loading screen (no `SIM_FRAME` data) writes are deferred until the stream
  resumes — unlike `_customsDirty`, which forces a `Reconfigure` return, writes
  have no wakeup. Acceptable because writes only matter in-flight; do **not**
  add a poll/timeout unless a paused-but-moving case surfaces.
* **Exception attribution — bounded.** Record `sendId → alias` for each write so
  a later `RecvIdException` names the target. `NextSendId()` mints a fresh id
  per send ([:653](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs#L653)), and a
  live axis writes ~every frame (§6), so a plain dictionary would grow
  unbounded (~3600 entries/min at 60 fps) for a lookup that only ever needs
  *recent* ids. Keep it **bounded**: a small ring / LRU of the last N sendIds
  (N ≈ a few hundred), or retain only the ids emitted in the most recent drain.
  Never an unbounded `Dictionary<uint,string>`.

### 4.3 Runtime dispatch — `CheckMsfsVarOutChanges`

Mirror of `CheckConfigOutChanges`
([DiyFfbPlugin.cs:1693-1793](../../SimHubPlugin/DiyFfbPlugin.cs#L1693-L1793)):

* Graph eval produces `lastGraphEvaluation.MsfsVarOutputs` (alias → raw graph
  value). Called every eval from
  [`EvaluateActiveGraph`](../../SimHubPlugin/DiyFfbPlugin.cs#L2503-L2506),
  right after `CheckConfigOutChanges`.
* For each entry: look up the port's `InMin/InMax/OutMin/OutMax` (carried in a
  plugin-side map built when scanning `MsfsVarOut` nodes), apply the linear
  map + clamp (§4.1), then diff the **mapped** value against a
  `_lastMsfsVarOutValues` dict (1e-6 tolerance, same as ConfigOut). On change,
  `_msfsClient.WriteValue(alias, mapped)`.
* Diffing the mapped (post-map) value means an input jitter smaller than one
  output LSB doesn't spam the pipe.

**Registration scan** — a sibling of
[`UpdateMsfsCustomVars`](../../SimHubPlugin/DiyFfbPlugin.cs#L1209-L1235):
scan the active graph's `MsfsVarOut` nodes for `(alias, SimVar, Unit)` per
input port → `client.SetWritableVars(list)`; also build the alias → range-map
(+ per-alias prefix) lookup used by `CheckMsfsVarOutChanges`. Call it from the
same sites as the read scan (graph load, editor Apply, vehicle change:
[:1199](../../SimHubPlugin/DiyFfbPlugin.cs#L1199),
[:2382](../../SimHubPlugin/DiyFfbPlugin.cs#L2382),
[:3750](../../SimHubPlugin/DiyFfbPlugin.cs#L3750)).

**Publish the range-map lookup atomically — it crosses threads.** The scan runs
on the graph-load / editor-Apply / vehicle-change thread, but
`CheckMsfsVarOutChanges` reads the lookup on the eval (game-data) thread. This
is new shared mutable state: `_lastConfigOutValues` is safe today only because
it is touched *solely* on the eval thread, and the range-map dict is not. Build
a **new immutable dictionary** in the scan and publish it by a single reference
assignment (a plain `volatile` field swap is enough — the reader only ever
needs a self-consistent snapshot, never a partially-populated dict). Do **not**
mutate a shared dict in place from the scan thread while the eval thread
enumerates it.

**Clear on graph reload:** clear `_lastMsfsVarOutValues` alongside
`_lastConfigOutValues`
([:2308](../../SimHubPlugin/DiyFfbPlugin.cs#L2308)) so stale writes don't leak
across graph switches.

**Also clear on reconnect / re-registration — not only graph reload.** Unlike
`ConfigOut` (whose in-process tier has no reconnect boundary), the client's
write *defines* are rebuilt on every `SetWritableVars`→`Configure` pass and on
every MSFS reconnect, leaving the sim at default/fresh define state while
`_lastMsfsVarOutValues` still holds the pre-disconnect values. Change-detection
would then suppress re-sending any value that hasn't changed, so a **held**
value (a trim-cancel offset at rest, an axis parked mid-travel) is never
delivered to the reconnected sim until it next moves. A live axis self-heals on
the next frame; a steady value does not. **Therefore force a full re-push
whenever the write set is (re)registered or the client reconnects** — clear
`_lastMsfsVarOutValues` at those points too, so the first eval after
re-registration writes every declared target regardless of whether its value
changed.

**Signal mechanism (pinned).** The client exposes a `volatile int
WriteGeneration`, incremented once at the end of every `Configure` (i.e. every
write-define rebuild / reconnect / `B:` re-enumerate), on the worker thread.
`CheckMsfsVarOutChanges` snapshots it each eval into a plugin field
`_lastWriteGeneration`; when the snapshot differs from the stored value, it
clears `_lastMsfsVarOutValues` (subject to the `B:` exemption below) before the
change-detection pass, then stores the new generation. This is edge-triggered
off a monotonically-increasing counter, so a missed intermediate value can't
strand the plugin — any change since the last eval forces exactly one re-push.
It reuses the same worker-owns-state discipline as `_customsDirty` (worker
writes, other threads read a snapshot), and needs no client→plugin callback.

**The forced re-push applies to `A:`/`L:` var writes only — never to `B:`
input events.** An `A:`/`L:` write is an idempotent *state assignment*:
re-sending the current value re-establishes the sim state and is harmless. A
`B:` input event is an *actuation* — many bindings are toggles, presets, or
triggers (`B:PEDALS_PRESET_SET` is the driving example), and re-firing one on
every reconnect / aircraft swap / editor Apply is an unsolicited side effect
(a spurious preset change, a toggled state). So when clearing
`_lastMsfsVarOutValues` for a re-push, **skip `B:`-prefixed aliases**: they
keep their last-sent value in the dict and re-fire only on a genuine graph
change. (The plugin knows each alias's prefix from the same scan that builds
the range-map lookup, so the exemption is a prefix test, no client round-trip.)

### 4.4 Settability, graceful failure & warnings

Unlike reads, there is **no pre-flight probe for settability** — the `PERIOD_ONCE`
read probe (Plan 23 §4.2) tells you a var *exists*, not that it's *writable*.

**There is also no success acknowledgement.** `SetDataOnSimObject` and
`SetInputEvent` produce no confirming recv — a *silent* write **is** the success
signal, and `RecvIdException` is the only feedback the transport ever emits.
Consequently, for the two cases that don't raise (`L:` typos, and any write
whose value is simply wrong), read-back verification against a paired
`MsfsVarDef` during discovery is the **primary** check, not a fallback.

| Stage | Detects | Surface |
| --- | --- | --- |
| Edit-time | empty alias/target, duplicate alias, `InMax == InMin` (degenerate map), unknown prefix | `MsfsVarOut` inspector marks the port; `validate_graphs.py` error |
| Runtime (bad/non-settable `A:`) | `SetDataOnSimObject` raises `RecvIdException` (e.g. `NAME_UNRECOGNIZED`, or a set-unsupported code) | read loop logs it; the `sendId → alias` map names the var; editor banner (reuse `IncludeError`-style dep prop) |
| Runtime (`L:` write) | LVARs are settable and don't raise; a wrong name silently creates/sets an unused LVAR | no direct signal — cross-check by **reading it back** with a paired `MsfsVarDef` on the same name during discovery |
| Runtime (`B:` absent on aircraft) | `EnumerateInputEvents` returns no matching name → hash never resolves | `WriteValue` for that alias is dropped + logged once; editor red port. Re-resolved on aircraft change |

The write path **never blocks or breaks the read stream**: a failed write is a
logged exception, the pipe stays healthy, `SIM_FRAME` sampling continues.

### 4.5 `B:` input events — the second transport (Input Event API)

`B:` variables are MSFS **Input Events** (aircraft model-behavior bindings).
They are **not** addressable by `SetDataOnSimObject` — you cannot put
`B:PEDALS_PRESET_SET` in a data definition. Writing one is a three-step API,
addressed by a **64-bit hash**, not a name:

1. **`EnumerateInputEvents(requestId)`** → async reply
   `RECV_ENUMERATE_INPUT_EVENTS`, a **list-type** message (paged:
   `dwArraySize` / `dwEntryNumber` / `dwOutOf`, same shape as the SDK's
   `RECV_LIST_TEMPLATE`) whose entries are
   `{ char Name[64]; UINT64 Hash; INPUT_EVENT_TYPE type }` for the **current
   aircraft**. The parser must accumulate across pages until `dwEntryNumber`
   reaches `dwOutOf`.
2. **Cache `name → hash`** for the declared `B:` targets. Aircraft-specific —
   the list changes on aircraft swap, so it is rebuilt on every
   re-registration (§4.2/§9.2), exactly like the read probe.
3. **`SetInputEvent(hash, cbUnitSize, value)`** — `cbUnitSize = 8`, value a
   FLOAT64 (the API also accepts a string form; not needed here).

**Protocol additions** ([SimConnectProtocol.cs](../../SimHubPlugin/Msfs/SimConnectProtocol.cs)):
two outbound builders (`WriteEnumerateInputEvents`, `WriteSetInputEventFloat64`),
one inbound parser (`TryReadEnumerateInputEvents` → yields the page's
`{name, hash}` records), plus the new send-ids, the new recv-id, and the
`INPUT_EVENT_TYPE` enum.

> **Clean-room caution (heightened).** Unlike `SetDataOnSimObject` (`0x10`,
> sequential and confidently placed), the input-event function-ids and recv-id
> were **added later in the SunRise protocol** and are *not* obvious from the
> ids we already ship. Derive each from node-simconnect
> (`enumerateInputEvents` / `setInputEvent` / the `RECV_ENUMERATE_INPUT_EVENTS`
> layout) per the Plan 19 §9 clean-room rule, and **live-validate** the
> enumerate round-trip + a real `SetInputEvent` against MSFS 2024 before
> trusting it. This is the single largest unknown in the plan.

**Client** (extends §4.2, same worker-thread ownership):

* `SetWritableVars` routes `B:` entries to an input-event resolver. During
  `Configure`, after the handshake, issue `EnumerateInputEvents` on a scratch
  request id and drain the paged reply with a bounded timeout (mirror the
  `PERIOD_ONCE` probe loop), building `alias → hash` for the declared `B:`
  names. A `B:` name absent from the enumeration never resolves → recorded in
  `FailedVars`, its `WriteValue`s dropped-and-logged.
* **The enumerate drain is the plan's real resilience surface — it must fail
  open, never wedge the connection.** Everything downstream of a resolved hash
  is trivial (route to `SetInputEvent`) or trivially safe (unresolved alias →
  drop-and-log). The one place a `B:` bug can take down the *whole* MSFS path
  is the paged-reply accumulation, because it runs inside `Configure` **before
  the read stream comes up**. Hard requirements on that loop:
  * **Bounded, always-terminating drain.** Accumulate pages until
    `dwEntryNumber` reaches `dwOutOf`, but cap on both a wall-clock timeout
    **and** a max-page/entry count. A malformed header, a page that never
    arrives, `dwOutOf == 0`, or an entry count that disagrees with
    `dwArraySize` must all exit the loop, not spin or block.
  * **Timeout/malformed ⇒ degrade, don't abort.** On any give-up, leave every
    declared `B:` alias *pending* (droppable-and-logged) and **let
    `Configure` finish and the read stream start normally.** A failed or
    empty enumerate must never prevent the connection from establishing or
    touch the `A:`/`L:`/read paths — it only means "no `B:` resolved yet,"
    which the aircraft-change re-run (next bullet) later fixes.
  * **Never trust the reply blindly.** Bound `Name[64]` reads, ignore entries
    whose type/hash is obviously garbage, and treat a short packet as
    end-of-list, not a parse error that kills the loop.
* **The enumeration is aircraft-scoped and `Configure` can run before the
  aircraft is fully loaded**, so a single enumerate at connect time can return
  an empty/wrong list and dump every `B:` alias into `FailedVars`. The resolver
  must therefore re-run on the **same delayed / aircraft-change re-registration
  trigger the read-side late-LVAR mitigation uses** (Plan 23 §9.2), not only
  once at `Configure`. Treat an unresolved `B:` alias as *pending*, not
  permanently failed: it stays droppable-and-logged until a later enumerate
  resolves it (or the port is confirmed absent on the current aircraft). This is
  the write-side analogue of re-probing LVARs that appear only after the model
  finishes initialising.
* `WriteValue` for a `B:`-routed alias sends `SetInputEvent(hash, value)` using
  the mapped value from §4.3 — no data definition involved. Same coalescing and
  `sendId → alias` exception attribution as the var path.

**Still worth checking per target:** some `B:` bindings are backed by an
`L:`/`A:` var; if a target is reachable as an LVAR, the simpler
`SetDataOnSimObject` path avoids the enumerate/hash round-trip entirely. Prefer
it when it exists — but the Input Event path is required for the many bindings
that are *not* var-backed.

---

## 5. Phases

* **Phase 0 — Write transport.** Add `SetDataOnSimObject` (`0x10`) +
  `WriteFloat64` to the protocol; add `SetWritableVars` / `WriteValue` /
  worker-thread write define registration + drain to the client;
  `sendId → alias` map for exception logging. **Validate in isolation:** wire a
  tiny test caller to write a known-settable var (e.g. an elevator trim var)
  and confirm the value lands in-sim (read it back via an existing default or a
  `MsfsVarDef`), and that a bad name raises a logged exception without
  disturbing the read stream. No graph feature yet. **Gate:** the read path is
  byte-for-byte unchanged when no writes are declared.

* **Phase 1 — `MsfsVarOut` model + runtime dispatch, `A:`/`L:` path (no palette
  UI).** Add the `MsfsVarOut` kind + four range-map `GraphPort` fields;
  serializer DTO + `isSignalNode`/`Convert` handling (ConfigOut-style, input
  sink); `MsfsVarOutputs` from the converter; `CheckMsfsVarOutChanges` + the
  registration/range-map scan feeding `SetWritableVars`; clear-on-reload. The
  corrected-axis math is authored in the graph — no C# feature layer. **Prove
  it end-to-end by hand-authoring the node JSON:** leave an axis unbound in
  MSFS, read the raw HID axis in the graph, compute a corrected value, write it
  via `MsfsVarOut`, confirm the sim control tracks it; a bad name warns and
  leaves everything else live. Trim-cancellation is just this pattern with a
  specific correction.

* **Phase 2 — `B:` input event transport (Input Event API) — the payoff
  phase.** The driving transport (§4.5): `EnumerateInputEvents` builders +
  paged-reply parser, `SetInputEvent`, the `alias → hash` resolver in the
  client's `Configure`, and prefix-routing in `SetWritableVars`/`WriteValue`.
  Node, dispatch, and range map from Phase 1 are reused unchanged — a `B:` port
  is authored identically. **Highest-uncertainty phase** (unvalidated
  function-ids); gate on a live enumerate + `SetInputEvent` round-trip against a
  real aircraft binding before proceeding. **Prove it:** wire a `B:` target,
  confirm the binding fires (verified in MSFS's dev tools / behaviors debugger,
  not via read-back — `B:` events have no read path); a name absent on the
  aircraft is dropped + logged, not fatal; a timed-out/empty enumerate still
  brings the connection up with those aliases pending.

* **Phase 3 — Editor authoring.** Palette `AddNode` entry + `GetTitleBarColor`
  case; an inspector port grid with columns *alias / target / Unit / InMin /
  InMax / OutMin / OutMax*, modeled on the **ConfigOut** editable-port editor
  (add/remove field). Unit preset dropdown (reuse Plan 23's list), disabled for
  `B:` rows; edit-time syntax + prefix + degenerate-map validation.

* **Phase 4 — Warning surfaces.** Reuse the `IncludeError`-style banner
  dep-props + per-port marking; new runtime→editor channel carrying write
  `FailedVars` (non-settable `A:`, unresolved `B:` hash) so the offending
  `MsfsVarOut` port paints red. SimHub log lines; `validate_graphs.py` checks
  for well-formed ports and unique aliases.

---

## 6. Risks & Mitigations

* **Unvalidated write protocol (`0x10`).** A wrong body layout could raise
  exceptions or, worse, write garbage into the sim. Mitigation: clean-room
  cross-check + live validation in phase 0 against a known-settable var, read
  back to confirm, before any real use.
* **Input Event API ids are the single biggest unknown (Phase 2).** The
  enumerate/set function-ids and the `RECV_ENUMERATE_INPUT_EVENTS` layout are
  later-protocol additions, not inferable from the ids we ship. Mitigation:
  derive from node-simconnect (clean-room) and gate Phase 2 on a live
  enumerate + `SetInputEvent` round-trip; keep it isolated so a wrong constant
  can't affect the `A:`/`L:` path or reads. If it proves intractable, the
  `A:`/`L:` transport still ships.
* **`B:` hash resolution timing.** Input events resolve to hashes only after an
  async enumerate that runs at (re)registration; a `B:` write issued before
  resolution, or for an event absent on the aircraft, must drop-and-log, never
  block. Late-initialising aircraft may need the same delayed re-register as
  Plan 23's late-LVAR mitigation.
* **Writing a non-settable `A:` var.** Silent (async exception only).
  Mitigation: `sendId → alias` logging + editor red-port surface (phase 4);
  document that many control-position vars are read-only and trim vars are the
  settable ones.
* **Write feedback loop.** Writing a corrected value that the sim then folds
  back into a var the graph reads, which changes the correction, which changes
  the write… Mitigation: keep the correction feed-forward — don't build a graph
  that servos a var it also writes. The axis-binding case is naturally
  feed-forward (raw HID in → corrected axis out); flag the risk for anyone
  reading a var they also drive.
* **Write rate.** For a live axis binding, the value changes nearly every
  frame, so a per-frame `SetDataOnSimObject` is the **expected steady state**,
  not a fault — this is exactly what a HID axis binding does in-sim, and it's
  negligible on a local pipe. Coalescing + the post-map 1e-6 diff simply avoid
  redundant writes when the axis is still.
* **Mid-flight re-registration gap.** Rebuilding write defines on graph/vehicle
  change or reconnect briefly drops the write target. Mitigation: same
  debounce/guard as the read path (Plan 23 §9.2). A *changing* value self-heals
  on the next frame, but a **held** value would stay stale — so clear
  `_lastMsfsVarOutValues` on every re-registration/reconnect (§4.3) to force a
  full re-push, not just rely on the next changed value. **Scope the re-push to
  `A:`/`L:` writes**: re-firing a `B:` toggle/preset on reconnect is a spurious
  actuation, so `B:` aliases are exempt (§4.3, RD6).
* **`L:` writes are unverifiable offline.** A typo'd LVAR write silently sets an
  unused variable. Mitigation: pair with a read-back `MsfsVarDef` during
  discovery (§4.4).

---

## 7. Testing

* **Unit (C#):** `WriteSetDataOnSimObjectFloat64` and
  `WriteSetInputEventFloat64` emit the exact expected byte layout (header +
  body + LE double) for representative values; `TryReadEnumerateInputEvents`
  reassembles a multi-page reply into the full `{name, hash}` set. Range-map
  math: identity, inverted (`OutMin > OutMax`), clamp at both ends, degenerate
  (`InMax == InMin`).
* **Enumerate-drain resilience (the load-bearing `B:` test).** Feed the drain
  loop synthetic replies and assert it always terminates and leaves the client
  usable: a well-formed multi-page reply resolves every hash; a malformed
  header, a truncated/short page, `dwOutOf == 0`, an entry count disagreeing
  with `dwArraySize`, and a page that never arrives (timeout) each exit the
  loop, leave the declared `B:` aliases *pending*, and let `Configure` complete
  so the read stream still comes up. No input can spin, block, or abort the
  connection.
* **Transport (integration):** write a known-settable trim var, read it back
  (default slot or `MsfsVarDef`), assert it tracks; write a bogus `A:` name,
  assert a logged exception + healthy read stream. Enumerate input events on a
  real aircraft, resolve a known `B:` name's hash, `SetInputEvent`, and confirm
  the binding fires; a `B:` name absent on the aircraft resolves to no hash and
  drops-and-logs.
* **Dispatch:** `CheckMsfsVarOutChanges` writes only on change (post-map
  tolerance), coalesces same-frame updates, routes by prefix, and clears on
  graph reload.
* **End-to-end:** an `MsfsVarOut` driven by a param maps + writes; the sim
  value (var) or binding (`B:`) matches the mapped/clamped expectation across
  the input range.
* **Regression:** `validate_graphs.py` flags a malformed `MsfsVarOut` port
  (empty alias/target), an unknown prefix, `InMax == InMin`, and duplicate
  aliases.

---

## 8. Files Touched (estimate)

| File | Phase | Change |
| --- | --- | --- |
| [SimConnectProtocol.cs](../../SimHubPlugin/Msfs/SimConnectProtocol.cs) | 0, 2 | (0) `SendIdSetDataOnSimObject=0x10`, `DataSetFlagDefault`, `WriteSetDataOnSimObjectFloat64`, `WriteFloat64`; (2) enumerate/set-input-event send-ids + recv-id, `WriteEnumerateInputEvents`, `WriteSetInputEventFloat64`, `TryReadEnumerateInputEvents` |
| [MsfsSimConnectClient.cs](../../SimHubPlugin/Msfs/MsfsSimConnectClient.cs) | 0, 2 | (0) write define id-space (**`ClearDataDefinition` each write define before re-add**), `SetWritableVars` (+ `_writablesDirty` OR'd into the `ReadLoop` reconfigure gate), `WriteValue` (coalesced), register write defines in `Configure`, drain in `ReadLoop`, **bounded** `sendId → alias` map, **`volatile int WriteGeneration` bumped per `Configure`** for the plugin cache clear; (2) `B:` prefix routing, enumerate + `alias → hash` resolver (**fail-open bounded drain**; re-run on aircraft-change/delayed re-register, not only `Configure`), `SetInputEvent` on drain |
| [DiyFfbPlugin.cs](../../SimHubPlugin/DiyFfbPlugin.cs) | 1 | `MsfsVarOutputs` consume; `CheckMsfsVarOutChanges` (+ `_lastMsfsVarOutValues`, clear-on-reload **and** on `WriteGeneration` change, **`B:` aliases exempt from the forced re-push**); `MsfsVarOut` scan → `SetWritableVars` + range-map/prefix lookup **published as an atomic `volatile` snapshot** (built on scan thread, read on eval thread); call sites alongside `UpdateMsfsCustomVars` |
| [GraphModel.cs](../../SimHubPlugin/GraphEditor/GraphModel.cs) | 1 | `MsfsVarOut` kind + `InMin/InMax/OutMin/OutMax` `GraphPort` fields |
| [GraphSerializer.cs](../../SimHubPlugin/GraphEditor/GraphSerializer.cs) | 1 | `GraphPortDto` range-map fields + `ShouldSerialize*`/`FromModel`/`ToModel`; `MsfsVarOut` as ConfigOut-style (non-signal) node; V1→V2 migration case |
| [GraphRuntimeConverter.cs](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs) | 1 | `MapNodeType` case + `Convert` else-if: `MsfsVarOut` input ports feed `MsfsVarOutputs` (output sink, no downstream port) |
| GraphValidator.cs | 1 | kind dispatch for `MsfsVarOut` |
| GraphEditor (palette `AddNode`, `GetTitleBarColor`, port-grid inspector w/ range-map columns, warning banner + red ports) | 3-4 | author `MsfsVarOut`; validation; write-`FailedVars` runtime→editor channel |
| [validate_graphs.py](../../SimHubPlugin/graphs/validate_graphs.py) | 4 | `MsfsVarOut` port well-formedness, unique aliases, valid prefix, degenerate map |

---

## 9. Resolved Decisions

1. **`MsfsVarOut` is top-level only** (matches `MsfsVarDef`, Plan 23 §9.1).
   No cross-include alias namespacing.
2. **Range mapping is per-port and applied at the write boundary**, not a
   separate scale node and not inside graph eval — keeps eval pure and the
   mapping colocated with the port that owns it. **Always clamp to output
   range; no extrapolation, no opt-out.**
3. **Change-detect on the mapped value**, reusing the `ConfigOut` 1e-6
   tolerance — bounds write rate to output resolution.
4. **`B:` input events are in scope via a second transport** (Input Event API,
   §4.5), routed by name prefix behind the same node, landing as Phase 2.
   Prefer an `L:`/`A:` backing var when one exists (simpler path), but the
   Input Event path is required for non-var-backed bindings.
5. **No settability probe.** Writability (and `B:` presence) isn't testable
   pre-flight beyond the enumerate; rely on async exception logging + read-back
   verification during discovery.
6. **Write-value cache is cleared on graph reload *and* on reconnect /
   re-registration**, so a held (unchanging) value is re-pushed to a
   freshly-reconnected sim instead of being suppressed by change-detection.
   This is where `MsfsVarOut` deliberately diverges from `ConfigOut`, whose
   in-process tier has no reconnect boundary. **The forced re-push is scoped to
   `A:`/`L:` var writes only; `B:` input events are exempt** — re-pushing an
   idempotent state assignment is harmless, but re-firing a toggle/preset/trigger
   actuation on every reconnect is an unwanted side effect (§4.3).
7. **`sendId → alias` attribution is bounded** (ring/LRU of recent ids), never
   an unbounded map — per-frame writes would otherwise leak for the session.
8. **Write defines are `ClearDataDefinition`'d before every re-add** (§4.2).
   `AddToDataDefinition` appends, and a reconfigure reassigns defineIds, so
   skipping the clear leaves a stale datum and the single-FLOAT64 write targets
   the wrong one — silently. Clear the used write band at the top of `Configure`.
9. **The plugin-side range-map/prefix lookup is published as an atomic
   `volatile` snapshot** (§4.3): built on the scan thread, read on the eval
   thread, never mutated in place. Unlike `_lastConfigOutValues` (eval-thread
   only), this lookup genuinely crosses threads.
10. **Re-push is edge-triggered off a `volatile int WriteGeneration`** the
    client bumps per `Configure` (§4.3); the plugin snapshots it each eval and
    clears `_lastMsfsVarOutValues` on change. Monotonic counter, not a flag, so
    a coalesced burst of reconfigures still forces exactly one re-push.

---

## 10. Notes

* **X-Plane symmetry (future).** Writing datarefs is the analogous X-Plane
  feature; if `MsfsVarDef` generalizes to a `VarDef` with a `SignalGroup`
  (Plan 23 §10), `MsfsVarOut` should generalize to a `VarOut` the same way.
  X-Plane transport work is separate and out of scope.
* **This is the counterpart to Plan 23's explicit exclusion** ("Writing
  SimVars/LVARs … is a distinct, higher-risk feature — explicitly excluded",
  Plan 23 §10). The higher risk is concentrated in the two unvalidated
  transports — the `0x10` `SetDataOnSimObject` body (Phase 0) and, more so, the
  later-protocol Input Event ids (Phase 2). Gate each on its own live
  validation before anything depends on it; the phases are ordered so the
  `A:`/`L:` path ships and proves the node even if the input-event ids take
  longer to pin down.
