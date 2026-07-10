# Plan 24 — Testing Guide (MSFS Variable / Input-Event Writeback)

Companion to [24_msfs_variable_writeback.md](24_msfs_variable_writeback.md). Follow
the phases **in order** — each gates the next. The two wire formats are marked
`UNVALIDATED` in code and must be confirmed against **live MSFS 2024** before they
can be trusted.

**Log to watch throughout:** `C:\Program Files (x86)\SimHub\Logs\SimHub.txt`
(lines prefixed `[MsfsSimConnect]`).

---

## 0. Preconditions

- SimHub plugin deployed: `C:\Program Files (x86)\SimHub\DiyFfbPlugin.dll` (rebuild
  with SimHub **closed** if the copy step failed).
- MSFS 2024 running, in a flight (not the main menu) — writes only apply in-flight.
- In-process SimConnect client selected in plugin settings (not the bridge EXE path).
- A graph loaded for the active aircraft so the editor's **MSFS Vars Out** node is
  available (top-level graphs only).

**Sanity gate (regression):** with **no** `MsfsVarOut` node declared, the read path
must be byte-for-byte unchanged — telemetry streams normally, no new log noise.

---

## Phase 0 / 1 — `A:` / `L:` writes (validated transport, do this first)

This proves the shared node + range map + dispatch against a **known-settable** var
before the higher-risk `B:` path.

### Steps

1. In the graph editor, right-click → **Add MSFS Vars Out** (top-level only).
2. Add/edit one write target:
   - **Alias:** `Elev.Trim` (label + warning key; wire a source into this port)
   - **Target:** `ELEVATOR TRIM PCT`
   - **Unit:** `percent over 100`
   - **Range map:** `In -1 → 1`, `Out -1 → 1` (identity to start; adjust once it tracks)
3. Wire a source into the port (a `Param` slider is easiest for a first test).
4. **Apply.**
5. Move the source and watch the sim's elevator trim.

### Read-back verification (the real check)

`SetDataOnSimObject` produces **no success reply** — a silent write *is* success. So
verify the value actually landed, don't just check "no exception":

- Add a paired **MSFS Vars** (`MsfsVarDef`) reading the same `ELEVATOR TRIM PCT`,
  surface it in the editor live readout, and confirm it reflects what you wrote; **or**
- Read the trim position off an existing default SimVar / the in-sim indicator.

### Log expectations

- `writables registered: N var define(s), 0 input event(s)` on Apply / vehicle change.
- A bogus `A:` name → `write failed: alias '…' exception=… sendId=…`, and the read
  stream stays healthy (telemetry keeps flowing). The offending port paints red in
  the editor.

### If it fails

A wrong value with **no** exception points at the unvalidated `0x10` body — suspect,
in order: `ArrayCount`, `cbUnitSize`, then `Flags` ordering in
`WriteSetDataOnSimObjectFloat64` ([SimConnectProtocol.cs](../../SimHubPlugin/Msfs/SimConnectProtocol.cs)).
Cross-check against node-simconnect `setDataOnSimObject` before changing anything.

> **`L:` caveat:** an LVAR write never raises — a typo silently sets an unused
> variable. There is no offline check; verify by reading it back with a paired
> `MsfsVarDef` on the same name.

---

## Phase 2 — `B:` input events (the driving use case; highest uncertainty)

Only start once `A:`/`L:` is confirmed. The enumerate/hash constants and the
`RECV_ENUMERATE_INPUT_EVENTS` layout are later-protocol additions and **unvalidated**.

### Steps

1. On an aircraft you know exposes the binding, add a **MSFS Vars Out** write target:
   - **Alias:** `Pedals.Preset`
   - **Target:** `B:PEDALS_PRESET_SET` (or any `B:` binding real for this aircraft)
   - **Unit:** ignored for `B:`
   - **Range map:** e.g. `In 0 → 1` ⇒ `Out 0 → N` (a preset selector); `0..1 → 0..1`
     for a trigger.
2. Wire a source, **Apply**.
3. Actuate and confirm the binding fires — **verified in MSFS's dev tools / behaviors
   debugger**, not via read-back (`B:` events have no read path).

### Log expectations

- `writables registered: … , M input event(s)` — `M > 0` means the enumerate
  resolved the hash(es).
- A name absent on the aircraft → `N B: input event(s) unresolved … (pending)` and a
  red editor port; it is **dropped-and-logged**, never fatal, and re-resolves on
  aircraft change.

### Resilience checks (fail-open is the requirement)

- **Connection always comes up.** Even if the enumerate times out / returns nothing,
  `Configure` must finish and the read stream must start — `B:` aliases just stay
  *pending*. A wedged connection here is a bug in the enumerate drain, not expected.
- **Aircraft swap.** Switch to an aircraft without the binding, then back — the hash
  re-resolves on re-registration; a stale-hash write mid-swap logs an exception but
  isn't fatal.

### If it fails

`M` stays 0 for a binding you *know* exists → the enumerate ids / list layout are
wrong. This path is fully isolated: a wrong constant here cannot affect the
`A:`/`L:` path or reads. Cross-check `enumerateInputEvents` (0x4f), `setInputEvent`
(0x51), `RECV_ID_ENUMERATE_INPUT_EVENTS` (34) and the entry layout
(`name[64] + hash(u64) + type(u32)`) against node-simconnect before editing.

---

## Cross-cutting behaviours to verify

- **Range map + clamp.** Drive the source past `InMin`/`InMax`; the sim value must
  saturate at `[min(OutMin,OutMax), max(OutMin,OutMax)]` — never extrapolate.
- **Change-detection.** A steady source produces no repeated writes (watch that the
  log isn't spamming); a moving axis writes ~per frame — that's the expected steady
  state, not a fault.
- **Reconnect re-push (`A:`/`L:` only).** Park an `A:`/`L:` write at a non-zero value,
  restart MSFS / reconnect: the held value must be re-sent once on reconnect (forced
  re-push via `WriteGeneration`).
- **`B:` re-push exemption.** On that same reconnect / aircraft swap / editor Apply, a
  `B:` toggle/preset must **not** re-fire — re-firing an actuation is the bug this
  exemption prevents.
- **Graph switch.** Switching graphs must not leak writes from the old graph
  (`_lastMsfsVarOutValues` cleared).

---

## Offline / regression

- **Graph validator:** `python SimHubPlugin/graphs/validate_graphs.py` — flags empty
  alias/target, unknown target prefix, degenerate range map (`InMax == InMin`), and
  duplicate aliases on `MsfsVarOut` ports.
- **Serialization round-trip:** save + reload a graph with a configured `MsfsVarOut`
  node; alias/target/unit and any non-default range-map fields must survive.

---

## After validation

Once each transport is confirmed live, record the now-known-good constants in the
Plan 19 validated-constants memory (`plan_19_msfs_simconnect_validated`) and drop the
`UNVALIDATED` cautions on `SendIdSetDataOnSimObject`, the Input Event ids, and the
`RECV_ENUMERATE_INPUT_EVENTS` layout in
[SimConnectProtocol.cs](../../SimHubPlugin/Msfs/SimConnectProtocol.cs).
