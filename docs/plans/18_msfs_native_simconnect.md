# MSFS Bridge: In-Process SimConnect (drop EXE + UDP)

Replace the out-of-process [MsfsPlugin/MsfsFfbDataProvider.cpp](MsfsPlugin/MsfsFfbDataProvider.cpp)
+ UDP loopback contract with a SimConnect client that runs **inside the
SimHub plugin's own process**. The bridge EXE, the 152-byte packet, the
process-spawn lifecycle, and the UDP listener thread all go away.

**Prerequisite:** Plan 17 (MSFS heli signals) — already shipped phase 1a/1b.
That plan delivers the value (raw SimVars → graph). This plan refactors
the plumbing only; no signal semantics change.

**Goal:** the plugin DLL talks to MSFS directly via the official managed
SimConnect wrapper, on its own worker thread, with the same connect /
disconnect / reconnect behaviour the EXE has today. Graphs see the
identical `MSFS.*` signal stream.

---

## 1. Why

The EXE+UDP design was a Plan-17-phase-1 expedient. It worked, it shipped,
and now its costs are visible:

* **Two deployment artefacts.** Plugin DLL + `MsfsFfbDataProvider.exe` +
  `SimConnect.dll` next to it. Users who hand-copy the plugin miss the
  bridge silently — MSFS support just doesn't work, no error.
* **Lifecycle complexity.** [DiyFfbPlugin.cs:1196-1253](SimHubPlugin/DiyFfbPlugin.cs#L1196-L1253)
  is a 60-line process spawner / killer / orphan-cleanup. The bridge's
  own outer reconnect loop ([MsfsFfbDataProvider.cpp:390-407](MsfsPlugin/MsfsFfbDataProvider.cpp#L390-L407))
  is a second copy of the same retry logic the X-Plane plugin doesn't need.
* **UDP serialise/deserialise.** Each sample marshals 35 doubles → 35 floats
  → 152 bytes → socket → 152 bytes → 35 floats → POCO. Pure overhead;
  same address space, same machine, same process tree.
* **Two build chains.** MSBuild (C# plugin) + MSBuild + MSFS SDK (C++ bridge).
  The C++ side needs `MSFS_SDK` set, ships its own vcxproj, has its own
  Debug/Release tree.
* **No shared logging.** Bridge logs to stdout + OutputDebugString; plugin
  logs to SimHub.txt. Debugging a SimVar that "doesn't look right" means
  attaching to two processes.
* **Loss of telemetry on bridge crash.** If the EXE crashes, the plugin
  silently stops receiving — no signal-source dropdown indication, no
  log. Today only the freshness check ([DiyFfbPlugin.cs:1428-1431](SimHubPlugin/DiyFfbPlugin.cs#L1428-L1431))
  catches it.

In-process eliminates all six. There is no functional benefit to the
EXE that we'd be giving up.

---

## 2. The Hard Constraint: SimHub is x86

`SimHubWPF.exe` is a **32-bit .NET assembly** (PE32, Intel 80386). The
plugin loads into that process, so any unmanaged DLL the plugin pulls
in must also be x86. The current bridge sidesteps this by running as a
separate x64 process — that's why the EXE exists at all in some sense.

**Implications:**

* We need the **x86** build of `SimConnect.dll`, not the x64 one we
  currently ship from `MsfsPlugin\Release\`.
* The MSFS 2024 SDK ships both. Confirm path during phase 0:
  `<SDK>\SimConnect SDK\lib\` historically holds the x64 DLL/LIB;
  the x86 build is typically in a sibling folder (`lib\x86\` or
  similar — verify against the actual 2024 SDK layout).
* The MSFS server accepts x86 SimConnect clients — this is unchanged
  from MSFS 2020 and is how every legacy add-on (FSUIPC, MobiFlight,
  etc.) connects.
* **If the 2024 SDK no longer ships an x86 SimConnect.dll,** the plan
  pivots: keep the bridge EXE alive solely as a marshalling shim, but
  collapse UDP → named pipe / shared memory in-process IPC. Decide at
  the end of phase 0. (Almost certainly the SDK still ships x86 — Asobo
  hasn't publicly stated they're dropping it.)

This constraint dictates approach selection (§3) and a non-trivial
chunk of phase 0.

---

## 3. Approach Selection

Three ways to talk to SimConnect from a .NET process. Surveyed in order
of risk, then a recommendation.

### 3.1 Option A — Managed wrapper (`Microsoft.FlightSimulator.SimConnect.dll`)

The official .NET wrapper shipped in `<SDK>\SimConnect SDK\lib\managed\`.
P/Invokes the native `SimConnect.dll` under the hood; exposes the API as
events and `RegisterDataDefineStruct<T>` calls.

* **Pro:** official, supported by Asobo, used by every C# add-on
  (MobiFlight, SimVar Project, etc.). Drops the EXE in ~300 lines of C#.
* **Pro:** the managed wrapper exposes `EventHandle` + `ReceiveMessage`
  so we can pump it from a dedicated worker thread without a Windows
  message loop — important for a SimHub plugin that has no UI loop of
  its own.
* **Con:** still requires the native `SimConnect.dll` to be next to the
  plugin DLL. So we drop the EXE but keep one unmanaged DLL.
* **Con:** ILRepack interaction. The wrapper itself can't be IL-merged
  (P/Invoke pinning would break). It ships as a separate file.

### 3.2 Option B — Direct P/Invoke

Hand-roll `[DllImport("SimConnect.dll")]` declarations against the C
exports. Skip the managed wrapper.

* **Pro:** one fewer file (no managed wrapper DLL alongside).
* **Con:** ~400 lines of marshalling for what the wrapper gives us free.
* **Con:** every SimConnect API revision (Asobo ships periodic SDK
  updates) requires us to re-check our hand-rolled signatures.
* **Verdict:** rejected. The wrapper is a thin shim; reimplementing it
  buys nothing.

### 3.3 Option C — Pure C# implementation (no SimConnect.dll at all)

The SimConnect protocol is a binary stream over named pipe
(`\\.\pipe\Microsoft Flight Simulator\SimConnect`) or TCP. Reverse-
engineered C# clients exist (FlyByWire's MobiFlight fork, `CoolFsConnect`,
`SimConnect.Net`).

* **Pro:** drops `SimConnect.dll` entirely. Plugin ships as a single DLL.
* **Pro:** no x86/x64 mismatch headaches; pure managed code is bitness-
  agnostic.
* **Con:** undocumented wire format. Asobo can change it in any sim
  update; our plugin breaks until we update the protocol parser.
* **Con:** MSFS 2024 may already have changed the protocol vs 2020.
  Unknown until tested.
* **Con:** maintenance burden falls on us, not Microsoft.
* **Verdict:** not now. Re-evaluate **only** if (a) shipping the x86
  SimConnect.dll proves logistically painful, or (b) we hit a hard
  version-skew bug with the official client.

### 3.4 Recommendation

**Option A.** Drop the EXE; keep the native `SimConnect.dll` as a single
unmanaged dependency shipped next to the plugin DLL. The managed wrapper
is the smallest, safest, most-supported path. We can always graduate to
Option C later if SimConnect.dll deployment becomes a real problem —
but the dependency is already shipping today (next to the EXE), so the
delta is zero.

---

## 4. Target Architecture

### Before (today)

```text
MSFS 2024 ──SimConnect──► MsfsFfbDataProvider.exe ──UDP──► SimHub plugin
                          (separate x64 process,            (UdpClient on
                          SimConnect.dll x64)                127.0.0.1:27016)
                                                            ParseMsfsPacket
                                                            → latestMsfsPacket
```

### After

```text
MSFS 2024 ──SimConnect──► SimHub plugin (x86)
                          ├── MsfsSimConnectClient (worker thread)
                          │     ├── SimConnect open / dispatch / reconnect
                          │     └── RegisterDataDefineStruct<SimVarSample>
                          └── latestMsfsPacket (unchanged downstream)
```

The downstream path — `latestMsfsPacket` → `BuildMsfsInputs` → graph —
is **untouched**. The whole change lives in the `Msfs*` plumbing layer.

### `MsfsSimConnectClient`

New file [SimHubPlugin/MsfsSimConnectClient.cs](SimHubPlugin/MsfsSimConnectClient.cs).
Owns the SimConnect handle, the worker thread, and the connect/reconnect
state machine. Public surface kept minimal:

```csharp
internal sealed class MsfsSimConnectClient : IDisposable
{
    public MsfsSimConnectClient(Action<MsfsUdpPacket> onSample, Action<string> log);
    public void Start();      // spawns worker thread, returns immediately
    public void Stop();       // signals worker, waits, releases handles
    public bool IsConnected { get; }
    public DateTime LastSampleUtc { get; }
    public void Dispose();    // = Stop()
}
```

Internals mirror the current EXE one-for-one:

* `Open` loop with 2-second retry until MSFS appears
* `RegisterDataDefineStruct<SimVarSample>` with the same 35 SimVar
  definitions currently in [MsfsFfbDataProvider.cpp:137-173](MsfsPlugin/MsfsFfbDataProvider.cpp#L137-L173)
* `RequestDataOnSimObject(SIMCONNECT_PERIOD.SIM_FRAME)` for ~50 Hz updates
* `OnRecvSimobjectData` event → unpack struct → build `MsfsUdpPacket` →
  invoke `onSample` callback (synchronous on worker thread; callback
  acquires `msfsLock` to publish)
* `OnRecvQuit` → tear down, set disconnected, restart the open loop

**Dispatch model.** SimConnect supports three: callback (current C++
EXE), threaded event with `WaitForSingleObject` + `ReceiveMessage`,
WindowProc message pump. We use **threaded event** (Option 2): the
managed wrapper exposes `EventHandle` (an `IntPtr` to a kernel event
object), `WaitForSingleObject` on it from our worker, then
`ReceiveMessage()` to drain queued messages, which fires the
`OnRecv*` events on the calling thread.

WindowProc is a non-starter — SimHub gives us no message loop to hook.
Callback mode also requires a message loop on most implementations.
Threaded event is the documented, dispatch-loop-free option.

### `DiyFfbPlugin` integration

Trim the existing MSFS plumbing in [DiyFfbPlugin.cs](SimHubPlugin/DiyFfbPlugin.cs):

| Today | After |
| --- | --- |
| `_msfsBridgeProcess` field | removed |
| `msfsUdpClient`, `msfsUdpThread`, `msfsUdpCts` fields | removed |
| `StartMsfsBridgeProcess` / `StopMsfsBridgeProcess` | removed |
| `StartMsfsUdpReceiver` / `StopMsfsUdpReceiver` | removed |
| `MsfsUdpLoop` / `ParseMsfsPacket` | removed |
| (new) `_msfsClient` field of type `MsfsSimConnectClient` | added |
| (new) `StartMsfsClient` / `StopMsfsClient` | added |
| `MsfsUdpPacket` POCO + `latestMsfsPacket` + `GetLatestMsfsPacket` + `IsMsfsTelemetryFresh` | **unchanged** |
| `MsfsPacketMagic`, `MsfsPacketVersion`, `MsfsPacketSizeBytes` constants | removed |
| Call sites in `Init` / shutdown | swapped 1:1 |

The POCO stays. It's a perfectly fine in-memory snapshot type; renaming
it (e.g. to `MsfsTelemetrySnapshot`) is optional churn — defer.

### Settings

[DiyFfbPluginSettings.cs:147-148](SimHubPlugin/DiyFfbPluginSettings.cs#L147-L148):

```csharp
public bool MsfsUdpEnabled = true;    // → rename MsfsSimConnectEnabled
public int  MsfsUdpPort    = 27016;   // → removed
```

Settings file is auto-migrated on first load: missing field → default true,
old field silently ignored (SimHub's `SaveCommonSettings` is forgiving).
No user-visible migration UI needed.

---

## 5. Build & Deployment

### What's added

* **NuGet / reference:** `Microsoft.FlightSimulator.SimConnect.dll` (the
  managed wrapper). The MSFS SDK ships this as a redistributable;
  vendor it into `SimHubPlugin/lib/` and reference by HintPath (same
  pattern as the InputManagerCS / vJoyInterfaceWrap references in
  [DiyFfbPlugin.csproj:69-72,161-164](SimHubPlugin/DiyFfbPlugin.csproj#L69-L72)).
* **Post-build copy:** the **x86** `SimConnect.dll` from the SDK into
  the plugin output dir (`C:\Program Files (x86)\SimHub\`). Replace the
  existing `DeployMsfsBridge` target ([DiyFfbPlugin.csproj:472-481](SimHubPlugin/DiyFfbPlugin.csproj#L472-L481))
  with a target that copies just `SimConnect.dll` from
  `$(MsfsSdkRoot)\SimConnect SDK\lib\x86\` (verified path TBD in §7).

### What's deleted

* Entire `MsfsPlugin/` directory: `.sln`, `.vcxproj`, `.cpp`, `Release/`
  build artefacts. Git rm.
* The `DeployMsfsBridge` MSBuild target in [DiyFfbPlugin.csproj:467-481](SimHubPlugin/DiyFfbPlugin.csproj#L467-L481).
* SimHubPlugin/bin gets `MsfsFfbDataProvider.exe` + `MsfsFfbDataProvider.pdb`
  removed (currently tracked — see `git status` modifications on
  `SimHubPlugin/bin/`).

### ILRepack interaction

`Microsoft.FlightSimulator.SimConnect.dll` **must not** be IL-merged
into `DiyFfbPlugin.dll`. The wrapper holds native P/Invoke handles
that ILRepack can break (PInvoke target name resolution, COM interop
tables). The post-build line in [DiyFfbPlugin.csproj:462](SimHubPlugin/DiyFfbPlugin.csproj#L462)
already excludes everything not explicitly listed, so the default is
correct — just **don't add** `Microsoft.FlightSimulator.SimConnect.dll`
to the ilrepack inputs. Confirm by inspecting the output DLL: the
managed wrapper should remain a separate file alongside the plugin.

### DLL search path

When SimHub loads `DiyFfbPlugin.dll`, the managed wrapper P/Invokes
`SimConnect.dll`. Windows resolves unmanaged DLLs via the standard
search order, which **includes the application directory** (= the
SimHub install root = where our plugin DLL lives). So dropping
`SimConnect.dll` next to the plugin DLL works without `SetDllDirectory`
gymnastics.

If, in testing, the load fails (`DllNotFoundException`), the fallback
is a `[ModuleInitializer]` (.NET 5+ — not available on net48) or a
static constructor that calls `SetDllDirectory` to the assembly
location. Decide based on actual phase-1 behaviour.

---

## 6. Phasing

| Phase | Scope | Effort |
| --- | --- | --- |
| **0 — Phase 0: x86 SDK verification** | Confirm MSFS 2024 SDK ships an **x86** `SimConnect.dll` + `Microsoft.FlightSimulator.SimConnect.dll`. Build a 100-line throwaway C# console app that opens SimConnect, registers one SimVar (AIRSPEED INDICATED), and prints values. Run inside an x86 host (csc /platform:x86). | 0.5 day |
| **1 — In-process client, dual-mode** | Add `MsfsSimConnectClient`. Wire it next to the existing UDP path. Settings: `MsfsSimConnectEnabled` (new, default true) wins; falls back to UDP receiver if disabled. Both populate `latestMsfsPacket`. Lets us A/B compare. | 2 days |
| **2 — Parity validation** | Fly a heli for ~5 minutes with both paths streaming. Log each `MsfsUdpPacket` produced by both, compare field-by-field, assert <0.01% per-field divergence (allowing for sample-time skew). Same `heli_unboosted_msfs.json` evaluation must produce identical force output to within float epsilon. | 1 day |
| **3 — Cut over** | Remove the UDP receiver code, the EXE spawn code, the `MsfsUdpEnabled`/`MsfsUdpPort` settings, the `DeployMsfsBridge` MSBuild target. Add post-build copy of `SimConnect.dll` (x86) only. | 0.5 day |
| **4 — Delete bridge** | `git rm -r MsfsPlugin/`. Update HANDOFF.md. Single commit titled `drop MsfsFfbDataProvider EXE`. | 0.5 day |

**Total: ~4.5 days.** Phases 0 and 1 are the only ones with material
unknowns; 2–4 are mechanical.

Phase 0 is non-negotiable. If the x86 SDK probe fails, the whole plan
needs to pivot (see §2 contingency) and budget will roughly double.

---

## 7. Open Questions

1. **x86 SimConnect.dll path in MSFS 2024 SDK.** Verify in phase 0.
   2020 SDK shipped `SimConnect SDK\lib\SimConnect.dll` as x64 and a
   `lib\x86\` subfolder for x86. 2024 SDK *should* mirror this but
   the public docs don't enumerate the lib layout explicitly.
   Resolution: install the 2024 SDK, list the lib directory tree.

2. **Managed wrapper version vs MSFS 2024 protocol.** The wrapper ships
   in `<SDK>\SimConnect SDK\lib\managed\Microsoft.FlightSimulator.SimConnect.dll`.
   It's tied to the SDK version. Using a wrapper from the 2020 SDK
   against MSFS 2024 may work (the protocol is largely backward-
   compatible) or may miss new APIs. Use the **2024 SDK's** wrapper
   to stay aligned with the runtime.

3. **Plugin DLL update with MSFS running.** Today the user kills SimHub
   → bridge EXE exits → DLL is unlocked. After this change, the
   plugin DLL holds `SimConnect.dll` loaded into SimHub's process
   for as long as SimHub runs. Replacing the plugin still works
   (SimHub copies the DLL on next launch), but live-replace via
   `xcopy` while SimHub is running will fail. This already isn't
   supported, so no behavioural change — note for the doc only.

4. **AppDomain unload during plugin reload.** SimHub plugin reloads
   should call `End()` which calls `Stop()` which calls
   `SimConnect.Dispose()` which `FreeLibrary`s the native handle.
   Test the SimHub "reload plugins" path explicitly — if it leaks
   the SimConnect handle, second connection attempt will fail.

5. **Threading model in the managed wrapper.** The wrapper's
   `ReceiveMessage()` blocks until the next message or returns immediately
   if none queued. We pair it with `WaitForSingleObject(EventHandle,
   timeout)` so the worker can periodically check a shutdown flag
   without busy-spinning. Confirm `EventHandle` is exposed by the
   2024-SDK wrapper (it was in 2020).

6. **Bridge logging migration.** Today the bridge logs to stdout
   (visible if launched manually) and `OutputDebugString` (visible
   via DebugView). After migration, all logs route through
   `SimHub.Logging.Current` → `SimHub.txt`. Lower the verbosity
   compared to the bridge's startup banner — SimHub.txt is shared
   with the rest of the plugin and the user.

7. **What about ATC MODEL / ATC TYPE auto-detection?** Plan 17 §10 Q7
   deferred per-aircraft constant lookup until phase 3 of plan 17.
   In-process SimConnect makes this trivially easier (just
   `RegisterDataDefineStruct` with `ATC TYPE` as a string SimVar and
   pipe through to `activeCarId`). Note as a follow-up; out of scope
   for this plan.

8. **Does anyone else need the EXE?** Search for references — if the
   bridge EXE is launched by any third-party script or documented
   externally, deleting it is a breaking change. Grep confirms it's
   only referenced from [DiyFfbPlugin.cs:1196-1253](SimHubPlugin/DiyFfbPlugin.cs#L1196-L1253)
   and [DiyFfbPlugin.csproj:467-481](SimHubPlugin/DiyFfbPlugin.csproj#L467-L481).
   Safe to delete.

---

## 8. Files Anticipated to Change

**Phase 1 (new + parallel path):**

* **New:** [SimHubPlugin/MsfsSimConnectClient.cs](SimHubPlugin/MsfsSimConnectClient.cs)
* **New:** `SimHubPlugin/lib/Microsoft.FlightSimulator.SimConnect.dll` (vendored)
* [SimHubPlugin/DiyFfbPlugin.cs](SimHubPlugin/DiyFfbPlugin.cs) —
  add `_msfsClient`, `StartMsfsClient`, `StopMsfsClient`; keep UDP
  receiver intact behind a settings flag
* [SimHubPlugin/DiyFfbPluginSettings.cs](SimHubPlugin/DiyFfbPluginSettings.cs) —
  add `MsfsSimConnectEnabled` (default true); leave `MsfsUdpEnabled`
* [SimHubPlugin/DiyFfbPlugin.csproj](SimHubPlugin/DiyFfbPlugin.csproj) —
  add `Reference` to the managed wrapper

**Phase 3 (cutover):**

* [SimHubPlugin/DiyFfbPlugin.cs](SimHubPlugin/DiyFfbPlugin.cs) — strip
  `~280 lines` (`StartMsfsBridgeProcess` / `StopMsfsBridgeProcess` /
  `StartMsfsUdpReceiver` / `StopMsfsUdpReceiver` / `MsfsUdpLoop` /
  `ParseMsfsPacket` / `_msfsBridgeProcess` / UDP fields / `MsfsPacket*`
  constants)
* [SimHubPlugin/DiyFfbPluginSettings.cs](SimHubPlugin/DiyFfbPluginSettings.cs) —
  drop `MsfsUdpEnabled`, `MsfsUdpPort`
* [SimHubPlugin/DiyFfbPlugin.csproj](SimHubPlugin/DiyFfbPlugin.csproj) —
  replace `DeployMsfsBridge` target with a `Copy SimConnect.dll` target

**Phase 4 (delete):**

* `git rm -r MsfsPlugin/`
* `git rm SimHubPlugin/bin/MsfsFfbDataProvider.exe SimHubPlugin/bin/MsfsFfbDataProvider.pdb`
  (currently tracked per `git status`)

---

## 9. Validation

**Parity (phase 2 gate).** Side-by-side capture of 5 minutes of MSFS
helicopter flight covering hover, ETL transition, cruise, autorotation
entry. Log every `MsfsUdpPacket` produced by both the EXE+UDP path and
the in-process client, joined by sequence number. Acceptance: for
each of the 35 float SimVars, per-sample absolute difference < 1e-5
(allows for double→float rounding noise; anything larger means a
SimVar definition diverged).

**Connection lifecycle (phase 3 gate).** Scripted:

1. SimHub start, MSFS not running → plugin logs "waiting for MSFS",
   no errors.
2. Launch MSFS → plugin logs "connected", `latestMsfsPacket`
   freshness becomes true within 5 seconds.
3. Quit MSFS → plugin logs "MSFS quit, reconnecting", freshness
   becomes false within 1 second.
4. Re-launch MSFS → plugin reconnects within 5 seconds.
5. SimHub stop while MSFS running → no errors in SimHub.txt; check
   for orphan handles via Process Explorer.

**Graph parity (phase 3 gate).** Same `heli_unboosted_msfs.json` graph,
same aircraft, same MSFS save — verify the live force-output value at
`outputForce_pitch` matches a recorded reference run from the EXE-based
build to within ±0.5% (the rest is sample-time jitter).

**Plugin reload (phase 3 gate).** Use SimHub's plugin manager to
disable then re-enable DiyFfbPlugin while MSFS is running. The
in-process client must re-establish connection cleanly. If it doesn't,
investigate `SimConnect.Dispose` + `FreeLibrary` ordering.

---

## 10. Rollback Plan

Each phase commits independently. If phase 2 parity validation fails,
revert phase 1 with a single `git revert <sha>` — the EXE path is
still wired and functional behind the settings flag through phases
1–2. Only phase 3 begins to delete the UDP code, and only phase 4
removes the bridge sources. If we need to roll back after phase 4,
the bridge sources are recoverable from git history (`git show`).
