# MSFS Bridge: Pure-C# SimConnect (drop EXE *and* SimConnect.dll)

Talk to MSFS 2024 directly from the SimHub plugin over the SimConnect
**named pipe**, using a hand-rolled C# implementation of the SimConnect
wire protocol. No `MsfsFfbDataProvider.exe`, no `SimConnect.dll`, no
managed wrapper — single-DLL deployment.

**Relationship to [Plan 18](18_msfs_native_simconnect.md):** plan 18
takes the same goal (drop the EXE) but keeps `SimConnect.dll` as an
unmanaged dependency via the official managed wrapper. This plan goes
further: eliminate the native DLL too. The two plans are mutually
exclusive — pick one. §3 of this doc defines the decision criteria.

**Prerequisite:** Plan 17 (the bridge ships raw SimVars, derivations
live in the plugin). Already shipped.

**Out of scope:** any SimConnect feature beyond read-only periodic
SimVar polling (no events, no set-data, no AI, no weather, no traffic).

---

## 1. What "Option C" Means Concretely

The SimConnect server inside MSFS exposes its client API over a Windows
named pipe at:

```text
\\.\pipe\Microsoft Flight Simulator\SimConnect
```

This is the same endpoint the official `SimConnect.dll` connects to.
The DLL is just a marshalling layer between a C-style API and a binary
RPC protocol over that pipe. **Nothing in the protocol requires the
DLL.** A pure managed client opens the pipe with
`NamedPipeClientStream`, speaks the wire format directly, and lifts the
35 `double` SimVars out of incoming response packets.

Result: the plugin DLL is the only file we ship. No EXE, no SDK
redistributable. Single artefact, no x86/x64 mismatch (managed code is
bitness-agnostic; we just talk bytes to a kernel pipe).

---

## 2. Why (vs Plan 18)

Plan 18 §1 already covers the EXE+UDP costs — re-read it for the
common motivation. Beyond that, going pure C# additionally buys:

* **One file to ship.** Plan 18 still requires `SimConnect.dll` next
  to the plugin DLL. Users who hand-copy the plugin without the DLL
  still hit a silent failure (`DllNotFoundException` at first MSFS
  connect). Pure C# has nothing to forget.
* **No x86/x64 trap.** Plan 18 depends on the MSFS 2024 SDK shipping
  an x86 `SimConnect.dll` (verified in plan 18 phase 0, but
  fundamentally a constraint we don't control). Pure C# sidesteps the
  whole question.
* **No SDK install gate for contributors.** Anyone building the plugin
  today needs `MSFS_SDK` pointing somewhere. Pure C# removes that
  build-time dependency entirely.
* **No `[DllImport]` lifetime issues.** Plan 18 §7 Q4 worries about
  AppDomain unload, `FreeLibrary`, and handle leaks across plugin
  reload. Pure C# has none of that — a `NamedPipeClientStream` is a
  managed resource that the GC + `Dispose` handle cleanly.

What we **give up** (and §3 weighs):

* **Undocumented wire format.** Microsoft has never published the
  SimConnect packet layout. Everything we know is reverse-engineered.
* **MSFS update risk.** Asobo can change the protocol in any sim
  update. Plan 18's wrapper is updated by Microsoft; our hand-rolled
  parser is updated by us.
* **Larger initial effort.** Plan 18 is a refactor (~4.5 days).
  This is an implementation (~10 days, see §7).

---

## 3. When to Pick This Over Plan 18

**Pick Plan 18 if** any of:

* You want this shipping fast (4.5 days vs 10).
* You're comfortable with one unmanaged DLL next to the plugin DLL.
* You expect to add features beyond read-only SimVars (events,
  set-data) in the next 6 months.

**Pick Plan 19 if** any of:

* "Single-file plugin" has product value beyond the engineering
  cost — e.g., distribution via SimHub's plugin marketplace where
  multi-file installs are friction.
* Plan 18 phase 0 reveals the MSFS 2024 SDK doesn't ship an x86
  `SimConnect.dll` and you don't want to keep the bridge EXE.
* You're willing to take on the maintenance burden of a hand-rolled
  protocol parser to remove an external dependency.

**Honest default:** Plan 18. The marginal value of "no native DLL" is
small unless deployment friction is a real user complaint, and we
haven't seen evidence it is. This plan exists so the option is
documented and the analysis is real, not as the recommended path.

---

## 4. Prior Art

| Project | Language | License | Pure (no DLL) | MSFS 2024 evidence | Status |
| --- | --- | --- | --- | --- | --- |
| [EvenAR/node-simconnect](https://github.com/EvenAR/node-simconnect) | TypeScript | LGPL-3.0 | Yes (net.Socket over pipe) | Yes — current `SunRise` build constant `(282174, 999)`, releases through 2026 | **Active** |
| [wegylexy/SimConnect](https://github.com/wegylexy/SimConnect) | C# (net5) | MIT | Yes (`NamedPipeClientStream`) | No — protocol list stops at FSX SE | Abandoned 2022 |
| [mharj/jsimconnect](https://github.com/mharj/jsimconnect) | Java | LGPL-2.1 | Yes | MSFS 2020 only | Stale 2021 |
| `lc0277/jsimconnect` (FlightGear / SourceForge) | Java | LGPL | Yes | FSX / P3D era | Historical reference |

**Most useful basis:** `node-simconnect`. It's the only actively-maintained
pure implementation with MSFS-2024-confirmed protocol constants. Read it
to **understand** the protocol; do not paste code from it (license — see
§9).

`wegylexy/SimConnect` is the closest C# starting point structurally
(`NamedPipeClientStream`, header layout, packet-builder pattern) but its
opcode tables and protocol-version constants are stuck at FSX. Treat it
as a code-shape reference: same idea, different constants.

---

## 5. Wire Protocol — What We Need To Know

Confirmed via the prior-art audit. Anything marked **(verify in phase 0)**
must be cross-checked against `node-simconnect` source before locking in
the C# implementation.

### 5.1 Transport

* **Named pipe** `\\.\pipe\Microsoft Flight Simulator\SimConnect`,
  bidirectional, byte-stream.
* Opened with
  `new NamedPipeClientStream(".", "Microsoft Flight Simulator\\SimConnect", PipeDirection.InOut, PipeOptions.Asynchronous)`.
* `ConnectAsync()` with a 2-second timeout; on
  `TimeoutException` / `IOException`, retry — same loop topology as the
  current EXE.
* MSFS 2024 also exposes IPv4 and IPv6 SimConnect endpoints; the
  registry-port dance from FSX/MSFS-2020 (`SimConnect_Port_IPv4`) is
  **deprecated** per 2024 docs. We use the pipe only.

### 5.2 Packet Header (16 bytes, little-endian)

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 4 | total packet size in bytes (header + body) |
| 4 | 4 | protocol version (use `4` for SunRise / MSFS 2024) |
| 8 | 4 | message id, OR'd with magic `0xF0000000` for outgoing |
| 12 | 4 | sequential send-id (client-side counter; echoed in responses for correlation) |

Body follows the header, message-specific.

Strings are **fixed-length null-padded ASCII** at protocol-defined
sizes (typically 256 for SimVar names and unit names — **verify in
phase 0**).

### 5.3 Messages We Implement

Outbound (we send):

| Logical message | Body |
| --- | --- |
| `Open` | `appName[256]` + protocol-version int + build-major (`282174`) + build-minor (`999`) + build-rev + reserved |
| `AddToDataDefinition` | `defID` (uint32) + `datumName[256]` + `unitsName[256]` + `datumType` (FLOAT64 = some int — **verify**) + `epsilon` (float32) + `datumID` (uint32, -1 = auto) |
| `RequestDataOnSimObject` | `requestID` (uint32) + `defID` + `objectID` (= 0, `SIMCONNECT_OBJECT_ID_USER`) + `period` (= 4, `PERIOD_SIM_FRAME`) + `flags` (= 0) + `origin` (= 0) + `interval` (= 0) + `limit` (= 0) |
| `ClearDataDefinition` | `defID` |
| (keepalive) | not required — MSFS does not require pings; the read loop just blocks |

Inbound (we receive):

| Message | What we do |
| --- | --- |
| `RECV_OPEN` | handshake ack — record server build numbers, log them, proceed |
| `RECV_SIMOBJECT_DATA` | the periodic sample — extract `requestID`, copy the 35-double payload into `SimVarSample`, hand off to the callback |
| `RECV_EXCEPTION` | log the exception code + `dwSendID` (which of our requests it refers to), surface to SimHub.txt at warning level |
| `RECV_QUIT` | MSFS shut down — close pipe, fall back to reconnect loop |

Everything else (`RECV_EVENT*`, `RECV_AIRPORT_LIST`, etc.) is discarded.

### 5.4 Opcode IDs

The numeric send-ids and recv-ids are **not** in the public docs.
`node-simconnect`'s `src/enums/SendID.ts` and `src/enums/RecvID.ts`
hold the authoritative reverse-engineered list. **Phase 1's first
task** is to copy those tables into our C# (verbatim integers, not
verbatim source — see §9 license posture).

### 5.5 MSFS 2024 Handshake

The Open packet must declare protocol `4` and build `(282174, 999)` to
get a `RECV_OPEN` ack from MSFS 2024 ("SunRise"). Older protocol values
work backward-compatibly for read-only SimVar polling, but advertising
the SunRise build is the safe path and matches what every current
client does.

If the server replies with `RECV_EXCEPTION` code `0x14`
(`VERSION_MISMATCH`), we lower the advertised version and retry once —
defensive only; not expected to trigger against released MSFS 2024.

---

## 6. Architecture

Two-layer split, isolating wire-protocol code from policy:

```text
┌─────────────────────────────────────────────────────────┐
│ DiyFfbPlugin (existing)                                 │
│   - latestMsfsPacket (POCO, unchanged)                  │
│   - GetLatestMsfsPacket / IsMsfsTelemetryFresh          │
│   - BuildMsfsInputs (unchanged)                         │
└──────────────────────▲──────────────────────────────────┘
                       │ Action<MsfsUdpPacket> onSample
┌──────────────────────┴──────────────────────────────────┐
│ MsfsSimConnectClient (new — policy layer)               │
│   - worker thread                                       │
│   - connect / handshake / register / request loop       │
│   - dispatch RECV_* → callbacks                         │
│   - reconnect on QUIT or pipe-broken                    │
└──────────────────────▲──────────────────────────────────┘
                       │ ISimConnectProtocol interface
┌──────────────────────┴──────────────────────────────────┐
│ SimConnectProtocol (new — pure wire format)             │
│   - PacketWriter, PacketReader                          │
│   - BuildOpenPacket / BuildAddDataDef / BuildRequest    │
│   - ParseRecvHeader / ParseSimObjectData / ParseException│
│   - Knows nothing about MSFS or SimHub                  │
└─────────────────────────────────────────────────────────┘
                       │ Stream
                  NamedPipeClientStream
```

### `SimConnectProtocol`

Stateless byte-pushing layer. Encodes outbound packets to a
`Span<byte>`, decodes inbound packet headers + bodies. Unit-testable
without a running MSFS — feed it canned byte buffers, assert it
produces the right struct.

Public surface:

```csharp
internal static class SimConnectProtocol
{
    public const int HeaderSize = 16;
    public const uint OutboundMagic = 0xF0000000;
    public const int ProtocolVersion = 4;
    public const int SunRiseBuildMajor = 282174;
    public const int SunRiseBuildMinor = 999;

    public static int WriteOpen(Span<byte> dst, string appName, uint sendId);
    public static int WriteAddDataDefinition(Span<byte> dst, uint defId,
        string datumName, string unitsName, int datumType, float epsilon,
        uint datumId, uint sendId);
    public static int WriteRequestDataOnSimObject(Span<byte> dst,
        uint requestId, uint defId, uint objectId, uint period,
        uint flags, uint sendId);

    public static bool TryReadHeader(ReadOnlySpan<byte> src,
        out int totalSize, out int protocol, out uint recvId, out uint sendId);
    public static bool TryReadSimObjectData(ReadOnlySpan<byte> body,
        out uint requestId, out uint objectId, out uint defineId,
        out int defineCount, out ReadOnlySpan<byte> payload);
    public static bool TryReadException(ReadOnlySpan<byte> body,
        out uint exceptionCode, out uint sendId, out uint index);
}
```

No I/O, no allocations beyond temporary byte buffers, no MSFS knowledge.

### `MsfsSimConnectClient`

The policy / lifecycle layer. Owns the pipe, the worker thread, and
the connect/reconnect state machine.

Public surface (identical to plan 18's `MsfsSimConnectClient`):

```csharp
internal sealed class MsfsSimConnectClient : IDisposable
{
    public MsfsSimConnectClient(Action<MsfsUdpPacket> onSample,
        Action<string> log);
    public void Start();
    public void Stop();
    public bool IsConnected { get; }
    public DateTime LastSampleUtc { get; }
}
```

Worker loop structure:

```text
while (running) {
  // 1. Connect
  if (!ConnectPipe()) { Sleep(2s); continue; }

  // 2. Handshake
  if (!SendOpen() || !AwaitRecvOpen(timeout: 3s)) {
    CloseAndRetry(); continue;
  }

  // 3. Register 35 SimVars
  for (int i = 0; i < kSimVars.Length; i++)
    SendAddToDataDefinition(defId: 1, kSimVars[i]);

  // 4. Subscribe at SIM_FRAME period
  SendRequestDataOnSimObject(requestId: 1, defId: 1,
                             objectId: 0, period: SIM_FRAME);

  // 5. Read loop
  while (running) {
    if (!ReadOnePacket(out var pkt)) break;  // pipe broken
    switch (pkt.RecvId) {
      case RECV_SIMOBJECT_DATA:
        ParseSampleAndPublish(pkt.Body);
        break;
      case RECV_QUIT:
        log("MSFS quit, reconnecting"); goto reconnect;
      case RECV_EXCEPTION:
        LogException(pkt.Body); break;
      default: break;  // ignore EVENT, AIRPORT_LIST, etc.
    }
  }
  reconnect:
  ClosePipe();
}
```

Worker is a single thread. The pipe is opened in **asynchronous** mode
so reads/writes can be cancelled cleanly via `Stop()` — without that,
`Stream.Read` blocks indefinitely waiting for the next packet.

### Threading & cancellation

* Worker thread (`IsBackground = true`).
* `CancellationTokenSource` plumbed into `ReadAsync(..., ct).GetAwaiter().GetResult()` — `.NET Framework 4.8` doesn't have `Stream.ReadAsync(Memory<byte>)`, but `Stream.ReadAsync(byte[], int, int, CancellationToken)` exists since 4.5.
* `Stop()`: signal CTS, dispose pipe (causes the in-flight read to throw `OperationCanceledException`), `Join(2s)`.

### Sample publishing

`SimVarSample` is a `[StructLayout(LayoutKind.Sequential, Pack = 1)]`
struct of 35 doubles — copied straight out of the SIMOBJECT_DATA
payload with `MemoryMarshal.Read<SimVarSample>(payload)`. Then the
worker constructs an `MsfsUdpPacket` (the existing POCO) and invokes
the `onSample` callback.

The POCO retains its name (`MsfsUdpPacket`) for diff minimality —
renaming to `MsfsTelemetrySnapshot` is optional churn, defer.

---

## 7. Implementation Phasing

| Phase | Scope | Effort |
| --- | --- | --- |
| **0 — Spike** | 200-line C# console app: open pipe, hand-roll Open packet for build `(282174, 999)`, receive + dump `RECV_OPEN` ack, exit. Validates transport assumption and 16-byte header layout against live MSFS 2024. | 1 day |
| **1 — Opcode table extraction** | Read `node-simconnect` enums (`SendID.ts`, `RecvID.ts`, `SimConnectException.ts`, `SimConnectPeriod.ts`, `SimConnectDataType.ts`); reproduce as C# `internal enum` declarations with the integer values. Clean-room: read the table → write the numbers, do not copy source. | 0.5 day |
| **2 — Protocol layer** | Implement `SimConnectProtocol` (§6). Unit tests with canned byte buffers — open packet for known input must match a byte sequence captured from the official client; same for AddDataDefinition, RequestData. Capture the reference bytes by running the EXE bridge under Wireshark-on-named-pipes or via API hooking; alternative: cross-check against `node-simconnect`'s built byte arrays for the same inputs. | 2 days |
| **3 — Client layer** | Implement `MsfsSimConnectClient` (§6). Worker thread, connect/handshake/register/read loop, reconnect on QUIT. Wire to a temporary CLI harness that prints each `RECV_SIMOBJECT_DATA` sample. | 2 days |
| **4 — Plugin integration** | Same shape as plan 18 phase 1: drop the new client in alongside the existing UDP path, gated by `MsfsSimConnectMode` setting (`Bridge` / `InProcess`). Both populate `latestMsfsPacket`. | 1 day |
| **5 — Parity validation** | Same as plan 18 phase 2: 5-minute side-by-side flight, log every `MsfsUdpPacket` from both paths, assert <1e-5 per-field divergence. Catches any opcode/struct-layout error. | 1 day |
| **6 — Cutover & delete** | Same as plan 18 phases 3+4: remove UDP code, remove EXE source, remove `SimConnect.dll` post-build copy. | 1 day |
| **7 — Hardening** | Run for ≥1 week of mixed flight (fixed-wing, heli, hover, cruise, sim quit/restart, MSFS update if one drops mid-window). Fix anything that breaks. | ongoing |

**Total dedicated work: ~8.5 days, plus hardening.** Roughly 2× plan
18's cost — and that's optimistic if the protocol turns out to differ
from the prior-art notes in places we didn't anticipate.

Phase 0 is the cheap go/no-go. If we can't get a `RECV_OPEN` ack in
one day, the protocol assumptions are wrong and the whole plan needs
to pivot to plan 18.

---

## 8. Risks

### 8.1 Protocol drift (high)

MSFS sim updates can change the wire format. The history is mixed: the
core packet layout has been stable since FSX (2008), but Asobo has
added new message types and adjusted struct sizes (e.g., the
`SIMCONNECT_ICAO` ident field grew in 2024). For our minimal API
surface, the risk is bounded — 35 doubles × `RECV_SIMOBJECT_DATA` is
about the most boring message in the protocol — but bounded ≠ zero.

**Mitigation:** keep the EXE bridge sources in git history. If a
future MSFS sim update breaks our parser, the fallback is a one-day
revert to plan 18's approach. The bridge code is small enough that
maintaining a dormant copy is cheap.

### 8.2 Opcode IDs not in our research (medium)

The agent that surveyed prior art could not enumerate the exact integer
values for every `SendID` / `RecvID` we need. We're relying on phase 1
to extract them from `node-simconnect` source. If `node-simconnect`'s
table has gaps for what we need (unlikely — they do read-only
SimVar polling too), we fall back to byte-diffing captures of the
official `SimConnect.dll`'s output for our exact calls.

**Mitigation:** phase 0 spike must validate at least the Open + recv-ack
opcodes against a live MSFS. If those work, the rest follows the same
pattern.

### 8.3 Named-pipe access on hardened MSFS installs (low)

MSFS Marketplace builds and Game Pass builds run under different
process tokens. The named pipe should be accessible to any local user
session, but verify against both Steam and MS Store builds in phase 5.

### 8.4 Performance under sustained load (low)

SimConnect at SIM_FRAME (~50 Hz) with 35 doubles is 56-byte payloads at
50 Hz — 2.8 KB/s. Trivial. The `Stream.Read` path with a pre-allocated
buffer plus `MemoryMarshal.Read<SimVarSample>` is allocation-free per
sample. Confirm with `dotnet-counters` in phase 5; expect zero GC
pressure attributable to MSFS.

### 8.5 Plugin-reload handle leak (low, vs plan 18 medium)

SimHub's "reload plugins" path disposes the plugin instance. We dispose
the `NamedPipeClientStream`, which releases the kernel pipe handle —
no `FreeLibrary` involved, no AppDomain unload subtleties. Lower risk
than plan 18 here.

---

## 9. License Posture (Important)

`node-simconnect` is **LGPL-3.0**. `jsimconnect` is **LGPL-2.1**.
`wegylexy/SimConnect` is MIT but doesn't cover MSFS 2024. The DIY
project root has no explicit `LICENSE` file at the time of writing,
but the SimHub plugin links against MIT-licensed dependencies and
should remain compatible with whatever the project ultimately
declares.

**Clean-room rule:** read the LGPL sources to understand the protocol
— integer values, packet field order, opcode tables, handshake
mechanics — and **reproduce them from that understanding** in our own
C# code. Do **not** copy LGPL source verbatim into our tree, nor
machine-translate it. Document in commit messages that the protocol
constants were derived from public reverse-engineering literature and
cross-checked against `node-simconnect` and `wegylexy/SimConnect`
behaviour. (Cite them; don't copy them.)

This is the standard approach for reimplementing reverse-engineered
protocols. The protocol facts themselves are not copyrightable; only
the specific expression in source code is.

If a project-wide LICENSE decision changes to GPL/LGPL anyway, the
constraint goes away — but the work is the same either way, so just
do the clean-room port.

---

## 10. Files Anticipated to Change

**New:**

* [SimHubPlugin/Msfs/SimConnectProtocol.cs](SimHubPlugin/Msfs/SimConnectProtocol.cs) — wire format
* [SimHubPlugin/Msfs/MsfsSimConnectClient.cs](SimHubPlugin/Msfs/MsfsSimConnectClient.cs) — policy / lifecycle
* `SimHubPlugin/Msfs/SimConnectEnums.cs` — extracted opcode + period + datatype + exception tables (~30 small enums)
* `SimHubPlugin/Tests/SimConnectProtocolTests.cs` — unit tests against canned byte buffers (if we have a test project; if not, a `#if DEBUG` self-check method)

**Modified (phase 4):**

* [SimHubPlugin/DiyFfbPlugin.cs](SimHubPlugin/DiyFfbPlugin.cs) — replace `StartMsfsBridgeProcess` / `StartMsfsUdpReceiver` call sites with `StartMsfsClient`. Keep UDP receiver intact behind setting flag during phases 4–5.
* [SimHubPlugin/DiyFfbPluginSettings.cs](SimHubPlugin/DiyFfbPluginSettings.cs) — add `MsfsSimConnectMode` enum (`Bridge` / `InProcess`), default `InProcess`.

**Deleted (phase 6):**

* The entire UDP receiver block in [DiyFfbPlugin.cs:1192-1418](SimHubPlugin/DiyFfbPlugin.cs#L1192-L1418).
* `MsfsPacket*` constants.
* `MsfsUdpEnabled` / `MsfsUdpPort` settings.
* The `DeployMsfsBridge` MSBuild target in [DiyFfbPlugin.csproj:467-481](SimHubPlugin/DiyFfbPlugin.csproj#L467-L481).
* `git rm -r MsfsPlugin/`.
* `git rm SimHubPlugin/bin/MsfsFfbDataProvider.exe SimHubPlugin/bin/MsfsFfbDataProvider.pdb`.

Crucially: **no new `SimConnect.dll` post-build copy.** That's the
whole point.

---

## 11. Validation

Same gates as plan 18 §9, with these additions specific to this plan:

* **Open handshake spike (phase 0 gate).** Throwaway console app
  connects, sends Open with SunRise build, receives `RECV_OPEN` from
  MSFS 2024 within 3 seconds. If it doesn't: stop, switch to plan 18.
* **Protocol unit tests (phase 2 gate).** ≥1 round-trip test per
  message type. Feed `SimConnectProtocol.WriteOpen(...)` known inputs,
  assert the byte array matches a captured reference. Same for
  AddDataDefinition and RequestData. The reference bytes come from
  either (a) running the existing EXE bridge while capturing its pipe
  output, or (b) constructing the equivalent input through
  `node-simconnect` in Node and dumping the buffer it sends.
* **Long-run stability (phase 7 gate).** A continuous ≥1-hour MSFS
  flight session with the in-process client must produce zero dropped
  samples (sequence gaps), zero pipe-broken reconnects (unless the
  user actively quit MSFS), and zero unhandled exceptions in
  SimHub.txt.

---

## 12. Rollback / Kill Switch

* Phases 4–5 keep the UDP path wired behind the `MsfsSimConnectMode`
  setting. Single config flip restores the EXE bridge.
* Phase 6 deletes the UDP path. After phase 6, rollback is `git revert`.
* The bridge sources stay in git history forever — they're recoverable
  via `git show <sha>:MsfsPlugin/MsfsFfbDataProvider.cpp` if ever
  needed.
* In the worst case (sim update breaks our parser, user reports
  in-flight FFB loss), the short-term mitigation is a hotfix flipping
  the default `MsfsSimConnectMode` back to `Bridge`. That requires
  shipping a new plugin DLL but no new bridge build — the bridge is
  already in everyone's install from the pre-phase-6 release.

This is why phase 6 (deleting the bridge sources) must wait until
phase 7 hardening clears at least 1 week of mixed-aircraft real-world
use. Don't burn the lifeboat until you're sure the new ship floats.

---

## 13. Open Questions

1. **Exact opcode integer values.** Plan deferred to phase 1, extracted
   from `node-simconnect` source. If those don't cover all our recv-IDs,
   we capture from the official client by hooking `WriteFile` on the
   pipe handle in a debug build of the EXE bridge — the captured bytes
   tell us everything.

2. **`SIMCONNECT_DATATYPE_FLOAT64` numeric value.** Stated as "some int"
   in §5.3 — verify the actual enum value before phase 2 tests will
   pass.

3. **String field sizes.** SimVar name and units name field widths are
   widely cited as 256 chars each but should be confirmed against
   `node-simconnect`'s `SimConnectPacketBuilder.ts` before phase 2.

4. **Pipe-creation race.** If the plugin starts before MSFS, the pipe
   doesn't exist yet. `NamedPipeClientStream.Connect(timeout)` should
   handle this cleanly (returns `TimeoutException`), but verify against
   `IOException` ERROR_FILE_NOT_FOUND on first attempt. Wrap both into
   the retry loop.

5. **Multiple simultaneous clients.** MSFS supports many SimConnect
   clients in parallel; we just need to confirm that opening the named
   pipe doesn't lock anyone else out. Should be the case (MSFS uses
   `PIPE_UNLIMITED_INSTANCES`) — but verify phase 0.

6. **Future SimVar additions.** If plan 17 phase 3 / plan 18 §7 Q7
   ever wants to read `ATC TYPE` (a string SimVar), the protocol layer
   needs to grow string-payload support in SIMOBJECT_DATA. Not in
   scope for this plan — note as a follow-up.

7. **Test harness.** No existing C# unit test project in
   SimHubPlugin/. Phase 2's unit tests either bootstrap one (adds a
   small csproj + xUnit + a CI runner) or use a `#if DEBUG` static
   constructor that runs the assertions on plugin load and writes
   results to SimHub.txt. Decide before phase 2 — the bootstrap is a
   small chunk of yak-shaving but pays back across the project.
