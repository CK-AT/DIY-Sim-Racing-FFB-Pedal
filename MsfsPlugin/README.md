# MSFS SimConnect Bridge

Standalone EXE that connects to MSFS 2024 via SimConnect, samples helicopter
SimVars, and streams a 152-byte UDP packet at sim-frame rate (~50 Hz) to the
SimHub plugin on `127.0.0.1:27016`. **Raw SimVars only** — all derivations
(BladeAlph / VRS / Slap / Propwash / Torque) live in the SimHub plugin
(`GraphSignals.BuildMsfsInputs`) so per-aircraft tuning happens through the
existing graph-param mechanism without rebuilding C++.

See [docs/plans/17_msfs2024_helicopter_signals.md](../docs/plans/17_msfs2024_helicopter_signals.md)
for the full design.

## Build

Prereqs:

- Visual Studio 2022/2025 with the **Desktop C++** workload.
- **MSFS 2024 SDK** (or 2020). Standard install location:
  `C:\MSFS 2024 SDK\`. The project reads the `MSFS_SDK` environment
  variable; if unset, it falls back to `C:\MSFS 2024 SDK\`. Override
  with `msbuild /p:MsfsSdkRoot="...\"` if your SDK lives elsewhere.

Build via IDE: open `MsfsFfbDataProvider.sln`, select `Release|x64`, build.

Build via command line:

```powershell
msbuild MsfsFfbDataProvider.sln /p:Configuration=Release /p:Platform=x64
```

Output: `Release\MsfsFfbDataProvider.exe`.

## Run

1. Start MSFS 2024 and load into a flight (any aircraft — heli or fixed-wing).
2. Run `MsfsFfbDataProvider.exe`. The console will print connection status.
   If MSFS isn't running, the bridge retries every 2 s.
3. In SimHub, the DiyFfb plugin's MSFS UDP receiver listens on port 27016
   by default. Settings: `MsfsUdpEnabled`, `MsfsUdpPort` in plugin settings.

## Config file (optional)

Create `MsfsFfbDataProvider.cfg` next to the EXE to override the UDP target:

```ini
host=127.0.0.1
port=27016
```

Per-aircraft derivation tuning (rotor tip speed, ETL speed, slap onset,
max torque, etc.) lives in the SimHub plugin as graph params under the
`Aircraft.*` group — tune through the graph param UI per aircraft profile,
not here.

## Packet format

152-byte little-endian binary packet, raw SimVars only. See
`MsfsFfbDataProvider.cpp` `MsfsFfbPacket` struct and
`SimHubPlugin/DiyFfbPlugin.cs` `MsfsUdpPacket` / `ParseMsfsPacket`
for the contract.

Magic word `0x4D464642` ("MFFB"), version `1`.
