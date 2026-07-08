# Plan 11: Migrate Grip Button Bindings from DirectInput to SimHub Input Mappings

> **Implementation note (post-merge):** During implementation we discovered
> SimHub exposes `AddInputMapping(name, inputPressed, inputReleased)` —
> a separate primitive purpose-built for held-state inputs, with explicit
> press/release callbacks. The original plan below proposed `AddAction` +
> `During` mode + a heartbeat/timeout heuristic; the shipped code uses
> `AddInputMapping` instead, which is simpler (no heartbeat, no Input-mode
> selection by the user) and semantically exact. The action callback args
> are `(SimHub.Plugins.PluginManager, string)` — not used by our handlers.
> See [DiyFfbPlugin.cs §grip-binding init](../../SimHubPlugin/DiyFfbPlugin.cs).

## Goal

Replace the custom DirectInput-based grip-button binding system with native
SimHub actions. The graph-input shape stays unchanged (`Grip.TrimHat.Up` etc.
write 0.0 / 1.0 into the input dict); only the input-acquisition path
swaps.

## Motivation

Today the plugin runs a parallel input stack:

- [ButtonInputReader.cs](../../SimHubPlugin/ButtonInputReader.cs) — DirectInput
  device enumeration + 60 Hz polling (323 LOC)
- [Controls/GripBindingControl.xaml(.cs)](../../SimHubPlugin/Controls/GripBindingControl.xaml.cs)
  — press-to-bind UI with a 50 ms timer loop (182 LOC)
- [ButtonBinding.cs](../../SimHubPlugin/ButtonBinding.cs) — persisted binding
  shape (joystick / keyboard / hat) (46 LOC)
- `DiyFfbPluginSettings.GripButtonBindings` (Dictionary<string, ButtonBinding>)
  — settings storage

SimHub provides every one of those natively via `AddAction`:

- Device enumeration: SimHub already discovers joysticks, keyboards, MIDI,
  Stream Deck, HID-generic, telemetry triggers.
- Press-to-bind: SimHub's standard control-binding panel.
- Storage: SimHub's controls config, transferable across SimHub profiles.
- Polling: the **`During`** input mode invokes the registered action callback
  while the button is held, at SimHub's tick rate (~60 Hz) — semantically
  identical to the plugin's current `Poll()`-based read.

Net delete: ~530 LOC, plus the `SharpDX.DirectInput` reference at
[DiyFfbPlugin.csproj:164](../../SimHubPlugin/DiyFfbPlugin.csproj#L164) which
no other code path uses.

## How `During` mode replaces the polling loop

**Today** (60 Hz):

```text
plugin tick
  → _buttonInputReader.Poll()        (refresh device state cache)
  → BuildGripInputs(reader, bindings, inputs)
       for each signal:
         inputs[signal] = reader.IsPressed(bindings[signal]) ? 1.0 : 0.0
```

**Tomorrow** (60 Hz):

```text
SimHub fires Grip.TrimHat.Up action  (while user holds the bound button)
  → _gripHeartbeats["Grip.TrimHat.Up"] = DateTime.UtcNow

plugin tick
  → BuildGripInputs(_gripHeartbeats, now, inputs)
       for each signal:
         held = (now - heartbeats[signal]).TotalMs < kHeldTimeoutMs
         inputs[signal] = held ? 1.0 : 0.0
```

The graph reads the same boolean shape; downstream nodes are untouched.
One-tick release latency (≤ 50 ms) — invisible for trim.

## Phase 1: Action registration

In [DiyFfbPlugin.Init()](../../SimHubPlugin/DiyFfbPlugin.cs), alongside the
existing `FlightControl.SafetyDamperToggle` action registration, add a
loop:

```csharp
foreach (var signal in GraphSignalCatalog.GripSignalNames)
{
    string capturedSignal = signal;  // capture for closure
    this.AddAction(signal, (a, b) =>
    {
        _gripHeartbeats[capturedSignal] = DateTime.UtcNow;
    });
}
```

Action names match the graph signal names verbatim — dots are permitted
(confirmed by the live `FlightControl.SafetyDamperToggle` registration at
[DiyFfbPlugin.cs:3815](../../SimHubPlugin/DiyFfbPlugin.cs#L3815)). Six new
actions registered:

| Action name             | Recommended input mode   | Rationale                                         |
|-------------------------|--------------------------|---------------------------------------------------|
| `Grip.TrimHat.Up`       | `During`                 | Held-state for trim integration                   |
| `Grip.TrimHat.Down`     | `During`                 | Held-state                                        |
| `Grip.TrimHat.Left`     | `During`                 | Held-state                                        |
| `Grip.TrimHat.Right`    | `During`                 | Held-state                                        |
| `Grip.ForceTrimRelease` | `During`                 | Held while releasing trim                         |
| `Grip.TrimReset`        | `Pressed` / `ShortPress` | One-shot edge; graph rising-edge node consumes it |

`TrimReset` works fine through the same heartbeat mechanism: `Pressed` mode
fires once → heartbeat set → BuildGripInputs returns 1.0 for one tick →
0.0 thereafter. Graph's edge-detect node (or whoever consumes
`Grip.TrimReset`) sees a single rising edge — same as today's behavior on
a button tap.

## Phase 2: Heartbeat-based BuildGripInputs

Add to `DiyFfbPlugin`:

```csharp
private readonly Dictionary<string, DateTime> _gripHeartbeats =
    new Dictionary<string, DateTime>();
private const int GripHeldTimeoutMs = 50;
```

Refactor [GraphSignalCatalog.BuildGripInputs](../../SimHubPlugin/GraphSignals.cs#L92):

```csharp
public static void BuildGripInputs(
    IReadOnlyDictionary<string, DateTime> heartbeats,
    DateTime now,
    IDictionary<string, double> inputs)
{
    foreach (var signalName in GripSignalNames)
    {
        double value = 0.0;
        if (heartbeats != null
            && heartbeats.TryGetValue(signalName, out var lastFired)
            && (now - lastFired).TotalMilliseconds < 50)  // GripHeldTimeoutMs
        {
            value = 1.0;
        }
        inputs[signalName] = value;
    }
}
```

Update the two call sites:

- [DiyFfbPlugin.cs:2319-2327](../../SimHubPlugin/DiyFfbPlugin.cs#L2319-L2327) `BuildGraphInputs(GameData)` — drop the `Poll()` and `SetActiveBindings()` calls; pass `_gripHeartbeats, DateTime.UtcNow, graphInputs`.
- [DiyFfbPlugin.cs:2329-2342](../../SimHubPlugin/DiyFfbPlugin.cs#L2329-L2342) `GetLiveGraphInputs()` — same refactor.

## Phase 3: Remove DirectInput infrastructure

Delete:

- [ButtonInputReader.cs](../../SimHubPlugin/ButtonInputReader.cs) (323 LOC)
- [Controls/GripBindingControl.xaml](../../SimHubPlugin/Controls/GripBindingControl.xaml) +
  [.xaml.cs](../../SimHubPlugin/Controls/GripBindingControl.xaml.cs) (182 LOC)
- [ButtonBinding.cs](../../SimHubPlugin/ButtonBinding.cs) — defer one release;
  see Phase 5.

Remove from `DiyFfbPlugin.cs`:

- `_buttonInputReader` field at [line 130](../../SimHubPlugin/DiyFfbPlugin.cs#L130) +
  internal accessor at [line 132](../../SimHubPlugin/DiyFfbPlugin.cs#L132)
- Init at [line 3631](../../SimHubPlugin/DiyFfbPlugin.cs#L3631) (`new ButtonInputReader()`)
- Dispose at [line 872-873](../../SimHubPlugin/DiyFfbPlugin.cs#L872-L873)
- Call sites at [lines 2323-2325](../../SimHubPlugin/DiyFfbPlugin.cs#L2323-L2325) +
  [2337](../../SimHubPlugin/DiyFfbPlugin.cs#L2337)

Remove from `DiyFfbPlugin.csproj`:

- `<Reference Include="SharpDX.DirectInput">` at [line 164](../../SimHubPlugin/DiyFfbPlugin.csproj#L164).
  `SharpDX` (parent at line 161) — keep only if other code uses it; survey
  shows nothing else does, so likely droppable too.
- `<Compile Include="ButtonInputReader.cs" />` and the `GripBindingControl`
  Page entry.

## Phase 4: UI replacement

Replace the `<diycontrols:GripBindingControl x:Name="uc_grip_binding" .../>`
at [DiyFfbPluginUI.xaml:948](../../SimHubPlugin/DiyFfbPluginUI.xaml#L948) with
a static help panel:

```text
Grip controls now bind through SimHub's standard Controls panel.

Search SimHub Controls for actions starting with "Grip.":
  • Grip.TrimHat.Up / Down / Left / Right  → set Input mode to During
  • Grip.ForceTrimRelease                  → set Input mode to During
  • Grip.TrimReset                         → set Input mode to Pressed (or ShortPress)

The "During" input mode is required for the trim-hat directions because
the firmware integrates trim while you hold the button. "Pressed" alone
will only register a single tick — the trim won't move.
```

Keep the panel small; one tab in the existing config UI is fine. No code
behind beyond static text. The signal names and recommended modes belong
in this UI text so users don't need the plan doc.

## Phase 5: Settings migration / rebind requirement

`DiyFfbPluginSettings.GripButtonBindings` cannot be programmatically
ported to SimHub — different binding format, different storage, different
device-identification scheme. **Existing users must rebind once.**

Handling tiers:

1. **Mark the field obsolete, keep it deserializable.** Tag with
   `[Obsolete("Replaced by SimHub control bindings — see plan 11")]`.
   Settings.json files from older versions still load (the field
   deserializes into a now-unused dict). One minor release later, remove
   the field and `ButtonBinding.cs`.
2. **One-time on-load notification.** On plugin Init, if
   `Settings.GripButtonBindings` is non-empty, log a one-time SimHub log
   message: *"DiyFfb: grip bindings from a previous version were found.
   Rebind via SimHub Controls — see the Help tab."* Optionally show a
   non-modal toast if SimHub exposes that API.
3. **Release-note callout.** Prominent line in the release notes /
   `HANDOFF.md` upgrade section.

## Phase 6: Documentation

- Update `Plugin_Design.md` grip-binding section to reference SimHub
  actions instead of `ButtonInputReader`.
- Update plan 08 / plan 09 (vibration plans) if they reference the grip
  signal acquisition path.
- Add a short upgrade note: "Plan 11 — rebind grip controls in SimHub."

## Open questions

1. **SimHub action-name format.** Verify whether '.' is permitted in
   `AddAction` names. If yes, register as `Grip.TrimHat.Up`; if no, use
   the underscore form. Either way, the *graph input name* stays
   `Grip.TrimHat.Up` — the dict key is the graph signal, not the action
   name.
2. **`During` tick rate.** Confirm SimHub's `During` cadence on a real
   binding. If meaningfully slower than 60 Hz, the 50 ms timeout may
   need adjustment (or graph-side smoothing for the integration node).
   Easy sanity test: bind in `During`, log `(now - lastFired)` in the
   action callback, observe.
3. **Action enumeration in SimHub UI.** Confirm registered actions show
   up under a "DiyFfb" category; if SimHub flat-lists them, the
   `Grip_*` prefix at least groups them alphabetically.
4. **Action callback signature.** Existing code uses `(a, b) => { ... }`
   for the `SafetyDamperToggle` action. Confirm the parameter types and
   whether SimHub passes any context (e.g. press/release event metadata)
   that we could use to skip the heartbeat heuristic.

## Risks

1. **Forced rebind on upgrade.** Every existing user must rebind grip
   controls once after upgrading. No automated migration is feasible.
   Mitigation: release-note prominence + on-load log message (Phase 5
   tier 2).
2. **`During` mode discoverability.** Users may default-bind without
   selecting `During` and find trim-hat doesn't accumulate. Mitigation:
   the Phase 4 help text lists the correct mode per action explicitly.
3. **Action-name collisions across plugins.** Unlikely (the `Grip_`
   prefix is plugin-specific) but worth a smoke test in a SimHub
   instance with several plugins active.
4. **Loss of in-plugin device debugging.** Today the binding control's
   "Bind..." flow surfaces device-detection issues (no joystick acquired,
   key code not detected). After migration, those become SimHub's
   problem — generally a better place for them, but troubleshooting
   moves to a different surface area for the user.

## Implementation order

The cleanest path is a single PR rather than parallel-coexistence,
because Phase 1+2 (heartbeat) and Phase 3 (delete reader) read from
different sources — running both would require a feature flag and
double the test surface.

1. **Phase 1+2 in one commit**: register actions, refactor
   `BuildGripInputs`, wire `_gripHeartbeats`. Plugin still includes the
   old `ButtonInputReader` files but they're no longer called — easy
   bisect target if anything regresses.
2. **Smoke test on real hardware**: bind a button via SimHub Controls
   in `During` mode, verify trim-hat held semantics match the previous
   build.
3. **Phase 3 in a follow-up commit**: delete `ButtonInputReader`,
   `GripBindingControl`, the SharpDX.DirectInput reference, and the
   `_buttonInputReader` field/init/dispose. Plus csproj `<Compile>` and
   `<Reference>` removal.
4. **Phase 4 commit**: swap `GripBindingControl` for the help panel in
   `DiyFfbPluginUI.xaml`.
5. **Phase 5 commit**: mark `GripButtonBindings` `[Obsolete]`, add
   on-load log warning when non-empty.
6. **Phase 6 commit**: doc updates.

Total estimated change: **−530 / +80 LOC** plus one csproj reference
removed.

## What stays unchanged

- `GraphSignalCatalog.GripSignalNames` — same six signal names, same
  graph-input dict keys.
- All graph templates and saved graphs that reference `Grip.TrimHat.Up`
  etc. — no template migration needed.
- The downstream trim-integration logic in graph nodes — still reads
  the same booleans from the same input names.
- `FlightControlConfig.trim_offset` wire format — unchanged.
- The `FlightFfbAction.trim_offset` per-frame field — unchanged.
