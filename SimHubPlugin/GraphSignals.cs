using System;
using System.Collections.Generic;
using GameReaderCommon;

namespace DiyFfb
{
    public static class GraphSignalCatalog
    {
        public static readonly IReadOnlyList<string> InputNames = GraphSignalCatalogData.InputNames;
        public static readonly IReadOnlyList<string> OutputNames = GraphSignalCatalogData.OutputNames;
        public static readonly IReadOnlyList<string> InputGroups = GraphSignalCatalogData.InputGroups;
        public static readonly IReadOnlyList<string> OutputGroups = GraphSignalCatalogData.OutputGroups;
        public static readonly IReadOnlyList<string> ParamGroups = GraphSignalCatalogData.ParamGroups;

        public static IReadOnlyList<string> GetInputSignalsForGroup(string group) =>
            GraphSignalCatalogData.GetInputSignalsForGroup(group);

        public static IReadOnlyList<string> GetOutputSignalsForGroup(string group) =>
            GraphSignalCatalogData.GetOutputSignalsForGroup(group);

        public static bool IsValidInputSignal(string fullName) =>
            GraphSignalCatalogData.IsValidInputSignal(fullName);

        public static bool IsValidOutputSignal(string fullName) =>
            GraphSignalCatalogData.IsValidOutputSignal(fullName);

        public static string BuildSignalName(string group, string suffix) =>
            GraphSignalCatalogData.BuildSignalName(group, suffix);

        public static void BuildXPlaneInputs(DiyFfbPlugin plugin, GameData data, IDictionary<string, double> inputs)
        {
            if (plugin == null || inputs == null)
            {
                return;
            }

            var packet = plugin.GetLatestXPlanePacket();
            if (packet == null)
            {
                return;
            }

            int rotorIndex = plugin.ResolveXPlaneRotorIndexForGraph(packet);
            double torqueNm = 0.0;
            double omegaRad = 0.0;
            if (rotorIndex >= 0 && rotorIndex < packet.TorqueNm.Length)
            {
                torqueNm = packet.TorqueNm[rotorIndex];
                omegaRad = packet.OmegaRad[rotorIndex];
            }

            // Telemetry inputs using new hierarchical naming
            inputs["XPlane.Speed.IAS"] = packet.IasKts;
            inputs["XPlane.Angle.Alpha"] = packet.AlphaDeg;
            inputs["XPlane.Angle.Beta"] = packet.BetaDeg;
            inputs["XPlane.Trim.Elevator"] = packet.ElevTrimNorm;
            inputs["XPlane.Trim.Aileron"] = packet.AilTrimNorm;
            inputs["XPlane.Trim.Rudder"] = packet.RudTrimNorm;
            inputs["XPlane.Rate.Roll"] = packet.PRate;   // p = roll rate
            inputs["XPlane.Rate.Pitch"] = packet.QRate;  // q = pitch rate
            inputs["XPlane.Rate.Yaw"] = packet.RRate;    // r = yaw rate
            inputs["XPlane.G_Nrml"] = packet.GNrml;
            inputs["XPlane.AeroTorque.Roll"] = packet.LAero;
            inputs["XPlane.AeroTorque.Pitch"] = packet.MAero;
            inputs["XPlane.AeroTorque.Yaw"] = packet.NAero;
            inputs["XPlane.MainRotor.Torque"] = torqueNm;
            double rpm = DiyFfbPlugin.ToRpm((float)omegaRad);
            inputs["XPlane.MainRotor.Speed"] = rpm;
            // Rotor vibration signals (indexed by rotorIndex, same as torque/speed)
            inputs["XPlane.Rotor.BladeAlphPitch"] = rotorIndex >= 0 && rotorIndex < packet.CyclicElevBladAlph.Length ? packet.CyclicElevBladAlph[rotorIndex] : 0.0;
            inputs["XPlane.Rotor.BladeAlphRoll"]  = rotorIndex >= 0 && rotorIndex < packet.CyclicAilnBladAlph.Length ? packet.CyclicAilnBladAlph[rotorIndex] : 0.0;
            inputs["XPlane.Rotor.Slap"]           = rotorIndex >= 0 && rotorIndex < packet.RotorBladeSlapRat.Length ? packet.RotorBladeSlapRat[rotorIndex] : 0.0;
            inputs["XPlane.Rotor.VRS"]            = rotorIndex >= 0 && rotorIndex < packet.VortexRingState.Length ? packet.VortexRingState[rotorIndex] : 0.0;
            inputs["XPlane.Rotor.Propwash"]       = rotorIndex >= 0 && rotorIndex < packet.PropwashMtrSec.Length ? packet.PropwashMtrSec[rotorIndex] : 0.0;
            inputs["XPlane.OnGround"] = packet.OnGround ? 1.0 : 0.0;
        }

        // Plan 17/20: populate MSFS.* graph inputs from the latest UDP packet.
        // Pure passthrough — per plan 20, all rotor derivations (BladeAlph /
        // VRS / Slap / MainRotor.Torque) live in graph subgraph
        // _embedded/msfs_derivations.json. Propwash was unused; dropped.
        public static void BuildMsfsInputs(DiyFfbPlugin plugin, GameData data, IDictionary<string, double> inputs)
        {
            if (plugin == null || inputs == null)
            {
                return;
            }

            var packet = plugin.GetLatestMsfsPacket();
            if (packet == null)
            {
                return;
            }

            inputs["MSFS.Speed.IAS"] = packet.IasKts;
            inputs["MSFS.Speed.TAS"] = packet.TasKts;
            inputs["MSFS.Angle.Alpha"] = packet.AlphaDeg;
            inputs["MSFS.Angle.Beta"] = packet.BetaDeg;
            inputs["MSFS.Attitude.Pitch"] = packet.PitchRad;
            inputs["MSFS.Attitude.Bank"] = packet.BankRad;
            inputs["MSFS.Rate.Roll"] = packet.PRateRadS;
            inputs["MSFS.Rate.Pitch"] = packet.QRateRadS;
            inputs["MSFS.Rate.Yaw"] = packet.RRateRadS;
            inputs["MSFS.G_Nrml"] = packet.GForce;
            inputs["MSFS.VVI.World"] = packet.VviWorldFps;
            inputs["MSFS.Velocity.BodyX"] = packet.VelocityBodyXFps;
            inputs["MSFS.Velocity.BodyY"] = packet.VelocityBodyYFps;
            inputs["MSFS.Velocity.BodyZ"] = packet.VelocityBodyZFps;
            inputs["MSFS.GroundSpeed"] = packet.GroundVelocityKts;
            inputs["MSFS.Weight.Total"] = packet.TotalWeightLb;
            inputs["MSFS.Air.Density"] = packet.AmbientDensitySlugsFt3;
            inputs["MSFS.MainRotor.Speed"] = packet.MainRotorRpm;
            inputs["MSFS.TailRotor.Speed"] = packet.TailRotorRpm;
            inputs["MSFS.Eng.TorquePct"] = packet.EngTorquePct;
            inputs["MSFS.Collective.Position"] = packet.CollectivePosPct;
            inputs["MSFS.Collective.BladePitchPct"] = packet.RotorCollectiveBladePitchPct;
            inputs["MSFS.Cyclic.BladePitchPct"] = packet.RotorCyclicBladePitchPct;
            inputs["MSFS.Cyclic.MaxPitchAngle"] = packet.RotorCyclicBladeMaxPitchPosRad;
            inputs["MSFS.TailRotor.PedalPosition"] = packet.TailRotorPedalPct;
            inputs["MSFS.TailRotor.BladePitchPct"] = packet.TailRotorBladePitchPct;
            inputs["MSFS.Disk.PitchAngle"] = packet.DiskPitchAngleRad;
            inputs["MSFS.Disk.BankAngle"] = packet.DiskBankAngleRad;
            inputs["MSFS.Disk.ConingPct"] = packet.DiskConingPct;
            inputs["MSFS.Rotor.LateralTrim"] = packet.RotorLateralTrimPct;
            inputs["MSFS.Rotor.LongitudinalTrim"] = packet.RotorLongitudinalTrimPct;
            inputs["MSFS.Rotor.RotationAngle"] = packet.RotorRotationAngleRad;
            inputs["MSFS.Trim.Elevator"] = packet.ElevTrimPct;
            inputs["MSFS.Trim.Aileron"] = packet.AilTrimPct;
            inputs["MSFS.Trim.Rudder"] = packet.RudTrimPct;
            inputs["MSFS.OnGround"] = packet.OnGround ? 1.0 : 0.0;

            // Plan 23: graph-declared custom vars (MsfsVarDef), keyed by alias.
            if (packet.Custom != null)
            {
                foreach (var kv in packet.Custom)
                {
                    inputs["MSFS." + kv.Key] = kv.Value;
                }
            }
        }

        /// <summary>
        /// All logical grip signal names that can be bound to physical buttons.
        /// </summary>
        public static readonly IReadOnlyList<string> GripSignalNames = new[]
        {
            "Grip.TrimHat.Up",
            "Grip.TrimHat.Down",
            "Grip.TrimHat.Left",
            "Grip.TrimHat.Right",
            "Grip.ForceTrimRelease",
            "Grip.TrimReset"
        };

        // Plan 11: read grip-button signal values from a held-state dict populated by
        // SimHub AddInputMapping callbacks (inputPressed → true, inputReleased → false).
        // SimHub enforces "during" semantics on input mappings, so no heartbeat/timeout
        // heuristic is needed — the bool is authoritative.
        public static void BuildGripInputs(
            IReadOnlyDictionary<string, bool> heldState,
            IDictionary<string, double> inputs)
        {
            foreach (var signalName in GripSignalNames)
            {
                bool held = heldState != null
                            && heldState.TryGetValue(signalName, out var v)
                            && v;
                inputs[signalName] = held ? 1.0 : 0.0;
            }
        }

        public static void BuildAxisInputs(DiyFfbPlugin plugin, IDictionary<string, double> inputs)
        {
            if (plugin == null || inputs == null) return;
            // Position is in mm (contact point position from ESP32).
            // Center is (pos_min + pos_max) / 2 from the function's config.
            // Force is in N (load-cell measured force from AxisState). Single-axis
            // functions use the primary linked axis; the flight pedals span multiple
            // axes, so force is split by direction (subtractive=left, additive=right)
            // while position stays single (the two pedals are mirrored).
            inputs["Axis.FlightStickPitch.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickPitch);
            inputs["Axis.FlightStickPitch.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickPitch);
            inputs["Axis.FlightStickPitch.Force"] = plugin.GetFunctionForce(FunctionID.FlightStickPitch);
            inputs["Axis.FlightStickRoll.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickRoll);
            inputs["Axis.FlightStickRoll.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickRoll);
            inputs["Axis.FlightStickRoll.Force"] = plugin.GetFunctionForce(FunctionID.FlightStickRoll);
            inputs["Axis.FlightPedals.Position"] = plugin.GetFunctionPosition(FunctionID.FlightPedals);
            inputs["Axis.FlightPedals.Center"] = plugin.GetFunctionCenter(FunctionID.FlightPedals);
            inputs["Axis.FlightPedals.Left.Force"] = plugin.GetPedalForce(FunctionID.FlightPedals, left: true);
            inputs["Axis.FlightPedals.Right.Force"] = plugin.GetPedalForce(FunctionID.FlightPedals, left: false);
            inputs["Axis.FlightStickCollective.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickCollective);
            inputs["Axis.FlightStickCollective.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickCollective);
            inputs["Axis.FlightStickCollective.Force"] = plugin.GetFunctionForce(FunctionID.FlightStickCollective);
        }
    }
}
