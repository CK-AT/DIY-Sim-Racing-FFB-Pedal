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

        // Plan 17: populate MSFS.* graph inputs from the latest UDP packet.
        // Mirror of BuildXPlaneInputs but ALSO computes the tier-C derived
        // signals (BladeAlph / VRS / Slap / Propwash / Torque) from raw
        // SimVars, using per-aircraft tuning constants read from graph params
        // (so persistence falls out of the existing GraphParamValues path).
        // Defaults are MD 500E-class (plan 17 §3, plan 04 §3 reference table).
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

            // Raw signals — straight copy.
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

            // Tier-C derivations (plan 17 §3). Per-aircraft tuning constants
            // read from graph params (Aircraft.* group); defaults are
            // MD 500E-class, calibrated against plan 04 §3 reference table.
            double rotorTipSpeedKts = plugin.GetGraphParamValue("Aircraft.RotorTipSpeedKts", 290.0);
            double kSpeedPitch       = plugin.GetGraphParamValue("Aircraft.KSpeedPitch", 17.0);
            double kSpeedRoll        = plugin.GetGraphParamValue("Aircraft.KSpeedRoll", 5.0);
            double etlSpeedKts       = plugin.GetGraphParamValue("Aircraft.EtlSpeedKts", 60.0);
            double descentMaxFpm     = plugin.GetGraphParamValue("Aircraft.DescentMaxFpm", 1500.0);
            double slapOnsetKts      = plugin.GetGraphParamValue("Aircraft.SlapOnsetKts", 20.0);
            double kSlap             = plugin.GetGraphParamValue("Aircraft.KSlap", 0.03);
            double slapMax           = plugin.GetGraphParamValue("Aircraft.SlapMax", 0.1);
            double discAreaM2        = plugin.GetGraphParamValue("Aircraft.DiscAreaM2", 49.0);
            double maxTorqueNm       = plugin.GetGraphParamValue("Aircraft.MaxTorqueNm", 800.0);

            // §3.1 BladeAlph — tier C (IAS only). Sign matches X-Plane: negative
            // in forward flight, scaling -k_speed_pitch * mu (mu_hover ≈ 0).
            double mu = rotorTipSpeedKts > 0.0 && packet.IasKts > 0.0
                ? packet.IasKts / rotorTipSpeedKts : 0.0;
            inputs["MSFS.Rotor.BladeAlphPitch"] = -kSpeedPitch * mu;
            inputs["MSFS.Rotor.BladeAlphRoll"]  = -kSpeedRoll * packet.BetaDeg * mu;

            // §3.2 VRS — 0.50 hover-descent → 0.25 cruise, matches X-Plane shape.
            // VVI is fps in earth frame (positive = climb); descent_fpm = -vvi × 60.
            double descentFpm = packet.VviWorldFps < 0.0 ? -packet.VviWorldFps * 60.0 : 0.0;
            double fwdFactor = ClampD(1.0 - packet.IasKts / Math.Max(etlSpeedKts, 1.0), 0.0, 1.0);
            double descFactor = ClampD(descentFpm / Math.Max(descentMaxFpm, 1.0), 0.0, 1.0);
            inputs["MSFS.Rotor.VRS"] = 0.25 + 0.25 * fwdFactor * (0.5 + 0.5 * descFactor);

            // §3.3 Slap — linear IAS ramp past onset, clamped.
            double overOnset = packet.IasKts - slapOnsetKts;
            inputs["MSFS.Rotor.Slap"] = overOnset <= 0.0
                ? 0.0
                : ClampD(kSlap * overOnset / 100.0, 0.0, slapMax);

            // §3.4 Propwash — momentum theory T ≈ weight × G; v_i = √(T/(2ρA)).
            double weightN = packet.TotalWeightLb * 4.4482;
            double rotorLiftN = weightN * packet.GForce;
            double rhoKgM3 = packet.AmbientDensitySlugsFt3 * 515.379;
            if (rhoKgM3 <= 0.0) rhoKgM3 = 1.225;
            double propwash = 0.0;
            if (rotorLiftN > 0.0 && discAreaM2 > 0.0)
            {
                double viHoverSq = rotorLiftN / (2.0 * rhoKgM3 * discAreaM2);
                if (viHoverSq > 0.0)
                {
                    double viHover = Math.Sqrt(viHoverSq);
                    double vFwd = packet.IasKts * 0.5144;
                    double ratio = vFwd / viHover;
                    propwash = viHover / Math.Sqrt(1.0 + ratio * ratio);
                }
            }
            inputs["MSFS.Rotor.Propwash"] = propwash;

            // §3.5 Torque — ENG TORQUE PERCENT (0..100) × max_torque_nm / 100.
            inputs["MSFS.MainRotor.Torque"] = (packet.EngTorquePct / 100.0) * maxTorqueNm;
        }

        private static double ClampD(double v, double lo, double hi)
        {
            return v < lo ? lo : (v > hi ? hi : v);
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
            // Both resolved via function → linked axis / config mapping.
            inputs["Axis.FlightStickPitch.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickPitch);
            inputs["Axis.FlightStickPitch.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickPitch);
            inputs["Axis.FlightStickRoll.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickRoll);
            inputs["Axis.FlightStickRoll.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickRoll);
            inputs["Axis.FlightPedals.Position"] = plugin.GetFunctionPosition(FunctionID.FlightPedals);
            inputs["Axis.FlightPedals.Center"] = plugin.GetFunctionCenter(FunctionID.FlightPedals);
            inputs["Axis.FlightStickCollective.Position"] = plugin.GetFunctionPosition(FunctionID.FlightStickCollective);
            inputs["Axis.FlightStickCollective.Center"] = plugin.GetFunctionCenter(FunctionID.FlightStickCollective);
        }
    }
}
