using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb
{
    public static class GraphSignalCatalogData
    {
        // Input signal groups (top-level namespaces)
        public static readonly IReadOnlyList<string> InputGroups = new[] { "XPlane", "Grip", "Axis" };

        // Output signal groups (function names)
        public static readonly IReadOnlyList<string> OutputGroups = new[]
        {
            "FlightStickPitch",
            "FlightStickRoll",
            "FlightPedals",
            "FlightStickCollective"
        };

        // FunctionScope options for Include nodes: empty string = "(none)" / unscoped
        public static readonly IReadOnlyList<string> FunctionScopeOptions = new[]
        {
            "",
            "FlightStickPitch",
            "FlightStickRoll",
            "FlightPedals",
            "FlightStickCollective"
        };

        // Parameter groups (from signal catalog spec)
        public static readonly IReadOnlyList<string> ParamGroups = new[]
        {
            "Aircraft",
            "System",
            "Vehicle",
            // Function names are also valid param groups
            "FlightStickPitch",
            "FlightStickRoll",
            "FlightPedals",
            "FlightStickCollective",
            "Cyclic"  // Shared by helicopter cyclic pitch and roll
        };

        public static readonly IReadOnlyList<string> InputNames = new[]
        {
            "XPlane.Speed.IAS",
            "XPlane.Angle.Alpha",
            "XPlane.Angle.Beta",
            "XPlane.Trim.Elevator",
            "XPlane.Trim.Aileron",
            "XPlane.Trim.Rudder",
            "XPlane.Rate.Roll",
            "XPlane.Rate.Pitch",
            "XPlane.Rate.Yaw",
            "XPlane.G_Nrml",
            "XPlane.AeroTorque.Roll",
            "XPlane.AeroTorque.Pitch",
            "XPlane.AeroTorque.Yaw",
            "XPlane.MainRotor.Torque",
            "XPlane.MainRotor.Speed",
            "XPlane.Rotor.BladeAlphPitch",
            "XPlane.Rotor.BladeAlphRoll",
            "XPlane.Rotor.Slap",
            "XPlane.Rotor.VRS",
            "XPlane.Rotor.Propwash",
            "XPlane.Rotor.FundamentalHz",
            "XPlane.OnGround",
            "Grip.TrimHat.Up",
            "Grip.TrimHat.Down",
            "Grip.TrimHat.Left",
            "Grip.TrimHat.Right",
            "Grip.ForceTrimRelease",
            "Grip.TrimReset",
            "Axis.FlightStickPitch.Position",
            "Axis.FlightStickPitch.Center",
            "Axis.FlightStickRoll.Position",
            "Axis.FlightStickRoll.Center",
            "Axis.FlightPedals.Position",
            "Axis.FlightPedals.Center",
            "Axis.FlightStickCollective.Position",
            "Axis.FlightStickCollective.Center"
        };

        public static readonly IReadOnlyList<string> OutputNames = new[]
        {
            "FlightStickPitch.SpringGain",
            "FlightStickPitch.DamperGain",
            "FlightStickPitch.Friction",
            "FlightStickPitch.LoadForce",
            "FlightStickPitch.TrimOffset",
            "FlightStickPitch.BuffetAmplitude",
            "FlightStickPitch.VibSlot1",
            "FlightStickPitch.VibSlot2",
            "FlightStickPitch.VibSlot3",
            "FlightStickPitch.VibSlot4",
            "FlightStickPitch.VibSlot5",
            "FlightStickPitch.Vib2Slot1",
            "FlightStickPitch.Vib2Slot2",
            "FlightStickRoll.SpringGain",
            "FlightStickRoll.DamperGain",
            "FlightStickRoll.Friction",
            "FlightStickRoll.LoadForce",
            "FlightStickRoll.TrimOffset",
            "FlightStickRoll.BuffetAmplitude",
            "FlightStickRoll.VibSlot1",
            "FlightStickRoll.VibSlot2",
            "FlightStickRoll.VibSlot3",
            "FlightStickRoll.VibSlot4",
            "FlightStickRoll.VibSlot5",
            "FlightStickRoll.Vib2Slot1",
            "FlightStickRoll.Vib2Slot2",
            "FlightPedals.SpringGain",
            "FlightPedals.DamperGain",
            "FlightPedals.Friction",
            "FlightPedals.LoadForce",
            "FlightPedals.TrimOffset",
            "FlightPedals.BuffetAmplitude",
            "FlightPedals.VibSlot1",
            "FlightPedals.VibSlot2",
            "FlightPedals.VibSlot3",
            "FlightPedals.VibSlot4",
            "FlightPedals.VibSlot5",
            "FlightPedals.Vib2Slot1",
            "FlightPedals.Vib2Slot2",
            "FlightStickCollective.SpringGain",
            "FlightStickCollective.DamperGain",
            "FlightStickCollective.Friction",
            "FlightStickCollective.LoadForce",
            "FlightStickCollective.TrimOffset",
            "FlightStickCollective.VibSlot1",
            "FlightStickCollective.VibSlot2",
            "FlightStickCollective.VibSlot3",
            "FlightStickCollective.VibSlot4",
            "FlightStickCollective.VibSlot5",
            "FlightStickCollective.Vib2Slot1",
            "FlightStickCollective.Vib2Slot2",
            // Shared scope: DDS fundamentals broadcast by gateway to all axes.
            // Routed via plain (non-scoped) Output node — excluded from
            // per-function OutputSuffixes filter below.
            "Shared.VibFundamental",
            "Shared.Vib2Fundamental"
        };

        // ConfigType options for ConfigOut nodes
        public static readonly IReadOnlyList<string> ConfigTypeOptions = new[]
        {
            "",
            "FlightStick",
            "FlightPedals"
        };

        /// <summary>
        /// Maps a FunctionScope value to the config type it implies.
        /// E.g., "FlightStickPitch" → "FlightStick", "FlightPedals" → "FlightPedals".
        /// </summary>
        public static string GetConfigTypeForScope(string functionScope)
        {
            switch (functionScope ?? "")
            {
                case "FlightStickPitch":
                case "FlightStickRoll":
                case "FlightStickCollective":
                    return "FlightStick";
                case "FlightPedals":
                    return "FlightPedals";
                default:
                    return "";
            }
        }

        /// <summary>
        /// All distinct output signal suffixes across all function groups.
        /// Used by scoped Output nodes where the group is determined by the parent Include's FunctionScope.
        /// </summary>
        public static readonly IReadOnlyList<string> OutputSuffixes =
            OutputNames
                .Where(n => !n.StartsWith("Shared.", StringComparison.Ordinal))
                .Select(n => n.Substring(n.IndexOf('.') + 1))
                .Distinct()
                .OrderBy(s => s)
                .ToArray();

        /// <summary>
        /// Gets signal suffixes for a given input group.
        /// E.g., for "XPlane" returns ["Speed.IAS", "Angle.Alpha", ...].
        /// </summary>
        public static IReadOnlyList<string> GetInputSignalsForGroup(string group)
        {
            if (string.IsNullOrEmpty(group)) return Array.Empty<string>();
            string prefix = group + ".";
            return InputNames
                .Where(n => n.StartsWith(prefix, StringComparison.Ordinal))
                .Select(n => n.Substring(prefix.Length))
                .ToArray();
        }

        /// <summary>
        /// Gets signal suffixes for a given output group.
        /// E.g., for "FlightStickPitch" returns ["SpringGain", "DamperGain", ...].
        /// </summary>
        public static IReadOnlyList<string> GetOutputSignalsForGroup(string group)
        {
            if (string.IsNullOrEmpty(group)) return Array.Empty<string>();
            string prefix = group + ".";
            return OutputNames
                .Where(n => n.StartsWith(prefix, StringComparison.Ordinal))
                .Select(n => n.Substring(prefix.Length))
                .ToArray();
        }

        /// <summary>
        /// Checks if the given full signal name is a valid input signal.
        /// </summary>
        public static bool IsValidInputSignal(string fullName)
        {
            return !string.IsNullOrEmpty(fullName) && InputNames.Contains(fullName);
        }

        /// <summary>
        /// Checks if the given full signal name is a valid output signal.
        /// </summary>
        public static bool IsValidOutputSignal(string fullName)
        {
            return !string.IsNullOrEmpty(fullName) && OutputNames.Contains(fullName);
        }

        /// <summary>
        /// Builds full signal name from group and suffix.
        /// </summary>
        public static string BuildSignalName(string group, string suffix)
        {
            if (string.IsNullOrEmpty(group) || string.IsNullOrEmpty(suffix))
                return "";
            return group + "." + suffix;
        }
    }
}
