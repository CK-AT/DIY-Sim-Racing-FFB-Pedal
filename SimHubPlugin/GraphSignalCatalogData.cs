using System;
using System.Collections.Generic;
using System.Linq;

namespace User.PluginSdkDemo
{
    public static class GraphSignalCatalogData
    {
        // Input signal groups (top-level namespaces)
        public static readonly IReadOnlyList<string> InputGroups = new[] { "XPlane" };

        // Output signal groups (function names)
        public static readonly IReadOnlyList<string> OutputGroups = new[]
        {
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
            "XPlane.OnGround"
        };

        public static readonly IReadOnlyList<string> OutputNames = new[]
        {
            "FlightStickPitch.SpringGain",
            "FlightStickPitch.DamperGain",
            "FlightStickPitch.Friction",
            "FlightStickPitch.LoadForce",
            "FlightStickPitch.TrimOffset",
            "FlightStickPitch.BuffetAmplitude",
            "FlightStickRoll.SpringGain",
            "FlightStickRoll.DamperGain",
            "FlightStickRoll.Friction",
            "FlightStickRoll.LoadForce",
            "FlightStickRoll.TrimOffset",
            "FlightStickRoll.BuffetAmplitude",
            "FlightPedals.SpringGain",
            "FlightPedals.DamperGain",
            "FlightPedals.Friction",
            "FlightPedals.LoadForce",
            "FlightPedals.TrimOffset",
            "FlightPedals.BuffetAmplitude",
            "FlightStickCollective.SpringGain",
            "FlightStickCollective.DamperGain",
            "FlightStickCollective.Friction",
            "FlightStickCollective.LoadForce",
            "FlightStickCollective.TrimOffset"
        };

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
