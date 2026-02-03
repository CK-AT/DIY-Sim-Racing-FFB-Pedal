using System.Collections.Generic;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Determines which layer a field change should be routed to by default.
    /// Each field has a "home layer" based on its category.
    /// </summary>
    public static class FieldRouter
    {
        // User-level fields: personal preferences that follow the user
        private static readonly HashSet<string> UserFields = new HashSet<string>
        {
            "output_min",
            "output_max",
            "base.output_min",
            "base.output_max",
            "simulated_mass",
            "friction",
            "static_balance_tuning",
            "static_balance_tuning.enabled",
            "static_balance_tuning.gain"
        };

        // Hardware-level fields: rarely changed, tied to physical hardware
        private static readonly HashSet<string> HardwareFields = new HashSet<string>
        {
            "kinematic_parameters",
            "static_balance_config",
            "linked_axes",
            "base.linked_axes",
            "controller_output_axis",
            "base.controller_output_axis",
            "steps_per_mm",
            "mm_per_rev",
            "f_max_loadcell",
            "b_loadcell_inverted",
            "b_motor_inverted"
        };

        /// <summary>
        /// Get the default target layer for a field based on its category.
        /// </summary>
        /// <param name="fieldPath">
        /// Dot-separated field path (e.g., "output_min", "shifter_config.gate_width").
        /// </param>
        /// <returns>Target layer for changes to this field.</returns>
        public static ConfigLayer GetTargetLayer(string fieldPath)
        {
            if (string.IsNullOrEmpty(fieldPath))
                return ConfigLayer.Profile;

            var normalizedPath = fieldPath.ToLowerInvariant();

            // Check User fields first
            if (UserFields.Contains(normalizedPath))
                return ConfigLayer.User;

            // Check Hardware fields
            if (HardwareFields.Contains(normalizedPath))
                return ConfigLayer.Hardware;

            // Check prefixes for nested fields
            if (normalizedPath.StartsWith("kinematic_parameters.") ||
                normalizedPath.StartsWith("static_balance_config."))
                return ConfigLayer.Hardware;

            if (normalizedPath.StartsWith("static_balance_tuning."))
                return ConfigLayer.User;

            // Default to Profile layer for vehicle-specific tuning
            // This includes: shifter_config, detent positions, force curves, etc.
            return ConfigLayer.Profile;
        }

        /// <summary>
        /// Check if a field is user-tunable (can be overridden at User layer).
        /// </summary>
        public static bool IsUserTunable(string fieldPath)
        {
            var layer = GetTargetLayer(fieldPath);
            return layer == ConfigLayer.User;
        }

        /// <summary>
        /// Check if a field is hardware-level (should rarely be overridden).
        /// </summary>
        public static bool IsHardwareField(string fieldPath)
        {
            var layer = GetTargetLayer(fieldPath);
            return layer == ConfigLayer.Hardware;
        }

        /// <summary>
        /// Get all user-tunable field paths for display in UI.
        /// </summary>
        public static IEnumerable<string> GetUserTunableFields()
        {
            return UserFields;
        }
    }
}
