using System;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Provides information about which configuration layer a field value comes from.
    /// Used by UI to display layer badges ([P] for Profile, [U] for User).
    /// </summary>
    public class ConfigLayerProvider
    {
        private readonly Func<int, FunctionConfigOverrides> _getProfileOverrides;
        private readonly Func<int, FunctionConfigOverrides> _getUserOverrides;

        /// <summary>
        /// Create a ConfigLayerProvider with delegates to access profile and user overrides.
        /// </summary>
        /// <param name="getProfileOverrides">Delegate to get profile overrides for a function ID.</param>
        /// <param name="getUserOverrides">Delegate to get user overrides for a function ID.</param>
        public ConfigLayerProvider(
            Func<int, FunctionConfigOverrides> getProfileOverrides,
            Func<int, FunctionConfigOverrides> getUserOverrides)
        {
            _getProfileOverrides = getProfileOverrides ?? throw new ArgumentNullException(nameof(getProfileOverrides));
            _getUserOverrides = getUserOverrides ?? throw new ArgumentNullException(nameof(getUserOverrides));
        }

        /// <summary>
        /// Get the source layer for a specific field on a function.
        /// Returns the highest-priority layer that has an override for this field,
        /// or null if the field uses the hardware default.
        /// </summary>
        /// <param name="functionId">The function ID.</param>
        /// <param name="fieldName">The field name (e.g., "OutputMin", "SimulatedMass").</param>
        /// <returns>User if user override exists, Profile if profile override exists, null otherwise.</returns>
        public ConfigLayer? GetFieldSourceLayer(int functionId, string fieldName)
        {
            // Check user layer first (highest priority)
            var userOverrides = _getUserOverrides(functionId);
            if (userOverrides != null && HasFieldValue(userOverrides, fieldName))
                return ConfigLayer.User;

            // Check profile layer
            var profileOverrides = _getProfileOverrides(functionId);
            if (profileOverrides != null && HasFieldValue(profileOverrides, fieldName))
                return ConfigLayer.Profile;

            // No override - hardware default
            return null;
        }

        /// <summary>
        /// Check if a field has a non-null value in the given overrides.
        /// </summary>
        private static bool HasFieldValue(FunctionConfigOverrides overrides, string fieldName)
        {
            if (overrides == null)
                return false;

            switch (fieldName)
            {
                case "OutputMin":
                case "output_min":
                    return overrides.OutputMin.HasValue;

                case "OutputMax":
                case "output_max":
                    return overrides.OutputMax.HasValue;

                case "SimulatedMass":
                case "simulated_mass":
                    return overrides.SimulatedMass.HasValue;

                case "Friction":
                case "friction":
                    return overrides.Friction.HasValue;

                case "StaticBalanceEnabled":
                case "static_balance_tuning.enabled":
                    return overrides.StaticBalanceTuning?.Enabled.HasValue == true;

                case "StaticBalanceGain":
                case "static_balance_tuning.gain":
                    return overrides.StaticBalanceTuning?.Gain.HasValue == true;

                case "StaticBalanceTuning":
                case "static_balance_tuning":
                    return overrides.StaticBalanceTuning != null && !overrides.StaticBalanceTuning.IsEmpty;

                default:
                    return false;
            }
        }

        /// <summary>
        /// Get the badge text for a layer (for UI display).
        /// </summary>
        public static string GetLayerBadgeText(ConfigLayer? layer)
        {
            switch (layer)
            {
                case ConfigLayer.User:
                    return "[U]";
                case ConfigLayer.Profile:
                    return "[P]";
                case ConfigLayer.Hardware:
                    return "[H]";
                default:
                    return null;
            }
        }

        /// <summary>
        /// Get the tooltip text for a layer badge.
        /// </summary>
        public static string GetLayerTooltip(ConfigLayer? layer)
        {
            switch (layer)
            {
                case ConfigLayer.User:
                    return "User preference override";
                case ConfigLayer.Profile:
                    return "Vehicle profile override";
                case ConfigLayer.Hardware:
                    return "Hardware default";
                default:
                    return "Using hardware default";
            }
        }
    }
}
