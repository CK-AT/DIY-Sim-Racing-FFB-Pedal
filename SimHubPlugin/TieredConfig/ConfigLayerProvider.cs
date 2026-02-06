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
        private readonly Func<int, FunctionConfig> _getBaseline;

        /// <summary>
        /// Create a ConfigLayerProvider with delegates to access profile and user overrides.
        /// </summary>
        /// <param name="getProfileOverrides">Delegate to get profile overrides for a function ID.</param>
        /// <param name="getUserOverrides">Delegate to get user overrides for a function ID.</param>
        /// <param name="getBaseline">Delegate to get baseline config for a function ID.</param>
        public ConfigLayerProvider(
            Func<int, FunctionConfigOverrides> getProfileOverrides,
            Func<int, FunctionConfigOverrides> getUserOverrides,
            Func<int, FunctionConfig> getBaseline = null)
        {
            _getProfileOverrides = getProfileOverrides ?? throw new ArgumentNullException(nameof(getProfileOverrides));
            _getUserOverrides = getUserOverrides ?? throw new ArgumentNullException(nameof(getUserOverrides));
            _getBaseline = getBaseline; // Optional - if null, baseline values won't be available
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

                case "flight_stick.motion_range":
                    return overrides.FlightStickMotionRange != null && !overrides.FlightStickMotionRange.IsEmpty;

                case "flight_pedals.motion_range":
                    return overrides.FlightPedalsMotionRange != null && !overrides.FlightPedalsMotionRange.IsEmpty;

                case "flight_stick.damping":
                    return overrides.FlightStickDamping.HasValue;

                case "flight_stick.centering_spring_const":
                    return overrides.FlightStickCenteringSpringConst.HasValue;

                case "flight_pedals.damping":
                    return overrides.FlightPedalsDamping.HasValue;

                case "flight_pedals.centering_spring_const":
                    return overrides.FlightPedalsCenteringSpringConst.HasValue;

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

        /// <summary>
        /// Check if a specific layer has a value for a field.
        /// </summary>
        public bool HasFieldValue(int functionId, string fieldPath, ConfigLayer layer)
        {
            switch (layer)
            {
                case ConfigLayer.User:
                    var userOverrides = _getUserOverrides(functionId);
                    return userOverrides != null && HasFieldValue(userOverrides, fieldPath);

                case ConfigLayer.Profile:
                    var profileOverrides = _getProfileOverrides(functionId);
                    return profileOverrides != null && HasFieldValue(profileOverrides, fieldPath);

                case ConfigLayer.Hardware:
                    // Hardware layer always has a value if baseline exists
                    return _getBaseline != null && _getBaseline(functionId) != null;

                default:
                    return false;
            }
        }

        /// <summary>
        /// Get the field value from a specific layer.
        /// </summary>
        public object GetFieldValue(int functionId, string fieldPath, ConfigLayer layer)
        {
            var field = OverrideFieldRegistry.GetField(fieldPath);
            if (field == null)
                return null;

            switch (layer)
            {
                case ConfigLayer.User:
                    var userOverrides = _getUserOverrides(functionId);
                    return userOverrides != null ? field.GetValue(userOverrides) : null;

                case ConfigLayer.Profile:
                    var profileOverrides = _getProfileOverrides(functionId);
                    return profileOverrides != null ? field.GetValue(profileOverrides) : null;

                case ConfigLayer.Hardware:
                    if (_getBaseline == null)
                        return null;
                    var baseline = _getBaseline(functionId);
                    return baseline != null ? GetFieldValueFromConfig(baseline, fieldPath) : null;

                default:
                    return null;
            }
        }

        /// <summary>
        /// Extract field value from a FunctionConfig based on field path.
        /// </summary>
        private object GetFieldValueFromConfig(FunctionConfig config, string fieldPath)
        {
            if (config == null)
                return null;

            switch (fieldPath)
            {
                case "output_min": return config.Base?.OutputMin;
                case "output_max": return config.Base?.OutputMax;
                case "simulated_mass": return config.SimulatedMass;
                case "friction": return config.Friction;
                case "static_balance_tuning.enabled": return config.StaticBalanceTuning?.Enabled;
                case "static_balance_tuning.gain": return config.StaticBalanceTuning?.Gain;
                // Add more fields as needed
                default: return null;
            }
        }
    }
}
