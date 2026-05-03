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

            // No override - baseline default
            return null;
        }

        /// <summary>
        /// Check if a field has a non-null value in the given overrides.
        /// </summary>
        private static bool HasFieldValue(FunctionConfigOverrides overrides, string fieldName)
        {
            return OverrideFieldRegistry.HasValue(overrides, fieldName);
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
                case ConfigLayer.Baseline:
                    return "[B]";
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
                case ConfigLayer.Baseline:
                    return "Baseline default";
                default:
                    return "Using baseline default";
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

                case ConfigLayer.Baseline:
                    // Baseline layer always has a value if baseline exists
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

                case ConfigLayer.Baseline:
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
                case "flight_control.motion_range":
                case "flight_pedals.motion_range":
                case "flight_stick.motion_range":
                    if (config.FlightControl != null) return $"{config.FlightControl.PosMin}-{config.FlightControl.PosMax}mm";
                    return null;
                case "flight_control.damping":
                case "flight_pedals.damping":
                case "flight_stick.damping": return config.FlightControl?.Damping;
                case "flight_control.centering_spring_const":
                case "flight_pedals.centering_spring_const":
                case "flight_stick.centering_spring_const": return config.FlightControl?.CenteringSpringConst;
                case "aux_function.rudder_brake.force_range":
                    return config.AuxFunction?.RudderBrake != null ? $"{config.AuxFunction.RudderBrake.FMin:F1}-{config.AuxFunction.RudderBrake.FMax:F1}N" : null;
                case "force_curve": return config.AutomotivePedal?.ForceCurveConfig;
                case "damper_config.positive_factor": return config.AutomotivePedal?.DamperConfig?.PositiveFactor;
                case "damper_config.negative_factor": return config.AutomotivePedal?.DamperConfig?.NegativeFactor;
                case "shifter_config": return config.Shifter;
                default: return null;
            }
        }

        /// <summary>
        /// Write a field value from the source config into the target config.
        /// Used by BakeFieldToBaseline to copy effective values into baselines.
        /// </summary>
        public static void WriteFieldToFunctionConfig(
            FunctionConfig target, string fieldPath, FunctionConfig source)
        {
            if (target == null || source == null)
                return;

            switch (fieldPath)
            {
                case "output_min":
                    if (source.Base != null)
                    {
                        if (target.Base == null) target.Base = new FunctionBase();
                        target.Base.OutputMin = source.Base.OutputMin;
                    }
                    break;
                case "output_max":
                    if (source.Base != null)
                    {
                        if (target.Base == null) target.Base = new FunctionBase();
                        target.Base.OutputMax = source.Base.OutputMax;
                    }
                    break;
                case "simulated_mass":
                    target.SimulatedMass = source.SimulatedMass;
                    break;
                case "friction":
                    target.Friction = source.Friction;
                    break;
                case "static_balance_tuning.enabled":
                    if (source.StaticBalanceTuning != null)
                    {
                        if (target.StaticBalanceTuning == null)
                            target.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning();
                        target.StaticBalanceTuning.Enabled = source.StaticBalanceTuning.Enabled;
                    }
                    break;
                case "static_balance_tuning.gain":
                    if (source.StaticBalanceTuning != null)
                    {
                        if (target.StaticBalanceTuning == null)
                            target.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning();
                        target.StaticBalanceTuning.Gain = source.StaticBalanceTuning.Gain;
                    }
                    break;
                case "force_curve":
                    if (source.AutomotivePedal != null)
                    {
                        if (target.AutomotivePedal == null)
                            target.AutomotivePedal = new AutomotivePedalConfig();
                        target.AutomotivePedal.ForceCurveConfig = source.AutomotivePedal.ForceCurveConfig?.Clone();
                    }
                    break;
                case "damper_config.positive_factor":
                    if (source.AutomotivePedal?.DamperConfig != null)
                    {
                        if (target.AutomotivePedal == null)
                            target.AutomotivePedal = new AutomotivePedalConfig();
                        if (target.AutomotivePedal.DamperConfig == null)
                            target.AutomotivePedal.DamperConfig = new DamperConfig();
                        target.AutomotivePedal.DamperConfig.PositiveFactor = source.AutomotivePedal.DamperConfig.PositiveFactor;
                    }
                    break;
                case "damper_config.negative_factor":
                    if (source.AutomotivePedal?.DamperConfig != null)
                    {
                        if (target.AutomotivePedal == null)
                            target.AutomotivePedal = new AutomotivePedalConfig();
                        if (target.AutomotivePedal.DamperConfig == null)
                            target.AutomotivePedal.DamperConfig = new DamperConfig();
                        target.AutomotivePedal.DamperConfig.NegativeFactor = source.AutomotivePedal.DamperConfig.NegativeFactor;
                    }
                    break;
                case "flight_control.motion_range":
                case "flight_pedals.motion_range":
                case "flight_stick.motion_range":
                    if (source.FlightControl != null)
                    {
                        if (target.FlightControl == null)
                            target.FlightControl = new FlightControlConfig();
                        target.FlightControl.PosMin = source.FlightControl.PosMin;
                        target.FlightControl.PosMax = source.FlightControl.PosMax;
                    }
                    break;
                case "flight_control.damping":
                case "flight_pedals.damping":
                case "flight_stick.damping":
                    if (source.FlightControl != null)
                    {
                        if (target.FlightControl == null)
                            target.FlightControl = new FlightControlConfig();
                        target.FlightControl.Damping = source.FlightControl.Damping;
                    }
                    break;
                case "flight_control.centering_spring_const":
                case "flight_pedals.centering_spring_const":
                case "flight_stick.centering_spring_const":
                    if (source.FlightControl != null)
                    {
                        if (target.FlightControl == null)
                            target.FlightControl = new FlightControlConfig();
                        target.FlightControl.CenteringSpringConst = source.FlightControl.CenteringSpringConst;
                    }
                    break;
                case "aux_function.rudder_brake.force_range":
                    if (source.AuxFunction?.RudderBrake != null)
                    {
                        if (target.AuxFunction == null)
                            target.AuxFunction = new AuxFunctionConfig();
                        if (target.AuxFunction.RudderBrake == null)
                            target.AuxFunction.RudderBrake = new RudderBrakeConfig();
                        target.AuxFunction.RudderBrake.FMin = source.AuxFunction.RudderBrake.FMin;
                        target.AuxFunction.RudderBrake.FMax = source.AuxFunction.RudderBrake.FMax;
                    }
                    break;
                case "shifter_config":
                    if (source.Shifter != null)
                        target.Shifter = source.Shifter.Clone();
                    break;
            }
        }

    }
}
