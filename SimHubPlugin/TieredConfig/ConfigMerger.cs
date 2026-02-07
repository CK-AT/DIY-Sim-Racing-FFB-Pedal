using System;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure functions for merging config overrides into base configs.
    /// Resolution: User > Profile > Baseline (first non-null wins).
    /// </summary>
    public static class ConfigMerger
    {
        /// <summary>
        /// Merge axis parameter overrides into a base AxisConfig.
        /// Returns a new config; does not mutate the original.
        /// </summary>
        /// <remarks>
        /// For kinematics and static balance, we do full replacement (not field-by-field merge)
        /// because these are complex computed structures where partial updates don't make sense.
        /// </remarks>
        public static AxisConfig MergeAxisOverrides(AxisConfig baseConfig, AxisParameterOverrides overrides)
        {
            if (baseConfig == null)
                throw new ArgumentNullException(nameof(baseConfig));

            if (overrides == null || overrides.IsEmpty)
                return baseConfig.Clone();

            var merged = baseConfig.Clone();

            // Full replacement for KinematicParameters if override is provided
            if (overrides.Kinematics != null)
            {
                merged.KinematicParameters = overrides.Kinematics.Clone();
            }

            // Full replacement for StaticBalanceConfig if override is provided
            if (overrides.StaticBalance != null)
            {
                merged.StaticBalanceConfig = overrides.StaticBalance.Clone();
            }

            return merged;
        }

        /// <summary>
        /// Merge function config delta into a base FunctionConfig.
        /// Returns a new config; does not mutate the original.
        /// </summary>
        public static FunctionConfig MergeFunctionConfig(FunctionConfig baseConfig, FunctionConfigOverrides delta)
        {
            if (baseConfig == null)
                throw new ArgumentNullException(nameof(baseConfig));

            if (delta == null || delta.IsEmpty)
                return baseConfig.Clone();

            var merged = baseConfig.Clone();

            // Merge scalar fields from FunctionBase
            if (merged.Base != null)
            {
                if (delta.OutputMin.HasValue)
                    merged.Base.OutputMin = delta.OutputMin.Value;
                if (delta.OutputMax.HasValue)
                    merged.Base.OutputMax = delta.OutputMax.Value;
            }

            // Merge common physics parameters
            if (delta.SimulatedMass.HasValue)
                merged.SimulatedMass = delta.SimulatedMass.Value;
            if (delta.Friction.HasValue)
                merged.Friction = delta.Friction.Value;

            // Merge static balance tuning if present
            if (delta.StaticBalanceTuning != null && !delta.StaticBalanceTuning.IsEmpty)
            {
                ApplyStaticBalanceTuningOverrides(merged, delta.StaticBalanceTuning);
            }

            // Merge AutomotivePedals-specific overrides
            if (merged.AutomotivePedal != null)
            {
                ApplyAutomotivePedalOverrides(merged.AutomotivePedal, delta);
            }

            // Merge FlightPedals-specific overrides
            if (merged.FlightPedals != null)
            {
                ApplyFlightPedalsOverrides(merged.FlightPedals, merged.AuxFunction, delta);
            }

            // Merge FlightStick-specific overrides (mode-specific)
            ApplyFlightStickOverrides(merged, delta);

            // Merge Shifter-specific overrides
            if (merged.Shifter != null && delta.ShifterConfig != null)
            {
                merged.Shifter = delta.ShifterConfig.Clone();
            }
            if (merged.AuxFunction?.ShifterDetect != null && delta.ShifterDetectConfig != null)
            {
                merged.AuxFunction.ShifterDetect = delta.ShifterDetectConfig.Clone();
            }

            return merged;
        }

        /// <summary>
        /// Merge all three layers: Baseline -> Profile -> User.
        /// Returns a new config; does not mutate any original.
        /// </summary>
        public static FunctionConfig MergeAllLayers(
            FunctionConfig baseline,
            FunctionConfigOverrides profile,
            FunctionConfigOverrides user)
        {
            if (baseline == null)
                throw new ArgumentNullException(nameof(baseline));

            // Start with baseline
            var merged = baseline.Clone();

            // Apply profile overrides (if any)
            if (profile != null && !profile.IsEmpty)
                merged = MergeFunctionConfig(merged, profile);

            // Apply user overrides (highest priority)
            if (user != null && !user.IsEmpty)
                merged = MergeFunctionConfig(merged, user);

            return merged;
        }

        /// <summary>
        /// Apply static balance tuning overrides to a FunctionConfig.
        /// </summary>
        private static void ApplyStaticBalanceTuningOverrides(
            FunctionConfig config,
            StaticBalanceTuningOverrides overrides)
        {
            if (config.StaticBalanceTuning == null)
                config.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning();

            var tuning = config.StaticBalanceTuning;

            if (overrides.Enabled.HasValue)
                tuning.Enabled = overrides.Enabled.Value;
            if (overrides.Gain.HasValue)
                tuning.Gain = overrides.Gain.Value;
        }

        /// <summary>
        /// Apply AutomotivePedal-specific overrides.
        /// NOTE: Does NOT update Base.OutputMin/Max - controls derive these from ForceCurveConfig.
        /// </summary>
        private static void ApplyAutomotivePedalOverrides(
            AutomotivePedalConfig config,
            FunctionConfigOverrides delta)
        {
            // Merge damper config
            if (delta.DamperConfig != null && !delta.DamperConfig.IsEmpty)
            {
                if (config.DamperConfig == null)
                    config.DamperConfig = new DamperConfig();

                if (delta.DamperConfig.PositiveFactor.HasValue)
                    config.DamperConfig.PositiveFactor = delta.DamperConfig.PositiveFactor.Value;
                if (delta.DamperConfig.NegativeFactor.HasValue)
                    config.DamperConfig.NegativeFactor = delta.DamperConfig.NegativeFactor.Value;
            }

            // Merge force curve (full replacement)
            if (delta.ForceCurve != null)
            {
                config.ForceCurveConfig = delta.ForceCurve.Clone();
            }
        }

        /// <summary>
        /// Apply FlightPedals-specific overrides.
        /// NOTE: Does NOT update Base.OutputMin/Max - controls derive these from PosNearLim/PosFarLim.
        /// </summary>
        private static void ApplyFlightPedalsOverrides(
            FlightPedalsConfig config,
            AuxFunctionConfig auxConfig,
            FunctionConfigOverrides delta)
        {
            // Merge motion range
            if (delta.FlightPedalsMotionRange != null && !delta.FlightPedalsMotionRange.IsEmpty)
            {
                if (delta.FlightPedalsMotionRange.NearLim.HasValue)
                    config.PosNearLim = delta.FlightPedalsMotionRange.NearLim.Value;
                if (delta.FlightPedalsMotionRange.FarLim.HasValue)
                    config.PosFarLim = delta.FlightPedalsMotionRange.FarLim.Value;
            }

            // Merge damping
            if (delta.FlightPedalsDamping.HasValue)
                config.Damping = delta.FlightPedalsDamping.Value;

            // Merge centering spring constant
            if (delta.FlightPedalsCenteringSpringConst.HasValue)
                config.CenteringSpringConst = delta.FlightPedalsCenteringSpringConst.Value;

            // Merge rudder brake force range (aux_function)
            if (auxConfig != null && delta.RudderBrakeForceRange != null && !delta.RudderBrakeForceRange.IsEmpty)
            {
                if (auxConfig.RudderBrake == null)
                    auxConfig.RudderBrake = new RudderBrakeConfig();

                if (delta.RudderBrakeForceRange.Min.HasValue)
                    auxConfig.RudderBrake.FMin = delta.RudderBrakeForceRange.Min.Value;
                if (delta.RudderBrakeForceRange.Max.HasValue)
                    auxConfig.RudderBrake.FMax = delta.RudderBrakeForceRange.Max.Value;
            }
        }

        /// <summary>
        /// Apply FlightStick-specific overrides (mode-dependent: Pitch/Roll/Collective).
        /// NOTE: Does NOT update Base.OutputMin/Max - controls derive these from PosMin/PosMax.
        /// </summary>
        private static void ApplyFlightStickOverrides(
            FunctionConfig merged,
            FunctionConfigOverrides delta)
        {
            // Helper to apply overrides to any flight stick config type
            void ApplyToConfig<T>(T config) where T : class
            {
                if (config == null) return;

                // Use dynamic to work with any FlightStick config type
                dynamic cfg = config;

                // Merge motion range
                if (delta.FlightStickMotionRange != null && !delta.FlightStickMotionRange.IsEmpty)
                {
                    if (delta.FlightStickMotionRange.Min.HasValue)
                        cfg.PosMin = delta.FlightStickMotionRange.Min.Value;
                    if (delta.FlightStickMotionRange.Max.HasValue)
                        cfg.PosMax = delta.FlightStickMotionRange.Max.Value;
                }

                // Merge damping
                if (delta.FlightStickDamping.HasValue)
                    cfg.Damping = delta.FlightStickDamping.Value;

                // Merge centering spring constant
                if (delta.FlightStickCenteringSpringConst.HasValue)
                    cfg.CenteringSpringConst = delta.FlightStickCenteringSpringConst.Value;
            }

            // Apply to the appropriate config based on function type
            switch (merged.Base.FunctionId)
            {
                case FunctionID.FlightStickPitch:
                    if (merged.FlightStickPitch == null)
                        merged.FlightStickPitch = new FlightStickPitchConfig();
                    ApplyToConfig(merged.FlightStickPitch);
                    break;
                case FunctionID.FlightStickRoll:
                    if (merged.FlightStickRoll == null)
                        merged.FlightStickRoll = new FlightStickRollConfig();
                    ApplyToConfig(merged.FlightStickRoll);
                    break;
                case FunctionID.FlightStickCollective:
                    if (merged.FlightStickCollective == null)
                        merged.FlightStickCollective = new FlightStickCollectiveConfig();
                    ApplyToConfig(merged.FlightStickCollective);
                    break;
            }
        }
    }
}
