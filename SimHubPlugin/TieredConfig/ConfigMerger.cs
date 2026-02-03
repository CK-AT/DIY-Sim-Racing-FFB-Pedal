using System;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure functions for merging config overrides into base configs.
    /// Resolution: User > Profile > Hardware (first non-null wins).
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

            return merged;
        }

        /// <summary>
        /// Merge all three layers: Hardware (base) -> Profile -> User.
        /// Returns a new config; does not mutate any original.
        /// </summary>
        public static FunctionConfig MergeAllLayers(
            FunctionConfig hardware,
            FunctionConfigOverrides profile,
            FunctionConfigOverrides user)
        {
            if (hardware == null)
                throw new ArgumentNullException(nameof(hardware));

            // Start with hardware base
            var merged = hardware.Clone();

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
    }
}
