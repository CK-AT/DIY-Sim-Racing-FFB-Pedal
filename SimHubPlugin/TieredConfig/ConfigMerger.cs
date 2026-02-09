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
                AutomotivePedalProcessor.ApplyOverrides(merged.AutomotivePedal, delta);
            }

            // Merge FlightPedals-specific overrides
            if (merged.FlightPedals != null)
            {
                FlightPedalsProcessor.ApplyOverrides(merged.FlightPedals, merged.AuxFunction, delta);
            }

            // Merge FlightStick-specific overrides (mode-specific)
            FlightStickProcessor.ApplyOverrides(merged, delta);

            // Merge Shifter-specific overrides
            if (merged.Shifter != null && delta.ShifterConfig != null)
            {
                merged.Shifter = delta.ShifterConfig.Clone();
            }
            if (merged.AuxFunction?.ShifterDetect != null && delta.ShifterDetectConfig != null)
            {
                merged.AuxFunction.ShifterDetect = delta.ShifterDetectConfig.Clone();
            }

            // Reconcile derived fields (posIdle/posEnd/outputMin/outputMax)
            // that depend on merged force curve / motion range values.
            ReconcileDerivedFields(merged);

            return merged;
        }

        /// <summary>
        /// Recompute fields derived from force curve / motion range after merge.
        /// Each function type has its own derivation rules (mirrored from UI controls).
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            if (config?.Base == null) return;

            switch (config.Base.FunctionId)
            {
                case FunctionID.BrakePedal:
                case FunctionID.AcceleratorPedal:
                case FunctionID.ClutchPedal:
                    AutomotivePedalProcessor.ReconcileDerivedFields(config);
                    break;
                case FunctionID.FlightPedals:
                    FlightPedalsProcessor.ReconcileDerivedFields(config);
                    break;
                case FunctionID.FlightStickPitch:
                case FunctionID.FlightStickRoll:
                case FunctionID.FlightStickCollective:
                    FlightStickProcessor.ReconcileDerivedFields(config);
                    break;
                case FunctionID.Shifter:
                    ReconcileShifter(config);
                    break;
            }
        }


        /// <summary>
        /// Shifter: outputMin/Max from PosX or PosY range based on Sequential flag.
        /// Mirrors ShifterConfigControl.UpdateOutputRange.
        /// </summary>
        private static void ReconcileShifter(FunctionConfig config)
        {
            var sh = config.Shifter;
            if (sh == null) return;

            if (sh.Sequential)
            {
                config.Base.OutputMin = sh.PosYMin;
                config.Base.OutputMax = sh.PosYMax;
            }
            else
            {
                config.Base.OutputMin = sh.PosXMin;
                config.Base.OutputMax = sh.PosXMax;
            }
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

    }
}
