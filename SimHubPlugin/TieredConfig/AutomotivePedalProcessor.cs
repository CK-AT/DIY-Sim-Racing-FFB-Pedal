namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for AutomotivePedal-specific config processing.
    /// Single source of truth for derived field reconciliation and override application.
    /// Called by both ConfigMerger (backend merge) and AutomotivePedalConfigControl (UI real-time).
    /// </summary>
    public static class AutomotivePedalProcessor
    {
        /// <summary>
        /// Recompute PosIdle/PosEnd and OutputMin/OutputMax from force curve values.
        /// PosIdle/PosEnd always track ForceCurveConfig.PosMin/PosMax.
        /// OutputMin/OutputMax depend on OutputMode (Force vs Travel).
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            var ap = config?.AutomotivePedal;
            var fc = ap?.ForceCurveConfig;
            if (fc == null) return;

            ap.PosIdle = fc.PosMin;
            ap.PosEnd = fc.PosMax;

            if (config.Base == null) return;

            switch (config.Base.OutputMode)
            {
                case OutputMode.Force:
                    config.Base.OutputMin = fc.FMin;
                    config.Base.OutputMax = fc.FMax;
                    break;
                case OutputMode.Travel:
                    config.Base.OutputMin = fc.PosMin;
                    config.Base.OutputMax = fc.PosMax;
                    break;
            }
        }

        /// <summary>
        /// Apply AutomotivePedal-specific overrides (damper config, force curve).
        /// Base.OutputMin/Max and PosIdle/PosEnd are reconciled by ReconcileDerivedFields.
        /// </summary>
        public static void ApplyOverrides(AutomotivePedalConfig config, FunctionConfigOverrides delta)
        {
            if (config == null || delta == null) return;

            // Merge damper config field-by-field
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
    }
}
