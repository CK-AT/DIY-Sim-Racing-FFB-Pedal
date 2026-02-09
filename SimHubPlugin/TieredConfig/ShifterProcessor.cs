namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for Shifter-specific config processing.
    /// Single source of truth for derived field reconciliation and override application.
    /// Called by both ConfigMerger (backend merge) and ShifterConfigControl (UI real-time).
    /// </summary>
    public static class ShifterProcessor
    {
        /// <summary>
        /// Recompute OutputMin/OutputMax from PosX or PosY range based on Sequential flag.
        /// Mirrors ShifterConfigControl.UpdateOutputRange.
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            if (config?.Base == null) return;

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
        /// Apply Shifter-specific overrides (full replacement of ShifterConfig and ShifterDetectConfig).
        /// </summary>
        public static void ApplyOverrides(FunctionConfig merged, FunctionConfigOverrides delta)
        {
            if (merged == null || delta == null) return;

            if (merged.Shifter != null && delta.ShifterConfig != null)
            {
                merged.Shifter = delta.ShifterConfig.Clone();
            }
            if (merged.AuxFunction?.ShifterDetect != null && delta.ShifterDetectConfig != null)
            {
                merged.AuxFunction.ShifterDetect = delta.ShifterDetectConfig.Clone();
            }
        }
    }
}
