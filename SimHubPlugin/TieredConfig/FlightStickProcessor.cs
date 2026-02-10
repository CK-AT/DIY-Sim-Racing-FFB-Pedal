namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for FlightStick-specific config processing.
    /// Single source of truth for derived field reconciliation and override application.
    /// Called by both ConfigMerger (backend merge) and FlightStickConfigControl (UI real-time).
    /// </summary>
    public static class FlightStickProcessor
    {
        /// <summary>
        /// Recompute OutputMin/OutputMax from PosMin/PosMax.
        /// Mirrors FlightStickConfigControl.SwitchFunction derived field logic.
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            if (config?.Base == null) return;

            var stick = config.FlightStick;
            if (stick != null)
            {
                config.Base.OutputMin = stick.PosMin;
                config.Base.OutputMax = stick.PosMax;
            }
        }

        /// <summary>
        /// Apply FlightStick-specific overrides (motion range, damping, centering spring).
        /// Base.OutputMin/Max are reconciled by ReconcileDerivedFields after this.
        /// </summary>
        public static void ApplyOverrides(FunctionConfig merged, FunctionConfigOverrides delta)
        {
            if (merged?.Base == null || delta == null) return;

            if (merged.FlightStick == null)
                merged.FlightStick = new FlightStickConfig();

            var cfg = merged.FlightStick;

            if (delta.FlightStickMotionRange != null && !delta.FlightStickMotionRange.IsEmpty)
            {
                if (delta.FlightStickMotionRange.Min.HasValue)
                    cfg.PosMin = delta.FlightStickMotionRange.Min.Value;
                if (delta.FlightStickMotionRange.Max.HasValue)
                    cfg.PosMax = delta.FlightStickMotionRange.Max.Value;
            }
            if (delta.FlightStickDamping.HasValue)
                cfg.Damping = delta.FlightStickDamping.Value;
            if (delta.FlightStickCenteringSpringConst.HasValue)
                cfg.CenteringSpringConst = delta.FlightStickCenteringSpringConst.Value;
        }
    }
}
