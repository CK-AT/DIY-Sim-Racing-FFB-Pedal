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

            bool hasFlightStickOverrides =
                (delta.FlightStickMotionRange != null && !delta.FlightStickMotionRange.IsEmpty) ||
                delta.FlightStickDamping.HasValue ||
                delta.FlightStickCenteringSpringConst.HasValue ||
                delta.FlightStickPhaseOffset.HasValue ||
                delta.FlightStickVibHarmonicRatios != null ||
                delta.FlightStickVib2HarmonicRatios != null;

            if (!hasFlightStickOverrides)
                return;

            // If the oneof has a different arm set (e.g. AutomotivePedal), bail out to
            // avoid clobbering it. If the oneof is unset (None), it's safe to create
            // the FlightStick arm — graph-derived ConfigOut values (vibration ratios,
            // phase) need an arm to write into when the firmware sends a baseline
            // without one.
            if (merged.SpecificCase != FunctionConfig.SpecificOneofCase.None &&
                merged.SpecificCase != FunctionConfig.SpecificOneofCase.FlightStick)
                return;

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
            if (delta.FlightStickPhaseOffset.HasValue)
            {
                // Plugin-side overrides express phase offset in degrees for
                // human-friendliness; firmware/proto consume radians.
                const float kDegToRad = (float)(System.Math.PI / 180.0);
                cfg.PhaseOffset = delta.FlightStickPhaseOffset.Value * kDegToRad;
            }
            if (delta.FlightStickVibHarmonicRatios != null)
            {
                cfg.VibHarmonicRatios.Clear();
                foreach (var r in delta.FlightStickVibHarmonicRatios)
                    cfg.VibHarmonicRatios.Add(r ?? 0.0f);
            }
            if (delta.FlightStickVib2HarmonicRatios != null)
            {
                cfg.Vib2HarmonicRatios.Clear();
                foreach (var r in delta.FlightStickVib2HarmonicRatios)
                    cfg.Vib2HarmonicRatios.Add(r ?? 0.0f);
            }
        }
    }
}
