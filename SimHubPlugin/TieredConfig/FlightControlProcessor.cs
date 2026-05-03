namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for FlightControl config processing — applies to all
    /// four flight functions (FlightStickPitch/Roll/Collective + FlightPedals).
    /// Single source of truth for derived field reconciliation and override
    /// application. Called by ConfigMerger (backend merge) and the flight UI
    /// controls (UI real-time).
    /// </summary>
    public static class FlightControlProcessor
    {
        /// <summary>
        /// Recompute OutputMin/OutputMax from FlightControlConfig.PosMin/PosMax.
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            if (config?.Base == null) return;

            var fc = config.FlightControl;
            if (fc != null)
            {
                config.Base.OutputMin = fc.PosMin;
                config.Base.OutputMax = fc.PosMax;
            }
        }

        /// <summary>
        /// Apply FlightControl-specific overrides: motion range, damping, centering
        /// spring, DDS phase + harmonic ratios, plus rudder brake force range when
        /// the aux_function is rudder-brake.
        /// Base.OutputMin/Max are reconciled by ReconcileDerivedFields after this.
        /// </summary>
        public static void ApplyOverrides(FunctionConfig merged, FunctionConfigOverrides delta)
        {
            if (merged?.Base == null || delta == null) return;

            bool hasFlightControlOverrides =
                (delta.FlightControlMotionRange != null && !delta.FlightControlMotionRange.IsEmpty) ||
                delta.FlightControlDamping.HasValue ||
                delta.FlightControlCenteringSpringConst.HasValue ||
                delta.FlightControlPhaseOffset.HasValue ||
                delta.FlightControlVibHarmonicRatios != null ||
                delta.FlightControlVib2HarmonicRatios != null;

            // RudderBrake (aux_function on FlightPedals function) is handled
            // independently of the flight_control oneof arm presence.
            if (hasFlightControlOverrides)
            {
                // If the oneof has a different arm set (e.g. AutomotivePedal), bail
                // out to avoid clobbering it. If the oneof is unset (None), it's
                // safe to create the FlightControl arm so graph-derived ConfigOut
                // values (vibration ratios, phase) have somewhere to land when the
                // firmware sends a baseline without one.
                if (merged.SpecificCase != FunctionConfig.SpecificOneofCase.None &&
                    merged.SpecificCase != FunctionConfig.SpecificOneofCase.FlightControl)
                {
                    // skip — leave aux_function rudder brake handling to the block below
                }
                else
                {
                    if (merged.FlightControl == null)
                        merged.FlightControl = new FlightControlConfig();

                    var cfg = merged.FlightControl;

                    if (delta.FlightControlMotionRange != null && !delta.FlightControlMotionRange.IsEmpty)
                    {
                        if (delta.FlightControlMotionRange.Min.HasValue)
                            cfg.PosMin = delta.FlightControlMotionRange.Min.Value;
                        if (delta.FlightControlMotionRange.Max.HasValue)
                            cfg.PosMax = delta.FlightControlMotionRange.Max.Value;
                    }
                    if (delta.FlightControlDamping.HasValue)
                        cfg.Damping = delta.FlightControlDamping.Value;
                    if (delta.FlightControlCenteringSpringConst.HasValue)
                        cfg.CenteringSpringConst = delta.FlightControlCenteringSpringConst.Value;
                    if (delta.FlightControlPhaseOffset.HasValue)
                    {
                        // Plugin-side overrides express phase offset in degrees for
                        // human-friendliness; firmware/proto consume radians.
                        const float kDegToRad = (float)(System.Math.PI / 180.0);
                        cfg.PhaseOffset = delta.FlightControlPhaseOffset.Value * kDegToRad;
                    }
                    if (delta.FlightControlVibHarmonicRatios != null)
                    {
                        cfg.VibHarmonicRatios.Clear();
                        foreach (var r in delta.FlightControlVibHarmonicRatios)
                            cfg.VibHarmonicRatios.Add(r ?? 0.0f);
                    }
                    if (delta.FlightControlVib2HarmonicRatios != null)
                    {
                        cfg.Vib2HarmonicRatios.Clear();
                        foreach (var r in delta.FlightControlVib2HarmonicRatios)
                            cfg.Vib2HarmonicRatios.Add(r ?? 0.0f);
                    }
                }
            }

            // Merge rudder brake force range (aux_function on FlightPedals function)
            if (merged.AuxFunction != null && delta.RudderBrakeForceRange != null && !delta.RudderBrakeForceRange.IsEmpty)
            {
                if (merged.AuxFunction.RudderBrake == null)
                    merged.AuxFunction.RudderBrake = new RudderBrakeConfig();

                if (delta.RudderBrakeForceRange.Min.HasValue)
                    merged.AuxFunction.RudderBrake.FMin = delta.RudderBrakeForceRange.Min.Value;
                if (delta.RudderBrakeForceRange.Max.HasValue)
                    merged.AuxFunction.RudderBrake.FMax = delta.RudderBrakeForceRange.Max.Value;
            }
        }
    }
}
