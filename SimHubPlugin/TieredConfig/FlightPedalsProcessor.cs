namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for FlightPedals-specific config processing.
    /// Single source of truth for derived field reconciliation and override application.
    /// Called by both ConfigMerger (backend merge) and FlightPedalsConfigControl (UI real-time).
    /// </summary>
    public static class FlightPedalsProcessor
    {
        /// <summary>
        /// Recompute OutputMin/OutputMax from PosNearLim/PosFarLim.
        /// Mirrors FlightPedalsConfigControl.SwitchFunction derived field logic.
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            var fp = config?.FlightPedals;
            if (fp == null || config.Base == null) return;

            config.Base.OutputMin = fp.PosNearLim;
            config.Base.OutputMax = fp.PosFarLim;
        }

        /// <summary>
        /// Apply FlightPedals-specific overrides: motion range, damping, centering spring,
        /// and rudder brake force range.
        /// Base.OutputMin/Max are reconciled by ReconcileDerivedFields after this.
        /// </summary>
        public static void ApplyOverrides(
            FlightPedalsConfig config,
            AuxFunctionConfig auxConfig,
            FunctionConfigOverrides delta)
        {
            if (config == null || delta == null) return;

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
    }
}
