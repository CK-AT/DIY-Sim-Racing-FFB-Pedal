namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure static methods for FlightStick-specific config processing (Pitch/Roll/Collective).
    /// Single source of truth for derived field reconciliation and override application.
    /// Called by both ConfigMerger (backend merge) and FlightStickConfigControl (UI real-time).
    /// </summary>
    public static class FlightStickProcessor
    {
        /// <summary>
        /// Recompute OutputMin/OutputMax from PosMin/PosMax of the active sub-config.
        /// Mirrors FlightStickConfigControl.SwitchFunction derived field logic.
        /// </summary>
        public static void ReconcileDerivedFields(FunctionConfig config)
        {
            if (config?.Base == null) return;

            switch (config.Base.FunctionId)
            {
                case FunctionID.FlightStickPitch:
                    if (config.FlightStickPitch != null)
                    {
                        config.Base.OutputMin = config.FlightStickPitch.PosMin;
                        config.Base.OutputMax = config.FlightStickPitch.PosMax;
                    }
                    break;
                case FunctionID.FlightStickRoll:
                    if (config.FlightStickRoll != null)
                    {
                        config.Base.OutputMin = config.FlightStickRoll.PosMin;
                        config.Base.OutputMax = config.FlightStickRoll.PosMax;
                    }
                    break;
                case FunctionID.FlightStickCollective:
                    if (config.FlightStickCollective != null)
                    {
                        config.Base.OutputMin = config.FlightStickCollective.PosMin;
                        config.Base.OutputMax = config.FlightStickCollective.PosMax;
                    }
                    break;
            }
        }

        /// <summary>
        /// Apply FlightStick-specific overrides (motion range, damping, centering spring).
        /// Dispatches to the correct sub-config based on FunctionId.
        /// Base.OutputMin/Max are reconciled by ReconcileDerivedFields after this.
        /// </summary>
        public static void ApplyOverrides(FunctionConfig merged, FunctionConfigOverrides delta)
        {
            if (merged?.Base == null || delta == null) return;

            switch (merged.Base.FunctionId)
            {
                case FunctionID.FlightStickPitch:
                    if (merged.FlightStickPitch == null)
                        merged.FlightStickPitch = new FlightStickPitchConfig();
                    ApplyToPitch(merged.FlightStickPitch, delta);
                    break;
                case FunctionID.FlightStickRoll:
                    if (merged.FlightStickRoll == null)
                        merged.FlightStickRoll = new FlightStickRollConfig();
                    ApplyToRoll(merged.FlightStickRoll, delta);
                    break;
                case FunctionID.FlightStickCollective:
                    if (merged.FlightStickCollective == null)
                        merged.FlightStickCollective = new FlightStickCollectiveConfig();
                    ApplyToCollective(merged.FlightStickCollective, delta);
                    break;
            }
        }

        private static void ApplyToPitch(FlightStickPitchConfig cfg, FunctionConfigOverrides delta)
        {
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

        private static void ApplyToRoll(FlightStickRollConfig cfg, FunctionConfigOverrides delta)
        {
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

        private static void ApplyToCollective(FlightStickCollectiveConfig cfg, FunctionConfigOverrides delta)
        {
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
