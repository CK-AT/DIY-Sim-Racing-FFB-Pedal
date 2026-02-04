using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Type of override field for UI rendering.
    /// </summary>
    public enum OverrideFieldType
    {
        Float,      // Slider + textbox
        Bool,       // Checkbox
        Complex     // Custom editor (force curve, shifter config, kinematics)
    }

    /// <summary>
    /// Grouping for override fields in UI.
    /// </summary>
    public enum OverrideFieldGroup
    {
        OutputScaling,
        Physics,
        StaticBalanceTuning,
        ForceFeedback,
        Damper,
        AutomotivePedals,
        FlightPedals,
        FlightStick,
        Shifter,
        AxisGeometry,
        AxisCalibration
    }

    /// <summary>
    /// Metadata and accessors for a single override field.
    /// </summary>
    public sealed class OverrideFieldDefinition
    {
        // Identity
        public string Name { get; internal set; }              // "OutputMin"
        public string FieldPath { get; internal set; }         // "output_min"

        // Metadata
        public string DisplayName { get; internal set; }       // "Output Min"
        public string Tooltip { get; internal set; }
        public OverrideFieldType FieldType { get; internal set; }
        public OverrideFieldGroup Group { get; internal set; }

        // Routing
        public ConfigLayer DefaultLayer { get; internal set; }

        // Display formatting (optional, for complex types)
        /// <summary>
        /// Optional formatter for displaying complex values in tooltips.
        /// For scalar types, default ToString() is used if not specified.
        /// Example: (val) => $"{val.Min}-{val.Max}mm"
        /// </summary>
        public Func<object, string> FormatValue { get; internal set; }

        // Accessors (compiled delegates for FunctionConfigOverrides)
        internal Func<FunctionConfigOverrides, bool> HasValue { get; set; }
        internal Func<FunctionConfigOverrides, object> GetValue { get; set; }
        internal Action<FunctionConfigOverrides, object> SetValue { get; set; }
        internal Action<FunctionConfigOverrides> ClearValue { get; set; }
    }

    /// <summary>
    /// Unified registry for all function-level override fields.
    /// Provides field metadata, layer routing, and value accessors.
    /// </summary>
    public static class OverrideFieldRegistry
    {
        private static readonly Dictionary<string, OverrideFieldDefinition> _fieldsByPath
            = new Dictionary<string, OverrideFieldDefinition>(StringComparer.OrdinalIgnoreCase);

        private static readonly Dictionary<string, OverrideFieldDefinition> _fieldsByName
            = new Dictionary<string, OverrideFieldDefinition>(StringComparer.OrdinalIgnoreCase);

        static OverrideFieldRegistry()
        {
            RegisterAllFields();
        }

        private static void RegisterAllFields()
        {
            // === Output Scaling ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "OutputMin",
                FieldPath = "output_min",
                DisplayName = "Output Min",
                Tooltip = "Minimum output value (0-1)",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.OutputScaling,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.OutputMin.HasValue,
                GetValue = o => o.OutputMin,
                SetValue = (o, v) => o.OutputMin = (float?)v,
                ClearValue = o => o.OutputMin = null
            });

            RegisterField(new OverrideFieldDefinition
            {
                Name = "OutputMax",
                FieldPath = "output_max",
                DisplayName = "Output Max",
                Tooltip = "Maximum output value (0-1)",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.OutputScaling,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.OutputMax.HasValue,
                GetValue = o => o.OutputMax,
                SetValue = (o, v) => o.OutputMax = (float?)v,
                ClearValue = o => o.OutputMax = null
            });

            // === Physics ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "SimulatedMass",
                FieldPath = "simulated_mass",
                DisplayName = "Simulated Mass",
                Tooltip = "Simulated mass for physics calculations (kg)",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.Physics,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.SimulatedMass.HasValue,
                GetValue = o => o.SimulatedMass,
                SetValue = (o, v) => o.SimulatedMass = (float?)v,
                ClearValue = o => o.SimulatedMass = null
            });

            RegisterField(new OverrideFieldDefinition
            {
                Name = "Friction",
                FieldPath = "friction",
                DisplayName = "Friction",
                Tooltip = "Friction coefficient for force feedback",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.Physics,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.Friction.HasValue,
                GetValue = o => o.Friction,
                SetValue = (o, v) => o.Friction = (float?)v,
                ClearValue = o => o.Friction = null
            });

            // === Static Balance Tuning (nested fields) ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "StaticBalanceEnabled",
                FieldPath = "static_balance_tuning.enabled",
                DisplayName = "Enabled",
                Tooltip = "Enable static balance compensation",
                FieldType = OverrideFieldType.Bool,
                Group = OverrideFieldGroup.StaticBalanceTuning,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.StaticBalanceTuning?.Enabled.HasValue == true,
                GetValue = o => o.StaticBalanceTuning?.Enabled,
                SetValue = (o, v) =>
                {
                    if (o.StaticBalanceTuning == null)
                        o.StaticBalanceTuning = new StaticBalanceTuningOverrides();
                    o.StaticBalanceTuning.Enabled = (bool?)v;
                },
                ClearValue = o =>
                {
                    if (o.StaticBalanceTuning != null)
                    {
                        o.StaticBalanceTuning.Enabled = null;
                        if (o.StaticBalanceTuning.IsEmpty)
                            o.StaticBalanceTuning = null;
                    }
                }
            });

            RegisterField(new OverrideFieldDefinition
            {
                Name = "StaticBalanceGain",
                FieldPath = "static_balance_tuning.gain",
                DisplayName = "Gain",
                Tooltip = "Static balance compensation gain",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.StaticBalanceTuning,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.StaticBalanceTuning?.Gain.HasValue == true,
                GetValue = o => o.StaticBalanceTuning?.Gain,
                SetValue = (o, v) =>
                {
                    if (o.StaticBalanceTuning == null)
                        o.StaticBalanceTuning = new StaticBalanceTuningOverrides();
                    o.StaticBalanceTuning.Gain = (float?)v;
                },
                ClearValue = o =>
                {
                    if (o.StaticBalanceTuning != null)
                    {
                        o.StaticBalanceTuning.Gain = null;
                        if (o.StaticBalanceTuning.IsEmpty)
                            o.StaticBalanceTuning = null;
                    }
                }
            });

            // === AutomotivePedals ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "DamperPositiveFactor",
                FieldPath = "damper_config.positive_factor",
                DisplayName = "Damper Positive Factor",
                Tooltip = "Damping in positive/pressing direction ((N*s)/mm)",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.Damper,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.DamperConfig?.PositiveFactor.HasValue == true,
                GetValue = o => o.DamperConfig?.PositiveFactor,
                SetValue = (o, v) =>
                {
                    if (o.DamperConfig == null)
                        o.DamperConfig = new DamperConfigOverrides();
                    o.DamperConfig.PositiveFactor = (float?)v;
                },
                ClearValue = o =>
                {
                    if (o.DamperConfig != null)
                    {
                        o.DamperConfig.PositiveFactor = null;
                        if (o.DamperConfig.IsEmpty)
                            o.DamperConfig = null;
                    }
                }
            });

            RegisterField(new OverrideFieldDefinition
            {
                Name = "DamperNegativeFactor",
                FieldPath = "damper_config.negative_factor",
                DisplayName = "Damper Negative Factor",
                Tooltip = "Damping in negative/pulling direction ((N*s)/mm)",
                FieldType = OverrideFieldType.Float,
                Group = OverrideFieldGroup.Damper,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.DamperConfig?.NegativeFactor.HasValue == true,
                GetValue = o => o.DamperConfig?.NegativeFactor,
                SetValue = (o, v) =>
                {
                    if (o.DamperConfig == null)
                        o.DamperConfig = new DamperConfigOverrides();
                    o.DamperConfig.NegativeFactor = (float?)v;
                },
                ClearValue = o =>
                {
                    if (o.DamperConfig != null)
                    {
                        o.DamperConfig.NegativeFactor = null;
                        if (o.DamperConfig.IsEmpty)
                            o.DamperConfig = null;
                    }
                }
            });

            RegisterField(new OverrideFieldDefinition
            {
                Name = "ForceCurve",
                FieldPath = "force_curve",
                DisplayName = "Force Curve",
                Tooltip = "Spline force curve configuration",
                FieldType = OverrideFieldType.Complex,
                Group = OverrideFieldGroup.ForceFeedback,
                DefaultLayer = ConfigLayer.Profile,
                HasValue = o => o.ForceCurve != null,
                GetValue = o => o.ForceCurve,
                SetValue = (o, v) => o.ForceCurve = (SplineForceCurveConfig)v,
                ClearValue = o => o.ForceCurve = null,
                FormatValue = (val) =>
                {
                    var curve = val as SplineForceCurveConfig;
                    if (curve == null) return "(not set)";
                    return $"{curve.PosMin}-{curve.PosMax}mm, {curve.FMin}-{curve.FMax}N";
                }
            });

            // === FlightPedals ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "FlightPedalsConfig",
                FieldPath = "flight_pedals",
                DisplayName = "Flight Pedals Config",
                Tooltip = "Complete flight pedals configuration (motion range, damping, spring)",
                FieldType = OverrideFieldType.Complex,
                Group = OverrideFieldGroup.FlightPedals,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.FlightPedalsConfig != null,
                GetValue = o => o.FlightPedalsConfig,
                SetValue = (o, v) => o.FlightPedalsConfig = (FlightPedalsConfig)v,
                ClearValue = o => o.FlightPedalsConfig = null,
                FormatValue = (val) =>
                {
                    var cfg = val as FlightPedalsConfig;
                    if (cfg == null) return "(not set)";
                    return $"Range: {cfg.PosNearLim}-{cfg.PosFarLim}mm, Damping: {cfg.Damping}, Spring: {cfg.CenteringSpringConst}";
                }
            });

            // === FlightStick ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "FlightStickConfig",
                FieldPath = "flight_stick",
                DisplayName = "Flight Stick Config",
                Tooltip = "Complete flight stick configuration (motion range, damping, spring)",
                FieldType = OverrideFieldType.Complex,
                Group = OverrideFieldGroup.FlightStick,
                DefaultLayer = ConfigLayer.User,
                HasValue = o => o.FlightStickConfig != null,
                GetValue = o => o.FlightStickConfig,
                SetValue = (o, v) => o.FlightStickConfig = (FlightStickPitchConfig)v,
                ClearValue = o => o.FlightStickConfig = null,
                FormatValue = (val) =>
                {
                    var cfg = val as FlightStickPitchConfig;
                    if (cfg == null) return "(not set)";
                    return $"Range: {cfg.PosMin}-{cfg.PosMax}mm, Damping: {cfg.Damping}, Spring: {cfg.CenteringSpringConst}";
                }
            });

            // === Shifter ===

            RegisterField(new OverrideFieldDefinition
            {
                Name = "ShifterConfig",
                FieldPath = "shifter_config",
                DisplayName = "Shifter Config",
                Tooltip = "Complete shifter configuration (geometry, gates, detents)",
                FieldType = OverrideFieldType.Complex,
                Group = OverrideFieldGroup.Shifter,
                DefaultLayer = ConfigLayer.Profile,
                HasValue = o => o.ShifterConfig != null,
                GetValue = o => o.ShifterConfig,
                SetValue = (o, v) => o.ShifterConfig = (ShifterConfig)v,
                ClearValue = o => o.ShifterConfig = null,
                FormatValue = (val) =>
                {
                    var cfg = val as ShifterConfig;
                    if (cfg == null) return "(not set)";
                    return cfg.Sequential ? "Sequential mode" : $"H-Pattern ({cfg.GateSegments?.Count ?? 0} gates)";
                }
            });
        }

        private static void RegisterField(OverrideFieldDefinition field)
        {
            _fieldsByPath[field.FieldPath] = field;
            _fieldsByName[field.Name] = field;
        }

        // === Field Definitions ===

        /// <summary>
        /// Get field definition by path or name (case-insensitive).
        /// </summary>
        /// <param name="fieldPath">Field path like "output_min" or name like "OutputMin"</param>
        /// <returns>Field definition or null if not found</returns>
        public static OverrideFieldDefinition GetField(string fieldPath)
        {
            if (string.IsNullOrEmpty(fieldPath))
                return null;

            // Try path lookup first
            if (_fieldsByPath.TryGetValue(fieldPath, out var field))
                return field;

            // Try name lookup
            if (_fieldsByName.TryGetValue(fieldPath, out field))
                return field;

            return null;
        }

        /// <summary>
        /// Get all registered field definitions.
        /// </summary>
        public static IEnumerable<OverrideFieldDefinition> GetAllFields()
        {
            return _fieldsByPath.Values;
        }

        /// <summary>
        /// Get field definitions for a specific group.
        /// </summary>
        public static IEnumerable<OverrideFieldDefinition> GetFieldsByGroup(OverrideFieldGroup group)
        {
            return _fieldsByPath.Values.Where(f => f.Group == group);
        }

        // === Layer Routing ===

        /// <summary>
        /// Get the target layer for a field (where edits should be saved by default).
        /// </summary>
        public static ConfigLayer GetTargetLayer(string fieldPath)
        {
            var field = GetField(fieldPath);
            if (field != null)
                return field.DefaultLayer;

            // Fallback to FieldRouter for unknown fields
            return FieldRouter.GetTargetLayer(fieldPath);
        }

        /// <summary>
        /// Check if a field is user-tunable (User layer).
        /// </summary>
        public static bool IsUserTunable(string fieldPath)
        {
            return GetTargetLayer(fieldPath) == ConfigLayer.User;
        }

        // === Value Operations ===

        /// <summary>
        /// Get the value of a field from overrides.
        /// </summary>
        /// <param name="overrides">Override container</param>
        /// <param name="fieldPath">Field path or name</param>
        /// <returns>Field value or null if not set</returns>
        public static object GetValue(FunctionConfigOverrides overrides, string fieldPath)
        {
            if (overrides == null)
                return null;

            var field = GetField(fieldPath);
            if (field?.GetValue == null)
                return null;

            return field.GetValue(overrides);
        }

        /// <summary>
        /// Set the value of a field in overrides.
        /// </summary>
        /// <param name="overrides">Override container</param>
        /// <param name="fieldPath">Field path or name</param>
        /// <param name="value">Value to set</param>
        public static void SetValue(FunctionConfigOverrides overrides, string fieldPath, object value)
        {
            if (overrides == null)
                throw new ArgumentNullException(nameof(overrides));

            var field = GetField(fieldPath);
            if (field?.SetValue == null)
                throw new ArgumentException($"Unknown field or field does not support SetValue: {fieldPath}", nameof(fieldPath));

            field.SetValue(overrides, value);
        }

        /// <summary>
        /// Clear the value of a field in overrides.
        /// </summary>
        /// <param name="overrides">Override container</param>
        /// <param name="fieldPath">Field path or name</param>
        public static void ClearValue(FunctionConfigOverrides overrides, string fieldPath)
        {
            if (overrides == null)
                return;

            var field = GetField(fieldPath);
            if (field?.ClearValue == null)
                return;

            field.ClearValue(overrides);
        }

        /// <summary>
        /// Check if a field has a non-null value in overrides.
        /// </summary>
        /// <param name="overrides">Override container</param>
        /// <param name="fieldPath">Field path or name</param>
        /// <returns>True if field has a value</returns>
        public static bool HasValue(FunctionConfigOverrides overrides, string fieldPath)
        {
            if (overrides == null)
                return false;

            var field = GetField(fieldPath);
            if (field?.HasValue == null)
                return false;

            return field.HasValue(overrides);
        }

        // === Path Normalization ===

        /// <summary>
        /// Normalize field name to field path.
        /// E.g., "OutputMin" -> "output_min"
        /// </summary>
        public static string NormalizeFieldPath(string fieldName)
        {
            if (string.IsNullOrEmpty(fieldName))
                return fieldName;

            var field = GetField(fieldName);
            return field?.FieldPath ?? fieldName.ToLowerInvariant();
        }
    }
}
