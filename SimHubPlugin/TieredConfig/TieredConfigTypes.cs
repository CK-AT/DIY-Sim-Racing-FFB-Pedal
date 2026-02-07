using System.Collections.Generic;
using Newtonsoft.Json;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Shared protobuf JSON formatter/parser for types that don't round-trip through JSON.NET.
    /// </summary>
    internal static class ProtobufJsonHelper
    {
        internal static readonly Google.Protobuf.JsonFormatter Formatter =
            new Google.Protobuf.JsonFormatter(Google.Protobuf.JsonFormatter.Settings.Default);
        internal static readonly Google.Protobuf.JsonParser Parser =
            new Google.Protobuf.JsonParser(Google.Protobuf.JsonParser.Settings.Default);

        internal static string ToJson(Google.Protobuf.IMessage message)
        {
            return message == null ? null : Formatter.Format(message);
        }

        internal static T FromJson<T>(string json) where T : Google.Protobuf.IMessage<T>, new()
        {
            if (string.IsNullOrEmpty(json)) return default;
            try { return Parser.Parse<T>(json); }
            catch { return default; }
        }
    }
    /// <summary>
    /// Configuration layer in the override hierarchy.
    /// Resolution order: User > Profile > Baseline (first non-null wins).
    /// </summary>
    public enum ConfigLayer
    {
        Baseline,   // ESP32 EEPROM defaults
        Profile,    // Vehicle-specific settings (AircraftFfbProfile)
        User        // Personal preferences (UserPreferences)
    }

    /// <summary>
    /// User-level preferences that follow the user across vehicles.
    /// Contains function config overrides for personal tuning.
    /// </summary>
    public class UserPreferences
    {
        /// <summary>
        /// Function config overrides keyed by function ID.
        /// Only user-tunable fields are stored here.
        /// </summary>
        public Dictionary<int, FunctionConfigOverrides> FunctionOverrides { get; set; }
            = new Dictionary<int, FunctionConfigOverrides>();
    }

    /// <summary>
    /// Delta overlay for FunctionConfig. All fields are nullable.
    /// Non-null values override the corresponding field in the base config.
    /// </summary>
    public class FunctionConfigOverrides
    {
        // Output scaling (user-tunable)
        public float? OutputMin { get; set; }
        public float? OutputMax { get; set; }

        // Common physics parameters (user-tunable)
        public float? SimulatedMass { get; set; }
        public float? Friction { get; set; }

        // Static balance tuning (user-tunable)
        public StaticBalanceTuningOverrides StaticBalanceTuning { get; set; }

        // AutomotivePedals overrides
        [JsonIgnore]
        public SplineForceCurveConfig ForceCurve { get; set; }
        [JsonProperty("ForceCurveJson")]
        public string ForceCurveJson
        {
            get => ProtobufJsonHelper.ToJson(ForceCurve);
            set => ForceCurve = ProtobufJsonHelper.FromJson<SplineForceCurveConfig>(value);
        }
        public DamperConfigOverrides DamperConfig { get; set; }

        // FlightPedals overrides
        public MotionRangeOverrides FlightPedalsMotionRange { get; set; }
        public float? FlightPedalsDamping { get; set; }
        public float? FlightPedalsCenteringSpringConst { get; set; }

        // FlightStick overrides (Pitch/Roll/Collective all use same structure)
        public MotionRangeOverrides FlightStickMotionRange { get; set; }
        public float? FlightStickDamping { get; set; }
        public float? FlightStickCenteringSpringConst { get; set; }

        // RudderBrake overrides (aux_function in FlightPedals)
        public ForceRangeOverrides RudderBrakeForceRange { get; set; }

        // Shifter overrides
        [JsonIgnore]
        public ShifterConfig ShifterConfig { get; set; }
        [JsonProperty("ShifterConfigJson")]
        public string ShifterConfigJson
        {
            get => ProtobufJsonHelper.ToJson(ShifterConfig);
            set => ShifterConfig = ProtobufJsonHelper.FromJson<ShifterConfig>(value);
        }
        [JsonIgnore]
        public ShifterDetectConfig ShifterDetectConfig { get; set; }
        [JsonProperty("ShifterDetectConfigJson")]
        public string ShifterDetectConfigJson
        {
            get => ProtobufJsonHelper.ToJson(ShifterDetectConfig);
            set => ShifterDetectConfig = ProtobufJsonHelper.FromJson<ShifterDetectConfig>(value);
        }

        /// <summary>
        /// Returns true if all override fields are null/empty.
        /// </summary>
        public bool IsEmpty =>
            OutputMin == null &&
            OutputMax == null &&
            SimulatedMass == null &&
            Friction == null &&
            (StaticBalanceTuning == null || StaticBalanceTuning.IsEmpty) &&
            ForceCurve == null &&
            (DamperConfig == null || DamperConfig.IsEmpty) &&
            (FlightPedalsMotionRange == null || FlightPedalsMotionRange.IsEmpty) &&
            FlightPedalsDamping == null &&
            FlightPedalsCenteringSpringConst == null &&
            (FlightStickMotionRange == null || FlightStickMotionRange.IsEmpty) &&
            FlightStickDamping == null &&
            FlightStickCenteringSpringConst == null &&
            (RudderBrakeForceRange == null || RudderBrakeForceRange.IsEmpty) &&
            ShifterConfig == null &&
            ShifterDetectConfig == null;
    }

    /// <summary>
    /// Delta overlay for StaticBalanceTuning parameters.
    /// Matches FunctionConfig.Types.StaticBalanceTuning protobuf fields.
    /// </summary>
    public class StaticBalanceTuningOverrides
    {
        public bool? Enabled { get; set; }
        public float? Gain { get; set; }

        public bool IsEmpty => Enabled == null && Gain == null;
    }

    /// <summary>
    /// Delta overlay for DamperConfig parameters (AutomotivePedals).
    /// </summary>
    public class DamperConfigOverrides
    {
        public float? PositiveFactor { get; set; }
        public float? NegativeFactor { get; set; }

        public bool IsEmpty => PositiveFactor == null && NegativeFactor == null;
    }

    /// <summary>
    /// Delta overlay for motion range (FlightPedals/FlightStick).
    /// Stores position limits as a pair.
    /// </summary>
    public class MotionRangeOverrides
    {
        public int? NearLim { get; set; }  // FlightPedals: pos_near_lim
        public int? FarLim { get; set; }   // FlightPedals: pos_far_lim

        public int? Min { get; set; }
        public int? Max { get; set; }

        public bool IsEmpty => NearLim == null && FarLim == null && Min == null && Max == null;
    }

    /// <summary>
    /// Delta overlay for rudder brake force range.
    /// </summary>
    public class ForceRangeOverrides
    {
        public float? Min { get; set; }  // f_min
        public float? Max { get; set; }  // f_max

        public bool IsEmpty => Min == null && Max == null;
    }

    /// <summary>
    /// Per-axis parameter overrides for use by functions.
    /// Allows functions to override axis physics without modifying the hardware config.
    /// Uses protobuf types directly for serialization compatibility.
    /// </summary>
    public class AxisParameterOverrides
    {
        /// <summary>
        /// Kinematic parameters override (linkage geometry, travel limits).
        /// If non-null, replaces the axis's kinematic_parameters entirely.
        /// </summary>
        [JsonIgnore]
        public KinematicParameters Kinematics { get; set; }
        [JsonProperty("KinematicsJson")]
        public string KinematicsJson
        {
            get => ProtobufJsonHelper.ToJson(Kinematics);
            set => Kinematics = ProtobufJsonHelper.FromJson<KinematicParameters>(value);
        }

        /// <summary>
        /// The GeneralKinematicConfig geometry that produced these Kinematics.
        /// Stored as JSON string because protobuf RepeatedField doesn't round-trip through JSON.NET.
        /// Used to restore the kinematics editor when switching functions.
        /// </summary>
        public string GeometryJson { get; set; }

        /// <summary>
        /// Static balance config override (position-dependent force compensation).
        /// If non-null, replaces the axis's static_balance_config entirely.
        /// </summary>
        [JsonIgnore]
        public AxisConfig.Types.StaticBalanceConfig StaticBalance { get; set; }
        [JsonProperty("StaticBalanceJson")]
        public string StaticBalanceJson
        {
            get => ProtobufJsonHelper.ToJson(StaticBalance);
            set => StaticBalance = ProtobufJsonHelper.FromJson<AxisConfig.Types.StaticBalanceConfig>(value);
        }

        /// <summary>
        /// Returns true if no overrides are defined.
        /// </summary>
        public bool IsEmpty => Kinematics == null && StaticBalance == null;

        /// <summary>
        /// Returns true if this override has any effective content.
        /// </summary>
        public bool HasOverrides => !IsEmpty;
    }
}
