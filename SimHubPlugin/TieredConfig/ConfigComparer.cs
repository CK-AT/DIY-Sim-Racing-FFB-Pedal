using System;
using System.Collections.Generic;
using Google.Protobuf;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Pure functions for comparing configs to detect changes.
    /// Used for diff-checking before sending configs to ESP32.
    /// </summary>
    public static class ConfigComparer
    {
        /// <summary>
        /// Floating point comparison tolerance for config values.
        /// Chosen to handle typical floating point arithmetic errors.
        /// </summary>
        public const float FloatTolerance = 1e-6f;

        /// <summary>
        /// Compare two protobuf messages for equality.
        /// Uses protobuf's built-in deep comparison.
        /// </summary>
        public static bool AreEqual<T>(T a, T b) where T : IMessage<T>
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;
            return a.Equals(b);
        }

        /// <summary>
        /// Compare two AxisConfig objects for equality.
        /// </summary>
        public static bool AreEqual(AxisConfig a, AxisConfig b)
        {
            return AreEqual<AxisConfig>(a, b);
        }

        /// <summary>
        /// Compare two FunctionConfig objects for equality.
        /// </summary>
        public static bool AreEqual(FunctionConfig a, FunctionConfig b)
        {
            return AreEqual<FunctionConfig>(a, b);
        }

        /// <summary>
        /// Compare two FunctionConfigOverrides for equality.
        /// Checks all override fields including protobuf-backed ones.
        /// </summary>
        public static bool AreEqual(FunctionConfigOverrides a, FunctionConfigOverrides b)
        {
            if (a == null && b == null) return true;
            if ((a == null || a.IsEmpty) && (b == null || b.IsEmpty)) return true;
            if (a == null || b == null) return false;

            return NullableFloatEqual(a.OutputMin, b.OutputMin) &&
                   NullableFloatEqual(a.OutputMax, b.OutputMax) &&
                   NullableFloatEqual(a.SimulatedMass, b.SimulatedMass) &&
                   NullableFloatEqual(a.Friction, b.Friction) &&
                   AreEqual(a.StaticBalanceTuning, b.StaticBalanceTuning) &&
                   AreEqual(a.ForceCurve, b.ForceCurve) &&
                   AreEqual(a.DamperConfig, b.DamperConfig) &&
                   AreEqual(a.FlightControlMotionRange, b.FlightControlMotionRange) &&
                   NullableFloatEqual(a.FlightControlDamping, b.FlightControlDamping) &&
                   NullableFloatEqual(a.FlightControlCenteringSpringConst, b.FlightControlCenteringSpringConst) &&
                   NullableFloatEqual(a.FlightControlPhaseOffset, b.FlightControlPhaseOffset) &&
                   NullableFloatArrayEqual(a.FlightControlVibHarmonicRatios, b.FlightControlVibHarmonicRatios) &&
                   NullableFloatArrayEqual(a.FlightControlVib2HarmonicRatios, b.FlightControlVib2HarmonicRatios) &&
                   AreEqual(a.RudderBrakeForceRange, b.RudderBrakeForceRange) &&
                   AreEqual(a.AbsEffect, b.AbsEffect) &&
                   AreEqual(a.ShifterConfig, b.ShifterConfig) &&
                   AreEqual(a.ShifterDetectConfig, b.ShifterDetectConfig);
        }

        /// <summary>
        /// Compare two StaticBalanceTuningOverrides for equality.
        /// </summary>
        public static bool AreEqual(StaticBalanceTuningOverrides a, StaticBalanceTuningOverrides b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;

            return a.Enabled == b.Enabled &&
                   NullableFloatEqual(a.Gain, b.Gain);
        }

        /// <summary>
        /// Compare two DamperConfigOverrides for equality.
        /// </summary>
        public static bool AreEqual(DamperConfigOverrides a, DamperConfigOverrides b)
        {
            if (a == null && b == null) return true;
            if ((a == null || a.IsEmpty) && (b == null || b.IsEmpty)) return true;
            if (a == null || b == null) return false;

            return NullableFloatEqual(a.PositiveFactor, b.PositiveFactor) &&
                   NullableFloatEqual(a.NegativeFactor, b.NegativeFactor);
        }

        /// <summary>
        /// Compare two MotionRangeOverrides for equality.
        /// </summary>
        public static bool AreEqual(MotionRangeOverrides a, MotionRangeOverrides b)
        {
            if (a == null && b == null) return true;
            if ((a == null || a.IsEmpty) && (b == null || b.IsEmpty)) return true;
            if (a == null || b == null) return false;

            return NullableIntEqual(a.Min, b.Min) &&
                   NullableIntEqual(a.Max, b.Max);
        }

        /// <summary>
        /// Compare two ForceRangeOverrides for equality.
        /// </summary>
        public static bool AreEqual(ForceRangeOverrides a, ForceRangeOverrides b)
        {
            if (a == null && b == null) return true;
            if ((a == null || a.IsEmpty) && (b == null || b.IsEmpty)) return true;
            if (a == null || b == null) return false;

            return NullableFloatEqual(a.Min, b.Min) &&
                   NullableFloatEqual(a.Max, b.Max);
        }

        /// <summary>
        /// Compare two AxisParameterOverrides for equality.
        /// Uses protobuf's built-in equality for the protobuf message fields.
        /// </summary>
        public static bool AreEqual(AxisParameterOverrides a, AxisParameterOverrides b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;

            return AreEqual(a.Kinematics, b.Kinematics) &&
                   AreEqual(a.StaticBalance, b.StaticBalance) &&
                   AreEqualProto(a.OscillationGuard, b.OscillationGuard) &&
                   NullableFloatEqual(a.MinDamping, b.MinDamping);
        }

        private static bool AreEqualProto(AxisConfig.Types.OscillationGuard a, AxisConfig.Types.OscillationGuard b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;
            return a.Equals(b);
        }

        /// <summary>
        /// Compare two KinematicParameters for equality.
        /// </summary>
        public static bool AreEqual(KinematicParameters a, KinematicParameters b)
        {
            return AreEqual<KinematicParameters>(a, b);
        }

        /// <summary>
        /// Compare two StaticBalanceConfig for equality.
        /// </summary>
        public static bool AreEqual(AxisConfig.Types.StaticBalanceConfig a, AxisConfig.Types.StaticBalanceConfig b)
        {
            return AreEqual<AxisConfig.Types.StaticBalanceConfig>(a, b);
        }

        /// <summary>
        /// Compare two nullable floats with tolerance for floating point errors.
        /// </summary>
        public static bool NullableFloatEqual(float? a, float? b)
        {
            if (!a.HasValue && !b.HasValue) return true;
            if (!a.HasValue || !b.HasValue) return false;
            return FloatEqual(a.Value, b.Value);
        }

        /// <summary>
        /// Compare two nullable float arrays element-wise with tolerance.
        /// Both null is equal; one null vs empty/all-null on the other side is also equal.
        /// </summary>
        public static bool NullableFloatArrayEqual(float?[] a, float?[] b)
        {
            int aLen = a?.Length ?? 0;
            int bLen = b?.Length ?? 0;
            int len = aLen > bLen ? aLen : bLen;
            for (int i = 0; i < len; i++)
            {
                float? av = (a != null && i < a.Length) ? a[i] : null;
                float? bv = (b != null && i < b.Length) ? b[i] : null;
                if (!NullableFloatEqual(av, bv)) return false;
            }
            return true;
        }

        /// <summary>
        /// Compare two nullable ints for equality.
        /// </summary>
        public static bool NullableIntEqual(int? a, int? b)
        {
            if (!a.HasValue && !b.HasValue) return true;
            if (!a.HasValue || !b.HasValue) return false;
            return a.Value == b.Value;
        }

        /// <summary>
        /// Compare two floats with tolerance for floating point errors.
        /// </summary>
        public static bool FloatEqual(float a, float b)
        {
            return Math.Abs(a - b) < FloatTolerance;
        }

        /// <summary>
        /// Compare two lists of floats with tolerance.
        /// </summary>
        public static bool ListsEqual(IList<float> a, IList<float> b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;
            if (a.Count != b.Count) return false;

            for (int i = 0; i < a.Count; i++)
            {
                if (!FloatEqual(a[i], b[i]))
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Check if a FunctionConfig has meaningful differences from another.
        /// Used for diff-checking before automatic sends.
        /// </summary>
        public static bool HasChanges(FunctionConfig current, FunctionConfig lastSent)
        {
            return !AreEqual(current, lastSent);
        }

        /// <summary>
        /// Check if an AxisConfig has meaningful differences from another.
        /// Used for diff-checking before automatic sends.
        /// </summary>
        public static bool HasChanges(AxisConfig current, AxisConfig lastSent)
        {
            return !AreEqual(current, lastSent);
        }
    }
}
