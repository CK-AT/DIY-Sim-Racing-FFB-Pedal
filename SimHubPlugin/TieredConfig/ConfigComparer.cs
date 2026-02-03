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
        /// </summary>
        public static bool AreEqual(FunctionConfigOverrides a, FunctionConfigOverrides b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;

            return NullableFloatEqual(a.OutputMin, b.OutputMin) &&
                   NullableFloatEqual(a.OutputMax, b.OutputMax) &&
                   NullableFloatEqual(a.SimulatedMass, b.SimulatedMass) &&
                   NullableFloatEqual(a.Friction, b.Friction) &&
                   AreEqual(a.StaticBalanceTuning, b.StaticBalanceTuning);
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
        /// Compare two AxisParameterOverrides for equality.
        /// Uses protobuf's built-in equality for the protobuf message fields.
        /// </summary>
        public static bool AreEqual(AxisParameterOverrides a, AxisParameterOverrides b)
        {
            if (a == null && b == null) return true;
            if (a == null || b == null) return false;

            return AreEqual(a.Kinematics, b.Kinematics) &&
                   AreEqual(a.StaticBalance, b.StaticBalance);
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
