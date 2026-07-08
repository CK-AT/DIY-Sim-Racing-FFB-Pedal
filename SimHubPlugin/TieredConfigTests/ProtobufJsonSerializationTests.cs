using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb.TieredConfig;
using Newtonsoft.Json;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Verifies that protobuf types with RepeatedField&lt;T&gt; survive JSON.NET round-trips
    /// through the [JsonIgnore] + companion *Json property pattern.
    /// </summary>
    public static class ProtobufJsonSerializationTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // AxisParameterOverrides
                TestRunner.RunTest("AxisOverrides_KinematicsRoundTrip", AxisOverrides_KinematicsRoundTrip),
                TestRunner.RunTest("AxisOverrides_StaticBalanceRoundTrip", AxisOverrides_StaticBalanceRoundTrip),
                TestRunner.RunTest("AxisOverrides_NullFieldsRoundTrip", AxisOverrides_NullFieldsRoundTrip),
                TestRunner.RunTest("AxisOverrides_MixedNullRoundTrip", AxisOverrides_MixedNullRoundTrip),

                // FunctionConfigOverrides
                TestRunner.RunTest("FuncOverrides_ForceCurveRoundTrip", FuncOverrides_ForceCurveRoundTrip),
                TestRunner.RunTest("FuncOverrides_ShifterConfigRoundTrip", FuncOverrides_ShifterConfigRoundTrip),
                TestRunner.RunTest("FuncOverrides_ShifterDetectConfigRoundTrip", FuncOverrides_ShifterDetectConfigRoundTrip),
                TestRunner.RunTest("FuncOverrides_AbsEffectRoundTrip", FuncOverrides_AbsEffectRoundTrip),

                // Plan 10 legacy JSON migration (FlightStick* / FlightPedals* / NearLim/FarLim)
                TestRunner.RunTest("FuncOverrides_LegacyFlightStickJsonMigrates", FuncOverrides_LegacyFlightStickJsonMigrates),
                TestRunner.RunTest("FuncOverrides_LegacyFlightPedalsJsonMigrates", FuncOverrides_LegacyFlightPedalsJsonMigrates),
                TestRunner.RunTest("FuncOverrides_LegacyJsonRoundTripStripsOldKeys", FuncOverrides_LegacyJsonRoundTripStripsOldKeys),
                TestRunner.RunTest("MotionRange_LegacyNearFarLimMigrates", MotionRange_LegacyNearFarLimMigrates),
            };
        }

        // === AxisParameterOverrides ===

        private static void AxisOverrides_KinematicsRoundTrip()
        {
            var original = new AxisParameterOverrides
            {
                Kinematics = new KinematicParameters
                {
                    ContactPointPosMinAbs = -100,
                    ContactPointPosMaxAbs = 100,
                }
            };
            original.Kinematics.CoeffsForceFactorOverContactPointPos.AddRange(new[] { 1.0, 0.5, 0.25 });
            original.Kinematics.CoeffsSledPosOverContactPointPos.AddRange(new[] { 0.1, 0.2, 0.3, 0.4, 0.5 });

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<AxisParameterOverrides>(json);

            AssertNotNull(restored.Kinematics, "Kinematics should not be null");
            AssertEqual(-100, restored.Kinematics.ContactPointPosMinAbs, "ContactPointPosMinAbs");
            AssertEqual(100, restored.Kinematics.ContactPointPosMaxAbs, "ContactPointPosMaxAbs");
            AssertSequenceEqual(
                new[] { 1.0, 0.5, 0.25 },
                restored.Kinematics.CoeffsForceFactorOverContactPointPos.ToArray(),
                "CoeffsForceFactorOverContactPointPos");
            AssertSequenceEqual(
                new[] { 0.1, 0.2, 0.3, 0.4, 0.5 },
                restored.Kinematics.CoeffsSledPosOverContactPointPos.ToArray(),
                "CoeffsSledPosOverContactPointPos");
        }

        private static void AxisOverrides_StaticBalanceRoundTrip()
        {
            var original = new AxisParameterOverrides
            {
                StaticBalance = new AxisConfig.Types.StaticBalanceConfig
                {
                    XCenter = 50.0f,
                    XHalfRange = 25.0f,
                }
            };
            original.StaticBalance.Coeffs.AddRange(new[] { 0.1f, -0.2f, 0.3f, -0.4f });

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<AxisParameterOverrides>(json);

            AssertNotNull(restored.StaticBalance, "StaticBalance should not be null");
            AssertEqual(50.0f, restored.StaticBalance.XCenter, "XCenter");
            AssertEqual(25.0f, restored.StaticBalance.XHalfRange, "XHalfRange");
            AssertSequenceEqual(
                new[] { 0.1f, -0.2f, 0.3f, -0.4f },
                restored.StaticBalance.Coeffs.ToArray(),
                "Coeffs");
        }

        private static void AxisOverrides_NullFieldsRoundTrip()
        {
            var original = new AxisParameterOverrides();

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<AxisParameterOverrides>(json);

            AssertNull(restored.Kinematics, "Kinematics should remain null");
            AssertNull(restored.StaticBalance, "StaticBalance should remain null");
            AssertTrue(restored.IsEmpty, "Should be empty");
        }

        private static void AxisOverrides_MixedNullRoundTrip()
        {
            var original = new AxisParameterOverrides
            {
                Kinematics = new KinematicParameters { ContactPointPosMinAbs = -50 },
                StaticBalance = null
            };
            original.Kinematics.CoeffsForceFactorOverContactPointPos.Add(2.0);

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<AxisParameterOverrides>(json);

            AssertNotNull(restored.Kinematics, "Kinematics should be preserved");
            AssertEqual(1, restored.Kinematics.CoeffsForceFactorOverContactPointPos.Count,
                "Should have 1 coefficient");
            AssertNull(restored.StaticBalance, "StaticBalance should remain null");
        }

        // === FunctionConfigOverrides ===

        private static void FuncOverrides_ForceCurveRoundTrip()
        {
            var original = new FunctionConfigOverrides
            {
                ForceCurve = new SplineForceCurveConfig
                {
                    PosMin = 0,
                    PosMax = 100,
                    FMin = 5.0f,
                    FMax = 80.0f,
                }
            };
            original.ForceCurve.FRelPoints.AddRange(new uint[] { 10, 25, 50, 75, 90, 100 });
            original.ForceCurve.CubicSplineParamsA.AddRange(new[] { 0.1f, 0.2f, 0.3f });
            original.ForceCurve.CubicSplineParamsB.AddRange(new[] { 0.4f, 0.5f, 0.6f });

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(json);

            AssertNotNull(restored.ForceCurve, "ForceCurve should not be null");
            AssertEqual(0, restored.ForceCurve.PosMin, "PosMin");
            AssertEqual(100, restored.ForceCurve.PosMax, "PosMax");
            AssertEqual(5.0f, restored.ForceCurve.FMin, "FMin");
            AssertEqual(80.0f, restored.ForceCurve.FMax, "FMax");
            AssertSequenceEqual(
                new uint[] { 10, 25, 50, 75, 90, 100 },
                restored.ForceCurve.FRelPoints.ToArray(),
                "FRelPoints");
            AssertSequenceEqual(
                new[] { 0.1f, 0.2f, 0.3f },
                restored.ForceCurve.CubicSplineParamsA.ToArray(),
                "CubicSplineParamsA");
            AssertSequenceEqual(
                new[] { 0.4f, 0.5f, 0.6f },
                restored.ForceCurve.CubicSplineParamsB.ToArray(),
                "CubicSplineParamsB");
        }

        private static void FuncOverrides_ShifterConfigRoundTrip()
        {
            var original = new FunctionConfigOverrides
            {
                ShifterConfig = new ShifterConfig
                {
                    PosXMin = -30,
                    PosXMax = 30,
                    PosYMin = -50,
                    PosYMax = 50,
                    Damping = 1.5f,
                    MaxForce = 10.0f,
                }
            };
            original.ShifterConfig.GateSegments.Add(new ShifterGateSegment
            {
                X0 = 0, Y0 = -50, X1 = 0, Y1 = 50,
                HalfWidth = 5, SpringCenter = 2.0f, SpringWall = 8.0f
            });
            original.ShifterConfig.Detents.Add(new ShifterDetentPoint
            {
                X = 0, Y = 40, Radius = 5, Spring = 3.0f
            });

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(json);

            AssertNotNull(restored.ShifterConfig, "ShifterConfig should not be null");
            AssertEqual(-30, restored.ShifterConfig.PosXMin, "PosXMin");
            AssertEqual(1, restored.ShifterConfig.GateSegments.Count, "GateSegments count");
            AssertEqual(0, restored.ShifterConfig.GateSegments[0].X0, "GateSegment.X0");
            AssertEqual(5u, restored.ShifterConfig.GateSegments[0].HalfWidth, "GateSegment.HalfWidth");
            AssertEqual(1, restored.ShifterConfig.Detents.Count, "Detents count");
            AssertEqual(40, restored.ShifterConfig.Detents[0].Y, "Detent.Y");
        }

        private static void FuncOverrides_ShifterDetectConfigRoundTrip()
        {
            var original = new FunctionConfigOverrides
            {
                ShifterDetectConfig = new ShifterDetectConfig
                {
                    Hysteresis = 10,
                }
            };
            original.ShifterDetectConfig.GearSlots.Add(new ShifterGearSlot
            {
                CenterX = 0, CenterY = 40, HalfWidth = 8, HalfHeight = 10, Gear = ShifterGear._1
            });
            original.ShifterDetectConfig.GearSlots.Add(new ShifterGearSlot
            {
                CenterX = 0, CenterY = -40, HalfWidth = 8, HalfHeight = 10, Gear = ShifterGear._2
            });

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(json);

            AssertNotNull(restored.ShifterDetectConfig, "ShifterDetectConfig should not be null");
            AssertEqual(10u, restored.ShifterDetectConfig.Hysteresis, "Hysteresis");
            AssertEqual(2, restored.ShifterDetectConfig.GearSlots.Count, "GearSlots count");
            AssertEqual(ShifterGear._1, restored.ShifterDetectConfig.GearSlots[0].Gear, "Slot[0].Gear");
            AssertEqual(ShifterGear._2, restored.ShifterDetectConfig.GearSlots[1].Gear, "Slot[1].Gear");
            AssertEqual(-40, restored.ShifterDetectConfig.GearSlots[1].CenterY, "Slot[1].CenterY");
        }

        private static void FuncOverrides_AbsEffectRoundTrip()
        {
            var original = new FunctionConfigOverrides
            {
                AbsEffect = new ABSEffectConfig
                {
                    Enabled = true,
                    Mode = ABSMode.Force,
                    Freq = 30,
                    Ampl = 50,
                    Pattern = ABSPattern.Sawtooth,
                    SimLevel = 75,
                }
            };

            var json = JsonConvert.SerializeObject(original);
            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(json);

            AssertNotNull(restored.AbsEffect, "AbsEffect should not be null");
            AssertTrue(restored.AbsEffect.Enabled, "Enabled");
            AssertEqual(ABSMode.Force, restored.AbsEffect.Mode, "Mode");
            AssertEqual(30u, restored.AbsEffect.Freq, "Freq");
            AssertEqual(50u, restored.AbsEffect.Ampl, "Ampl");
            AssertEqual(ABSPattern.Sawtooth, restored.AbsEffect.Pattern, "Pattern");
            AssertEqual(75u, restored.AbsEffect.SimLevel, "SimLevel");
        }

        // === Plan 10 legacy JSON migration ===

        // FunctionConfigOverrides.OnDeserialized routes legacy property names captured
        // by [JsonExtensionData] into the new FlightControl* properties. Verifies a
        // saved profile from before Plan 10 deserializes into the new shape.
        private static void FuncOverrides_LegacyFlightStickJsonMigrates()
        {
            string legacyJson = "{\"FlightStickDamping\":1.25,\"FlightStickCenteringSpringConst\":3.5," +
                                "\"FlightStickPhaseOffset\":90.0," +
                                "\"FlightStickMotionRange\":{\"Min\":-30,\"Max\":30}," +
                                "\"FlightStickVibHarmonicRatios\":[1.0,2.0,3.0,null,null]," +
                                "\"FlightStickVib2HarmonicRatios\":[0.5,1.5]}";

            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(legacyJson);

            AssertEqual(1.25f, restored.FlightControlDamping ?? 0f, "Damping migrated");
            AssertEqual(3.5f, restored.FlightControlCenteringSpringConst ?? 0f, "Centering spring migrated");
            AssertEqual(90.0f, restored.FlightControlPhaseOffset ?? 0f, "Phase offset migrated");
            AssertNotNull(restored.FlightControlMotionRange, "MotionRange migrated");
            AssertEqual(-30, restored.FlightControlMotionRange.Min ?? 0, "MotionRange.Min");
            AssertEqual(30, restored.FlightControlMotionRange.Max ?? 0, "MotionRange.Max");
            AssertNotNull(restored.FlightControlVibHarmonicRatios, "Vib1 harm ratios migrated");
            AssertEqual(1.0f, restored.FlightControlVibHarmonicRatios[0] ?? 0f, "Vib1 slot 0");
            AssertEqual(2.0f, restored.FlightControlVibHarmonicRatios[1] ?? 0f, "Vib1 slot 1");
            AssertEqual(3.0f, restored.FlightControlVibHarmonicRatios[2] ?? 0f, "Vib1 slot 2");
            AssertNotNull(restored.FlightControlVib2HarmonicRatios, "Vib2 harm ratios migrated");
            AssertEqual(0.5f, restored.FlightControlVib2HarmonicRatios[0] ?? 0f, "Vib2 slot 0");
            AssertEqual(1.5f, restored.FlightControlVib2HarmonicRatios[1] ?? 0f, "Vib2 slot 1");
        }

        private static void FuncOverrides_LegacyFlightPedalsJsonMigrates()
        {
            // Pre-Plan 10 pedal profile: FlightPedalsDamping/CenteringSpringConst at top
            // level, MotionRange uses NearLim/FarLim instead of Min/Max.
            string legacyJson = "{\"FlightPedalsDamping\":0.75,\"FlightPedalsCenteringSpringConst\":2.1," +
                                "\"FlightPedalsMotionRange\":{\"NearLim\":5,\"FarLim\":85}}";

            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(legacyJson);

            AssertEqual(0.75f, restored.FlightControlDamping ?? 0f, "Pedal damping migrated");
            AssertEqual(2.1f, restored.FlightControlCenteringSpringConst ?? 0f, "Pedal centering spring migrated");
            AssertNotNull(restored.FlightControlMotionRange, "Pedal motion range migrated");
            AssertEqual(5, restored.FlightControlMotionRange.Min ?? 0, "NearLim migrated to Min");
            AssertEqual(85, restored.FlightControlMotionRange.Max ?? 0, "FarLim migrated to Max");
        }

        // After load+resave the JSON should contain only the new property names —
        // the [JsonExtensionData] dict is set to null in OnDeserialized to prevent
        // legacy keys from re-emerging.
        private static void FuncOverrides_LegacyJsonRoundTripStripsOldKeys()
        {
            string legacyJson = "{\"FlightStickDamping\":1.0,\"FlightPedalsDamping\":2.0}";

            var restored = JsonConvert.DeserializeObject<FunctionConfigOverrides>(legacyJson);
            string resaved = JsonConvert.SerializeObject(restored);

            AssertTrue(!resaved.Contains("FlightStickDamping"),
                "Resaved JSON must not contain legacy FlightStickDamping key");
            AssertTrue(!resaved.Contains("FlightPedalsDamping"),
                "Resaved JSON must not contain legacy FlightPedalsDamping key");
            AssertTrue(resaved.Contains("FlightControlDamping"),
                "Resaved JSON should contain canonical FlightControlDamping key");
        }

        // MotionRangeOverrides has its own OnDeserialized that maps NearLim/FarLim
        // (legacy pedal field names) into Min/Max.
        private static void MotionRange_LegacyNearFarLimMigrates()
        {
            string legacyJson = "{\"NearLim\":12,\"FarLim\":78}";

            var restored = JsonConvert.DeserializeObject<MotionRangeOverrides>(legacyJson);

            AssertEqual(12, restored.Min ?? 0, "NearLim should migrate to Min");
            AssertEqual(78, restored.Max ?? 0, "FarLim should migrate to Max");
        }

        // === Assertion helpers ===

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition) throw new InvalidOperationException($"AssertTrue failed: {message}");
        }

        private static void AssertEqual<T>(T expected, T actual, string message)
        {
            if (!EqualityComparer<T>.Default.Equals(expected, actual))
                throw new InvalidOperationException($"AssertEqual failed: {message}. Expected={expected}, Actual={actual}");
        }

        private static void AssertNotNull(object obj, string message)
        {
            if (obj == null) throw new InvalidOperationException($"AssertNotNull failed: {message}");
        }

        private static void AssertNull(object obj, string message)
        {
            if (obj != null) throw new InvalidOperationException($"AssertNull failed: {message}");
        }

        private static void AssertSequenceEqual<T>(T[] expected, T[] actual, string message)
        {
            if (!expected.SequenceEqual(actual))
            {
                var expectedStr = string.Join(", ", expected);
                var actualStr = string.Join(", ", actual);
                throw new InvalidOperationException(
                    $"AssertSequenceEqual failed: {message}. Expected=[{expectedStr}], Actual=[{actualStr}]");
            }
        }
    }
}
