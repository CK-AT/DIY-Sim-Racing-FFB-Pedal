using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for ConfigComparer pure functions.
    /// </summary>
    public static class ConfigComparerTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // AxisConfig equality tests
                TestRunner.RunTest("AreEqual_IdenticalAxisConfigs_ReturnsTrue", AreEqual_IdenticalAxisConfigs_ReturnsTrue),
                TestRunner.RunTest("AreEqual_DifferentKinematics_ReturnsFalse", AreEqual_DifferentKinematics_ReturnsFalse),
                TestRunner.RunTest("AreEqual_DifferentStaticBalance_ReturnsFalse", AreEqual_DifferentStaticBalance_ReturnsFalse),
                TestRunner.RunTest("AreEqual_BothNullAxisConfigs_ReturnsTrue", AreEqual_BothNullAxisConfigs_ReturnsTrue),
                TestRunner.RunTest("AreEqual_OneNullAxisConfig_ReturnsFalse", AreEqual_OneNullAxisConfig_ReturnsFalse),

                // FunctionConfig equality tests
                TestRunner.RunTest("AreEqual_IdenticalFunctionConfigs_ReturnsTrue", AreEqual_IdenticalFunctionConfigs_ReturnsTrue),
                TestRunner.RunTest("AreEqual_DifferentOutputMin_ReturnsFalse", AreEqual_DifferentOutputMin_ReturnsFalse),

                // FunctionConfigOverrides equality tests
                TestRunner.RunTest("AreEqual_IdenticalOverrides_ReturnsTrue", AreEqual_IdenticalOverrides_ReturnsTrue),
                TestRunner.RunTest("AreEqual_DifferentOverrides_ReturnsFalse", AreEqual_DifferentOverrides_ReturnsFalse),
                TestRunner.RunTest("AreEqual_BothNullOverrides_ReturnsTrue", AreEqual_BothNullOverrides_ReturnsTrue),
                TestRunner.RunTest("AreEqual_OneNullOverrides_ReturnsFalse", AreEqual_OneNullOverrides_ReturnsFalse),
                TestRunner.RunTest("AreEqual_EmptyOverrides_AreEqual", AreEqual_EmptyOverrides_AreEqual),

                // AxisParameterOverrides equality tests
                TestRunner.RunTest("AreEqual_IdenticalAxisOverrides_ReturnsTrue", AreEqual_IdenticalAxisOverrides_ReturnsTrue),
                TestRunner.RunTest("AreEqual_DifferentAxisOverrides_ReturnsFalse", AreEqual_DifferentAxisOverrides_ReturnsFalse),
                TestRunner.RunTest("AreEqual_BothNullAxisOverrides_ReturnsTrue", AreEqual_BothNullAxisOverrides_ReturnsTrue),
                TestRunner.RunTest("AreEqual_EmptyAxisOverrides_AreEqual", AreEqual_EmptyAxisOverrides_AreEqual),

                // Float comparison tests
                TestRunner.RunTest("FloatEqual_IdenticalValues_ReturnsTrue", FloatEqual_IdenticalValues_ReturnsTrue),
                TestRunner.RunTest("FloatEqual_WithinTolerance_ReturnsTrue", FloatEqual_WithinTolerance_ReturnsTrue),
                TestRunner.RunTest("FloatEqual_OutsideTolerance_ReturnsFalse", FloatEqual_OutsideTolerance_ReturnsFalse),
                TestRunner.RunTest("NullableFloatEqual_BothNull_ReturnsTrue", NullableFloatEqual_BothNull_ReturnsTrue),
                TestRunner.RunTest("NullableFloatEqual_OneNull_ReturnsFalse", NullableFloatEqual_OneNull_ReturnsFalse),

                // List comparison tests
                TestRunner.RunTest("ListsEqual_IdenticalLists_ReturnsTrue", ListsEqual_IdenticalLists_ReturnsTrue),
                TestRunner.RunTest("ListsEqual_DifferentLengths_ReturnsFalse", ListsEqual_DifferentLengths_ReturnsFalse),
                TestRunner.RunTest("ListsEqual_DifferentValues_ReturnsFalse", ListsEqual_DifferentValues_ReturnsFalse),
                TestRunner.RunTest("ListsEqual_BothNull_ReturnsTrue", ListsEqual_BothNull_ReturnsTrue),
                TestRunner.RunTest("ListsEqual_OneNull_ReturnsFalse", ListsEqual_OneNull_ReturnsFalse),

                // HasChanges tests
                TestRunner.RunTest("HasChanges_SameConfig_ReturnsFalse", HasChanges_SameConfig_ReturnsFalse),
                TestRunner.RunTest("HasChanges_DifferentConfig_ReturnsTrue", HasChanges_DifferentConfig_ReturnsTrue),
            };
        }

        // === AxisConfig Equality Tests ===

        private static void AreEqual_IdenticalAxisConfigs_ReturnsTrue()
        {
            var config1 = CreateAxisConfig(minPos: -50, maxPos: 50);
            var config2 = CreateAxisConfig(minPos: -50, maxPos: 50);

            AssertTrue(ConfigComparer.AreEqual(config1, config2), "Identical configs should be equal");
        }

        private static void AreEqual_DifferentKinematics_ReturnsFalse()
        {
            var config1 = CreateAxisConfig(minPos: -50, maxPos: 50);
            var config2 = CreateAxisConfig(minPos: -60, maxPos: 60);

            AssertFalse(ConfigComparer.AreEqual(config1, config2), "Different kinematics should not be equal");
        }

        private static void AreEqual_DifferentStaticBalance_ReturnsFalse()
        {
            var config1 = CreateAxisConfig(minPos: -50, maxPos: 50);
            config1.StaticBalanceConfig = new AxisConfig.Types.StaticBalanceConfig { XCenter = 10.0f, XHalfRange = 50.0f };

            var config2 = CreateAxisConfig(minPos: -50, maxPos: 50);
            config2.StaticBalanceConfig = new AxisConfig.Types.StaticBalanceConfig { XCenter = 20.0f, XHalfRange = 70.0f };

            AssertFalse(ConfigComparer.AreEqual(config1, config2), "Different static balance should not be equal");
        }

        private static void AreEqual_BothNullAxisConfigs_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.AreEqual((AxisConfig)null, (AxisConfig)null), "Both null should be equal");
        }

        private static void AreEqual_OneNullAxisConfig_ReturnsFalse()
        {
            var config = CreateAxisConfig();
            AssertFalse(ConfigComparer.AreEqual(config, null), "Config vs null should not be equal");
            AssertFalse(ConfigComparer.AreEqual(null, config), "Null vs config should not be equal");
        }

        // === FunctionConfig Equality Tests ===

        private static void AreEqual_IdenticalFunctionConfigs_ReturnsTrue()
        {
            var config1 = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var config2 = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);

            AssertTrue(ConfigComparer.AreEqual(config1, config2), "Identical function configs should be equal");
        }

        private static void AreEqual_DifferentOutputMin_ReturnsFalse()
        {
            var config1 = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var config2 = CreateFunctionConfig(outputMin: 0.1f, outputMax: 1.0f);

            AssertFalse(ConfigComparer.AreEqual(config1, config2), "Different OutputMin should not be equal");
        }

        // === FunctionConfigOverrides Equality Tests ===

        private static void AreEqual_IdenticalOverrides_ReturnsTrue()
        {
            var o1 = new FunctionConfigOverrides { OutputMin = 0.1f, OutputMax = 0.9f };
            var o2 = new FunctionConfigOverrides { OutputMin = 0.1f, OutputMax = 0.9f };

            AssertTrue(ConfigComparer.AreEqual(o1, o2), "Identical overrides should be equal");
        }

        private static void AreEqual_DifferentOverrides_ReturnsFalse()
        {
            var o1 = new FunctionConfigOverrides { OutputMin = 0.1f };
            var o2 = new FunctionConfigOverrides { OutputMin = 0.2f };

            AssertFalse(ConfigComparer.AreEqual(o1, o2), "Different overrides should not be equal");
        }

        private static void AreEqual_BothNullOverrides_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.AreEqual((FunctionConfigOverrides)null, (FunctionConfigOverrides)null),
                "Both null overrides should be equal");
        }

        private static void AreEqual_OneNullOverrides_ReturnsFalse()
        {
            var o1 = new FunctionConfigOverrides { OutputMin = 0.1f };

            AssertFalse(ConfigComparer.AreEqual(o1, null), "Override vs null should not be equal");
            AssertFalse(ConfigComparer.AreEqual(null, o1), "Null vs override should not be equal");
        }

        private static void AreEqual_EmptyOverrides_AreEqual()
        {
            var o1 = new FunctionConfigOverrides();
            var o2 = new FunctionConfigOverrides();

            AssertTrue(ConfigComparer.AreEqual(o1, o2), "Empty overrides should be equal");
        }

        // === AxisParameterOverrides Equality Tests ===

        private static void AreEqual_IdenticalAxisOverrides_ReturnsTrue()
        {
            var o1 = new AxisParameterOverrides { Kinematics = CreateKinematics(-50, 50) };
            var o2 = new AxisParameterOverrides { Kinematics = CreateKinematics(-50, 50) };

            AssertTrue(ConfigComparer.AreEqual(o1, o2), "Identical axis overrides should be equal");
        }

        private static void AreEqual_DifferentAxisOverrides_ReturnsFalse()
        {
            var o1 = new AxisParameterOverrides { Kinematics = CreateKinematics(-50, 50) };
            var o2 = new AxisParameterOverrides { Kinematics = CreateKinematics(-60, 60) };

            AssertFalse(ConfigComparer.AreEqual(o1, o2), "Different axis overrides should not be equal");
        }

        private static void AreEqual_BothNullAxisOverrides_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.AreEqual((AxisParameterOverrides)null, (AxisParameterOverrides)null),
                "Both null axis overrides should be equal");
        }

        private static void AreEqual_EmptyAxisOverrides_AreEqual()
        {
            var o1 = new AxisParameterOverrides();
            var o2 = new AxisParameterOverrides();

            AssertTrue(ConfigComparer.AreEqual(o1, o2), "Empty axis overrides should be equal");
        }

        // === Float Comparison Tests ===

        private static void FloatEqual_IdenticalValues_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.FloatEqual(0.5f, 0.5f), "Identical floats should be equal");
        }

        private static void FloatEqual_WithinTolerance_ReturnsTrue()
        {
            // 0.1 + 0.2 can have floating point error
            float a = 0.1f + 0.2f;
            float b = 0.3f;

            AssertTrue(ConfigComparer.FloatEqual(a, b), "Floats within tolerance should be equal");
        }

        private static void FloatEqual_OutsideTolerance_ReturnsFalse()
        {
            AssertFalse(ConfigComparer.FloatEqual(0.5f, 0.6f), "Floats outside tolerance should not be equal");
        }

        private static void NullableFloatEqual_BothNull_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.NullableFloatEqual(null, null), "Both null should be equal");
        }

        private static void NullableFloatEqual_OneNull_ReturnsFalse()
        {
            AssertFalse(ConfigComparer.NullableFloatEqual(0.5f, null), "Value vs null should not be equal");
            AssertFalse(ConfigComparer.NullableFloatEqual(null, 0.5f), "Null vs value should not be equal");
        }

        // === List Comparison Tests ===

        private static void ListsEqual_IdenticalLists_ReturnsTrue()
        {
            var a = new List<float> { 1.0f, 2.0f, 3.0f };
            var b = new List<float> { 1.0f, 2.0f, 3.0f };

            AssertTrue(ConfigComparer.ListsEqual(a, b), "Identical lists should be equal");
        }

        private static void ListsEqual_DifferentLengths_ReturnsFalse()
        {
            var a = new List<float> { 1.0f, 2.0f, 3.0f };
            var b = new List<float> { 1.0f, 2.0f };

            AssertFalse(ConfigComparer.ListsEqual(a, b), "Different length lists should not be equal");
        }

        private static void ListsEqual_DifferentValues_ReturnsFalse()
        {
            var a = new List<float> { 1.0f, 2.0f, 3.0f };
            var b = new List<float> { 1.0f, 2.0f, 4.0f };

            AssertFalse(ConfigComparer.ListsEqual(a, b), "Lists with different values should not be equal");
        }

        private static void ListsEqual_BothNull_ReturnsTrue()
        {
            AssertTrue(ConfigComparer.ListsEqual(null, null), "Both null lists should be equal");
        }

        private static void ListsEqual_OneNull_ReturnsFalse()
        {
            var a = new List<float> { 1.0f };

            AssertFalse(ConfigComparer.ListsEqual(a, null), "List vs null should not be equal");
            AssertFalse(ConfigComparer.ListsEqual(null, a), "Null vs list should not be equal");
        }

        // === HasChanges Tests ===

        private static void HasChanges_SameConfig_ReturnsFalse()
        {
            var config1 = CreateAxisConfig(minPos: -50, maxPos: 50);
            var config2 = CreateAxisConfig(minPos: -50, maxPos: 50);

            AssertFalse(ConfigComparer.HasChanges(config1, config2), "Same configs should have no changes");
        }

        private static void HasChanges_DifferentConfig_ReturnsTrue()
        {
            var config1 = CreateAxisConfig(minPos: -50, maxPos: 50);
            var config2 = CreateAxisConfig(minPos: -60, maxPos: 60);

            AssertTrue(ConfigComparer.HasChanges(config1, config2), "Different configs should have changes");
        }

        // === Helper Methods ===

        private static AxisConfig CreateAxisConfig(int minPos = -100, int maxPos = 100)
        {
            return new AxisConfig
            {
                AxisId = AxisID._1,
                KinematicParameters = CreateKinematics(minPos, maxPos)
            };
        }

        private static KinematicParameters CreateKinematics(int minPos = -100, int maxPos = 100)
        {
            return new KinematicParameters
            {
                ContactPointPosMinAbs = minPos,
                ContactPointPosMaxAbs = maxPos
            };
        }

        private static FunctionConfig CreateFunctionConfig(float outputMin = 0.0f, float outputMax = 1.0f)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    OutputMin = outputMin,
                    OutputMax = outputMax
                }
            };
        }

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition)
                throw new InvalidOperationException(message);
        }

        private static void AssertFalse(bool condition, string message)
        {
            if (condition)
                throw new InvalidOperationException(message);
        }
    }
}
