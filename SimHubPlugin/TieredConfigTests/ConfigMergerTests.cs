using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for ConfigMerger pure functions.
    /// </summary>
    public static class ConfigMergerTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Axis override merge tests
                TestRunner.RunTest("Merge_OnlyKinematics_PreservesStaticBalance", Merge_OnlyKinematics_PreservesStaticBalance),
                TestRunner.RunTest("Merge_OnlyStaticBalance_PreservesKinematics", Merge_OnlyStaticBalance_PreservesKinematics),
                TestRunner.RunTest("Merge_BothOverrides_ReplacesAll", Merge_BothOverrides_ReplacesAll),
                TestRunner.RunTest("Merge_EmptyOverrides_ReturnsBaseUnchanged", Merge_EmptyOverrides_ReturnsBaseUnchanged),
                TestRunner.RunTest("Merge_NullOverrides_ReturnsBaseUnchanged", Merge_NullOverrides_ReturnsBaseUnchanged),
                TestRunner.RunTest("Merge_DoesNotMutateOriginal", Merge_DoesNotMutateOriginal),
                TestRunner.RunTest("Merge_NullBaseConfig_Throws", Merge_NullBaseConfig_Throws),

                // Function config delta merge tests
                TestRunner.RunTest("MergeFunctionConfig_OutputMinOverride", MergeFunctionConfig_OutputMinOverride),
                TestRunner.RunTest("MergeFunctionConfig_OutputMaxOverride", MergeFunctionConfig_OutputMaxOverride),
                TestRunner.RunTest("MergeFunctionConfig_MultipleOverrides", MergeFunctionConfig_MultipleOverrides),
                TestRunner.RunTest("MergeFunctionConfig_NullDelta_ReturnsBase", MergeFunctionConfig_NullDelta_ReturnsBase),
                TestRunner.RunTest("MergeFunctionConfig_EmptyDelta_ReturnsBase", MergeFunctionConfig_EmptyDelta_ReturnsBase),
                TestRunner.RunTest("MergeFunctionConfig_DoesNotMutateOriginal", MergeFunctionConfig_DoesNotMutateOriginal),

                // Three-layer merge tests
                TestRunner.RunTest("MergeAllLayers_UserWins", MergeAllLayers_UserWins),
                TestRunner.RunTest("MergeAllLayers_ProfileWins_WhenNoUser", MergeAllLayers_ProfileWins_WhenNoUser),
                TestRunner.RunTest("MergeAllLayers_HardwareWins_WhenNoOverrides", MergeAllLayers_HardwareWins_WhenNoOverrides),
                TestRunner.RunTest("MergeAllLayers_MixedLayers", MergeAllLayers_MixedLayers),

                // Static balance tuning tests
                TestRunner.RunTest("MergeFunctionConfig_StaticBalanceTuning", MergeFunctionConfig_StaticBalanceTuning),
            };
        }

        // === Axis Override Merge Tests ===

        private static void Merge_OnlyKinematics_PreservesStaticBalance()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.StaticBalanceConfig = CreateStaticBalance(xCenter: 10.0f, xHalfRange: 50.0f);
            var originalBalance = baseConfig.StaticBalanceConfig.Clone();

            var overrides = new AxisParameterOverrides
            {
                Kinematics = CreateKinematics(minPos: -50, maxPos: 50),
                StaticBalance = null
            };

            var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

            AssertEqual(-50, merged.KinematicParameters.ContactPointPosMinAbs, "Kinematics should be overridden");
            AssertTrue(ConfigComparer.AreEqual(originalBalance, merged.StaticBalanceConfig),
                "Static balance should be preserved when not overridden");
        }

        private static void Merge_OnlyStaticBalance_PreservesKinematics()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.KinematicParameters = CreateKinematics(minPos: -100, maxPos: 100);
            var originalKinematics = baseConfig.KinematicParameters.Clone();

            var overrides = new AxisParameterOverrides
            {
                Kinematics = null,
                StaticBalance = CreateStaticBalance(xCenter: 20.0f, xHalfRange: 80.0f)
            };

            var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

            AssertTrue(ConfigComparer.AreEqual(originalKinematics, merged.KinematicParameters),
                "Kinematics should be preserved when not overridden");
            AssertNear(20.0f, merged.StaticBalanceConfig.XCenter, 1e-6f, "Static balance should be overridden");
        }

        private static void Merge_BothOverrides_ReplacesAll()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.KinematicParameters = CreateKinematics(minPos: -10, maxPos: 10);
            baseConfig.StaticBalanceConfig = CreateStaticBalance(xCenter: 5.0f, xHalfRange: 10.0f);

            var overrides = new AxisParameterOverrides
            {
                Kinematics = CreateKinematics(minPos: -99, maxPos: 99),
                StaticBalance = CreateStaticBalance(xCenter: 50.0f, xHalfRange: 99.0f)
            };

            var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

            AssertEqual(-99, merged.KinematicParameters.ContactPointPosMinAbs, "Kinematics min should be overridden");
            AssertEqual(99, merged.KinematicParameters.ContactPointPosMaxAbs, "Kinematics max should be overridden");
            AssertNear(50.0f, merged.StaticBalanceConfig.XCenter, 1e-6f, "Static balance XCenter should be overridden");
            AssertNear(99.0f, merged.StaticBalanceConfig.XHalfRange, 1e-6f, "Static balance XHalfRange should be overridden");
        }

        private static void Merge_EmptyOverrides_ReturnsBaseUnchanged()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.KinematicParameters = CreateKinematics(minPos: -25, maxPos: 25);
            baseConfig.StaticBalanceConfig = CreateStaticBalance(xCenter: 15.0f, xHalfRange: 70.0f);

            var overrides = new AxisParameterOverrides
            {
                Kinematics = null,
                StaticBalance = null
            };

            var merged = ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

            AssertTrue(ConfigComparer.AreEqual(baseConfig, merged), "Empty overrides should return base unchanged");
        }

        private static void Merge_NullOverrides_ReturnsBaseUnchanged()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.KinematicParameters = CreateKinematics(minPos: -25, maxPos: 25);

            var merged = ConfigMerger.MergeAxisOverrides(baseConfig, null);

            AssertTrue(ConfigComparer.AreEqual(baseConfig, merged), "Null overrides should return base unchanged");
        }

        private static void Merge_DoesNotMutateOriginal()
        {
            var baseConfig = CreateAxisConfig();
            baseConfig.KinematicParameters = CreateKinematics(minPos: -30, maxPos: 30);
            var originalMin = baseConfig.KinematicParameters.ContactPointPosMinAbs;

            var overrides = new AxisParameterOverrides
            {
                Kinematics = CreateKinematics(minPos: -999, maxPos: 999)
            };

            ConfigMerger.MergeAxisOverrides(baseConfig, overrides);

            AssertEqual(originalMin, baseConfig.KinematicParameters.ContactPointPosMinAbs,
                "Original base config should not be mutated");
        }

        private static void Merge_NullBaseConfig_Throws()
        {
            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };

            AssertThrows<ArgumentNullException>(
                () => ConfigMerger.MergeAxisOverrides(null, overrides),
                "Null base config should throw");
        }

        // === Function Config Delta Merge Tests ===

        private static void MergeFunctionConfig_OutputMinOverride()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var delta = new FunctionConfigOverrides { OutputMin = 0.2f };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertNear(0.2f, merged.Base.OutputMin, 1e-6f, "OutputMin should be overridden");
            AssertNear(1.0f, merged.Base.OutputMax, 1e-6f, "OutputMax should be preserved");
        }

        private static void MergeFunctionConfig_OutputMaxOverride()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var delta = new FunctionConfigOverrides { OutputMax = 0.8f };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertNear(0.0f, merged.Base.OutputMin, 1e-6f, "OutputMin should be preserved");
            AssertNear(0.8f, merged.Base.OutputMax, 1e-6f, "OutputMax should be overridden");
        }

        private static void MergeFunctionConfig_MultipleOverrides()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            baseConfig.SimulatedMass = 1.0f;
            baseConfig.Friction = 0.5f;

            var delta = new FunctionConfigOverrides
            {
                OutputMin = 0.1f,
                OutputMax = 0.9f,
                SimulatedMass = 2.0f,
                Friction = 0.3f
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertNear(0.1f, merged.Base.OutputMin, 1e-6f, "OutputMin should be overridden");
            AssertNear(0.9f, merged.Base.OutputMax, 1e-6f, "OutputMax should be overridden");
            AssertNear(2.0f, merged.SimulatedMass, 1e-6f, "SimulatedMass should be overridden");
            AssertNear(0.3f, merged.Friction, 1e-6f, "Friction should be overridden");
        }

        private static void MergeFunctionConfig_NullDelta_ReturnsBase()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, null);

            AssertTrue(ConfigComparer.AreEqual(baseConfig, merged), "Null delta should return base unchanged");
        }

        private static void MergeFunctionConfig_EmptyDelta_ReturnsBase()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var delta = new FunctionConfigOverrides();

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(ConfigComparer.AreEqual(baseConfig, merged), "Empty delta should return base unchanged");
        }

        private static void MergeFunctionConfig_DoesNotMutateOriginal()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var originalMin = baseConfig.Base.OutputMin;

            var delta = new FunctionConfigOverrides { OutputMin = 0.5f };
            ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertNear(originalMin, baseConfig.Base.OutputMin, 1e-6f, "Original base should not be mutated");
        }

        // === Three-Layer Merge Tests ===

        private static void MergeAllLayers_UserWins()
        {
            var hardware = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var profile = new FunctionConfigOverrides { OutputMax = 0.9f };
            var user = new FunctionConfigOverrides { OutputMax = 0.8f };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

            AssertNear(0.8f, merged.Base.OutputMax, 1e-6f, "User should win over profile");
        }

        private static void MergeAllLayers_ProfileWins_WhenNoUser()
        {
            var hardware = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var profile = new FunctionConfigOverrides { OutputMax = 0.9f };
            var user = new FunctionConfigOverrides { OutputMax = null };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

            AssertNear(0.9f, merged.Base.OutputMax, 1e-6f, "Profile should win when no user override");
        }

        private static void MergeAllLayers_HardwareWins_WhenNoOverrides()
        {
            var hardware = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            var profile = new FunctionConfigOverrides();
            var user = new FunctionConfigOverrides();

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

            AssertNear(1.0f, merged.Base.OutputMax, 1e-6f, "Baseline should win when no overrides");
        }

        private static void MergeAllLayers_MixedLayers()
        {
            var hardware = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            hardware.SimulatedMass = 1.0f;
            hardware.Friction = 0.5f;

            var profile = new FunctionConfigOverrides
            {
                OutputMax = 0.9f,       // Will be overridden by user
                SimulatedMass = 2.0f    // Will stay (no user override)
            };

            var user = new FunctionConfigOverrides
            {
                OutputMax = 0.8f,       // User wins
                Friction = 0.3f         // User wins (no profile override)
            };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user);

            AssertNear(0.0f, merged.Base.OutputMin, 1e-6f, "OutputMin unchanged from hardware");
            AssertNear(0.8f, merged.Base.OutputMax, 1e-6f, "User wins for OutputMax");
            AssertNear(2.0f, merged.SimulatedMass, 1e-6f, "Profile wins for SimulatedMass (no user)");
            AssertNear(0.3f, merged.Friction, 1e-6f, "User wins for Friction");
        }

        // === Static Balance Tuning Tests ===

        private static void MergeFunctionConfig_StaticBalanceTuning()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            baseConfig.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning
            {
                Enabled = false,
                Gain = 0.5f
            };

            var delta = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides
                {
                    Enabled = true,
                    Gain = 0.8f
                }
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.StaticBalanceTuning.Enabled, "StaticBalanceTuning.Enabled should be overridden");
            AssertNear(0.8f, merged.StaticBalanceTuning.Gain, 1e-6f, "StaticBalanceTuning.Gain should be overridden");
        }

        // === Helper Methods ===

        private static AxisConfig CreateAxisConfig(int minPos = -100, int maxPos = 100)
        {
            return new AxisConfig
            {
                AxisId = AxisID._1,
                KinematicParameters = CreateKinematics(minPos, maxPos),
                StaticBalanceConfig = CreateStaticBalance()
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

        private static AxisConfig.Types.StaticBalanceConfig CreateStaticBalance(float xCenter = 0.0f, float xHalfRange = 50.0f)
        {
            return new AxisConfig.Types.StaticBalanceConfig
            {
                XCenter = xCenter,
                XHalfRange = xHalfRange
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

        private static void AssertEqual<T>(T expected, T actual, string message) where T : IEquatable<T>
        {
            if (!expected.Equals(actual))
                throw new InvalidOperationException($"{message}: expected {expected}, got {actual}");
        }

        private static void AssertNear(float expected, float actual, float tolerance, string message)
        {
            if (Math.Abs(expected - actual) > tolerance)
                throw new InvalidOperationException($"{message}: expected {expected:F6}, got {actual:F6}");
        }

        private static void AssertThrows<TException>(Action action, string message) where TException : Exception
        {
            try
            {
                action();
            }
            catch (TException)
            {
                return;
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"{message}: Threw {ex.GetType().Name} instead of {typeof(TException).Name}");
            }
            throw new InvalidOperationException($"{message}: No exception was thrown");
        }
    }
}
