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
                TestRunner.RunTest("MergeFunctionConfig_FlightPedalsOverrides_PreserveBaseline", MergeFunctionConfig_FlightPedalsOverrides_PreserveBaseline),

                // Three-layer merge tests
                TestRunner.RunTest("MergeAllLayers_UserWins", MergeAllLayers_UserWins),
                TestRunner.RunTest("MergeAllLayers_ProfileWins_WhenNoUser", MergeAllLayers_ProfileWins_WhenNoUser),
                TestRunner.RunTest("MergeAllLayers_HardwareWins_WhenNoOverrides", MergeAllLayers_HardwareWins_WhenNoOverrides),
                TestRunner.RunTest("MergeAllLayers_MixedLayers", MergeAllLayers_MixedLayers),

                // ConfigOut tier merge tests
                TestRunner.RunTest("MergeAllLayers_ConfigOutAlone_AppliesValue", MergeAllLayers_ConfigOutAlone_AppliesValue),
                TestRunner.RunTest("MergeAllLayers_ProfileBeatsConfigOut", MergeAllLayers_ProfileBeatsConfigOut),
                TestRunner.RunTest("MergeAllLayers_UserBeatsConfigOut", MergeAllLayers_UserBeatsConfigOut),
                TestRunner.RunTest("MergeAllLayers_FourTierStack", MergeAllLayers_FourTierStack),

                // Static balance tuning tests
                TestRunner.RunTest("MergeFunctionConfig_StaticBalanceTuning", MergeFunctionConfig_StaticBalanceTuning),

                // Oneof preservation tests
                TestRunner.RunTest("MergeFunctionConfig_AutomotivePedal_PreservesOneof", MergeFunctionConfig_AutomotivePedal_PreservesOneof),
                TestRunner.RunTest("MergeFunctionConfig_FlightPedals_PreservesOneof", MergeFunctionConfig_FlightPedals_PreservesOneof),
                TestRunner.RunTest("MergeFunctionConfig_FlightStickArmCreatedWhenOneofNone", MergeFunctionConfig_FlightStickArmCreatedWhenOneofNone),
                TestRunner.RunTest("MergeFunctionConfig_FlightStickOverride_DoesNotClobberOtherArm", MergeFunctionConfig_FlightStickOverride_DoesNotClobberOtherArm),
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

        private static void MergeFunctionConfig_FlightPedalsOverrides_PreserveBaseline()
        {
            var baseConfig = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            baseConfig.Base.FunctionId = FunctionID.FlightPedals;
            baseConfig.FlightPedals = new FlightPedalsConfig
            {
                PosNearLim = 41,
                PosFarLim = 91,
                Damping = 0.7f,
                CenteringSpringConst = 2.2f
            };

            var delta = new FunctionConfigOverrides
            {
                FlightPedalsMotionRange = new MotionRangeOverrides
                {
                    NearLim = 12
                }
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.FlightPedals != null, "FlightPedals config should be preserved");
            AssertEqual(12, merged.FlightPedals.PosNearLim, "PosNearLim should be overridden");
            AssertEqual(91, merged.FlightPedals.PosFarLim, "PosFarLim should remain from baseline");
            AssertNear(0.7f, merged.FlightPedals.Damping, 1e-6f, "Damping should remain from baseline");
            AssertNear(2.2f, merged.FlightPedals.CenteringSpringConst, 1e-6f, "Centering spring should remain from baseline");
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

        // === ConfigOut tier merge tests ===
        // ConfigOut sits below Profile/User: graph-derived values establish a
        // baseline-like overlay that explicit user overrides can still beat.

        private static void MergeAllLayers_ConfigOutAlone_AppliesValue()
        {
            var hardware = CreateFunctionConfig();
            hardware.SimulatedMass = 1.0f;
            var configOut = new FunctionConfigOverrides { SimulatedMass = 7.0f };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile: null, user: null, configOut: configOut);

            AssertNear(7.0f, merged.SimulatedMass, 1e-6f, "ConfigOut should apply when no profile/user override");
        }

        private static void MergeAllLayers_ProfileBeatsConfigOut()
        {
            var hardware = CreateFunctionConfig();
            hardware.SimulatedMass = 1.0f;
            var profile = new FunctionConfigOverrides { SimulatedMass = 5.0f };
            var configOut = new FunctionConfigOverrides { SimulatedMass = 7.0f };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user: null, configOut: configOut);

            AssertNear(5.0f, merged.SimulatedMass, 1e-6f, "Profile must override ConfigOut");
        }

        private static void MergeAllLayers_UserBeatsConfigOut()
        {
            var hardware = CreateFunctionConfig();
            hardware.SimulatedMass = 1.0f;
            var user = new FunctionConfigOverrides { SimulatedMass = 9.0f };
            var configOut = new FunctionConfigOverrides { SimulatedMass = 7.0f };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile: null, user: user, configOut: configOut);

            AssertNear(9.0f, merged.SimulatedMass, 1e-6f, "User must override ConfigOut");
        }

        private static void MergeAllLayers_FourTierStack()
        {
            var hardware = CreateFunctionConfig(outputMin: 0.0f, outputMax: 1.0f);
            hardware.SimulatedMass = 1.0f;
            hardware.Friction = 0.5f;

            // ConfigOut sets fields the user/profile have not touched
            var configOut = new FunctionConfigOverrides
            {
                Friction = 0.7f,        // No higher-tier override → ConfigOut wins
                SimulatedMass = 3.0f    // Profile will override
            };
            var profile = new FunctionConfigOverrides
            {
                SimulatedMass = 2.0f,   // User will override
                OutputMax = 0.9f        // User will override
            };
            var user = new FunctionConfigOverrides
            {
                SimulatedMass = 4.0f,   // Top-priority
                OutputMax = 0.8f
            };

            var merged = ConfigMerger.MergeAllLayers(hardware, profile, user, configOut);

            AssertNear(0.7f, merged.Friction, 1e-6f, "ConfigOut wins when no profile/user override");
            AssertNear(4.0f, merged.SimulatedMass, 1e-6f, "User beats profile beats ConfigOut");
            AssertNear(0.8f, merged.Base.OutputMax, 1e-6f, "User wins for OutputMax (no ConfigOut entry)");
            AssertNear(0.0f, merged.Base.OutputMin, 1e-6f, "OutputMin untouched");
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

        // === Oneof Preservation Tests ===

        /// <summary>
        /// Regression: FlightStickProcessor.ApplyOverrides used to create a new FlightStickConfig
        /// when merged.FlightStick was null, clobbering the AutomotivePedal oneof arm.
        /// </summary>
        private static void MergeFunctionConfig_AutomotivePedal_PreservesOneof()
        {
            var baseConfig = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.BrakePedal, OutputMin = 0, OutputMax = 100 },
                AutomotivePedal = new AutomotivePedalConfig
                {
                    DamperConfig = new DamperConfig { PositiveFactor = 0.05f, NegativeFactor = 0.05f },
                    ForceCurveConfig = new SplineForceCurveConfig { PosMin = 19, PosMax = 70, FMin = 10, FMax = 80 }
                }
            };

            var delta = new FunctionConfigOverrides
            {
                ForceCurve = new SplineForceCurveConfig { PosMin = 10, PosMax = 80, FMin = 5, FMax = 90 }
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.AutomotivePedal != null, "AutomotivePedal oneof should be preserved after merge");
            AssertTrue(merged.SpecificCase == FunctionConfig.SpecificOneofCase.AutomotivePedal,
                "SpecificCase should remain AutomotivePedal");
            AssertTrue(merged.FlightStick == null, "FlightStick should be null for AutomotivePedal config");
            AssertNear(10f, merged.AutomotivePedal.ForceCurveConfig.PosMin, 1e-6f, "Force curve override PosMin");
            AssertNear(0.05f, merged.AutomotivePedal.DamperConfig.PositiveFactor, 1e-6f, "Damper should be preserved from base");
        }

        private static void MergeFunctionConfig_FlightPedals_PreservesOneof()
        {
            var baseConfig = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.FlightPedals, OutputMin = 0, OutputMax = 100 },
                FlightPedals = new FlightPedalsConfig
                {
                    PosNearLim = 10, PosFarLim = 90, Damping = 0.1f
                }
            };

            var delta = new FunctionConfigOverrides { SimulatedMass = 2.0f };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.FlightPedals != null, "FlightPedals oneof should be preserved after merge");
            AssertTrue(merged.SpecificCase == FunctionConfig.SpecificOneofCase.FlightPedals,
                "SpecificCase should remain FlightPedals");
            AssertTrue(merged.FlightStick == null, "FlightStick should be null for FlightPedals config");
            AssertNear(2.0f, merged.SimulatedMass, 1e-6f, "SimulatedMass override should apply");
        }

        // Regression: when ESP32 sends a FlightStickPitch baseline without the FlightStick
        // oneof arm set (SpecificCase = None), graph-derived ConfigOut overrides for
        // vibration ratios were silently dropped because the processor refused to create
        // the arm. The fix creates the arm when the oneof is None and FlightStick-specific
        // overrides are present.
        private static void MergeFunctionConfig_FlightStickArmCreatedWhenOneofNone()
        {
            var baseConfig = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.FlightStickPitch, OutputMin = -100, OutputMax = 100 }
                // Note: no FlightStick arm set — SpecificCase == None
            };
            AssertTrue(baseConfig.SpecificCase == FunctionConfig.SpecificOneofCase.None,
                "Baseline must have no oneof arm for this test");

            var delta = new FunctionConfigOverrides
            {
                FlightStickVibHarmonicRatios = new float?[] { 1.0f, 2.0f, 3.0f, 5.0f, 10.0f },
                FlightStickPhaseOffset = 90.0f
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.FlightStick != null,
                "FlightStick arm should be created when oneof is None and overrides present");
            AssertTrue(merged.SpecificCase == FunctionConfig.SpecificOneofCase.FlightStick,
                "SpecificCase should be FlightStick after arm creation");
            AssertEqual(5, merged.FlightStick.VibHarmonicRatios.Count, "All 5 harm ratios should land");
            AssertNear(1.0f, merged.FlightStick.VibHarmonicRatios[0], 1e-6f, "HarmRatio1");
            AssertNear(2.0f, merged.FlightStick.VibHarmonicRatios[1], 1e-6f, "HarmRatio2");
            AssertNear(3.0f, merged.FlightStick.VibHarmonicRatios[2], 1e-6f, "HarmRatio3");
            AssertNear(5.0f, merged.FlightStick.VibHarmonicRatios[3], 1e-6f, "HarmRatio4");
            AssertNear(10.0f, merged.FlightStick.VibHarmonicRatios[4], 1e-6f, "HarmRatio5");
        }

        // Verifies the safety side: FlightStick overrides must NOT clobber a different
        // active oneof arm (e.g. AutomotivePedal). This case is what motivated the
        // original "don't create FlightStick" guard — the new logic still respects it.
        private static void MergeFunctionConfig_FlightStickOverride_DoesNotClobberOtherArm()
        {
            var baseConfig = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.BrakePedal, OutputMin = 0, OutputMax = 100 },
                AutomotivePedal = new AutomotivePedalConfig
                {
                    DamperConfig = new DamperConfig { PositiveFactor = 0.05f, NegativeFactor = 0.05f }
                }
            };

            // Pathological delta: someone wired a FlightStick override onto a brake-pedal
            // function (e.g. profile cross-contamination). Must not clobber.
            var delta = new FunctionConfigOverrides
            {
                FlightStickVibHarmonicRatios = new float?[] { 1.0f, 2.0f, 3.0f, 5.0f, 10.0f }
            };

            var merged = ConfigMerger.MergeFunctionConfig(baseConfig, delta);

            AssertTrue(merged.AutomotivePedal != null, "AutomotivePedal arm must survive");
            AssertTrue(merged.SpecificCase == FunctionConfig.SpecificOneofCase.AutomotivePedal,
                "SpecificCase must remain AutomotivePedal");
            AssertTrue(merged.FlightStick == null, "FlightStick arm must NOT be created over an existing arm");
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
