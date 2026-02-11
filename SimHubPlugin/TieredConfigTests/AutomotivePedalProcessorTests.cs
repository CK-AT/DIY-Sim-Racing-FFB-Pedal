using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    public static class AutomotivePedalProcessorTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Reconciliation tests
                TestRunner.RunTest("Reconcile_ForceMode_SetsOutputFromForce", Reconcile_ForceMode_SetsOutputFromForce),
                TestRunner.RunTest("Reconcile_TravelMode_SetsOutputFromPos", Reconcile_TravelMode_SetsOutputFromPos),
                TestRunner.RunTest("Reconcile_SetsPosIdleAndPosEnd", Reconcile_SetsPosIdleAndPosEnd),
                TestRunner.RunTest("Reconcile_NullForceCurve_NoOp", Reconcile_NullForceCurve_NoOp),
                TestRunner.RunTest("Reconcile_NullConfig_NoOp", Reconcile_NullConfig_NoOp),
                TestRunner.RunTest("Reconcile_NullBase_StillSetsPosFields", Reconcile_NullBase_StillSetsPosFields),

                // Override application tests
                TestRunner.RunTest("Apply_DamperPositiveFactor", Apply_DamperPositiveFactor),
                TestRunner.RunTest("Apply_DamperNegativeFactor", Apply_DamperNegativeFactor),
                TestRunner.RunTest("Apply_ForceCurveReplacement", Apply_ForceCurveReplacement),
                TestRunner.RunTest("Apply_NullDelta_NoOp", Apply_NullDelta_NoOp),
                TestRunner.RunTest("Apply_EmptyDelta_NoOp", Apply_EmptyDelta_NoOp),
                TestRunner.RunTest("Apply_DamperAutoCreated", Apply_DamperAutoCreated),

                // ABS effect override tests
                TestRunner.RunTest("Apply_AbsEffectReplacement", Apply_AbsEffectReplacement),
                TestRunner.RunTest("Apply_AbsEffectIsClone", Apply_AbsEffectIsClone),
                TestRunner.RunTest("Apply_AbsEffectPreservesOtherEffects", Apply_AbsEffectPreservesOtherEffects),
            };
        }

        // === Reconciliation Tests ===

        private static void Reconcile_ForceMode_SetsOutputFromForce()
        {
            var config = CreatePedalConfig(OutputMode.Force, posMin: 0, posMax: 100, fMin: 5.0f, fMax: 50.0f);

            AutomotivePedalProcessor.ReconcileDerivedFields(config);

            AssertNear(5.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be FMin in Force mode");
            AssertNear(50.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be FMax in Force mode");
        }

        private static void Reconcile_TravelMode_SetsOutputFromPos()
        {
            var config = CreatePedalConfig(OutputMode.Travel, posMin: 10, posMax: 90, fMin: 5.0f, fMax: 50.0f);

            AutomotivePedalProcessor.ReconcileDerivedFields(config);

            AssertNear(10.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be PosMin in Travel mode");
            AssertNear(90.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be PosMax in Travel mode");
        }

        private static void Reconcile_SetsPosIdleAndPosEnd()
        {
            var config = CreatePedalConfig(OutputMode.Force, posMin: 5, posMax: 95, fMin: 1.0f, fMax: 10.0f);

            AutomotivePedalProcessor.ReconcileDerivedFields(config);

            AssertEqual(5, config.AutomotivePedal.PosIdle, "PosIdle should equal ForceCurveConfig.PosMin");
            AssertEqual(95, config.AutomotivePedal.PosEnd, "PosEnd should equal ForceCurveConfig.PosMax");
        }

        private static void Reconcile_NullForceCurve_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = new FunctionBase { OutputMode = OutputMode.Force, OutputMin = 1.0f, OutputMax = 2.0f },
                AutomotivePedal = new AutomotivePedalConfig { ForceCurveConfig = null }
            };

            AutomotivePedalProcessor.ReconcileDerivedFields(config);

            AssertNear(1.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be unchanged");
            AssertNear(2.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be unchanged");
        }

        private static void Reconcile_NullConfig_NoOp()
        {
            // Should not throw
            AutomotivePedalProcessor.ReconcileDerivedFields(null);
        }

        private static void Reconcile_NullBase_StillSetsPosFields()
        {
            var config = new FunctionConfig
            {
                Base = null,
                AutomotivePedal = new AutomotivePedalConfig
                {
                    ForceCurveConfig = CreateForceCurve(posMin: 10, posMax: 80, fMin: 1.0f, fMax: 10.0f)
                }
            };

            AutomotivePedalProcessor.ReconcileDerivedFields(config);

            AssertEqual(10, config.AutomotivePedal.PosIdle, "PosIdle should be set even without Base");
            AssertEqual(80, config.AutomotivePedal.PosEnd, "PosEnd should be set even without Base");
        }

        // === Override Application Tests ===

        private static void Apply_DamperPositiveFactor()
        {
            var apConfig = new AutomotivePedalConfig
            {
                DamperConfig = new DamperConfig { PositiveFactor = 0.1f, NegativeFactor = 0.2f }
            };
            var delta = new FunctionConfigOverrides
            {
                DamperConfig = new DamperConfigOverrides { PositiveFactor = 0.5f }
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertNear(0.5f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "PositiveFactor should be overridden");
            AssertNear(0.2f, apConfig.DamperConfig.NegativeFactor, 1e-6f, "NegativeFactor should be preserved");
        }

        private static void Apply_DamperNegativeFactor()
        {
            var apConfig = new AutomotivePedalConfig
            {
                DamperConfig = new DamperConfig { PositiveFactor = 0.1f, NegativeFactor = 0.2f }
            };
            var delta = new FunctionConfigOverrides
            {
                DamperConfig = new DamperConfigOverrides { NegativeFactor = 0.7f }
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertNear(0.1f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "PositiveFactor should be preserved");
            AssertNear(0.7f, apConfig.DamperConfig.NegativeFactor, 1e-6f, "NegativeFactor should be overridden");
        }

        private static void Apply_ForceCurveReplacement()
        {
            var original = CreateForceCurve(posMin: 0, posMax: 100, fMin: 1.0f, fMax: 10.0f);
            var apConfig = new AutomotivePedalConfig { ForceCurveConfig = original };
            var replacement = CreateForceCurve(posMin: 5, posMax: 95, fMin: 2.0f, fMax: 20.0f);
            var delta = new FunctionConfigOverrides { ForceCurve = replacement };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertEqual(5, apConfig.ForceCurveConfig.PosMin, "ForceCurve PosMin should be replaced");
            AssertEqual(95, apConfig.ForceCurveConfig.PosMax, "ForceCurve PosMax should be replaced");
            AssertNear(2.0f, apConfig.ForceCurveConfig.FMin, 1e-6f, "ForceCurve FMin should be replaced");
            AssertNear(20.0f, apConfig.ForceCurveConfig.FMax, 1e-6f, "ForceCurve FMax should be replaced");
            // Verify it's a clone, not the same reference
            AssertTrue(!ReferenceEquals(replacement, apConfig.ForceCurveConfig),
                "ForceCurve should be a clone, not the same reference");
        }

        private static void Apply_NullDelta_NoOp()
        {
            var apConfig = new AutomotivePedalConfig
            {
                DamperConfig = new DamperConfig { PositiveFactor = 0.1f, NegativeFactor = 0.2f },
                ForceCurveConfig = CreateForceCurve(posMin: 0, posMax: 100, fMin: 1.0f, fMax: 10.0f)
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, null);

            AssertNear(0.1f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "PositiveFactor should be unchanged");
            AssertEqual(0, apConfig.ForceCurveConfig.PosMin, "ForceCurve PosMin should be unchanged");
        }

        private static void Apply_EmptyDelta_NoOp()
        {
            var apConfig = new AutomotivePedalConfig
            {
                DamperConfig = new DamperConfig { PositiveFactor = 0.1f, NegativeFactor = 0.2f },
                ForceCurveConfig = CreateForceCurve(posMin: 0, posMax: 100, fMin: 1.0f, fMax: 10.0f)
            };
            var delta = new FunctionConfigOverrides();

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertNear(0.1f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "PositiveFactor should be unchanged");
            AssertEqual(0, apConfig.ForceCurveConfig.PosMin, "ForceCurve PosMin should be unchanged");
        }

        private static void Apply_DamperAutoCreated()
        {
            var apConfig = new AutomotivePedalConfig { DamperConfig = null };
            var delta = new FunctionConfigOverrides
            {
                DamperConfig = new DamperConfigOverrides { PositiveFactor = 0.3f }
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertTrue(apConfig.DamperConfig != null, "DamperConfig should be auto-created");
            AssertNear(0.3f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "PositiveFactor should be set");
        }

        // === ABS Effect Override Tests ===

        private static void Apply_AbsEffectReplacement()
        {
            var apConfig = new AutomotivePedalConfig
            {
                AbsEffectConfig = new ABSEffectConfig { Enabled = false, Freq = 10, Ampl = 20 }
            };
            var delta = new FunctionConfigOverrides
            {
                AbsEffect = new ABSEffectConfig { Enabled = true, Freq = 30, Ampl = 50, Mode = ABSMode.Force }
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertTrue(apConfig.AbsEffectConfig.Enabled, "AbsEffect.Enabled should be overridden to true");
            AssertEqual((int)apConfig.AbsEffectConfig.Freq, 30, "AbsEffect.Freq should be overridden");
            AssertEqual((int)apConfig.AbsEffectConfig.Ampl, 50, "AbsEffect.Ampl should be overridden");
            AssertTrue(apConfig.AbsEffectConfig.Mode == ABSMode.Force, "AbsEffect.Mode should be overridden");
        }

        private static void Apply_AbsEffectIsClone()
        {
            var original = new ABSEffectConfig { Freq = 25 };
            var apConfig = new AutomotivePedalConfig
            {
                AbsEffectConfig = new ABSEffectConfig { Freq = 10 }
            };
            var delta = new FunctionConfigOverrides { AbsEffect = original };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertTrue(!ReferenceEquals(original, apConfig.AbsEffectConfig),
                "AbsEffectConfig should be a clone, not the same reference");
            AssertEqual((int)apConfig.AbsEffectConfig.Freq, 25, "Freq should match override value");
        }

        private static void Apply_AbsEffectPreservesOtherEffects()
        {
            var apConfig = new AutomotivePedalConfig
            {
                AbsEffectConfig = new ABSEffectConfig { Freq = 10 },
                RpmEffectConfig = new RPMEffectConfig { Amp = 42 },
                DamperConfig = new DamperConfig { PositiveFactor = 0.3f }
            };
            var delta = new FunctionConfigOverrides
            {
                AbsEffect = new ABSEffectConfig { Freq = 99 }
            };

            AutomotivePedalProcessor.ApplyOverrides(apConfig, delta);

            AssertEqual((int)apConfig.AbsEffectConfig.Freq, 99, "AbsEffect.Freq should be overridden");
            AssertEqual((int)apConfig.RpmEffectConfig.Amp, 42, "RpmEffect should be preserved");
            AssertNear(0.3f, apConfig.DamperConfig.PositiveFactor, 1e-6f, "DamperConfig should be preserved");
        }

        // === Helper Methods ===

        private static FunctionConfig CreatePedalConfig(OutputMode mode, int posMin, int posMax, float fMin, float fMax)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.BrakePedal,
                    OutputMode = mode,
                    OutputMin = 0,
                    OutputMax = 0
                },
                AutomotivePedal = new AutomotivePedalConfig
                {
                    ForceCurveConfig = CreateForceCurve(posMin, posMax, fMin, fMax)
                }
            };
        }

        private static SplineForceCurveConfig CreateForceCurve(int posMin, int posMax, float fMin, float fMax)
        {
            return new SplineForceCurveConfig
            {
                PosMin = posMin,
                PosMax = posMax,
                FMin = fMin,
                FMax = fMax
            };
        }

        private static void AssertTrue(bool condition, string message)
        {
            if (!condition)
                throw new Exception($"Assertion failed: {message}");
        }

        private static void AssertEqual(int expected, int actual, string message)
        {
            if (expected != actual)
                throw new Exception($"Assertion failed: {message} (expected {expected}, got {actual})");
        }

        private static void AssertNear(float expected, float actual, float tolerance, string message)
        {
            if (Math.Abs(expected - actual) > tolerance)
                throw new Exception($"Assertion failed: {message} (expected {expected}, got {actual})");
        }
    }
}
