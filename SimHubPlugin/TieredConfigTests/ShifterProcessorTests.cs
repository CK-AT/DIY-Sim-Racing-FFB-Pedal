using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    public static class ShifterProcessorTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Reconciliation tests
                TestRunner.RunTest("Reconcile_HPattern_SetsOutputFromPosX", Reconcile_HPattern_SetsOutputFromPosX),
                TestRunner.RunTest("Reconcile_Sequential_SetsOutputFromPosY", Reconcile_Sequential_SetsOutputFromPosY),
                TestRunner.RunTest("Reconcile_NullShifter_NoOp", Reconcile_NullShifter_NoOp),
                TestRunner.RunTest("Reconcile_NullConfig_NoOp", Reconcile_NullConfig_NoOp),
                TestRunner.RunTest("Reconcile_NullBase_NoOp", Reconcile_NullBase_NoOp),

                // Override application tests
                TestRunner.RunTest("Apply_ShifterConfig_FullReplacement", Apply_ShifterConfig_FullReplacement),
                TestRunner.RunTest("Apply_ShifterDetectConfig_FullReplacement", Apply_ShifterDetectConfig_FullReplacement),
                TestRunner.RunTest("Apply_BothConfigs_Replaced", Apply_BothConfigs_Replaced),
                TestRunner.RunTest("Apply_NullDelta_NoOp", Apply_NullDelta_NoOp),
                TestRunner.RunTest("Apply_NullMerged_NoOp", Apply_NullMerged_NoOp),
                TestRunner.RunTest("Apply_NullShifterOnMerged_NoReplace", Apply_NullShifterOnMerged_NoReplace),
                TestRunner.RunTest("Apply_NullShifterDetectOnMerged_NoReplace", Apply_NullShifterDetectOnMerged_NoReplace),
                TestRunner.RunTest("Apply_ShifterConfig_IsCloned", Apply_ShifterConfig_IsCloned),
            };
        }

        // === Reconciliation Tests ===

        private static void Reconcile_HPattern_SetsOutputFromPosX()
        {
            var config = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);

            ShifterProcessor.ReconcileDerivedFields(config);

            AssertEqual(-100, (int)config.Base.OutputMin, "OutputMin should be PosXMin");
            AssertEqual(100, (int)config.Base.OutputMax, "OutputMax should be PosXMax");
        }

        private static void Reconcile_Sequential_SetsOutputFromPosY()
        {
            var config = CreateShifterConfig(sequential: true, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);

            ShifterProcessor.ReconcileDerivedFields(config);

            AssertEqual(-200, (int)config.Base.OutputMin, "OutputMin should be PosYMin");
            AssertEqual(200, (int)config.Base.OutputMax, "OutputMax should be PosYMax");
        }

        private static void Reconcile_NullShifter_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.Shifter,
                    OutputMin = 1.0f,
                    OutputMax = 2.0f
                },
                Shifter = null
            };

            ShifterProcessor.ReconcileDerivedFields(config);

            AssertNear(1.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be unchanged");
            AssertNear(2.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be unchanged");
        }

        private static void Reconcile_NullConfig_NoOp()
        {
            // Should not throw
            ShifterProcessor.ReconcileDerivedFields(null);
        }

        private static void Reconcile_NullBase_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = null,
                Shifter = new ShifterConfig { PosXMin = -50, PosXMax = 50 }
            };

            // Should not throw
            ShifterProcessor.ReconcileDerivedFields(config);
        }

        // === Override Application Tests ===

        private static void Apply_ShifterConfig_FullReplacement()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);
            merged.Shifter.Damping = 0.5f;

            var replacement = new ShifterConfig { PosXMin = -50, PosXMax = 50, PosYMin = -80, PosYMax = 80, Damping = 1.0f };
            var delta = new FunctionConfigOverrides { ShifterConfig = replacement };

            ShifterProcessor.ApplyOverrides(merged, delta);

            AssertEqual(-50, merged.Shifter.PosXMin, "PosXMin should be from override");
            AssertEqual(50, merged.Shifter.PosXMax, "PosXMax should be from override");
            AssertNear(1.0f, merged.Shifter.Damping, 1e-6f, "Damping should be from override");
        }

        private static void Apply_ShifterDetectConfig_FullReplacement()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);
            merged.AuxFunction = new AuxFunctionConfig
            {
                ShifterDetect = new ShifterDetectConfig { Hysteresis = 10 }
            };

            var replacementDetect = new ShifterDetectConfig { Hysteresis = 25 };
            var delta = new FunctionConfigOverrides { ShifterDetectConfig = replacementDetect };

            ShifterProcessor.ApplyOverrides(merged, delta);

            AssertEqual(25, (int)merged.AuxFunction.ShifterDetect.Hysteresis, "Hysteresis should be from override");
        }

        private static void Apply_BothConfigs_Replaced()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);
            merged.AuxFunction = new AuxFunctionConfig
            {
                ShifterDetect = new ShifterDetectConfig { Hysteresis = 10 }
            };

            var delta = new FunctionConfigOverrides
            {
                ShifterConfig = new ShifterConfig { PosXMin = -30, PosXMax = 30, Damping = 2.0f },
                ShifterDetectConfig = new ShifterDetectConfig { Hysteresis = 50 }
            };

            ShifterProcessor.ApplyOverrides(merged, delta);

            AssertEqual(-30, merged.Shifter.PosXMin, "PosXMin should be from override");
            AssertNear(2.0f, merged.Shifter.Damping, 1e-6f, "Damping should be from override");
            AssertEqual(50, (int)merged.AuxFunction.ShifterDetect.Hysteresis, "Hysteresis should be from override");
        }

        private static void Apply_NullDelta_NoOp()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);

            ShifterProcessor.ApplyOverrides(merged, null);

            AssertEqual(-100, merged.Shifter.PosXMin, "PosXMin should be unchanged");
            AssertEqual(100, merged.Shifter.PosXMax, "PosXMax should be unchanged");
        }

        private static void Apply_NullMerged_NoOp()
        {
            var delta = new FunctionConfigOverrides
            {
                ShifterConfig = new ShifterConfig { PosXMin = -50, PosXMax = 50 }
            };

            // Should not throw
            ShifterProcessor.ApplyOverrides(null, delta);
        }

        private static void Apply_NullShifterOnMerged_NoReplace()
        {
            var merged = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.Shifter },
                Shifter = null
            };
            var delta = new FunctionConfigOverrides
            {
                ShifterConfig = new ShifterConfig { PosXMin = -50, PosXMax = 50 }
            };

            ShifterProcessor.ApplyOverrides(merged, delta);

            // Shifter stays null because the guard checks merged.Shifter != null
            AssertTrue(merged.Shifter == null, "Shifter should remain null when not present on merged");
        }

        private static void Apply_NullShifterDetectOnMerged_NoReplace()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);
            // No AuxFunction set
            merged.AuxFunction = null;

            var delta = new FunctionConfigOverrides
            {
                ShifterDetectConfig = new ShifterDetectConfig { Hysteresis = 25 }
            };

            ShifterProcessor.ApplyOverrides(merged, delta);

            // AuxFunction stays null because guard checks merged.AuxFunction?.ShifterDetect
            AssertTrue(merged.AuxFunction == null, "AuxFunction should remain null");
        }

        private static void Apply_ShifterConfig_IsCloned()
        {
            var merged = CreateShifterConfig(sequential: false, posXMin: -100, posXMax: 100, posYMin: -200, posYMax: 200);
            var original = new ShifterConfig { PosXMin = -50, PosXMax = 50, Damping = 1.0f };
            var delta = new FunctionConfigOverrides { ShifterConfig = original };

            ShifterProcessor.ApplyOverrides(merged, delta);

            // Mutating the original should not affect merged
            original.PosXMin = -999;
            AssertEqual(-50, merged.Shifter.PosXMin, "Merged should be a clone, not the same reference");
        }

        // === Helper Methods ===

        private static FunctionConfig CreateShifterConfig(bool sequential, int posXMin, int posXMax, int posYMin, int posYMax)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.Shifter,
                    OutputMin = 0,
                    OutputMax = 0
                },
                Shifter = new ShifterConfig
                {
                    Sequential = sequential,
                    PosXMin = posXMin,
                    PosXMax = posXMax,
                    PosYMin = posYMin,
                    PosYMax = posYMax,
                    Damping = 0.5f
                }
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
