using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    public static class FlightStickProcessorTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Reconciliation tests
                TestRunner.RunTest("Reconcile_Pitch_SetsOutputFromPosRange", Reconcile_Pitch_SetsOutputFromPosRange),
                TestRunner.RunTest("Reconcile_Roll_SetsOutputFromPosRange", Reconcile_Roll_SetsOutputFromPosRange),
                TestRunner.RunTest("Reconcile_Collective_SetsOutputFromPosRange", Reconcile_Collective_SetsOutputFromPosRange),
                TestRunner.RunTest("Reconcile_NullSubConfig_NoOp", Reconcile_NullSubConfig_NoOp),
                TestRunner.RunTest("Reconcile_NullConfig_NoOp", Reconcile_NullConfig_NoOp),
                TestRunner.RunTest("Reconcile_NullBase_NoOp", Reconcile_NullBase_NoOp),

                // Override application tests — Pitch
                TestRunner.RunTest("Apply_Pitch_MotionRange_Min", Apply_Pitch_MotionRange_Min),
                TestRunner.RunTest("Apply_Pitch_MotionRange_Max", Apply_Pitch_MotionRange_Max),
                TestRunner.RunTest("Apply_Pitch_MotionRange_Both", Apply_Pitch_MotionRange_Both),
                TestRunner.RunTest("Apply_Pitch_Damping", Apply_Pitch_Damping),
                TestRunner.RunTest("Apply_Pitch_CenteringSpringConst", Apply_Pitch_CenteringSpringConst),

                // Override application tests — Roll
                TestRunner.RunTest("Apply_Roll_MotionRange", Apply_Roll_MotionRange),
                TestRunner.RunTest("Apply_Roll_Damping", Apply_Roll_Damping),

                // Override application tests — Collective
                TestRunner.RunTest("Apply_Collective_MotionRange", Apply_Collective_MotionRange),
                TestRunner.RunTest("Apply_Collective_CenteringSpringConst", Apply_Collective_CenteringSpringConst),

                // Null/empty delta
                TestRunner.RunTest("Apply_NullDelta_NoOp", Apply_NullDelta_NoOp),
                TestRunner.RunTest("Apply_EmptyDelta_NoOp", Apply_EmptyDelta_NoOp),
                TestRunner.RunTest("Apply_NullBase_NoOp", Apply_NullBase_NoOp),

                // Auto-creation of sub-config
                TestRunner.RunTest("Apply_Pitch_AutoCreatesSubConfig", Apply_Pitch_AutoCreatesSubConfig),
            };
        }

        // === Reconciliation Tests ===

        private static void Reconcile_Pitch_SetsOutputFromPosRange()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);

            FlightStickProcessor.ReconcileDerivedFields(config);

            AssertNear(-20.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be PosMin");
            AssertNear(20.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be PosMax");
        }

        private static void Reconcile_Roll_SetsOutputFromPosRange()
        {
            var config = CreateRollConfig(posMin: -15, posMax: 15);

            FlightStickProcessor.ReconcileDerivedFields(config);

            AssertNear(-15.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be PosMin");
            AssertNear(15.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be PosMax");
        }

        private static void Reconcile_Collective_SetsOutputFromPosRange()
        {
            var config = CreateCollectiveConfig(posMin: 0, posMax: 50);

            FlightStickProcessor.ReconcileDerivedFields(config);

            AssertNear(0.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be PosMin");
            AssertNear(50.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be PosMax");
        }

        private static void Reconcile_NullSubConfig_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.FlightStickPitch,
                    OutputMin = 1.0f,
                    OutputMax = 2.0f
                },
                FlightStickPitch = null
            };

            FlightStickProcessor.ReconcileDerivedFields(config);

            AssertNear(1.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be unchanged");
            AssertNear(2.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be unchanged");
        }

        private static void Reconcile_NullConfig_NoOp()
        {
            // Should not throw
            FlightStickProcessor.ReconcileDerivedFields(null);
        }

        private static void Reconcile_NullBase_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = null,
                FlightStickPitch = new FlightStickPitchConfig { PosMin = -10, PosMax = 10 }
            };

            // Should not throw
            FlightStickProcessor.ReconcileDerivedFields(config);
        }

        // === Override Application Tests — Pitch ===

        private static void Apply_Pitch_MotionRange_Min()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = -30 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(-30, config.FlightStickPitch.PosMin, "PosMin should be overridden");
            AssertEqual(20, config.FlightStickPitch.PosMax, "PosMax should be preserved");
        }

        private static void Apply_Pitch_MotionRange_Max()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Max = 30 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(-20, config.FlightStickPitch.PosMin, "PosMin should be preserved");
            AssertEqual(30, config.FlightStickPitch.PosMax, "PosMax should be overridden");
        }

        private static void Apply_Pitch_MotionRange_Both()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = -25, Max = 25 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(-25, config.FlightStickPitch.PosMin, "PosMin should be overridden");
            AssertEqual(25, config.FlightStickPitch.PosMax, "PosMax should be overridden");
        }

        private static void Apply_Pitch_Damping()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            var delta = new FunctionConfigOverrides { FlightStickDamping = 1.5f };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertNear(1.5f, config.FlightStickPitch.Damping, 1e-6f, "Damping should be overridden");
        }

        private static void Apply_Pitch_CenteringSpringConst()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            var delta = new FunctionConfigOverrides { FlightStickCenteringSpringConst = 3.0f };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertNear(3.0f, config.FlightStickPitch.CenteringSpringConst, 1e-6f, "CenteringSpringConst should be overridden");
        }

        // === Override Application Tests — Roll ===

        private static void Apply_Roll_MotionRange()
        {
            var config = CreateRollConfig(posMin: -15, posMax: 15);
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = -10, Max = 10 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(-10, config.FlightStickRoll.PosMin, "PosMin should be overridden");
            AssertEqual(10, config.FlightStickRoll.PosMax, "PosMax should be overridden");
        }

        private static void Apply_Roll_Damping()
        {
            var config = CreateRollConfig(posMin: -15, posMax: 15);
            var delta = new FunctionConfigOverrides { FlightStickDamping = 2.0f };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertNear(2.0f, config.FlightStickRoll.Damping, 1e-6f, "Damping should be overridden");
        }

        // === Override Application Tests — Collective ===

        private static void Apply_Collective_MotionRange()
        {
            var config = CreateCollectiveConfig(posMin: 0, posMax: 50);
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = 5, Max = 45 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(5, config.FlightStickCollective.PosMin, "PosMin should be overridden");
            AssertEqual(45, config.FlightStickCollective.PosMax, "PosMax should be overridden");
        }

        private static void Apply_Collective_CenteringSpringConst()
        {
            var config = CreateCollectiveConfig(posMin: 0, posMax: 50);
            var delta = new FunctionConfigOverrides { FlightStickCenteringSpringConst = 0.5f };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertNear(0.5f, config.FlightStickCollective.CenteringSpringConst, 1e-6f, "CenteringSpringConst should be overridden");
        }

        // === Null/Empty Delta Tests ===

        private static void Apply_NullDelta_NoOp()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            config.FlightStickPitch.Damping = 0.5f;

            FlightStickProcessor.ApplyOverrides(config, null);

            AssertEqual(-20, config.FlightStickPitch.PosMin, "PosMin should be unchanged");
            AssertEqual(20, config.FlightStickPitch.PosMax, "PosMax should be unchanged");
            AssertNear(0.5f, config.FlightStickPitch.Damping, 1e-6f, "Damping should be unchanged");
        }

        private static void Apply_EmptyDelta_NoOp()
        {
            var config = CreatePitchConfig(posMin: -20, posMax: 20);
            config.FlightStickPitch.Damping = 0.5f;
            var delta = new FunctionConfigOverrides();

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertEqual(-20, config.FlightStickPitch.PosMin, "PosMin should be unchanged");
            AssertEqual(20, config.FlightStickPitch.PosMax, "PosMax should be unchanged");
            AssertNear(0.5f, config.FlightStickPitch.Damping, 1e-6f, "Damping should be unchanged");
        }

        private static void Apply_NullBase_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = null,
                FlightStickPitch = new FlightStickPitchConfig { PosMin = -20, PosMax = 20 }
            };
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = -30 }
            };

            // Should not throw
            FlightStickProcessor.ApplyOverrides(config, delta);

            // Sub-config unchanged because Base is null — can't dispatch
            AssertEqual(-20, config.FlightStickPitch.PosMin, "PosMin should be unchanged");
        }

        // === Auto-creation Tests ===

        private static void Apply_Pitch_AutoCreatesSubConfig()
        {
            var config = new FunctionConfig
            {
                Base = new FunctionBase { FunctionId = FunctionID.FlightStickPitch },
                FlightStickPitch = null
            };
            var delta = new FunctionConfigOverrides
            {
                FlightStickMotionRange = new MotionRangeOverrides { Min = -10, Max = 10 }
            };

            FlightStickProcessor.ApplyOverrides(config, delta);

            AssertTrue(config.FlightStickPitch != null, "FlightStickPitch should be auto-created");
            AssertEqual(-10, config.FlightStickPitch.PosMin, "PosMin should be set");
            AssertEqual(10, config.FlightStickPitch.PosMax, "PosMax should be set");
        }

        // === Helper Methods ===

        private static FunctionConfig CreatePitchConfig(int posMin, int posMax)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.FlightStickPitch,
                    OutputMin = 0,
                    OutputMax = 0
                },
                FlightStickPitch = new FlightStickPitchConfig
                {
                    PosMin = posMin,
                    PosMax = posMax,
                    Damping = 0.5f,
                    CenteringSpringConst = 1.5f
                }
            };
        }

        private static FunctionConfig CreateRollConfig(int posMin, int posMax)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.FlightStickRoll,
                    OutputMin = 0,
                    OutputMax = 0
                },
                FlightStickRoll = new FlightStickRollConfig
                {
                    PosMin = posMin,
                    PosMax = posMax,
                    Damping = 0.5f,
                    CenteringSpringConst = 1.5f
                }
            };
        }

        private static FunctionConfig CreateCollectiveConfig(int posMin, int posMax)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.FlightStickCollective,
                    OutputMin = 0,
                    OutputMax = 0
                },
                FlightStickCollective = new FlightStickCollectiveConfig
                {
                    PosMin = posMin,
                    PosMax = posMax,
                    Damping = 0.5f,
                    CenteringSpringConst = 1.5f
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
