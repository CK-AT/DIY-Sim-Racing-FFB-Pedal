using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    public static class FlightPedalsProcessorTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Reconciliation tests
                TestRunner.RunTest("Reconcile_SetsOutputFromMotionRange", Reconcile_SetsOutputFromMotionRange),
                TestRunner.RunTest("Reconcile_NullFlightPedals_NoOp", Reconcile_NullFlightPedals_NoOp),
                TestRunner.RunTest("Reconcile_NullConfig_NoOp", Reconcile_NullConfig_NoOp),
                TestRunner.RunTest("Reconcile_NullBase_NoOp", Reconcile_NullBase_NoOp),

                // Override application tests
                TestRunner.RunTest("Apply_MotionRange_NearLim", Apply_MotionRange_NearLim),
                TestRunner.RunTest("Apply_MotionRange_FarLim", Apply_MotionRange_FarLim),
                TestRunner.RunTest("Apply_MotionRange_Both", Apply_MotionRange_Both),
                TestRunner.RunTest("Apply_Damping", Apply_Damping),
                TestRunner.RunTest("Apply_CenteringSpringConst", Apply_CenteringSpringConst),
                TestRunner.RunTest("Apply_RudderBrakeForceRange", Apply_RudderBrakeForceRange),
                TestRunner.RunTest("Apply_RudderBrakeAutoCreated", Apply_RudderBrakeAutoCreated),
                TestRunner.RunTest("Apply_NullDelta_NoOp", Apply_NullDelta_NoOp),
                TestRunner.RunTest("Apply_EmptyDelta_NoOp", Apply_EmptyDelta_NoOp),
                TestRunner.RunTest("Apply_NullAuxConfig_SkipsRudderBrake", Apply_NullAuxConfig_SkipsRudderBrake),
            };
        }

        // === Reconciliation Tests ===

        private static void Reconcile_SetsOutputFromMotionRange()
        {
            var config = CreateFlightPedalsConfig(nearLim: 5, farLim: 45);

            FlightPedalsProcessor.ReconcileDerivedFields(config);

            AssertNear(5.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be PosNearLim");
            AssertNear(45.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be PosFarLim");
        }

        private static void Reconcile_NullFlightPedals_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = new FunctionBase { OutputMin = 1.0f, OutputMax = 2.0f },
                FlightPedals = null
            };

            FlightPedalsProcessor.ReconcileDerivedFields(config);

            AssertNear(1.0f, config.Base.OutputMin, 1e-6f, "OutputMin should be unchanged");
            AssertNear(2.0f, config.Base.OutputMax, 1e-6f, "OutputMax should be unchanged");
        }

        private static void Reconcile_NullConfig_NoOp()
        {
            // Should not throw
            FlightPedalsProcessor.ReconcileDerivedFields(null);
        }

        private static void Reconcile_NullBase_NoOp()
        {
            var config = new FunctionConfig
            {
                Base = null,
                FlightPedals = new FlightPedalsConfig { PosNearLim = 10, PosFarLim = 40 }
            };

            // Should not throw; OutputMin/Max are on Base so nothing to set
            FlightPedalsProcessor.ReconcileDerivedFields(config);
        }

        // === Override Application Tests ===

        private static void Apply_MotionRange_NearLim()
        {
            var fpConfig = new FlightPedalsConfig { PosNearLim = 0, PosFarLim = 50 };
            var delta = new FunctionConfigOverrides
            {
                FlightPedalsMotionRange = new MotionRangeOverrides { NearLim = 10 }
            };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertEqual(10, fpConfig.PosNearLim, "PosNearLim should be overridden");
            AssertEqual(50, fpConfig.PosFarLim, "PosFarLim should be preserved");
        }

        private static void Apply_MotionRange_FarLim()
        {
            var fpConfig = new FlightPedalsConfig { PosNearLim = 0, PosFarLim = 50 };
            var delta = new FunctionConfigOverrides
            {
                FlightPedalsMotionRange = new MotionRangeOverrides { FarLim = 40 }
            };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertEqual(0, fpConfig.PosNearLim, "PosNearLim should be preserved");
            AssertEqual(40, fpConfig.PosFarLim, "PosFarLim should be overridden");
        }

        private static void Apply_MotionRange_Both()
        {
            var fpConfig = new FlightPedalsConfig { PosNearLim = 0, PosFarLim = 50 };
            var delta = new FunctionConfigOverrides
            {
                FlightPedalsMotionRange = new MotionRangeOverrides { NearLim = 5, FarLim = 45 }
            };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertEqual(5, fpConfig.PosNearLim, "PosNearLim should be overridden");
            AssertEqual(45, fpConfig.PosFarLim, "PosFarLim should be overridden");
        }

        private static void Apply_Damping()
        {
            var fpConfig = new FlightPedalsConfig { Damping = 0.5f };
            var delta = new FunctionConfigOverrides { FlightPedalsDamping = 1.2f };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertNear(1.2f, fpConfig.Damping, 1e-6f, "Damping should be overridden");
        }

        private static void Apply_CenteringSpringConst()
        {
            var fpConfig = new FlightPedalsConfig { CenteringSpringConst = 1.5f };
            var delta = new FunctionConfigOverrides { FlightPedalsCenteringSpringConst = 2.5f };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertNear(2.5f, fpConfig.CenteringSpringConst, 1e-6f, "CenteringSpringConst should be overridden");
        }

        private static void Apply_RudderBrakeForceRange()
        {
            var fpConfig = new FlightPedalsConfig();
            var auxConfig = new AuxFunctionConfig
            {
                RudderBrake = new RudderBrakeConfig { FMin = 50, FMax = 150 }
            };
            var delta = new FunctionConfigOverrides
            {
                RudderBrakeForceRange = new ForceRangeOverrides { Min = 30, Max = 200 }
            };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, auxConfig, delta);

            AssertNear(30.0f, auxConfig.RudderBrake.FMin, 1e-6f, "RudderBrake FMin should be overridden");
            AssertNear(200.0f, auxConfig.RudderBrake.FMax, 1e-6f, "RudderBrake FMax should be overridden");
        }

        private static void Apply_RudderBrakeAutoCreated()
        {
            var fpConfig = new FlightPedalsConfig();
            var auxConfig = new AuxFunctionConfig { RudderBrake = null };
            var delta = new FunctionConfigOverrides
            {
                RudderBrakeForceRange = new ForceRangeOverrides { Min = 40 }
            };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, auxConfig, delta);

            AssertTrue(auxConfig.RudderBrake != null, "RudderBrake should be auto-created");
            AssertNear(40.0f, auxConfig.RudderBrake.FMin, 1e-6f, "RudderBrake FMin should be set");
        }

        private static void Apply_NullDelta_NoOp()
        {
            var fpConfig = new FlightPedalsConfig { PosNearLim = 5, PosFarLim = 45, Damping = 0.5f };

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, null);

            AssertEqual(5, fpConfig.PosNearLim, "PosNearLim should be unchanged");
            AssertEqual(45, fpConfig.PosFarLim, "PosFarLim should be unchanged");
            AssertNear(0.5f, fpConfig.Damping, 1e-6f, "Damping should be unchanged");
        }

        private static void Apply_EmptyDelta_NoOp()
        {
            var fpConfig = new FlightPedalsConfig { PosNearLim = 5, PosFarLim = 45, Damping = 0.5f };
            var delta = new FunctionConfigOverrides();

            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertEqual(5, fpConfig.PosNearLim, "PosNearLim should be unchanged");
            AssertEqual(45, fpConfig.PosFarLim, "PosFarLim should be unchanged");
            AssertNear(0.5f, fpConfig.Damping, 1e-6f, "Damping should be unchanged");
        }

        private static void Apply_NullAuxConfig_SkipsRudderBrake()
        {
            var fpConfig = new FlightPedalsConfig { Damping = 0.5f };
            var delta = new FunctionConfigOverrides
            {
                FlightPedalsDamping = 1.0f,
                RudderBrakeForceRange = new ForceRangeOverrides { Min = 30, Max = 200 }
            };

            // Should not throw — rudder brake skipped because auxConfig is null
            FlightPedalsProcessor.ApplyOverrides(fpConfig, null, delta);

            AssertNear(1.0f, fpConfig.Damping, 1e-6f, "Damping should still be overridden");
        }

        // === Helper Methods ===

        private static FunctionConfig CreateFlightPedalsConfig(int nearLim, int farLim)
        {
            return new FunctionConfig
            {
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.FlightPedals,
                    OutputMin = 0,
                    OutputMax = 0
                },
                FlightPedals = new FlightPedalsConfig
                {
                    PosNearLim = nearLim,
                    PosFarLim = farLim,
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
