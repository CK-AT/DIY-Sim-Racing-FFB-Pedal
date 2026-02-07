using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for FieldRouter layer routing logic.
    /// </summary>
    public static class FieldRouterTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // User field routing tests
                TestRunner.RunTest("GetTargetLayer_OutputMin_ReturnsUser", GetTargetLayer_OutputMin_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_OutputMax_ReturnsUser", GetTargetLayer_OutputMax_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_SimulatedMass_ReturnsUser", GetTargetLayer_SimulatedMass_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_Friction_ReturnsUser", GetTargetLayer_Friction_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_StaticBalanceTuning_ReturnsUser", GetTargetLayer_StaticBalanceTuning_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_StaticBalanceTuningNested_ReturnsUser", GetTargetLayer_StaticBalanceTuningNested_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_BaseOutputMin_ReturnsUser", GetTargetLayer_BaseOutputMin_ReturnsUser),

                // Baseline field routing tests
                TestRunner.RunTest("GetTargetLayer_KinematicParameters_ReturnsBaseline", GetTargetLayer_KinematicParameters_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_KinematicParametersNested_ReturnsBaseline", GetTargetLayer_KinematicParametersNested_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_StaticBalanceConfig_ReturnsBaseline", GetTargetLayer_StaticBalanceConfig_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_StaticBalanceConfigNested_ReturnsBaseline", GetTargetLayer_StaticBalanceConfigNested_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_LinkedAxes_ReturnsBaseline", GetTargetLayer_LinkedAxes_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_StepsPerMm_ReturnsBaseline", GetTargetLayer_StepsPerMm_ReturnsBaseline),
                TestRunner.RunTest("GetTargetLayer_FMaxLoadcell_ReturnsBaseline", GetTargetLayer_FMaxLoadcell_ReturnsBaseline),

                // Profile field routing tests (default)
                TestRunner.RunTest("GetTargetLayer_ShifterConfig_ReturnsProfile", GetTargetLayer_ShifterConfig_ReturnsProfile),
                TestRunner.RunTest("GetTargetLayer_UnknownField_ReturnsProfile", GetTargetLayer_UnknownField_ReturnsProfile),
                TestRunner.RunTest("GetTargetLayer_EmptyField_ReturnsProfile", GetTargetLayer_EmptyField_ReturnsProfile),
                TestRunner.RunTest("GetTargetLayer_NullField_ReturnsProfile", GetTargetLayer_NullField_ReturnsProfile),

                // Case insensitivity tests
                TestRunner.RunTest("GetTargetLayer_CaseInsensitive_OutputMin", GetTargetLayer_CaseInsensitive_OutputMin),
                TestRunner.RunTest("GetTargetLayer_CaseInsensitive_KinematicParameters", GetTargetLayer_CaseInsensitive_KinematicParameters),

                // IsUserTunable tests
                TestRunner.RunTest("IsUserTunable_UserField_ReturnsTrue", IsUserTunable_UserField_ReturnsTrue),
                TestRunner.RunTest("IsUserTunable_BaselineField_ReturnsFalse", IsUserTunable_BaselineField_ReturnsFalse),
                TestRunner.RunTest("IsUserTunable_ProfileField_ReturnsFalse", IsUserTunable_ProfileField_ReturnsFalse),

                // IsBaselineField tests
                TestRunner.RunTest("IsBaselineField_BaselineField_ReturnsTrue", IsBaselineField_BaselineField_ReturnsTrue),
                TestRunner.RunTest("IsBaselineField_UserField_ReturnsFalse", IsBaselineField_UserField_ReturnsFalse),
                TestRunner.RunTest("IsBaselineField_ProfileField_ReturnsFalse", IsBaselineField_ProfileField_ReturnsFalse),

                // GetUserTunableFields tests
                TestRunner.RunTest("GetUserTunableFields_ContainsExpectedFields", GetUserTunableFields_ContainsExpectedFields),
            };
        }

        // === User Field Routing Tests ===

        private static void GetTargetLayer_OutputMin_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("output_min");
            AssertEqual(ConfigLayer.User, layer, "output_min should route to User");
        }

        private static void GetTargetLayer_OutputMax_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("output_max");
            AssertEqual(ConfigLayer.User, layer, "output_max should route to User");
        }

        private static void GetTargetLayer_SimulatedMass_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("simulated_mass");
            AssertEqual(ConfigLayer.User, layer, "simulated_mass should route to User");
        }

        private static void GetTargetLayer_Friction_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("friction");
            AssertEqual(ConfigLayer.User, layer, "friction should route to User");
        }

        private static void GetTargetLayer_StaticBalanceTuning_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("static_balance_tuning");
            AssertEqual(ConfigLayer.User, layer, "static_balance_tuning should route to User");
        }

        private static void GetTargetLayer_StaticBalanceTuningNested_ReturnsUser()
        {
            var enabledLayer = FieldRouter.GetTargetLayer("static_balance_tuning.enabled");
            var gainLayer = FieldRouter.GetTargetLayer("static_balance_tuning.gain");
            var deepNestedLayer = FieldRouter.GetTargetLayer("static_balance_tuning.some_future_field");

            AssertEqual(ConfigLayer.User, enabledLayer, "static_balance_tuning.enabled should route to User");
            AssertEqual(ConfigLayer.User, gainLayer, "static_balance_tuning.gain should route to User");
            AssertEqual(ConfigLayer.User, deepNestedLayer, "static_balance_tuning.* nested fields should route to User");
        }

        private static void GetTargetLayer_BaseOutputMin_ReturnsUser()
        {
            var layer = FieldRouter.GetTargetLayer("base.output_min");
            AssertEqual(ConfigLayer.User, layer, "base.output_min should route to User");
        }

        // === Baseline Field Routing Tests ===

        private static void GetTargetLayer_KinematicParameters_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("kinematic_parameters");
            AssertEqual(ConfigLayer.Baseline, layer, "kinematic_parameters should route to Baseline");
        }

        private static void GetTargetLayer_KinematicParametersNested_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("kinematic_parameters.contact_point_pos_min");
            AssertEqual(ConfigLayer.Baseline, layer, "kinematic_parameters.* should route to Baseline");
        }

        private static void GetTargetLayer_StaticBalanceConfig_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("static_balance_config");
            AssertEqual(ConfigLayer.Baseline, layer, "static_balance_config should route to Baseline");
        }

        private static void GetTargetLayer_StaticBalanceConfigNested_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("static_balance_config.x_center");
            AssertEqual(ConfigLayer.Baseline, layer, "static_balance_config.* should route to Baseline");
        }

        private static void GetTargetLayer_LinkedAxes_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("linked_axes");
            AssertEqual(ConfigLayer.Baseline, layer, "linked_axes should route to Baseline");
        }

        private static void GetTargetLayer_StepsPerMm_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("steps_per_mm");
            AssertEqual(ConfigLayer.Baseline, layer, "steps_per_mm should route to Baseline");
        }

        private static void GetTargetLayer_FMaxLoadcell_ReturnsBaseline()
        {
            var layer = FieldRouter.GetTargetLayer("f_max_loadcell");
            AssertEqual(ConfigLayer.Baseline, layer, "f_max_loadcell should route to Baseline");
        }

        // === Profile Field Routing Tests (default) ===

        private static void GetTargetLayer_ShifterConfig_ReturnsProfile()
        {
            var layer = FieldRouter.GetTargetLayer("shifter_config.gate_width");
            AssertEqual(ConfigLayer.Profile, layer, "shifter_config.* should route to Profile");
        }

        private static void GetTargetLayer_UnknownField_ReturnsProfile()
        {
            var layer = FieldRouter.GetTargetLayer("some_unknown_field");
            AssertEqual(ConfigLayer.Profile, layer, "Unknown fields should default to Profile");
        }

        private static void GetTargetLayer_EmptyField_ReturnsProfile()
        {
            var layer = FieldRouter.GetTargetLayer("");
            AssertEqual(ConfigLayer.Profile, layer, "Empty field should default to Profile");
        }

        private static void GetTargetLayer_NullField_ReturnsProfile()
        {
            var layer = FieldRouter.GetTargetLayer(null);
            AssertEqual(ConfigLayer.Profile, layer, "Null field should default to Profile");
        }

        // === Case Insensitivity Tests ===

        private static void GetTargetLayer_CaseInsensitive_OutputMin()
        {
            var lower = FieldRouter.GetTargetLayer("output_min");
            var upper = FieldRouter.GetTargetLayer("OUTPUT_MIN");
            var mixed = FieldRouter.GetTargetLayer("Output_Min");

            AssertEqual(ConfigLayer.User, lower, "output_min lowercase should route to User");
            AssertEqual(ConfigLayer.User, upper, "OUTPUT_MIN uppercase should route to User");
            AssertEqual(ConfigLayer.User, mixed, "Output_Min mixed case should route to User");
        }

        private static void GetTargetLayer_CaseInsensitive_KinematicParameters()
        {
            var lower = FieldRouter.GetTargetLayer("kinematic_parameters");
            var upper = FieldRouter.GetTargetLayer("KINEMATIC_PARAMETERS");

            AssertEqual(ConfigLayer.Baseline, lower, "kinematic_parameters lowercase should route to Baseline");
            AssertEqual(ConfigLayer.Baseline, upper, "KINEMATIC_PARAMETERS uppercase should route to Baseline");
        }

        // === IsUserTunable Tests ===

        private static void IsUserTunable_UserField_ReturnsTrue()
        {
            AssertTrue(FieldRouter.IsUserTunable("output_min"), "output_min should be user tunable");
            AssertTrue(FieldRouter.IsUserTunable("friction"), "friction should be user tunable");
        }

        private static void IsUserTunable_BaselineField_ReturnsFalse()
        {
            AssertFalse(FieldRouter.IsUserTunable("kinematic_parameters"), "kinematic_parameters should not be user tunable");
        }

        private static void IsUserTunable_ProfileField_ReturnsFalse()
        {
            AssertFalse(FieldRouter.IsUserTunable("shifter_config"), "shifter_config should not be user tunable");
        }

        // === IsBaselineField Tests ===

        private static void IsBaselineField_BaselineField_ReturnsTrue()
        {
            AssertTrue(FieldRouter.IsBaselineField("kinematic_parameters"), "kinematic_parameters should be baseline field");
            AssertTrue(FieldRouter.IsBaselineField("steps_per_mm"), "steps_per_mm should be baseline field");
        }

        private static void IsBaselineField_UserField_ReturnsFalse()
        {
            AssertFalse(FieldRouter.IsBaselineField("output_min"), "output_min should not be baseline field");
        }

        private static void IsBaselineField_ProfileField_ReturnsFalse()
        {
            AssertFalse(FieldRouter.IsBaselineField("shifter_config"), "shifter_config should not be baseline field");
        }

        // === GetUserTunableFields Tests ===

        private static void GetUserTunableFields_ContainsExpectedFields()
        {
            var fields = FieldRouter.GetUserTunableFields().ToList();

            AssertTrue(fields.Contains("output_min"), "Should contain output_min");
            AssertTrue(fields.Contains("output_max"), "Should contain output_max");
            AssertTrue(fields.Contains("simulated_mass"), "Should contain simulated_mass");
            AssertTrue(fields.Contains("friction"), "Should contain friction");
            AssertTrue(fields.Contains("static_balance_tuning"), "Should contain static_balance_tuning");
            AssertTrue(fields.Count >= 5, "Should have at least 5 user-tunable fields");
        }

        // === Helper Methods ===

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

        private static void AssertEqual<T>(T expected, T actual, string message)
        {
            if (!EqualityComparer<T>.Default.Equals(expected, actual))
                throw new InvalidOperationException($"{message}: expected {expected}, got {actual}");
        }
    }
}
