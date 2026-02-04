using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for OverrideFieldRegistry field definitions and value operations.
    /// </summary>
    public static class OverrideFieldRegistryTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Field registration tests
                TestRunner.RunTest("GetField_OutputMin_ReturnsDefinition", GetField_OutputMin_ReturnsDefinition),
                TestRunner.RunTest("GetField_OutputMax_ReturnsDefinition", GetField_OutputMax_ReturnsDefinition),
                TestRunner.RunTest("GetField_SimulatedMass_ReturnsDefinition", GetField_SimulatedMass_ReturnsDefinition),
                TestRunner.RunTest("GetField_Friction_ReturnsDefinition", GetField_Friction_ReturnsDefinition),
                TestRunner.RunTest("GetField_StaticBalanceEnabled_ReturnsDefinition", GetField_StaticBalanceEnabled_ReturnsDefinition),
                TestRunner.RunTest("GetField_StaticBalanceGain_ReturnsDefinition", GetField_StaticBalanceGain_ReturnsDefinition),

                // Field lookup tests (case-insensitive)
                TestRunner.RunTest("GetField_ByPath_CaseInsensitive", GetField_ByPath_CaseInsensitive),
                TestRunner.RunTest("GetField_ByName_CaseInsensitive", GetField_ByName_CaseInsensitive),
                TestRunner.RunTest("GetField_UnknownField_ReturnsNull", GetField_UnknownField_ReturnsNull),
                TestRunner.RunTest("GetField_NullOrEmpty_ReturnsNull", GetField_NullOrEmpty_ReturnsNull),

                // GetAllFields tests
                TestRunner.RunTest("GetAllFields_ReturnsAllRegistered", GetAllFields_ReturnsAllRegistered),

                // GetFieldsByGroup tests
                TestRunner.RunTest("GetFieldsByGroup_OutputScaling_ReturnsCorrectFields", GetFieldsByGroup_OutputScaling_ReturnsCorrectFields),
                TestRunner.RunTest("GetFieldsByGroup_Physics_ReturnsCorrectFields", GetFieldsByGroup_Physics_ReturnsCorrectFields),
                TestRunner.RunTest("GetFieldsByGroup_StaticBalanceTuning_ReturnsCorrectFields", GetFieldsByGroup_StaticBalanceTuning_ReturnsCorrectFields),

                // Layer routing tests
                TestRunner.RunTest("GetTargetLayer_OutputMin_ReturnsUser", GetTargetLayer_OutputMin_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_SimulatedMass_ReturnsUser", GetTargetLayer_SimulatedMass_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_StaticBalanceTuning_ReturnsUser", GetTargetLayer_StaticBalanceTuning_ReturnsUser),
                TestRunner.RunTest("GetTargetLayer_UnknownField_FallbackToRouter", GetTargetLayer_UnknownField_FallbackToRouter),

                // IsUserTunable tests
                TestRunner.RunTest("IsUserTunable_OutputMin_ReturnsTrue", IsUserTunable_OutputMin_ReturnsTrue),
                TestRunner.RunTest("IsUserTunable_KinematicParameters_ReturnsFalse", IsUserTunable_KinematicParameters_ReturnsFalse),

                // Float field value operations
                TestRunner.RunTest("GetValue_OutputMin_ReturnsValue", GetValue_OutputMin_ReturnsValue),
                TestRunner.RunTest("GetValue_OutputMin_NotSet_ReturnsNull", GetValue_OutputMin_NotSet_ReturnsNull),
                TestRunner.RunTest("SetValue_OutputMin_SetsValue", SetValue_OutputMin_SetsValue),
                TestRunner.RunTest("ClearValue_OutputMin_ClearsValue", ClearValue_OutputMin_ClearsValue),
                TestRunner.RunTest("HasValue_OutputMin_WhenSet_ReturnsTrue", HasValue_OutputMin_WhenSet_ReturnsTrue),
                TestRunner.RunTest("HasValue_OutputMin_WhenNotSet_ReturnsFalse", HasValue_OutputMin_WhenNotSet_ReturnsFalse),

                // Nested field value operations (StaticBalanceTuning)
                TestRunner.RunTest("GetValue_StaticBalanceEnabled_ReturnsValue", GetValue_StaticBalanceEnabled_ReturnsValue),
                TestRunner.RunTest("SetValue_StaticBalanceEnabled_CreatesParent", SetValue_StaticBalanceEnabled_CreatesParent),
                TestRunner.RunTest("ClearValue_StaticBalanceEnabled_ClearsValue", ClearValue_StaticBalanceEnabled_ClearsValue),
                TestRunner.RunTest("ClearValue_StaticBalanceEnabled_RemovesParentIfEmpty", ClearValue_StaticBalanceEnabled_RemovesParentIfEmpty),
                TestRunner.RunTest("HasValue_StaticBalanceEnabled_WhenSet_ReturnsTrue", HasValue_StaticBalanceEnabled_WhenSet_ReturnsTrue),
                TestRunner.RunTest("HasValue_StaticBalanceGain_WhenSet_ReturnsTrue", HasValue_StaticBalanceGain_WhenSet_ReturnsTrue),

                // Path normalization tests
                TestRunner.RunTest("NormalizeFieldPath_OutputMin_ReturnsLowercase", NormalizeFieldPath_OutputMin_ReturnsLowercase),
                TestRunner.RunTest("NormalizeFieldPath_UnknownField_ReturnsLowercase", NormalizeFieldPath_UnknownField_ReturnsLowercase),

                // Field metadata tests
                TestRunner.RunTest("FieldDefinition_OutputMin_HasCorrectMetadata", FieldDefinition_OutputMin_HasCorrectMetadata),
                TestRunner.RunTest("FieldDefinition_StaticBalanceEnabled_HasCorrectType", FieldDefinition_StaticBalanceEnabled_HasCorrectType),

                // Edge case tests
                TestRunner.RunTest("SetValue_NullOverrides_ThrowsException", SetValue_NullOverrides_ThrowsException),
                TestRunner.RunTest("SetValue_UnknownField_ThrowsException", SetValue_UnknownField_ThrowsException),
                TestRunner.RunTest("GetValue_NullOverrides_ReturnsNull", GetValue_NullOverrides_ReturnsNull),
                TestRunner.RunTest("ClearValue_NullOverrides_DoesNotThrow", ClearValue_NullOverrides_DoesNotThrow),
            };
        }

        // === Field Registration Tests ===

        private static void GetField_OutputMin_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("output_min");
            AssertNotNull(field, "output_min field should be registered");
            AssertEqual("OutputMin", field.Name, "Field name should match");
            AssertEqual("output_min", field.FieldPath, "Field path should match");
        }

        private static void GetField_OutputMax_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("output_max");
            AssertNotNull(field, "output_max field should be registered");
            AssertEqual("OutputMax", field.Name, "Field name should match");
        }

        private static void GetField_SimulatedMass_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("simulated_mass");
            AssertNotNull(field, "simulated_mass field should be registered");
            AssertEqual("SimulatedMass", field.Name, "Field name should match");
        }

        private static void GetField_Friction_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("friction");
            AssertNotNull(field, "friction field should be registered");
            AssertEqual("Friction", field.Name, "Field name should match");
        }

        private static void GetField_StaticBalanceEnabled_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("static_balance_tuning.enabled");
            AssertNotNull(field, "static_balance_tuning.enabled field should be registered");
            AssertEqual("StaticBalanceEnabled", field.Name, "Field name should match");
        }

        private static void GetField_StaticBalanceGain_ReturnsDefinition()
        {
            var field = OverrideFieldRegistry.GetField("static_balance_tuning.gain");
            AssertNotNull(field, "static_balance_tuning.gain field should be registered");
            AssertEqual("StaticBalanceGain", field.Name, "Field name should match");
        }

        // === Field Lookup Tests ===

        private static void GetField_ByPath_CaseInsensitive()
        {
            var lower = OverrideFieldRegistry.GetField("output_min");
            var upper = OverrideFieldRegistry.GetField("OUTPUT_MIN");
            var mixed = OverrideFieldRegistry.GetField("Output_Min");

            AssertNotNull(lower, "Lowercase path should work");
            AssertNotNull(upper, "Uppercase path should work");
            AssertNotNull(mixed, "Mixed case path should work");
            AssertEqual(lower.Name, upper.Name, "Should return same field definition");
        }

        private static void GetField_ByName_CaseInsensitive()
        {
            var normal = OverrideFieldRegistry.GetField("OutputMin");
            var lower = OverrideFieldRegistry.GetField("outputmin");
            var upper = OverrideFieldRegistry.GetField("OUTPUTMIN");

            AssertNotNull(normal, "Normal name should work");
            AssertNotNull(lower, "Lowercase name should work");
            AssertNotNull(upper, "Uppercase name should work");
            AssertEqual("OutputMin", normal.Name, "Should return correct field");
        }

        private static void GetField_UnknownField_ReturnsNull()
        {
            var field = OverrideFieldRegistry.GetField("unknown_field");
            AssertNull(field, "Unknown field should return null");
        }

        private static void GetField_NullOrEmpty_ReturnsNull()
        {
            var nullField = OverrideFieldRegistry.GetField(null);
            var emptyField = OverrideFieldRegistry.GetField("");

            AssertNull(nullField, "Null field path should return null");
            AssertNull(emptyField, "Empty field path should return null");
        }

        // === GetAllFields Tests ===

        private static void GetAllFields_ReturnsAllRegistered()
        {
            var fields = OverrideFieldRegistry.GetAllFields().ToList();

            AssertTrue(fields.Count >= 6, "Should have at least 6 registered fields");
            AssertTrue(fields.Any(f => f.Name == "OutputMin"), "Should contain OutputMin");
            AssertTrue(fields.Any(f => f.Name == "OutputMax"), "Should contain OutputMax");
            AssertTrue(fields.Any(f => f.Name == "SimulatedMass"), "Should contain SimulatedMass");
            AssertTrue(fields.Any(f => f.Name == "Friction"), "Should contain Friction");
            AssertTrue(fields.Any(f => f.Name == "StaticBalanceEnabled"), "Should contain StaticBalanceEnabled");
            AssertTrue(fields.Any(f => f.Name == "StaticBalanceGain"), "Should contain StaticBalanceGain");
        }

        // === GetFieldsByGroup Tests ===

        private static void GetFieldsByGroup_OutputScaling_ReturnsCorrectFields()
        {
            var fields = OverrideFieldRegistry.GetFieldsByGroup(OverrideFieldGroup.OutputScaling).ToList();

            AssertTrue(fields.Count >= 2, "OutputScaling group should have at least 2 fields");
            AssertTrue(fields.Any(f => f.Name == "OutputMin"), "Should contain OutputMin");
            AssertTrue(fields.Any(f => f.Name == "OutputMax"), "Should contain OutputMax");
        }

        private static void GetFieldsByGroup_Physics_ReturnsCorrectFields()
        {
            var fields = OverrideFieldRegistry.GetFieldsByGroup(OverrideFieldGroup.Physics).ToList();

            AssertTrue(fields.Count >= 2, "Physics group should have at least 2 fields");
            AssertTrue(fields.Any(f => f.Name == "SimulatedMass"), "Should contain SimulatedMass");
            AssertTrue(fields.Any(f => f.Name == "Friction"), "Should contain Friction");
        }

        private static void GetFieldsByGroup_StaticBalanceTuning_ReturnsCorrectFields()
        {
            var fields = OverrideFieldRegistry.GetFieldsByGroup(OverrideFieldGroup.StaticBalanceTuning).ToList();

            AssertTrue(fields.Count >= 2, "StaticBalanceTuning group should have at least 2 fields");
            AssertTrue(fields.Any(f => f.Name == "StaticBalanceEnabled"), "Should contain StaticBalanceEnabled");
            AssertTrue(fields.Any(f => f.Name == "StaticBalanceGain"), "Should contain StaticBalanceGain");
        }

        // === Layer Routing Tests ===

        private static void GetTargetLayer_OutputMin_ReturnsUser()
        {
            var layer = OverrideFieldRegistry.GetTargetLayer("output_min");
            AssertEqual(ConfigLayer.User, layer, "output_min should target User layer");
        }

        private static void GetTargetLayer_SimulatedMass_ReturnsUser()
        {
            var layer = OverrideFieldRegistry.GetTargetLayer("simulated_mass");
            AssertEqual(ConfigLayer.User, layer, "simulated_mass should target User layer");
        }

        private static void GetTargetLayer_StaticBalanceTuning_ReturnsUser()
        {
            var layer = OverrideFieldRegistry.GetTargetLayer("static_balance_tuning.enabled");
            AssertEqual(ConfigLayer.User, layer, "static_balance_tuning.enabled should target User layer");
        }

        private static void GetTargetLayer_UnknownField_FallbackToRouter()
        {
            // Unknown fields should fallback to FieldRouter
            var layer = OverrideFieldRegistry.GetTargetLayer("unknown_field");
            AssertEqual(ConfigLayer.Profile, layer, "Unknown field should default to Profile via FieldRouter");
        }

        // === IsUserTunable Tests ===

        private static void IsUserTunable_OutputMin_ReturnsTrue()
        {
            AssertTrue(OverrideFieldRegistry.IsUserTunable("output_min"), "output_min should be user tunable");
        }

        private static void IsUserTunable_KinematicParameters_ReturnsFalse()
        {
            // This should fallback to FieldRouter which returns Hardware
            AssertFalse(OverrideFieldRegistry.IsUserTunable("kinematic_parameters"),
                "kinematic_parameters should not be user tunable");
        }

        // === Float Field Value Operations ===

        private static void GetValue_OutputMin_ReturnsValue()
        {
            var overrides = new FunctionConfigOverrides { OutputMin = 0.5f };
            var value = OverrideFieldRegistry.GetValue(overrides, "output_min");

            AssertNotNull(value, "Value should not be null");
            AssertEqual(0.5f, (float?)value, "Should return correct value");
        }

        private static void GetValue_OutputMin_NotSet_ReturnsNull()
        {
            var overrides = new FunctionConfigOverrides();
            var value = OverrideFieldRegistry.GetValue(overrides, "output_min");

            AssertNull(value, "Value should be null when not set");
        }

        private static void SetValue_OutputMin_SetsValue()
        {
            var overrides = new FunctionConfigOverrides();
            OverrideFieldRegistry.SetValue(overrides, "output_min", 0.3f);

            AssertEqual(0.3f, overrides.OutputMin, "OutputMin should be set");
        }

        private static void ClearValue_OutputMin_ClearsValue()
        {
            var overrides = new FunctionConfigOverrides { OutputMin = 0.5f };
            OverrideFieldRegistry.ClearValue(overrides, "output_min");

            AssertNull(overrides.OutputMin, "OutputMin should be null after clear");
        }

        private static void HasValue_OutputMin_WhenSet_ReturnsTrue()
        {
            var overrides = new FunctionConfigOverrides { OutputMin = 0.5f };
            AssertTrue(OverrideFieldRegistry.HasValue(overrides, "output_min"),
                "HasValue should return true when set");
        }

        private static void HasValue_OutputMin_WhenNotSet_ReturnsFalse()
        {
            var overrides = new FunctionConfigOverrides();
            AssertFalse(OverrideFieldRegistry.HasValue(overrides, "output_min"),
                "HasValue should return false when not set");
        }

        // === Nested Field Value Operations ===

        private static void GetValue_StaticBalanceEnabled_ReturnsValue()
        {
            var overrides = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides { Enabled = true }
            };
            var value = OverrideFieldRegistry.GetValue(overrides, "static_balance_tuning.enabled");

            AssertNotNull(value, "Value should not be null");
            AssertEqual(true, (bool?)value, "Should return correct value");
        }

        private static void SetValue_StaticBalanceEnabled_CreatesParent()
        {
            var overrides = new FunctionConfigOverrides();
            OverrideFieldRegistry.SetValue(overrides, "static_balance_tuning.enabled", true);

            AssertNotNull(overrides.StaticBalanceTuning, "Parent should be created");
            AssertEqual(true, overrides.StaticBalanceTuning.Enabled, "Enabled should be set");
        }

        private static void ClearValue_StaticBalanceEnabled_ClearsValue()
        {
            var overrides = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides { Enabled = true, Gain = 1.0f }
            };
            OverrideFieldRegistry.ClearValue(overrides, "static_balance_tuning.enabled");

            AssertNull(overrides.StaticBalanceTuning.Enabled, "Enabled should be null");
            AssertNotNull(overrides.StaticBalanceTuning, "Parent should remain (Gain still set)");
        }

        private static void ClearValue_StaticBalanceEnabled_RemovesParentIfEmpty()
        {
            var overrides = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides { Enabled = true }
            };
            OverrideFieldRegistry.ClearValue(overrides, "static_balance_tuning.enabled");

            AssertNull(overrides.StaticBalanceTuning, "Parent should be removed when empty");
        }

        private static void HasValue_StaticBalanceEnabled_WhenSet_ReturnsTrue()
        {
            var overrides = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides { Enabled = true }
            };
            AssertTrue(OverrideFieldRegistry.HasValue(overrides, "static_balance_tuning.enabled"),
                "HasValue should return true when set");
        }

        private static void HasValue_StaticBalanceGain_WhenSet_ReturnsTrue()
        {
            var overrides = new FunctionConfigOverrides
            {
                StaticBalanceTuning = new StaticBalanceTuningOverrides { Gain = 1.0f }
            };
            AssertTrue(OverrideFieldRegistry.HasValue(overrides, "static_balance_tuning.gain"),
                "HasValue should return true when set");
        }

        // === Path Normalization Tests ===

        private static void NormalizeFieldPath_OutputMin_ReturnsLowercase()
        {
            var normalized = OverrideFieldRegistry.NormalizeFieldPath("OutputMin");
            AssertEqual("output_min", normalized, "Should normalize to lowercase path");
        }

        private static void NormalizeFieldPath_UnknownField_ReturnsLowercase()
        {
            var normalized = OverrideFieldRegistry.NormalizeFieldPath("UnknownField");
            AssertEqual("unknownfield", normalized, "Unknown field should just be lowercased");
        }

        // === Field Metadata Tests ===

        private static void FieldDefinition_OutputMin_HasCorrectMetadata()
        {
            var field = OverrideFieldRegistry.GetField("output_min");

            AssertEqual(OverrideFieldType.Float, field.FieldType, "Should be Float type");
            AssertEqual(OverrideFieldGroup.OutputScaling, field.Group, "Should be OutputScaling group");
            AssertEqual(ConfigLayer.User, field.DefaultLayer, "Should target User layer");
            AssertNotNull(field.DisplayName, "DisplayName should be set");
            AssertNotNull(field.Tooltip, "Tooltip should be set");
        }

        private static void FieldDefinition_StaticBalanceEnabled_HasCorrectType()
        {
            var field = OverrideFieldRegistry.GetField("static_balance_tuning.enabled");

            AssertEqual(OverrideFieldType.Bool, field.FieldType, "Should be Bool type");
            AssertEqual(OverrideFieldGroup.StaticBalanceTuning, field.Group, "Should be StaticBalanceTuning group");
        }

        // === Edge Case Tests ===

        private static void SetValue_NullOverrides_ThrowsException()
        {
            try
            {
                OverrideFieldRegistry.SetValue(null, "output_min", 0.5f);
                throw new InvalidOperationException("Should have thrown ArgumentNullException");
            }
            catch (ArgumentNullException)
            {
                // Expected
            }
        }

        private static void SetValue_UnknownField_ThrowsException()
        {
            try
            {
                var overrides = new FunctionConfigOverrides();
                OverrideFieldRegistry.SetValue(overrides, "unknown_field", 0.5f);
                throw new InvalidOperationException("Should have thrown ArgumentException");
            }
            catch (ArgumentException)
            {
                // Expected
            }
        }

        private static void GetValue_NullOverrides_ReturnsNull()
        {
            var value = OverrideFieldRegistry.GetValue(null, "output_min");
            AssertNull(value, "Should return null for null overrides");
        }

        private static void ClearValue_NullOverrides_DoesNotThrow()
        {
            // Should not throw
            OverrideFieldRegistry.ClearValue(null, "output_min");
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

        private static void AssertNotNull(object obj, string message)
        {
            if (obj == null)
                throw new InvalidOperationException(message);
        }

        private static void AssertNull(object obj, string message)
        {
            if (obj != null)
                throw new InvalidOperationException(message);
        }
    }
}
