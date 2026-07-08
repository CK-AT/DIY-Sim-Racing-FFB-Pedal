using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for ChangeTracker pending change management.
    /// </summary>
    public static class ChangeTrackerTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Basic tracking tests
                TestRunner.RunTest("TrackChange_AddsToCorrectLayer", TrackChange_AddsToCorrectLayer),
                TestRunner.RunTest("TrackChange_AutoRoutes_UserField", TrackChange_AutoRoutes_UserField),
                TestRunner.RunTest("TrackChange_AutoRoutes_BaselineField", TrackChange_AutoRoutes_BaselineField),
                TestRunner.RunTest("TrackChange_AutoRoutes_ProfileField", TrackChange_AutoRoutes_ProfileField),
                TestRunner.RunTest("TrackChange_OverwritesSameField", TrackChange_OverwritesSameField),

                // HasUnsavedChanges tests
                TestRunner.RunTest("HasUnsavedChanges_FalseWhenEmpty", HasUnsavedChanges_FalseWhenEmpty),
                TestRunner.RunTest("HasUnsavedChanges_TrueAfterTrack", HasUnsavedChanges_TrueAfterTrack),
                TestRunner.RunTest("HasUnsavedChangesForLayer_Specific", HasUnsavedChangesForLayer_Specific),
                TestRunner.RunTest("HasUnsavedChangesForFunction_Specific", HasUnsavedChangesForFunction_Specific),

                // GetPendingChanges tests
                TestRunner.RunTest("GetPendingChanges_ByLayer_ReturnsCorrectChanges", GetPendingChanges_ByLayer_ReturnsCorrectChanges),
                TestRunner.RunTest("GetPendingChanges_ByFunction_ReturnsAllLayers", GetPendingChanges_ByFunction_ReturnsAllLayers),
                TestRunner.RunTest("GetAllPendingChanges_ReturnsAll", GetAllPendingChanges_ReturnsAll),
                TestRunner.RunTest("GetPendingChanges_ReturnsCopy", GetPendingChanges_ReturnsCopy),

                // Commit tests
                TestRunner.RunTest("CommitChanges_ClearsLayer", CommitChanges_ClearsLayer),
                TestRunner.RunTest("CommitChanges_ClearsLayerAndFunction", CommitChanges_ClearsLayerAndFunction),
                TestRunner.RunTest("CommitChanges_PreservesOtherLayers", CommitChanges_PreservesOtherLayers),

                // Discard tests
                TestRunner.RunTest("DiscardAll_ClearsEverything", DiscardAll_ClearsEverything),
                TestRunner.RunTest("DiscardChanges_ByLayer_ClearsOnlyLayer", DiscardChanges_ByLayer_ClearsOnlyLayer),
                TestRunner.RunTest("DiscardChanges_ByFunction_ClearsAllLayers", DiscardChanges_ByFunction_ClearsAllLayers),
                TestRunner.RunTest("DiscardChange_SingleField_RemovesOnlyField", DiscardChange_SingleField_RemovesOnlyField),
                TestRunner.RunTest("DiscardChange_LastField_RemovesFunction", DiscardChange_LastField_RemovesFunction),

                // Reroute tests
                TestRunner.RunTest("RerouteChange_MovesToNewLayer", RerouteChange_MovesToNewLayer),
                TestRunner.RunTest("RerouteChange_NonExistent_NoOp", RerouteChange_NonExistent_NoOp),
                TestRunner.RunTest("RerouteChange_CleansUpEmptySource", RerouteChange_CleansUpEmptySource),

                // Count tests
                TestRunner.RunTest("Count_ReturnsCorrectTotal", Count_ReturnsCorrectTotal),
            };
        }

        // === Basic Tracking Tests ===

        private static void TrackChange_AddsToCorrectLayer()
        {
            var tracker = new ChangeTracker();

            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);
            AssertTrue(changes.ContainsKey(1), "Should have function 1");
            AssertTrue(changes[1].ContainsKey("output_min"), "Should have field output_min");
            AssertEqual(0.5f, (float)changes[1]["output_min"], "Value should match");
        }

        private static void TrackChange_AutoRoutes_UserField()
        {
            var tracker = new ChangeTracker();

            tracker.TrackChange(1, "output_min", 0.5f);

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "output_min should route to User layer");
            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Should not be in Profile");
            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.Baseline), "Should not be in Baseline");
        }

        private static void TrackChange_AutoRoutes_BaselineField()
        {
            var tracker = new ChangeTracker();

            tracker.TrackChange(1, "kinematic_parameters", new object());

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Baseline), "kinematic_parameters should route to Baseline");
            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "Should not be in User");
        }

        private static void TrackChange_AutoRoutes_ProfileField()
        {
            var tracker = new ChangeTracker();

            tracker.TrackChange(1, "shifter_config.gate_width", 10.0f);

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Unknown field should route to Profile");
            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "Should not be in User");
        }

        private static void TrackChange_OverwritesSameField()
        {
            var tracker = new ChangeTracker();

            tracker.TrackChange(1, "output_min", 0.3f, ConfigLayer.User);
            tracker.TrackChange(1, "output_min", 0.7f, ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);
            AssertEqual(0.7f, (float)changes[1]["output_min"], "Should overwrite with latest value");
            AssertEqual(1, tracker.Count, "Should only have one change");
        }

        // === HasUnsavedChanges Tests ===

        private static void HasUnsavedChanges_FalseWhenEmpty()
        {
            var tracker = new ChangeTracker();

            AssertFalse(tracker.HasUnsavedChanges, "New tracker should have no changes");
        }

        private static void HasUnsavedChanges_TrueAfterTrack()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            AssertTrue(tracker.HasUnsavedChanges, "Should have changes after tracking");
        }

        private static void HasUnsavedChangesForLayer_Specific()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(2, "gate_width", 10.0f, ConfigLayer.Profile);

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "User layer should have changes");
            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Profile layer should have changes");
            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.Baseline), "Baseline layer should be empty");
        }

        private static void HasUnsavedChangesForFunction_Specific()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(2, "gate_width", 10.0f, ConfigLayer.Profile);

            AssertTrue(tracker.HasUnsavedChangesForFunction(1), "Function 1 should have changes");
            AssertTrue(tracker.HasUnsavedChangesForFunction(2), "Function 2 should have changes");
            AssertFalse(tracker.HasUnsavedChangesForFunction(999), "Non-existent function should have no changes");
        }

        // === GetPendingChanges Tests ===

        private static void GetPendingChanges_ByLayer_ReturnsCorrectChanges()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "output_max", 0.8f, ConfigLayer.User);
            tracker.TrackChange(2, "friction", 0.3f, ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);

            AssertEqual(2, changes.Count, "Should have 2 functions");
            AssertEqual(2, changes[1].Count, "Function 1 should have 2 fields");
            AssertEqual(1, changes[2].Count, "Function 2 should have 1 field");
        }

        private static void GetPendingChanges_ByFunction_ReturnsAllLayers()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "gate_width", 10.0f, ConfigLayer.Profile);
            tracker.TrackChange(1, "steps_per_mm", 100, ConfigLayer.Baseline);

            var changes = tracker.GetPendingChanges(1);

            AssertEqual(3, changes.Count, "Should have changes in 3 layers");
            AssertTrue(changes.ContainsKey(ConfigLayer.User), "Should have User layer");
            AssertTrue(changes.ContainsKey(ConfigLayer.Profile), "Should have Profile layer");
            AssertTrue(changes.ContainsKey(ConfigLayer.Baseline), "Should have Baseline layer");
        }

        private static void GetAllPendingChanges_ReturnsAll()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "gate_width", 10.0f, ConfigLayer.Profile);
            tracker.TrackChange(2, "friction", 0.3f, ConfigLayer.User);

            var changes = tracker.GetAllPendingChanges();

            AssertEqual(3, changes.Count, "Should have 3 total changes");
        }

        private static void GetPendingChanges_ReturnsCopy()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);
            changes.Clear();

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "Original should not be affected");
        }

        // === Commit Tests ===

        private static void CommitChanges_ClearsLayer()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(2, "output_max", 0.8f, ConfigLayer.User);

            tracker.CommitChanges(ConfigLayer.User);

            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "User layer should be empty after commit");
        }

        private static void CommitChanges_ClearsLayerAndFunction()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(2, "output_max", 0.8f, ConfigLayer.User);

            tracker.CommitChanges(ConfigLayer.User, 1);

            AssertTrue(tracker.HasUnsavedChangesForFunction(2), "Function 2 should still have changes");
            AssertFalse(tracker.HasUnsavedChangesForFunction(1), "Function 1 should be cleared");
        }

        private static void CommitChanges_PreservesOtherLayers()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "gate_width", 10.0f, ConfigLayer.Profile);

            tracker.CommitChanges(ConfigLayer.User);

            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Profile should still have changes");
        }

        // === Discard Tests ===

        private static void DiscardAll_ClearsEverything()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(2, "gate_width", 10.0f, ConfigLayer.Profile);
            tracker.TrackChange(3, "steps_per_mm", 100, ConfigLayer.Baseline);

            tracker.DiscardAll();

            AssertFalse(tracker.HasUnsavedChanges, "Should have no changes after discard all");
            AssertEqual(0, tracker.Count, "Count should be 0");
        }

        private static void DiscardChanges_ByLayer_ClearsOnlyLayer()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "gate_width", 10.0f, ConfigLayer.Profile);

            tracker.DiscardChanges(ConfigLayer.User);

            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "User should be cleared");
            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Profile should remain");
        }

        private static void DiscardChanges_ByFunction_ClearsAllLayers()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "gate_width", 10.0f, ConfigLayer.Profile);
            tracker.TrackChange(2, "friction", 0.3f, ConfigLayer.User);

            tracker.DiscardChanges(1);

            AssertFalse(tracker.HasUnsavedChangesForFunction(1), "Function 1 should be cleared");
            AssertTrue(tracker.HasUnsavedChangesForFunction(2), "Function 2 should remain");
        }

        private static void DiscardChange_SingleField_RemovesOnlyField()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "output_max", 0.8f, ConfigLayer.User);

            tracker.DiscardChange(1, "output_min", ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);
            AssertFalse(changes[1].ContainsKey("output_min"), "output_min should be removed");
            AssertTrue(changes[1].ContainsKey("output_max"), "output_max should remain");
        }

        private static void DiscardChange_LastField_RemovesFunction()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            tracker.DiscardChange(1, "output_min", ConfigLayer.User);

            var changes = tracker.GetPendingChanges(ConfigLayer.User);
            AssertFalse(changes.ContainsKey(1), "Function should be removed when last field is discarded");
        }

        // === Reroute Tests ===

        private static void RerouteChange_MovesToNewLayer()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            tracker.RerouteChange(1, "output_min", ConfigLayer.User, ConfigLayer.Profile);

            AssertFalse(tracker.HasUnsavedChangesForLayer(ConfigLayer.User), "User should be empty");
            AssertTrue(tracker.HasUnsavedChangesForLayer(ConfigLayer.Profile), "Profile should have the change");
            var changes = tracker.GetPendingChanges(ConfigLayer.Profile);
            AssertEqual(0.5f, (float)changes[1]["output_min"], "Value should be preserved");
        }

        private static void RerouteChange_NonExistent_NoOp()
        {
            var tracker = new ChangeTracker();

            tracker.RerouteChange(1, "nonexistent", ConfigLayer.User, ConfigLayer.Profile);

            AssertFalse(tracker.HasUnsavedChanges, "Should have no changes");
        }

        private static void RerouteChange_CleansUpEmptySource()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);

            tracker.RerouteChange(1, "output_min", ConfigLayer.User, ConfigLayer.Profile);

            var userChanges = tracker.GetPendingChanges(ConfigLayer.User);
            AssertFalse(userChanges.ContainsKey(1), "Empty function should be cleaned up from source layer");
        }

        // === Count Tests ===

        private static void Count_ReturnsCorrectTotal()
        {
            var tracker = new ChangeTracker();
            tracker.TrackChange(1, "output_min", 0.5f, ConfigLayer.User);
            tracker.TrackChange(1, "output_max", 0.8f, ConfigLayer.User);
            tracker.TrackChange(2, "gate_width", 10.0f, ConfigLayer.Profile);

            AssertEqual(3, tracker.Count, "Should count all fields across all layers");
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
