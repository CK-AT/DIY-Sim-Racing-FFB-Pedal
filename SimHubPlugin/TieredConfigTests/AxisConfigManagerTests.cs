using System;
using System.Collections.Generic;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for AxisConfigManager lifecycle management.
    /// </summary>
    public static class AxisConfigManagerTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // Base config management
                TestRunner.RunTest("SetBaseConfig_StoresConfig", SetBaseConfig_StoresConfig),
                TestRunner.RunTest("SetBaseConfig_NullThrows", SetBaseConfig_NullThrows),
                TestRunner.RunTest("GetBaseConfig_ReturnsClone", GetBaseConfig_ReturnsClone),
                TestRunner.RunTest("GetBaseConfig_MissingReturnsNull", GetBaseConfig_MissingReturnsNull),

                // Apply function overrides
                TestRunner.RunTest("ApplyFunctionOverride_MergesConfig", ApplyFunctionOverride_MergesConfig),
                TestRunner.RunTest("ApplyFunctionOverride_TracksActiveFunction", ApplyFunctionOverride_TracksActiveFunction),
                TestRunner.RunTest("ApplyFunctionOverride_NoBaseConfig_Throws", ApplyFunctionOverride_NoBaseConfig_Throws),
                TestRunner.RunTest("ApplyFunctionOverride_EmptyOverrides_NoOp", ApplyFunctionOverride_EmptyOverrides_NoOp),
                TestRunner.RunTest("ApplyFunctionOverride_FiresEvent", ApplyFunctionOverride_FiresEvent),

                // Clear function overrides
                TestRunner.RunTest("ClearFunctionOverride_RestoresBaseConfig", ClearFunctionOverride_RestoresBaseConfig),
                TestRunner.RunTest("ClearFunctionOverride_RemovesTracking", ClearFunctionOverride_RemovesTracking),
                TestRunner.RunTest("ClearFunctionOverride_NoActiveOverride_NoOp", ClearFunctionOverride_NoActiveOverride_NoOp),
                TestRunner.RunTest("ClearFunctionOverrides_ByFunctionId", ClearFunctionOverrides_ByFunctionId),

                // Diff checking
                TestRunner.RunTest("ApplyFunctionOverride_DiffCheck_NoSendIfUnchanged", ApplyFunctionOverride_DiffCheck_NoSendIfUnchanged),
                TestRunner.RunTest("ApplyFunctionOverride_NoDiffCheck_AlwaysSends", ApplyFunctionOverride_NoDiffCheck_AlwaysSends),

                // Reset
                TestRunner.RunTest("Reset_ClearsAllState", Reset_ClearsAllState),
                TestRunner.RunTest("ResetAxis_ClearsSpecificAxis", ResetAxis_ClearsSpecificAxis),

                // Query methods
                TestRunner.RunTest("HasFunctionOverride_ReturnsCorrectState", HasFunctionOverride_ReturnsCorrectState),
                TestRunner.RunTest("GetOverridingFunction_ReturnsCorrectId", GetOverridingFunction_ReturnsCorrectId),
                TestRunner.RunTest("GetActiveOverrides_ReturnsAll", GetActiveOverrides_ReturnsAll),
            };
        }

        // === Base Config Management ===

        private static void SetBaseConfig_StoresConfig()
        {
            var manager = new AxisConfigManager();
            var config = CreateAxisConfig(minPos: -50, maxPos: 50);

            manager.SetBaseConfig(1, config);

            var retrieved = manager.GetBaseConfig(1);
            AssertNotNull(retrieved, "Should store and retrieve config");
            AssertEqual(-50, retrieved.KinematicParameters.ContactPointPosMinAbs, "Config should match");
        }

        private static void SetBaseConfig_NullThrows()
        {
            var manager = new AxisConfigManager();

            AssertThrows<ArgumentNullException>(
                () => manager.SetBaseConfig(1, null),
                "Null config should throw");
        }

        private static void GetBaseConfig_ReturnsClone()
        {
            var manager = new AxisConfigManager();
            var config = CreateAxisConfig(minPos: -50, maxPos: 50);
            manager.SetBaseConfig(1, config);

            var retrieved = manager.GetBaseConfig(1);
            retrieved.KinematicParameters.ContactPointPosMinAbs = -999;

            var retrievedAgain = manager.GetBaseConfig(1);
            AssertEqual(-50, retrievedAgain.KinematicParameters.ContactPointPosMinAbs,
                "Modifying retrieved config should not affect stored config");
        }

        private static void GetBaseConfig_MissingReturnsNull()
        {
            var manager = new AxisConfigManager();

            var retrieved = manager.GetBaseConfig(999);

            AssertNull(retrieved, "Missing axis should return null");
        }

        // === Apply Function Overrides ===

        private static void ApplyFunctionOverride_MergesConfig()
        {
            var manager = new AxisConfigManager();
            var baseConfig = CreateAxisConfig(minPos: -50, maxPos: 50);
            manager.SetBaseConfig(1, baseConfig);

            var overrides = new AxisParameterOverrides
            {
                Kinematics = CreateKinematics(-99, 99)
            };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            var current = manager.GetCurrentConfig(1);
            AssertEqual(-99, current.KinematicParameters.ContactPointPosMinAbs, "Override should be applied");
        }

        private static void ApplyFunctionOverride_TracksActiveFunction()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertTrue(manager.HasFunctionOverride(1), "Should track override");
            AssertEqual(100, manager.GetOverridingFunction(1), "Should track function ID");
        }

        private static void ApplyFunctionOverride_NoBaseConfig_Throws()
        {
            var manager = new AxisConfigManager();
            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };

            AssertThrows<InvalidOperationException>(
                () => manager.ApplyFunctionOverride(1, 100, overrides),
                "Missing base config should throw");
        }

        private static void ApplyFunctionOverride_EmptyOverrides_NoOp()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            var overrides = new AxisParameterOverrides(); // empty
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertFalse(manager.HasFunctionOverride(1), "Empty overrides should not be tracked");
        }

        private static void ApplyFunctionOverride_FiresEvent()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            AxisConfigChangedEventArgs receivedArgs = null;
            manager.AxisConfigChanged += (s, e) => receivedArgs = e;

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics(-99, 99) };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertNotNull(receivedArgs, "Event should be fired");
            AssertEqual(1, receivedArgs.AxisId, "Event should have correct axis ID");
            AssertTrue(receivedArgs.HasFunctionOverride, "Event should indicate function override");
            AssertEqual(100, receivedArgs.OverridingFunctionId, "Event should have function ID");
        }

        // === Clear Function Overrides ===

        private static void ClearFunctionOverride_RestoresBaseConfig()
        {
            var manager = new AxisConfigManager();
            var baseConfig = CreateAxisConfig(minPos: -50, maxPos: 50);
            manager.SetBaseConfig(1, baseConfig);

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics(-99, 99) };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);
            manager.ClearFunctionOverride(1, diffCheck: false);

            var current = manager.GetCurrentConfig(1);
            AssertEqual(-50, current.KinematicParameters.ContactPointPosMinAbs, "Should restore base config");
        }

        private static void ClearFunctionOverride_RemovesTracking()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);
            manager.ClearFunctionOverride(1, diffCheck: false);

            AssertFalse(manager.HasFunctionOverride(1), "Should remove tracking");
            AssertNull(manager.GetOverridingFunction(1), "Should clear function ID");
        }

        private static void ClearFunctionOverride_NoActiveOverride_NoOp()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            int eventCount = 0;
            manager.AxisConfigChanged += (s, e) => eventCount++;

            manager.ClearFunctionOverride(1, diffCheck: false);

            AssertEqual(0, eventCount, "No event should fire if no active override");
        }

        private static void ClearFunctionOverrides_ByFunctionId()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());
            manager.SetBaseConfig(2, CreateAxisConfig());

            var overrides = new Dictionary<int, AxisParameterOverrides>
            {
                [1] = new AxisParameterOverrides { Kinematics = CreateKinematics() },
                [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
            };
            manager.ApplyFunctionOverrides(100, overrides, diffCheck: false);

            manager.ClearFunctionOverrides(100, diffCheck: false);

            AssertFalse(manager.HasFunctionOverride(1), "Should clear axis 1");
            AssertFalse(manager.HasFunctionOverride(2), "Should clear axis 2");
        }

        // === Diff Checking ===

        private static void ApplyFunctionOverride_DiffCheck_NoSendIfUnchanged()
        {
            var manager = new AxisConfigManager();
            var baseConfig = CreateAxisConfig(minPos: -50, maxPos: 50);
            manager.SetBaseConfig(1, baseConfig);

            // First apply
            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics(-99, 99) };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            int eventCount = 0;
            manager.AxisConfigChanged += (s, e) => eventCount++;

            // Apply same override again with diff check
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: true);

            AssertEqual(0, eventCount, "Should not fire event if config unchanged");
        }

        private static void ApplyFunctionOverride_NoDiffCheck_AlwaysSends()
        {
            var manager = new AxisConfigManager();
            var baseConfig = CreateAxisConfig(minPos: -50, maxPos: 50);
            manager.SetBaseConfig(1, baseConfig);

            // First apply
            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics(-99, 99) };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            int eventCount = 0;
            manager.AxisConfigChanged += (s, e) => eventCount++;

            // Apply same override again without diff check
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertEqual(1, eventCount, "Should fire event even if config unchanged when diffCheck=false");
        }

        // === Reset ===

        private static void Reset_ClearsAllState()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());
            manager.SetBaseConfig(2, CreateAxisConfig());

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            manager.Reset();

            AssertNull(manager.GetBaseConfig(1), "Should clear base configs");
            AssertNull(manager.GetBaseConfig(2), "Should clear base configs");
            AssertFalse(manager.HasFunctionOverride(1), "Should clear overrides");
            AssertEqual(0, manager.GetActiveOverrides().Count, "Should clear active overrides");
        }

        private static void ResetAxis_ClearsSpecificAxis()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());
            manager.SetBaseConfig(2, CreateAxisConfig());

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);
            manager.ApplyFunctionOverride(2, 101, overrides, diffCheck: false);

            manager.ResetAxis(1);

            AssertNull(manager.GetBaseConfig(1), "Should clear axis 1");
            AssertNotNull(manager.GetBaseConfig(2), "Should keep axis 2");
            AssertFalse(manager.HasFunctionOverride(1), "Should clear axis 1 override");
            AssertTrue(manager.HasFunctionOverride(2), "Should keep axis 2 override");
        }

        // === Query Methods ===

        private static void HasFunctionOverride_ReturnsCorrectState()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            AssertFalse(manager.HasFunctionOverride(1), "Should be false initially");

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertTrue(manager.HasFunctionOverride(1), "Should be true after apply");

            manager.ClearFunctionOverride(1, diffCheck: false);

            AssertFalse(manager.HasFunctionOverride(1), "Should be false after clear");
        }

        private static void GetOverridingFunction_ReturnsCorrectId()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());

            AssertNull(manager.GetOverridingFunction(1), "Should be null initially");

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);

            AssertEqual(100, manager.GetOverridingFunction(1), "Should return function ID");
        }

        private static void GetActiveOverrides_ReturnsAll()
        {
            var manager = new AxisConfigManager();
            manager.SetBaseConfig(1, CreateAxisConfig());
            manager.SetBaseConfig(2, CreateAxisConfig());

            var overrides = new AxisParameterOverrides { Kinematics = CreateKinematics() };
            manager.ApplyFunctionOverride(1, 100, overrides, diffCheck: false);
            manager.ApplyFunctionOverride(2, 101, overrides, diffCheck: false);

            var active = manager.GetActiveOverrides();

            AssertEqual(2, active.Count, "Should return all active overrides");
            AssertEqual(100, active[1], "Should have correct function for axis 1");
            AssertEqual(101, active[2], "Should have correct function for axis 2");
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
