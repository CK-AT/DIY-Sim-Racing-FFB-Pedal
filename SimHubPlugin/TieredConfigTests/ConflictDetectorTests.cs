using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb;
using DiyFfb.TieredConfig;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for ConflictDetector pure functions.
    /// </summary>
    public static class ConflictDetectorTests
    {
        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // No conflict scenarios
                TestRunner.RunTest("FindConflicts_Empty_NoConflicts", FindConflicts_Empty_NoConflicts),
                TestRunner.RunTest("FindConflicts_SingleFunction_NoConflict", FindConflicts_SingleFunction_NoConflict),
                TestRunner.RunTest("FindConflicts_MultipleFunctions_DifferentAxes_NoConflict", FindConflicts_MultipleFunctions_DifferentAxes_NoConflict),
                TestRunner.RunTest("FindConflicts_EmptyOverrides_NoConflict", FindConflicts_EmptyOverrides_NoConflict),

                // Conflict scenarios
                TestRunner.RunTest("FindConflicts_TwoFunctions_SameAxis_Kinematics_Conflict", FindConflicts_TwoFunctions_SameAxis_Kinematics_Conflict),
                TestRunner.RunTest("FindConflicts_TwoFunctions_SameAxis_StaticBalance_Conflict", FindConflicts_TwoFunctions_SameAxis_StaticBalance_Conflict),
                TestRunner.RunTest("FindConflicts_ThreeFunctions_SameAxis_Conflict", FindConflicts_ThreeFunctions_SameAxis_Conflict),
                TestRunner.RunTest("FindConflicts_MultipleAxes_MultipleConflicts", FindConflicts_MultipleAxes_MultipleConflicts),

                // Partial overlap (no conflict)
                TestRunner.RunTest("FindConflicts_OnlyKinematics_VsOnlyStaticBalance_NoConflict", FindConflicts_OnlyKinematics_VsOnlyStaticBalance_NoConflict),

                // Partial overlap (conflict)
                TestRunner.RunTest("FindConflicts_BothOverrideKinematics_Conflict", FindConflicts_BothOverrideKinematics_Conflict),

                // HasConflicts helper
                TestRunner.RunTest("HasConflicts_WhenConflict_ReturnsTrue", HasConflicts_WhenConflict_ReturnsTrue),
                TestRunner.RunTest("HasConflicts_WhenNoConflict_ReturnsFalse", HasConflicts_WhenNoConflict_ReturnsFalse),

                // GetLockedAxes helper
                TestRunner.RunTest("GetLockedAxes_ReturnsConflictingAxes", GetLockedAxes_ReturnsConflictingAxes),
                TestRunner.RunTest("GetLockedAxes_NoConflicts_ReturnsEmpty", GetLockedAxes_NoConflicts_ReturnsEmpty),
            };
        }

        // === No Conflict Scenarios ===

        private static void FindConflicts_Empty_NoConflicts()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>();

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(0, conflicts.Count, "Empty input should have no conflicts");
        }

        private static void FindConflicts_SingleFunction_NoConflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(0, conflicts.Count, "Single function should have no conflicts");
        }

        private static void FindConflicts_MultipleFunctions_DifferentAxes_NoConflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [3] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(0, conflicts.Count, "Functions on different axes should not conflict");
        }

        private static void FindConflicts_EmptyOverrides_NoConflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = null, StaticBalance = null }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(0, conflicts.Count, "Empty overrides don't conflict (they don't override anything)");
        }

        // === Conflict Scenarios ===

        private static void FindConflicts_TwoFunctions_SameAxis_Kinematics_Conflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics(-50, 50) }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics(-60, 60) }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(1, conflicts.Count, "Two functions overriding same axis kinematics should conflict");
            AssertEqual(2, conflicts[0].AxisId, "Conflict should be on axis 2");
            AssertContains(conflicts[0].ConflictingFunctionIds, 1, "Function 1 should be in conflict");
            AssertContains(conflicts[0].ConflictingFunctionIds, 2, "Function 2 should be in conflict");
        }

        private static void FindConflicts_TwoFunctions_SameAxis_StaticBalance_Conflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { StaticBalance = CreateStaticBalance(0.5f) }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { StaticBalance = CreateStaticBalance(0.7f) }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(1, conflicts.Count, "Two functions overriding same axis static balance should conflict");
        }

        private static void FindConflicts_ThreeFunctions_SameAxis_Conflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [3] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(1, conflicts.Count, "Three functions overriding same axis should produce one conflict");
            AssertEqual(3, conflicts[0].ConflictingFunctionIds.Count, "All three functions should be in the conflict");
        }

        private static void FindConflicts_MultipleAxes_MultipleConflicts()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() },
                    [3] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() },
                    [3] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(2, conflicts.Count, "Should have conflicts on both axes");
            var conflictingAxes = conflicts.Select(c => c.AxisId).OrderBy(x => x).ToList();
            AssertEqual(2, conflictingAxes[0], "First conflict should be axis 2");
            AssertEqual(3, conflictingAxes[1], "Second conflict should be axis 3");
        }

        // === Partial Overlap Tests ===

        private static void FindConflicts_OnlyKinematics_VsOnlyStaticBalance_NoConflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides
                    {
                        Kinematics = CreateKinematics(),
                        StaticBalance = null
                    }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides
                    {
                        Kinematics = null,
                        StaticBalance = CreateStaticBalance(0.5f)
                    }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(0, conflicts.Count, "Overriding different fields should not conflict");
        }

        private static void FindConflicts_BothOverrideKinematics_Conflict()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides
                    {
                        Kinematics = CreateKinematics(),
                        StaticBalance = null
                    }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides
                    {
                        Kinematics = CreateKinematics(),
                        StaticBalance = CreateStaticBalance(0.5f)
                    }
                }
            };

            var conflicts = ConflictDetector.FindAxisConflicts(activeFunctions);

            AssertEqual(1, conflicts.Count, "Both overriding kinematics should conflict");
        }

        // === HasConflicts Helper Tests ===

        private static void HasConflicts_WhenConflict_ReturnsTrue()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            AssertTrue(ConflictDetector.HasConflicts(activeFunctions), "Should detect conflicts");
        }

        private static void HasConflicts_WhenNoConflict_ReturnsFalse()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            AssertFalse(ConflictDetector.HasConflicts(activeFunctions), "Should not detect conflicts");
        }

        // === GetLockedAxes Helper Tests ===

        private static void GetLockedAxes_ReturnsConflictingAxes()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() },
                    [3] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                },
                [2] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var lockedAxes = ConflictDetector.GetLockedAxes(activeFunctions);

            AssertEqual(1, lockedAxes.Count, "Only axis 2 should be locked (axis 3 has no conflict)");
            AssertTrue(lockedAxes.Contains(2), "Axis 2 should be locked");
        }

        private static void GetLockedAxes_NoConflicts_ReturnsEmpty()
        {
            var activeFunctions = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>
            {
                [1] = new Dictionary<int, AxisParameterOverrides>
                {
                    [2] = new AxisParameterOverrides { Kinematics = CreateKinematics() }
                }
            };

            var lockedAxes = ConflictDetector.GetLockedAxes(activeFunctions);

            AssertEqual(0, lockedAxes.Count, "No conflicts should mean no locked axes");
        }

        // === Helper Methods ===

        private static KinematicParameters CreateKinematics(int minPos = -100, int maxPos = 100)
        {
            return new KinematicParameters
            {
                ContactPointPosMinAbs = minPos,
                ContactPointPosMaxAbs = maxPos
            };
        }

        private static AxisConfig.Types.StaticBalanceConfig CreateStaticBalance(float xCenter = 10.0f)
        {
            return new AxisConfig.Types.StaticBalanceConfig
            {
                XCenter = xCenter,
                XHalfRange = 50.0f
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

        private static void AssertContains<T>(IEnumerable<T> collection, T item, string message)
        {
            if (!collection.Contains(item))
                throw new InvalidOperationException($"{message}: collection does not contain {item}");
        }
    }
}
