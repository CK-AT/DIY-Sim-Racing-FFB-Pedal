using System;
using System.Collections.Generic;
using System.Linq;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    internal static class Program
    {
        private static int Main()
        {
            Console.WriteLine("Tiered Config Tests");
            Console.WriteLine("===================");
            Console.WriteLine();

            var allResults = new List<TestResult>();
            bool allPassed = true;

            // Run AutomotivePedalProcessor tests
            var processorResults = AutomotivePedalProcessorTests.RunAll();
            TestRunner.PrintResults("AutomotivePedalProcessor", processorResults);
            allResults.AddRange(processorResults);
            if (!processorResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run ConfigMerger tests
            var mergerResults = ConfigMergerTests.RunAll();
            TestRunner.PrintResults("ConfigMerger", mergerResults);
            allResults.AddRange(mergerResults);
            if (!mergerResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run ConfigComparer tests
            var comparerResults = ConfigComparerTests.RunAll();
            TestRunner.PrintResults("ConfigComparer", comparerResults);
            allResults.AddRange(comparerResults);
            if (!comparerResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run ConflictDetector tests
            var conflictResults = ConflictDetectorTests.RunAll();
            TestRunner.PrintResults("ConflictDetector", conflictResults);
            allResults.AddRange(conflictResults);
            if (!conflictResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run AxisConfigManager tests
            var axisManagerResults = AxisConfigManagerTests.RunAll();
            TestRunner.PrintResults("AxisConfigManager", axisManagerResults);
            allResults.AddRange(axisManagerResults);
            if (!axisManagerResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run ChangeTracker tests
            var changeTrackerResults = ChangeTrackerTests.RunAll();
            TestRunner.PrintResults("ChangeTracker", changeTrackerResults);
            allResults.AddRange(changeTrackerResults);
            if (!changeTrackerResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run FieldRouter tests
            var fieldRouterResults = FieldRouterTests.RunAll();
            TestRunner.PrintResults("FieldRouter", fieldRouterResults);
            allResults.AddRange(fieldRouterResults);
            if (!fieldRouterResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Run OverrideFieldRegistry tests
            var registryResults = OverrideFieldRegistryTests.RunAll();
            TestRunner.PrintResults("OverrideFieldRegistry", registryResults);
            allResults.AddRange(registryResults);
            if (!registryResults.TrueForAll(r => r.Passed))
                allPassed = false;

            // Summary
            Console.WriteLine("===================");
            int totalPassed = allResults.Count(r => r.Passed);
            int totalFailed = allResults.Count - totalPassed;
            Console.WriteLine($"Total: {totalPassed}/{allResults.Count} tests passed");

            if (totalFailed > 0)
            {
                Console.WriteLine();
                Console.WriteLine("Failed tests:");
                foreach (var result in allResults.Where(r => !r.Passed))
                {
                    Console.WriteLine($"  - {result.Name}: {result.Message}");
                }
            }

            return allPassed ? 0 : 1;
        }
    }
}
