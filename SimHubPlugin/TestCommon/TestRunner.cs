using System;
using System.Collections.Generic;

namespace SimHubPlugin.TestCommon
{
    /// <summary>
    /// Common test infrastructure shared across all test projects.
    /// </summary>
    public static class TestRunner
    {
        public static TestResult RunTest(string name, Func<bool> test)
        {
            try
            {
                return new TestResult(name, test());
            }
            catch (Exception ex)
            {
                return new TestResult(name, false, ex.Message);
            }
        }

        public static TestResult RunTest(string name, Action test)
        {
            try
            {
                test();
                return new TestResult(name, true);
            }
            catch (Exception ex)
            {
                return new TestResult(name, false, ex.Message);
            }
        }

        public static void PrintResults(string suiteName, List<TestResult> results)
        {
            Console.WriteLine($"{suiteName}:");
            int passed = 0;
            foreach (var result in results)
            {
                Console.WriteLine($"  [{(result.Passed ? "PASS" : "FAIL")}] {result.Name}{(string.IsNullOrWhiteSpace(result.Message) ? "" : $" - {result.Message}")}");
                if (result.Passed)
                {
                    passed++;
                }
            }
            Console.WriteLine($"  {passed}/{results.Count} tests passed.");
            Console.WriteLine();
        }
    }

    public sealed class TestResult
    {
        public string Name { get; }
        public bool Passed { get; }
        public string Message { get; }

        public TestResult(string name, bool passed, string message = "")
        {
            Name = name;
            Passed = passed;
            Message = message;
        }
    }
}
