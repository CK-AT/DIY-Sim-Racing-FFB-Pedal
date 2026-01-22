using System;
using System.Collections.Generic;

namespace DiyFfb.GraphTest
{
    public static class GraphTestRunner
    {
        public static void Run()
        {
            var results = new List<TestResult>
            {
                RunTest("Evaluator basic outputs", TestEvaluatorBasicOutputs),
                RunTest("Evaluator trace values", TestEvaluatorTraceValues),
                RunTest("JSON load/save roundtrip", TestJsonRoundtrip),
                RunTest("Validation catches missing output", TestIncludeOutputValidation),
                RunTest("Inline include mapping", TestInlineIncludeMapping),
                RunTest("Block library index", TestBlockLibraryIndex),
                RunTest("Schema version mismatch", TestSchemaVersionMismatch),
                RunTest("Unknown function validation", TestUnknownFunctionValidation),
                RunTest("Output missing src validation", TestOutputMissingSrc),
                RunTest("Include missing path validation", TestIncludeMissingPath),
                RunTest("Include mapping warnings", TestIncludeMappingWarnings),
                RunTest("Op arg count validation", TestOpArgValidation),
                RunTest("Clamp bound order warning", TestClampBoundOrderWarning)
            };

            PrintResults(results);
        }

        private static TestResult RunTest(string name, Func<bool> test)
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

        private static bool TestEvaluatorBasicOutputs()
        {
            var graph = BuildBaseGraph();
            var outputs = new GraphEvaluator(graph).Evaluate(
                new Dictionary<string, double> { ["ias_kts"] = 60.0, ["vref_kts"] = 60.0 },
                new Dictionary<string, double> { ["k_q"] = 1.0, ["k_rate"] = 0.5, ["k_friction"] = 0.1 });

            return outputs.ContainsKey("spring") &&
                   outputs.ContainsKey("damper") &&
                   outputs.ContainsKey("friction");
        }

        private static bool TestEvaluatorTraceValues()
        {
            var graph = BuildBaseGraph();
            var result = new GraphEvaluator(graph).EvaluateWithTrace(
                new Dictionary<string, double> { ["ias_kts"] = 80.0, ["vref_kts"] = 80.0 },
                new Dictionary<string, double> { ["k_q"] = 1.0, ["k_rate"] = 0.5, ["k_friction"] = 0.1 });

            return result.NodeValues.ContainsKey("qhat") &&
                   result.NodeValues.ContainsKey("spring") &&
                   result.Outputs.ContainsKey("spring");
        }

        private static bool TestJsonRoundtrip()
        {
            var graph = BuildBaseGraph();
            var saver = new GraphSaver();
            string json = saver.SaveToJson(graph);
            var loader = new GraphLoader();
            var loaded = loader.LoadFromJson(json, out var validation);
            return validation.IsValid && loaded.Nodes.Count == graph.Nodes.Count;
        }

        private static bool TestIncludeOutputValidation()
        {
            var graph = new GraphDefinition();
            graph.Nodes["in"] = new GraphNode { Id = "in", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["include"] = new GraphNode
            {
                Id = "include",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph(),
                InputMap = { ["cmd_force"] = "in" },
                OutputMap = { ["missing_out"] = "host_out" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "host_out" };

            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestInlineIncludeMapping()
        {
            var graph = new GraphDefinition();
            graph.Nodes["load"] = new GraphNode { Id = "load", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph(),
                InputMap = { ["cmd_force"] = "load" },
                OutputMap = { ["out_force"] = "force_out" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "force_out" };

            var outputs = new GraphEvaluator(graph, new GraphIncludeResolver(AppContext.BaseDirectory)).Evaluate(
                new Dictionary<string, double> { ["cmd_force"] = 2.0 },
                new Dictionary<string, double>());

            return outputs.TryGetValue("out", out var value) && Math.Abs(value - 2.0) < 0.0001;
        }

        private static bool TestBlockLibraryIndex()
        {
            string tempDir = System.IO.Path.Combine(System.IO.Path.GetTempPath(), "ffb_graph_test");
            System.IO.Directory.CreateDirectory(tempDir);
            var resolver = new GraphIncludeResolver(tempDir);

            var graph = BuildInlineGraph();
            var includeNode = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = graph,
                InputMap = { ["cmd_force"] = "cmd_force" },
                OutputMap = { ["out_force"] = "out_force" }
            };

            resolver.ResolveInclude(includeNode);
            string indexPath = System.IO.Path.Combine(tempDir, "graphs", "_embedded", "index.json");
            return System.IO.File.Exists(indexPath);
        }

        private static bool TestSchemaVersionMismatch()
        {
            var graph = BuildBaseGraph();
            graph.Version = 2;
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestUnknownFunctionValidation()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["bad_func"] = new GraphNode
            {
                Id = "bad_func",
                Type = NodeType.Func,
                Func = "unknown_func",
                Args = { "ias", "vref" }
            };
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad", Src = "bad_func" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestOutputMissingSrc()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestIncludeMissingPath()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = null
            };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestIncludeMappingWarnings()
        {
            var graph = new GraphDefinition();
            graph.Nodes["inc"] = new GraphNode
            {
                Id = "inc",
                Type = NodeType.Include,
                InlineGraph = BuildInlineGraph()
            };
            graph.Nodes["val"] = new GraphNode { Id = "val", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "val" };
            var validation = GraphValidator.Validate(graph);
            return validation.IsValid && validation.Warnings.Count > 0;
        }

        private static bool TestOpArgValidation()
        {
            var graph = BuildBaseGraph();
            graph.Nodes["bad_op"] = new GraphNode
            {
                Id = "bad_op",
                Type = NodeType.Op,
                Op = OpType.Mul,
                Args = { "ias" }
            };
            graph.Nodes["out_bad"] = new GraphNode { Id = "out_bad", Type = NodeType.Output, Name = "bad", Src = "bad_op" };
            var validation = GraphValidator.Validate(graph);
            return !validation.IsValid;
        }

        private static bool TestClampBoundOrderWarning()
        {
            var graph = new GraphDefinition();
            graph.Nodes["val"] = new GraphNode { Id = "val", Type = NodeType.Const, ConstValue = 0.5 };
            graph.Nodes["min"] = new GraphNode { Id = "min", Type = NodeType.Const, ConstValue = 2.0 };
            graph.Nodes["max"] = new GraphNode { Id = "max", Type = NodeType.Const, ConstValue = 1.0 };
            graph.Nodes["clamp"] = new GraphNode
            {
                Id = "clamp",
                Type = NodeType.Op,
                Op = OpType.Clamp,
                Args = { "val", "min", "max" }
            };
            graph.Nodes["out"] = new GraphNode { Id = "out", Type = NodeType.Output, Name = "out", Src = "clamp" };
            var validation = GraphValidator.Validate(graph);
            return validation.IsValid && validation.Warnings.Count > 0;
        }

        private static GraphDefinition BuildBaseGraph()
        {
            var graph = new GraphDefinition();
            graph.Nodes["ias"] = new GraphNode { Id = "ias", Type = NodeType.Input, Name = "ias_kts" };
            graph.Nodes["vref"] = new GraphNode { Id = "vref", Type = NodeType.Input, Name = "vref_kts" };
            graph.Nodes["qhat"] = new GraphNode
            {
                Id = "qhat",
                Type = NodeType.Func,
                Func = "qhat_eff",
                Args = { "ias", "vref" }
            };
            graph.Nodes["kq"] = new GraphNode { Id = "kq", Type = NodeType.Param, Name = "k_q" };
            graph.Nodes["kr"] = new GraphNode { Id = "kr", Type = NodeType.Param, Name = "k_rate" };
            graph.Nodes["kf"] = new GraphNode { Id = "kf", Type = NodeType.Param, Name = "k_friction" };
            graph.Nodes["spring"] = new GraphNode { Id = "spring", Type = NodeType.Op, Op = OpType.Mul, Args = { "kq", "qhat" } };
            graph.Nodes["damper"] = new GraphNode { Id = "damper", Type = NodeType.Op, Op = OpType.Mul, Args = { "kr", "qhat" } };
            graph.Nodes["friction"] = new GraphNode { Id = "friction", Type = NodeType.Op, Op = OpType.Mul, Args = { "kf", "qhat" } };
            graph.Nodes["out_spring"] = new GraphNode { Id = "out_spring", Type = NodeType.Output, Name = "spring", Src = "spring" };
            graph.Nodes["out_damper"] = new GraphNode { Id = "out_damper", Type = NodeType.Output, Name = "damper", Src = "damper" };
            graph.Nodes["out_friction"] = new GraphNode { Id = "out_friction", Type = NodeType.Output, Name = "friction", Src = "friction" };
            return graph;
        }

        private static GraphDefinition BuildInlineGraph()
        {
            var graph = new GraphDefinition();
            graph.Nodes["cmd"] = new GraphNode { Id = "cmd", Type = NodeType.Input, Name = "cmd_force" };
            graph.Nodes["out_force"] = new GraphNode { Id = "out_force", Type = NodeType.Output, Name = "out_force", Src = "cmd" };
            return graph;
        }

        private static void PrintResults(List<TestResult> results)
        {
            Console.WriteLine("GraphTest suite:");
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

        private sealed class TestResult
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
}
