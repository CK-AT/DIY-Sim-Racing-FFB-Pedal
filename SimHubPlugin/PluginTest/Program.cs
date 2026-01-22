using System;
using System.Collections.Generic;
using User.PluginSdkDemo.GraphEditor;

namespace User.PluginSdkDemo.PluginTest
{
    internal static class Program
    {
        private static int Main()
        {
            var results = new List<TestResult>
            {
                RunTest("GraphEditor JSON roundtrip", TestGraphEditorJsonRoundtrip),
                RunTest("Graph preview evaluation", TestGraphPreviewEvaluation)
            };

            PrintResults(results);
            return results.TrueForAll(r => r.Passed) ? 0 : 1;
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

        private static bool TestGraphEditorJsonRoundtrip()
        {
            var graph = new GraphDefinition();
            var input = new GraphNode
            {
                Id = "node_input",
                Title = "Input",
                Kind = GraphNodeKind.Input,
                X = 10,
                Y = 20
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            return validation.IsValid && loaded.Nodes.Count == 1 && loaded.Params.ContainsKey("k_q");
        }

        private static bool TestGraphPreviewEvaluation()
        {
            var graph = new GraphDefinition();
            var input = new GraphNode
            {
                Id = "in",
                Title = "ias_kts",
                Kind = GraphNodeKind.Input
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            var param = new GraphNode
            {
                Id = "param",
                Title = "k_q",
                Kind = GraphNodeKind.Param
            };
            param.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(param);

            var op = new GraphNode
            {
                Id = "mul",
                Title = "Mul",
                Kind = GraphNodeKind.Op,
                Op = "mul"
            };
            op.Ports.Add(new GraphPort { Name = "a", Kind = GraphPortKind.Input });
            op.Ports.Add(new GraphPort { Name = "b", Kind = GraphPortKind.Input });
            op.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(op);

            var output = new GraphNode
            {
                Id = "out",
                Title = "out_force",
                Kind = GraphNodeKind.Output
            };
            output.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
            graph.Nodes.Add(output);

            graph.Links.Add(new GraphLink { FromNodeId = "in", FromPort = "out", ToNodeId = "mul", ToPort = "a" });
            graph.Links.Add(new GraphLink { FromNodeId = "param", FromPort = "out", ToNodeId = "mul", ToPort = "b" });
            graph.Links.Add(new GraphLink { FromNodeId = "mul", FromPort = "out", ToNodeId = "out", ToPort = "in" });

            var evaluator = new GraphPreviewEvaluator();
            var result = evaluator.Evaluate(graph,
                new Dictionary<string, double> { ["ias_kts"] = 2.0 },
                new Dictionary<string, double> { ["k_q"] = 3.0 });

            return result.Outputs.Count > 0;
        }

        private static void PrintResults(List<TestResult> results)
        {
            Console.WriteLine("PluginTest suite:");
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
