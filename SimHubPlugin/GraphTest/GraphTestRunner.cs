using System;
using System.Collections.Generic;
using System.Linq;
using User.PluginSdkDemo;
using User.PluginSdkDemo.GraphEditor;
using GraphEditor = User.PluginSdkDemo.GraphEditor;
using SimHubPlugin.TestCommon;

namespace DiyFfb.GraphTest
{
    public static class GraphTestRunner
    {
        public static void Run()
        {
            var results = new List<TestResult>();

            // Runtime tests (evaluator, validation, includes)
            results.Add(TestRunner.RunTest("Evaluator basic outputs", TestEvaluatorBasicOutputs));
            results.Add(TestRunner.RunTest("Compiled evaluator matches outputs", TestCompiledEvaluatorMatches));
            results.Add(TestRunner.RunTest("Evaluator trace values", TestEvaluatorTraceValues));
            results.Add(TestRunner.RunTest("JSON load/save roundtrip", TestJsonRoundtrip));
            results.Add(TestRunner.RunTest("Validation catches missing output", TestIncludeOutputValidation));
            results.Add(TestRunner.RunTest("Inline include mapping", TestInlineIncludeMapping));
            results.Add(TestRunner.RunTest("Block library index", TestBlockLibraryIndex));
            results.Add(TestRunner.RunTest("Schema version mismatch", TestSchemaVersionMismatch));
            results.Add(TestRunner.RunTest("Unknown function validation", TestUnknownFunctionValidation));
            results.Add(TestRunner.RunTest("Output missing src validation", TestOutputMissingSrc));
            results.Add(TestRunner.RunTest("Include missing path validation", TestIncludeMissingPath));
            results.Add(TestRunner.RunTest("Include mapping warnings", TestIncludeMappingWarnings));
            results.Add(TestRunner.RunTest("Op arg count validation", TestOpArgValidation));
            results.Add(TestRunner.RunTest("Clamp bound order warning", TestClampBoundOrderWarning));
            results.Add(TestRunner.RunTest("Graph output names unique", TestGraphOutputNamesUnique));

            // Editor tests (JSON serialization, param schema, parameter resolution)
            results.Add(TestRunner.RunTest("GraphEditor JSON roundtrip", TestGraphEditorJsonRoundtrip));
            results.Add(TestRunner.RunTest("Graph param UI schema roundtrip", TestGraphParamUiRoundtrip));
            results.Add(TestRunner.RunTest("Graph preview evaluation", TestGraphPreviewEvaluation));
            results.Add(TestRunner.RunTest("ParamValues serialization roundtrip", TestParamValuesSerializationRoundtrip));
            results.Add(TestRunner.RunTest("Param resolution: include default", TestParamResolutionIncludeDefault));
            results.Add(TestRunner.RunTest("Param resolution: graph override", TestParamResolutionGraphOverride));
            results.Add(TestRunner.RunTest("Param resolution: vehicle override", TestParamResolutionVehicleOverride));
            results.Add(TestRunner.RunTest("Param resolution: three-tier cascade", TestParamResolutionThreeTierCascade));

            // v3 Include node tests
            results.Add(TestRunner.RunTest("ExtractInterface from graph", TestExtractInterfaceFromGraph));
            results.Add(TestRunner.RunTest("v3 Include ports not serialized", TestV3IncludePortsNotSerialized));
            results.Add(TestRunner.RunTest("v2 Include ports migration", TestV2IncludePortsMigration));

            // v4 Library graph tests
            results.Add(TestRunner.RunTest("Library graph interface extraction", TestLibraryGraphInterfaceExtraction));
            results.Add(TestRunner.RunTest("Library graph serialization", TestLibraryGraphSerialization));
            results.Add(TestRunner.RunTest("Library graph Title serialization", TestLibraryGraphTitleSerialization));
            results.Add(TestRunner.RunTest("Library graph runtime conversion", TestLibraryGraphRuntimeConversion));

            // Node duplication tests
            results.Add(TestRunner.RunTest("Node serialization preserves SignalGroup", TestNodeSignalGroupPreservation));
            results.Add(TestRunner.RunTest("Port serialization preserves SignalSuffix", TestPortSignalSuffixPreservation));

            TestRunner.PrintResults("FFB Graph Tests", results);
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

        private static bool TestCompiledEvaluatorMatches()
        {
            var graph = BuildBaseGraph();
            var inputs = new Dictionary<string, double> { ["ias_kts"] = 85.0, ["vref_kts"] = 70.0 };
            var parameters = new Dictionary<string, double> { ["k_q"] = 1.3, ["k_rate"] = 0.6, ["k_friction"] = 0.2 };
            var legacy = new GraphEvaluator(graph).Evaluate(inputs, parameters);
            var compiled = new GraphCompiledEvaluator(graph).Evaluate(inputs, parameters);

            return Math.Abs(legacy["spring"] - compiled["spring"]) < 1e-6 &&
                   Math.Abs(legacy["damper"] - compiled["damper"]) < 1e-6 &&
                   Math.Abs(legacy["friction"] - compiled["friction"]) < 1e-6;
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

        private static bool TestGraphOutputNamesUnique()
        {
            var outputs = new HashSet<string>(StringComparer.Ordinal);
            foreach (var name in GraphSignalCatalogData.OutputNames)
            {
                if (!outputs.Add(name))
                {
                    return false;
                }
            }

            return outputs.Contains("FlightStickPitch.SpringGain");
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

        // Editor tests (from PluginTest)

        private static bool TestGraphEditorJsonRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            var input = new GraphEditor.GraphNode
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
            var graph = new GraphEditor.GraphDefinition();
            var input = new GraphEditor.GraphNode
            {
                Id = "in",
                Title = "ias_kts",
                Kind = GraphNodeKind.Input
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            var param = new GraphEditor.GraphNode
            {
                Id = "param",
                Title = "k_q",
                Kind = GraphNodeKind.Param
            };
            param.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(param);

            var op = new GraphEditor.GraphNode
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

            var output = new GraphEditor.GraphNode
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

        private static bool TestGraphParamUiRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0,
                Ui = new GraphParamUi
                {
                    Widget = "slider",
                    Label = "Spring Gain",
                    Group = "FlightStickPitch",
                    Units = "N",
                    Step = 0.1,
                    Precision = 2,
                    LogScale = true
                }
            };
            graph.Params["k_q"].Ui.Options.Add(new GraphParamOption { Value = "A", Label = "Mode A" });

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);
            if (!validation.IsValid || !loaded.Params.TryGetValue("k_q", out var param))
            {
                return false;
            }

            return param.Ui != null &&
                   param.Ui.Widget == "slider" &&
                   param.Ui.Label == "Spring Gain" &&
                   param.Ui.Group == "FlightStickPitch" &&
                   param.Ui.Units == "N" &&
                   param.Ui.Step.HasValue &&
                   Math.Abs(param.Ui.Step.Value - 0.1) < 1e-9 &&
                   param.Ui.Precision == 2 &&
                   param.Ui.LogScale &&
                   param.Ui.Options.Count == 1 &&
                   param.Ui.Options[0].Label == "Mode A";
        }

        private static bool TestParamValuesSerializationRoundtrip()
        {
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_q"] = new GraphParam
            {
                Name = "k_q",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.Params["k_rate"] = new GraphParam
            {
                Name = "k_rate",
                DefaultValue = 0.5,
                Min = 0.0,
                Max = 2.0
            };

            // Set graph-level overrides
            graph.ParamValues["k_q"] = 1.5;
            graph.ParamValues["k_rate"] = 0.8;

            string json = GraphSerializer.Serialize(graph);
            var loaded = GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid || loaded.ParamValues == null)
            {
                return false;
            }

            return loaded.ParamValues.ContainsKey("k_q") &&
                   Math.Abs(loaded.ParamValues["k_q"] - 1.5) < 1e-9 &&
                   loaded.ParamValues.ContainsKey("k_rate") &&
                   Math.Abs(loaded.ParamValues["k_rate"] - 0.8) < 1e-9;
        }

        private static bool TestParamResolutionIncludeDefault()
        {
            // Test that include/graph default value is used when no overrides exist
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };

            // No graph or vehicle overrides
            double resolved = ResolveParamValue("k_spring", graph, null);
            return Math.Abs(resolved - 1.0) < 1e-9;
        }

        private static bool TestParamResolutionGraphOverride()
        {
            // Test that graph-level override takes precedence over default
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.ParamValues["k_spring"] = 2.0;

            // No vehicle override
            double resolved = ResolveParamValue("k_spring", graph, null);
            return Math.Abs(resolved - 2.0) < 1e-9;
        }

        private static bool TestParamResolutionVehicleOverride()
        {
            // Test that vehicle-level override takes precedence over graph override
            var graph = new GraphEditor.GraphDefinition();
            graph.Params["k_spring"] = new GraphParam
            {
                Name = "k_spring",
                DefaultValue = 1.0,
                Min = 0.0,
                Max = 5.0
            };
            graph.ParamValues["k_spring"] = 2.0;

            var vehicleOverrides = new Dictionary<string, double>
            {
                ["k_spring"] = 3.0
            };

            double resolved = ResolveParamValue("k_spring", graph, vehicleOverrides);
            return Math.Abs(resolved - 3.0) < 1e-9;
        }

        private static bool TestParamResolutionThreeTierCascade()
        {
            // Test the full three-tier cascade with multiple params at different levels
            var graph = new GraphEditor.GraphDefinition();

            // Param with default only
            graph.Params["p1"] = new GraphParam { Name = "p1", DefaultValue = 1.0, Min = 0.0, Max = 10.0 };

            // Param with default + graph override
            graph.Params["p2"] = new GraphParam { Name = "p2", DefaultValue = 2.0, Min = 0.0, Max = 10.0 };
            graph.ParamValues["p2"] = 2.5;

            // Param with default + graph override + vehicle override
            graph.Params["p3"] = new GraphParam { Name = "p3", DefaultValue = 3.0, Min = 0.0, Max = 10.0 };
            graph.ParamValues["p3"] = 3.5;

            var vehicleOverrides = new Dictionary<string, double>
            {
                ["p3"] = 3.9
            };

            double v1 = ResolveParamValue("p1", graph, vehicleOverrides);
            double v2 = ResolveParamValue("p2", graph, vehicleOverrides);
            double v3 = ResolveParamValue("p3", graph, vehicleOverrides);

            return Math.Abs(v1 - 1.0) < 1e-9 &&
                   Math.Abs(v2 - 2.5) < 1e-9 &&
                   Math.Abs(v3 - 3.9) < 1e-9;
        }

        /// <summary>
        /// Helper method that implements the three-tier parameter resolution logic.
        /// This mirrors the logic in DiyFfbPlugin.BuildGraphParams().
        /// </summary>
        private static double ResolveParamValue(string paramName, GraphEditor.GraphDefinition graph, Dictionary<string, double> vehicleOverrides)
        {
            if (!graph.Params.TryGetValue(paramName, out var param))
            {
                return 0.0;
            }

            // Tier 1: Include/graph default
            double value = param.DefaultValue;

            // Tier 2: Graph paramValues override
            if (graph.ParamValues != null && graph.ParamValues.TryGetValue(paramName, out var graphOverride))
            {
                value = graphOverride;
            }

            // Tier 3: Vehicle profile override
            if (vehicleOverrides != null && vehicleOverrides.TryGetValue(paramName, out var vehicleOverride))
            {
                value = vehicleOverride;
            }

            return value;
        }

        // v3 Include node tests

        private static bool TestExtractInterfaceFromGraph()
        {
            // Create a graph with Input/Output nodes to extract interface from
            var graph = new GraphEditor.GraphDefinition();

            // Add Input node with two output ports
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            inputNode.Ports.Add(new GraphPort { Name = "ias", Kind = GraphPortKind.Output, SignalSuffix = "Speed.IAS" });
            inputNode.Ports.Add(new GraphPort { Name = "alpha", Kind = GraphPortKind.Output, SignalSuffix = "Angle.Alpha" });
            graph.Nodes.Add(inputNode);

            // Add Output node with one input port
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphNodeKind.Output,
                SignalGroup = "FlightStickPitch"
            };
            outputNode.Ports.Add(new GraphPort { Name = "spring", Kind = GraphPortKind.Input, SignalSuffix = "SpringGain" });
            graph.Nodes.Add(outputNode);

            var iface = GraphSerializer.ExtractInterface(graph);

            return iface.IsValid &&
                   iface.Inputs.Count == 2 &&
                   iface.Inputs.Contains("Speed.IAS") &&
                   iface.Inputs.Contains("Angle.Alpha") &&
                   iface.Outputs.Count == 1 &&
                   iface.Outputs.Contains("SpringGain");
        }

        private static bool TestV3IncludePortsNotSerialized()
        {
            // Create graph with Include node that has ports
            var graph = new GraphEditor.GraphDefinition();
            var includeNode = new GraphEditor.GraphNode
            {
                Id = "inc",
                Kind = GraphNodeKind.Include,
                IncludePath = "subgraph.json"
            };
            includeNode.Ports.Add(new GraphPort { Name = "in1", Kind = GraphPortKind.Input });
            includeNode.Ports.Add(new GraphPort { Name = "out1", Kind = GraphPortKind.Output });
            graph.Nodes.Add(includeNode);

            // Serialize
            string json = GraphSerializer.Serialize(graph);

            // Check that Include node doesn't have Ports in JSON
            // (ports should be omitted for Include nodes in v3)
            return !json.Contains("\"Ports\"");
        }

        private static bool TestV2IncludePortsMigration()
        {
            // Simulate v2 JSON with Include node that has stored ports
            string v2Json = @"{
                ""Version"": 2,
                ""Nodes"": [{
                    ""Id"": ""inc"",
                    ""Kind"": ""Include"",
                    ""IncludePath"": ""test.json"",
                    ""X"": 100,
                    ""Y"": 200,
                    ""Ports"": [
                        {""Name"": ""in1"", ""Kind"": ""Input""},
                        {""Name"": ""out1"", ""Kind"": ""Output""}
                    ]
                }],
                ""Links"": [],
                ""Params"": []
            }";

            var graph = GraphSerializer.Deserialize(v2Json, out var validation);

            // v2 Include nodes should still load their ports from JSON
            var includeNode = graph.Nodes[0];

            return validation.IsValid &&
                   includeNode.Kind == GraphNodeKind.Include &&
                   includeNode.Ports.Count == 2 &&
                   includeNode.Ports[0].Name == "in1" &&
                   includeNode.Ports[1].Name == "out1";
        }

        // v4 Library graph tests

        private static bool TestLibraryGraphInterfaceExtraction()
        {
            // Create a library graph with freeform port names
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node with freeform port names (no SignalGroup/SignalSuffix)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                Title = "Inputs"
            };
            inputNode.Ports.Add(new GraphPort { Name = "speed", Kind = GraphPortKind.Output });
            inputNode.Ports.Add(new GraphPort { Name = "force", Kind = GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Add Output node with freeform port names
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphNodeKind.Output,
                Title = "Outputs"
            };
            outputNode.Ports.Add(new GraphPort { Name = "result", Kind = GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            var iface = GraphSerializer.ExtractInterface(graph);

            // For library graphs, interface should use port Names directly
            return iface.IsValid &&
                   iface.Inputs.Count == 2 &&
                   iface.Inputs.Contains("speed") &&
                   iface.Inputs.Contains("force") &&
                   iface.Outputs.Count == 1 &&
                   iface.Outputs.Contains("result");
        }

        private static bool TestLibraryGraphSerialization()
        {
            // Create library graph
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node (library graph: no SignalGroup, uses Name)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphNodeKind.Input,
                Title = "In"
            };
            inputNode.Ports.Add(new GraphPort { Name = "x", Kind = GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Serialize and check IsLibraryGraph is persisted
            string json = GraphSerializer.Serialize(graph);
            if (!json.Contains("\"IsLibraryGraph\": true"))
            {
                return false;
            }

            // Deserialize and verify
            var loaded = GraphSerializer.Deserialize(json, out var validation);

            return validation.IsValid &&
                   loaded.IsLibraryGraph &&
                   loaded.Nodes[0].Ports.Count == 1 &&
                   loaded.Nodes[0].Ports[0].Name == "x";
        }

        private static bool TestLibraryGraphTitleSerialization()
        {
            // BUG TEST: Library graph Input/Output nodes should preserve their Title
            // through serialization, but ShouldSerializeTitle() incorrectly excludes them.
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Add Input node with a custom Title
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                Title = "My Custom Input"  // This should persist
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort { Name = "x", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Add Output node with a custom Title
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                Title = "My Custom Output"  // This should persist
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort { Name = "result", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            // Find the loaded nodes
            var loadedInput = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            var loadedOutput = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Output);

            // Titles should be preserved for library graph Input/Output nodes
            return loadedInput != null &&
                   loadedInput.Title == "My Custom Input" &&
                   loadedOutput != null &&
                   loadedOutput.Title == "My Custom Output";
        }

        private static bool TestLibraryGraphRuntimeConversion()
        {
            // Test that library graph nodes convert correctly to runtime with freeform names
            var graph = new GraphEditor.GraphDefinition
            {
                IsLibraryGraph = true
            };

            // Input node with freeform port name (no SignalGroup)
            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                Title = "Inputs"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort { Name = "force", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(inputNode);

            // Const node
            var constNode = new GraphEditor.GraphNode
            {
                Id = "k",
                Kind = GraphEditor.GraphNodeKind.Const,
                ConstValue = 2.0
            };
            graph.Nodes.Add(constNode);

            // Op node (multiply)
            var opNode = new GraphEditor.GraphNode
            {
                Id = "mul",
                Kind = GraphEditor.GraphNodeKind.Op,
                Op = "mul"
            };
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "a", Kind = GraphEditor.GraphPortKind.Input });
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "b", Kind = GraphEditor.GraphPortKind.Input });
            opNode.Ports.Add(new GraphEditor.GraphPort { Name = "out", Kind = GraphEditor.GraphPortKind.Output });
            graph.Nodes.Add(opNode);

            // Output node with freeform port name
            var outputNode = new GraphEditor.GraphNode
            {
                Id = "out",
                Kind = GraphEditor.GraphNodeKind.Output,
                Title = "Outputs"
            };
            outputNode.Ports.Add(new GraphEditor.GraphPort { Name = "scaled", Kind = GraphEditor.GraphPortKind.Input });
            graph.Nodes.Add(outputNode);

            // Links
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "in", FromPort = "force", ToNodeId = "mul", ToPort = "a" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "k", ToNodeId = "mul", ToPort = "b" });
            graph.Links.Add(new GraphEditor.GraphLink { FromNodeId = "mul", FromPort = "out", ToNodeId = "out", ToPort = "scaled" });

            // Convert to runtime
            var runtime = GraphEditor.GraphRuntimeConverter.Convert(graph);

            // The input should use freeform name "force" (not a signal-catalog name)
            // Use string comparison to avoid enum type conflicts between local and imported types
            bool hasForceInput = runtime.Nodes.Values.Any(n =>
                n.Type.ToString() == "Input" && n.Name == "force");

            // The output should use freeform name "scaled"
            bool hasScaledOutput = runtime.Nodes.Values.Any(n =>
                n.Type.ToString() == "Output" && n.Name == "scaled");

            return hasForceInput && hasScaledOutput;
        }

        private static bool TestNodeSignalGroupPreservation()
        {
            // Test that SignalGroup is preserved through serialization roundtrip
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"  // This must persist
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "IAS_kts",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS_kts"
            });
            graph.Nodes.Add(inputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            var loadedNode = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            return loadedNode != null && loadedNode.SignalGroup == "XPlane";
        }

        private static bool TestPortSignalSuffixPreservation()
        {
            // Test that port SignalSuffix is preserved through serialization roundtrip
            var graph = new GraphEditor.GraphDefinition();

            var inputNode = new GraphEditor.GraphNode
            {
                Id = "in",
                Kind = GraphEditor.GraphNodeKind.Input,
                SignalGroup = "XPlane"
            };
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "IAS_kts",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "IAS_kts"  // This must persist
            });
            inputNode.Ports.Add(new GraphEditor.GraphPort
            {
                Name = "Alpha_deg",
                Kind = GraphEditor.GraphPortKind.Output,
                SignalSuffix = "Alpha_deg"
            });
            graph.Nodes.Add(inputNode);

            // Serialize and deserialize
            string json = GraphEditor.GraphSerializer.Serialize(graph);
            var loaded = GraphEditor.GraphSerializer.Deserialize(json, out var validation);

            if (!validation.IsValid)
            {
                return false;
            }

            var loadedNode = loaded.Nodes.FirstOrDefault(n => n.Kind == GraphEditor.GraphNodeKind.Input);
            if (loadedNode == null || loadedNode.Ports.Count != 2)
            {
                return false;
            }

            var port1 = loadedNode.Ports.FirstOrDefault(p => p.Name == "IAS_kts");
            var port2 = loadedNode.Ports.FirstOrDefault(p => p.Name == "Alpha_deg");

            return port1 != null && port1.SignalSuffix == "IAS_kts" &&
                   port2 != null && port2.SignalSuffix == "Alpha_deg";
        }

    }
}
