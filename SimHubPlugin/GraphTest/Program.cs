using System;
using System.Collections.Generic;

namespace DiyFfb.GraphTest
{
    public static class Program
    {
        public static void Main()
        {
            GraphTestRunner.Run();

            var graph = BuildSampleGraph();
            var resolver = new GraphIncludeResolver(AppContext.BaseDirectory);
            var evaluator = new GraphEvaluator(graph, resolver);

            var inputs = new Dictionary<string, double>
            {
                ["ias_kts"] = 120.0,
                ["vref_kts"] = 60.0,
                ["load_force"] = 15.0,
                ["hyd_pressure"] = 5.0
            };

            var parameters = new Dictionary<string, double>
            {
                ["k_q"] = 0.8,
                ["k_rate"] = 0.4,
                ["k_friction"] = 0.1,
                ["actuator_gain"] = 1.5,
                ["actuator_max"] = 20.0
            };

            var outputs = evaluator.Evaluate(inputs, parameters);
            Console.WriteLine("FFB graph test output:");
            foreach (var kvp in outputs)
            {
                Console.WriteLine($"  {kvp.Key} = {kvp.Value:F4}");
            }

            Console.WriteLine();
            Console.WriteLine("JSON load test:");
            var loader = new GraphLoader();
            var graphJson = BuildSampleGraphJson();
            var loaded = loader.LoadFromJson(graphJson, out var validation);
            Console.WriteLine($"  valid = {validation.IsValid}");
            foreach (var error in validation.Errors)
            {
                Console.WriteLine($"  error: {error}");
            }
            if (validation.IsValid)
            {
                var outputsJson = new GraphEvaluator(loaded, resolver).Evaluate(inputs, parameters);
                foreach (var kvp in outputsJson)
                {
                    Console.WriteLine($"  {kvp.Key} = {kvp.Value:F4}");
                }
            }

            Console.WriteLine();
            Console.WriteLine("JSON export test:");
            var saver = new GraphSaver();
            Console.WriteLine(saver.SaveToJson(graph));
        }

        private static GraphDefinition BuildSampleGraph()
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

            graph.Nodes["load_force"] = new GraphNode { Id = "load_force", Type = NodeType.Input, Name = "load_force" };
            graph.Nodes["hyd_pressure"] = new GraphNode { Id = "hyd_pressure", Type = NodeType.Input, Name = "hyd_pressure" };
            graph.Nodes["actuator"] = new GraphNode
            {
                Id = "actuator",
                Type = NodeType.Include,
                Path = "graphs/actuator.json",
                InputMap = { ["cmd_force"] = "load_force", ["pressure"] = "hyd_pressure" },
                OutputMap = { ["out_force"] = "actuator_force" }
            };
            graph.Nodes["out_load"] = new GraphNode { Id = "out_load", Type = NodeType.Output, Name = "load", Src = "actuator_force" };

            graph.Nodes["out_spring"] = new GraphNode { Id = "out_spring", Type = NodeType.Output, Name = "spring", Src = "spring" };
            graph.Nodes["out_damper"] = new GraphNode { Id = "out_damper", Type = NodeType.Output, Name = "damper", Src = "damper" };
            graph.Nodes["out_friction"] = new GraphNode { Id = "out_friction", Type = NodeType.Output, Name = "friction", Src = "friction" };
            return graph;
        }

        private static string BuildSampleGraphJson()
        {
            return @"{
  ""version"": 1,
  ""nodes"": [
    { ""id"": ""ias"", ""type"": ""Input"", ""name"": ""ias_kts"" },
    { ""id"": ""vref"", ""type"": ""Input"", ""name"": ""vref_kts"" },
    { ""id"": ""qhat"", ""type"": ""Func"", ""func"": ""qhat_eff"", ""args"": [""ias"", ""vref""] },
    { ""id"": ""kq"", ""type"": ""Param"", ""name"": ""k_q"" },
    { ""id"": ""kr"", ""type"": ""Param"", ""name"": ""k_rate"" },
    { ""id"": ""kf"", ""type"": ""Param"", ""name"": ""k_friction"" },
    { ""id"": ""spring"", ""type"": ""Op"", ""op"": ""Mul"", ""args"": [""kq"", ""qhat""] },
    { ""id"": ""damper"", ""type"": ""Op"", ""op"": ""Mul"", ""args"": [""kr"", ""qhat""] },
    { ""id"": ""friction"", ""type"": ""Op"", ""op"": ""Mul"", ""args"": [""kf"", ""qhat""] },
    { ""id"": ""load_force"", ""type"": ""Input"", ""name"": ""load_force"" },
    { ""id"": ""hyd_pressure"", ""type"": ""Input"", ""name"": ""hyd_pressure"" },
    {
      ""id"": ""actuator"",
      ""type"": ""Include"",
      ""path"": ""graphs/actuator.json"",
      ""inline"": {
        ""version"": 1,
        ""nodes"": [
          { ""id"": ""cmd_force"", ""type"": ""Input"", ""name"": ""cmd_force"" },
          { ""id"": ""pressure"", ""type"": ""Input"", ""name"": ""pressure"" },
          { ""id"": ""k_force"", ""type"": ""Param"", ""name"": ""actuator_gain"" },
          { ""id"": ""scaled"", ""type"": ""Op"", ""op"": ""Mul"", ""args"": [""cmd_force"", ""k_force""] },
          { ""id"": ""clamp_max"", ""type"": ""Param"", ""name"": ""actuator_max"" },
          { ""id"": ""out_force"", ""type"": ""Op"", ""op"": ""Clamp"", ""args"": [""scaled"", ""pressure"", ""clamp_max""] },
          { ""id"": ""out_force_out"", ""type"": ""Output"", ""name"": ""out_force"", ""src"": ""out_force"" }
        ]
      },
      ""inputs"": { ""cmd_force"": ""load_force"", ""pressure"": ""hyd_pressure"" },
      ""outputs"": { ""out_force"": ""actuator_force"" }
    },
    { ""id"": ""out_load"", ""type"": ""Output"", ""name"": ""load"", ""src"": ""actuator_force"" },
    { ""id"": ""out_spring"", ""type"": ""Output"", ""name"": ""spring"", ""src"": ""spring"" },
    { ""id"": ""out_damper"", ""type"": ""Output"", ""name"": ""damper"", ""src"": ""damper"" },
    { ""id"": ""out_friction"", ""type"": ""Output"", ""name"": ""friction"", ""src"": ""friction"" }
  ]
}";
        }
    }
}
