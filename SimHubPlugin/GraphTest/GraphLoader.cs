using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace DiyFfb.GraphTest
{
    public sealed class GraphLoader
    {
        public GraphDefinition LoadFromJson(string json, out GraphValidationResult validation)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                throw new ArgumentException("Graph JSON is empty.", nameof(json));
            }

            GraphDefinitionDto dto;
            try
            {
                dto = JsonConvert.DeserializeObject<GraphDefinitionDto>(json, new JsonSerializerSettings
                {
                    Converters = { new StringEnumConverter() }
                });
            }
            catch (JsonException ex)
            {
                throw new InvalidOperationException($"Graph JSON parse error: {ex.Message}", ex);
            }

            if (dto == null)
            {
                throw new InvalidOperationException("Failed to deserialize graph JSON.");
            }

            var graph = new GraphDefinition
            {
                Version = dto.Version <= 0 ? 1 : dto.Version
            };
            if (dto.Nodes != null)
            {
                foreach (var node in dto.Nodes)
                {
                    if (node == null || string.IsNullOrWhiteSpace(node.Id))
                    {
                        continue;
                    }

                    var graphNode = new GraphNode
                    {
                        Id = node.Id,
                        Type = node.Type,
                        Name = node.Name ?? string.Empty,
                        ConstValue = node.ConstValue,
                        Op = node.Op,
                        Func = node.Func ?? string.Empty,
                        Src = node.Src ?? string.Empty,
                        Path = node.Path ?? string.Empty,
                        Expr = node.Expr ?? string.Empty
                    };
                    if (node.Args != null)
                    {
                        graphNode.Args.AddRange(node.Args);
                    }
                    if (node.ArgNegate != null)
                    {
                        graphNode.ArgNegate.AddRange(node.ArgNegate);
                    }
                    if (node.Inline != null)
                    {
                        graphNode.InlineGraph = BuildDefinitionFromDto(node.Inline);
                    }
                    if (node.Inputs != null)
                    {
                        foreach (var kvp in node.Inputs)
                        {
                            graphNode.InputMap[kvp.Key] = kvp.Value;
                        }
                    }
                    if (node.Outputs != null)
                    {
                        foreach (var kvp in node.Outputs)
                        {
                            graphNode.OutputMap[kvp.Key] = kvp.Value;
                        }
                    }

                    graph.Nodes[graphNode.Id] = graphNode;
                }
            }

            validation = GraphValidator.Validate(graph);
            return graph;
        }

        internal GraphDefinition BuildDefinitionFromDto(GraphDefinitionDto dto)
        {
            var graph = new GraphDefinition
            {
                Version = dto.Version <= 0 ? 1 : dto.Version
            };

            if (dto.Nodes != null)
            {
                foreach (var node in dto.Nodes)
                {
                    if (node == null || string.IsNullOrWhiteSpace(node.Id))
                    {
                        continue;
                    }

                    var graphNode = new GraphNode
                    {
                        Id = node.Id,
                        Type = node.Type,
                        Name = node.Name ?? string.Empty,
                        ConstValue = node.ConstValue,
                        Op = node.Op,
                        Func = node.Func ?? string.Empty,
                        Src = node.Src ?? string.Empty,
                        Path = node.Path ?? string.Empty,
                        Expr = node.Expr ?? string.Empty
                    };
                    if (node.Args != null)
                    {
                        graphNode.Args.AddRange(node.Args);
                    }
                    if (node.ArgNegate != null)
                    {
                        graphNode.ArgNegate.AddRange(node.ArgNegate);
                    }
                    if (node.Inputs != null)
                    {
                        foreach (var kvp in node.Inputs)
                        {
                            graphNode.InputMap[kvp.Key] = kvp.Value;
                        }
                    }
                    if (node.Outputs != null)
                    {
                        foreach (var kvp in node.Outputs)
                        {
                            graphNode.OutputMap[kvp.Key] = kvp.Value;
                        }
                    }
                    if (node.Inline != null)
                    {
                        graphNode.InlineGraph = BuildDefinitionFromDto(node.Inline);
                    }

                    graph.Nodes[graphNode.Id] = graphNode;
                }
            }

            return graph;
        }
    }

    public sealed class GraphValidationResult
    {
        public List<string> Errors { get; } = new List<string>();
        public List<string> Warnings { get; } = new List<string>();
        public bool IsValid => Errors.Count == 0;
    }

    public static class GraphValidator
    {
        private static readonly HashSet<string> KnownFunctions = new HashSet<string>
        {
            "normalize",
            "qhat_eff",
            "torque_norm",
            "rpm_norm",
            "assist_loss",
            "buffet",
            "accumulator",
            "sample_hold",
            "edge_detect",
            "lag_asym"
        };

        private static readonly Dictionary<OpType, int> OpArgCounts = new Dictionary<OpType, int>
        {
            { OpType.Add, 2 },
            { OpType.Sub, 2 },
            { OpType.Mul, 2 },
            { OpType.Div, 2 },
            { OpType.Min, 2 },
            { OpType.Max, 2 },
            { OpType.Abs, 1 },
            { OpType.Neg, 1 },
            { OpType.Clamp, 3 },
            { OpType.Lerp, 3 },
            { OpType.Exp, 1 },
            { OpType.Sqrt, 1 },
            { OpType.Pow, 2 }
        };

        private static readonly HashSet<OpType> VariadicOps = new HashSet<OpType>
        {
            OpType.Add,
            OpType.Mul,
            OpType.Min,
            OpType.Max
        };

        public static GraphValidationResult Validate(GraphDefinition graph)
        {
            var result = new GraphValidationResult();
            if (graph == null)
            {
                result.Errors.Add("Graph is null.");
                return result;
            }

            if (graph.Version != 1)
            {
                result.Errors.Add($"Unsupported graph schema version: {graph.Version}.");
            }

            var includeOutputs = new HashSet<string>();
            var includeInputNames = new Dictionary<string, HashSet<string>>();
            var includeOutputNames = new Dictionary<string, HashSet<string>>();
            foreach (var node in graph.Nodes.Values)
            {
                if (node.Type != NodeType.Include)
                {
                    continue;
                }

                foreach (var mapping in node.OutputMap)
                {
                    if (!string.IsNullOrWhiteSpace(mapping.Value))
                    {
                        includeOutputs.Add(mapping.Value);
                    }
                }

                if (node.InlineGraph != null)
                {
                    includeInputNames[node.Id] = CollectNames(node.InlineGraph, NodeType.Input);
                    includeOutputNames[node.Id] = CollectNames(node.InlineGraph, NodeType.Output);
                }
            }

            foreach (var node in graph.Nodes.Values)
            {
                if ((node.Type == NodeType.Input || node.Type == NodeType.Param || node.Type == NodeType.Output) &&
                    string.IsNullOrWhiteSpace(node.Name))
                {
                    result.Errors.Add($"Node '{node.Id}' is missing a name.");
                }
                if (node.Type == NodeType.Output && string.IsNullOrWhiteSpace(node.Name))
                {
                    result.Errors.Add($"Output node '{node.Id}' is missing a name.");
                }
                if (node.Type == NodeType.Output && string.IsNullOrWhiteSpace(node.Src))
                {
                    result.Errors.Add($"Output node '{node.Id}' is missing a source.");
                }

                foreach (var arg in node.Args)
                {
                    if (!graph.Nodes.ContainsKey(arg))
                    {
                        result.Errors.Add($"Node '{node.Id}' references missing arg '{arg}'.");
                    }
                }

                if (!string.IsNullOrWhiteSpace(node.Src) &&
                    !graph.Nodes.ContainsKey(node.Src) &&
                    !includeOutputs.Contains(node.Src))
                {
                    result.Errors.Add($"Node '{node.Id}' references missing source '{node.Src}'.");
                }

                if (node.Type == NodeType.Func && string.IsNullOrWhiteSpace(node.Func))
                {
                    result.Errors.Add($"Func node '{node.Id}' is missing a function.");
                }
                if (node.Type == NodeType.Func && !string.IsNullOrWhiteSpace(node.Func) &&
                    !KnownFunctions.Contains(node.Func))
                {
                    result.Errors.Add($"Func node '{node.Id}' uses unknown function '{node.Func}'.");
                }
                if (node.Type == NodeType.Expr)
                {
                    foreach (var mapping in node.InputMap)
                    {
                        if (!graph.Nodes.ContainsKey(mapping.Value))
                        {
                            result.Errors.Add($"Expr node '{node.Id}' inport '{mapping.Key}' references missing node '{mapping.Value}'.");
                        }
                    }

                    var parsed = GraphExprSupport.TryParse(node.Expr, out string exprError);
                    if (parsed == null)
                    {
                        result.Errors.Add($"Expr node '{node.Id}' has an invalid formula: {exprError}.");
                    }
                    else
                    {
                        foreach (var name in GraphExprSupport.CollectIdentifiers(parsed))
                        {
                            if (!GraphExprSupport.IsConstant(name) && !node.InputMap.ContainsKey(name))
                            {
                                result.Errors.Add($"Expr node '{node.Id}' formula references '{name}', which is not a wired inport (available: [{string.Join(", ", node.InputMap.Keys)}]).");
                            }
                        }
                    }
                }
                if (node.Type == NodeType.Include && string.IsNullOrWhiteSpace(node.Path) && node.InlineGraph == null)
                {
                    result.Errors.Add($"Include node '{node.Id}' is missing a path.");
                }
                if (node.Type == NodeType.Include && node.InputMap.Count == 0)
                {
                    result.Warnings.Add($"Include node '{node.Id}' has no inputs mapped.");
                }
                if (node.Type == NodeType.Include && node.OutputMap.Count == 0)
                {
                    result.Warnings.Add($"Include node '{node.Id}' has no outputs mapped.");
                }
                if (node.Type == NodeType.Include)
                {
                    foreach (var mapping in node.InputMap)
                    {
                        if (!graph.Nodes.ContainsKey(mapping.Value))
                        {
                            result.Errors.Add($"Include node '{node.Id}' input '{mapping.Key}' references missing node '{mapping.Value}'.");
                        }
                    }

                    if (includeInputNames.TryGetValue(node.Id, out var subInputs))
                    {
                        foreach (var mapping in node.InputMap)
                        {
                            if (!subInputs.Contains(mapping.Key))
                            {
                                result.Errors.Add($"Include node '{node.Id}' input '{mapping.Key}' is not declared in the subgraph.");
                            }
                        }
                    }

                    if (includeOutputNames.TryGetValue(node.Id, out var subOutputs))
                    {
                        foreach (var mapping in node.OutputMap)
                        {
                            if (!subOutputs.Contains(mapping.Key))
                            {
                                result.Errors.Add($"Include node '{node.Id}' output '{mapping.Key}' is not declared in the subgraph.");
                            }
                        }
                    }
                }

                if (node.Type == NodeType.Op)
                {
                    if (!OpArgCounts.TryGetValue(node.Op, out var expected))
                    {
                        result.Errors.Add($"Op node '{node.Id}' uses unsupported op '{node.Op}'.");
                    }
                    else if (VariadicOps.Contains(node.Op))
                    {
                        if (node.Args.Count < expected)
                        {
                            result.Errors.Add($"Op node '{node.Id}' expects at least {expected} args but has {node.Args.Count}.");
                        }
                    }
                    else if (node.Args.Count != expected)
                    {
                        result.Errors.Add($"Op node '{node.Id}' expects {expected} args but has {node.Args.Count}.");
                    }
                    else if (node.Op == OpType.Clamp)
                    {
                        if (graph.Nodes.TryGetValue(node.Args[1], out var minNode) &&
                            graph.Nodes.TryGetValue(node.Args[2], out var maxNode) &&
                            minNode.Type == NodeType.Const && maxNode.Type == NodeType.Const &&
                            minNode.ConstValue > maxNode.ConstValue)
                        {
                            result.Warnings.Add($"Op node '{node.Id}' clamp bounds are inverted (min > max).");
                        }
                    }

                    if (node.ArgNegate.Count > 0)
                    {
                        if (node.ArgNegate.Count != node.Args.Count)
                        {
                            result.Errors.Add($"Op node '{node.Id}' negate list count ({node.ArgNegate.Count}) does not match args count ({node.Args.Count}).");
                        }
                        if (node.Op != OpType.Add && node.Op != OpType.Mul)
                        {
                            result.Errors.Add($"Op node '{node.Id}' uses negate flags with unsupported op '{node.Op}'.");
                        }
                    }
                }
            }

            if (result.Errors.Count == 0)
            {
                try
                {
                    _ = new GraphEvaluator(graph);
                }
                catch (Exception ex)
                {
                    result.Errors.Add($"Graph evaluator init failed: {ex.Message}");
                }
            }

            return result;
        }

        private static HashSet<string> CollectNames(GraphDefinition graph, NodeType type)
        {
            var names = new HashSet<string>();
            if (graph == null)
            {
                return names;
            }

            foreach (var node in graph.Nodes.Values)
            {
                if (node.Type == type && !string.IsNullOrWhiteSpace(node.Name))
                {
                    names.Add(node.Name);
                }
            }

            return names;
        }
    }

    internal sealed class GraphDefinitionDto
    {
        public int Version { get; set; } = 1;
        public List<GraphNodeDto> Nodes { get; set; } = new List<GraphNodeDto>();
    }

    internal sealed class GraphNodeDto
    {
        public string Id { get; set; } = string.Empty;
        public NodeType Type { get; set; }
        public string Name { get; set; } = string.Empty;
        public double ConstValue { get; set; }
        public OpType Op { get; set; }
        public string Func { get; set; } = string.Empty;
        public List<string> Args { get; set; } = new List<string>();
        public List<bool> ArgNegate { get; set; } = new List<bool>();
        public string Src { get; set; } = string.Empty;
        public string Path { get; set; } = string.Empty;
        public string Expr { get; set; } = string.Empty;
        public Dictionary<string, string> Inputs { get; set; } = new Dictionary<string, string>();
        public Dictionary<string, string> Outputs { get; set; } = new Dictionary<string, string>();
        public GraphDefinitionDto Inline { get; set; }
    }
}
