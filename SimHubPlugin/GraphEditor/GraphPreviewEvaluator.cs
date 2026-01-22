using DiyFfb.GraphTest;
using System;
using System.Collections.Generic;
using System.Linq;

namespace User.PluginSdkDemo.GraphEditor
{
    public sealed class GraphPreviewEvaluator
    {
        public GraphEvaluationResult Evaluate(GraphDefinition graph,
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            if (graph == null)
            {
                return new GraphEvaluationResult();
            }

            var runtime = GraphRuntimeConverter.Convert(graph);
            var evaluator = new GraphEvaluator(runtime);
            return evaluator.EvaluateWithTrace(inputs, parameters);
        }
    }

    internal static class GraphRuntimeConverter
    {
        public static DiyFfb.GraphTest.GraphDefinition Convert(GraphDefinition graph)
        {
            var runtime = new DiyFfb.GraphTest.GraphDefinition { Version = graph.Version };
            var nodes = graph.Nodes.ToDictionary(n => n.Id, n => n);

            foreach (var node in graph.Nodes)
            {
                var runtimeNode = new DiyFfb.GraphTest.GraphNode
                {
                    Id = node.Id,
                    Name = string.IsNullOrWhiteSpace(node.Title) ? node.Id : node.Title,
                    Type = MapNodeType(node.Kind),
                    ConstValue = node.ConstValue,
                    Func = node.Func ?? "",
                    Path = node.IncludePath ?? ""
                };

                if (node.Kind == GraphNodeKind.Op)
                {
                    runtimeNode.Op = MapOp(node.Op);
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, node.Id, port.Name, out var source))
                        {
                            runtimeNode.Args.Add(source);
                        }
                    }
                }
                else if (node.Kind == GraphNodeKind.Func)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, node.Id, port.Name, out var source))
                        {
                            runtimeNode.Args.Add(source);
                        }
                    }
                }
                else if (node.Kind == GraphNodeKind.Output)
                {
                    if (TryGetInputSource(nodes, graph.Links, node.Id, "in", out var source))
                    {
                        runtimeNode.Src = source;
                    }
                }
                else if (node.Kind == GraphNodeKind.Include)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, node.Id, port.Name, out var source))
                        {
                            runtimeNode.InputMap[port.Name] = source;
                        }
                    }

                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        if (!runtimeNode.OutputMap.ContainsKey(port.Name))
                        {
                            runtimeNode.OutputMap[port.Name] = BuildIncludeOutputId(node.Id, port.Name);
                        }
                    }
                }

                runtime.Nodes[runtimeNode.Id] = runtimeNode;
            }

            return runtime;
        }

        private static NodeType MapNodeType(GraphNodeKind kind)
        {
            switch (kind)
            {
                case GraphNodeKind.Input: return NodeType.Input;
                case GraphNodeKind.Param: return NodeType.Param;
                case GraphNodeKind.Const: return NodeType.Const;
                case GraphNodeKind.Op: return NodeType.Op;
                case GraphNodeKind.Func: return NodeType.Func;
                case GraphNodeKind.Include: return NodeType.Include;
                case GraphNodeKind.Output: return NodeType.Output;
                default: return NodeType.Const;
            }
        }

        private static OpType MapOp(string op)
        {
            switch ((op ?? "").Trim().ToLowerInvariant())
            {
                case "add":
                case "+": return OpType.Add;
                case "sub":
                case "-": return OpType.Sub;
                case "mul":
                case "*": return OpType.Mul;
                case "div":
                case "/": return OpType.Div;
                case "min": return OpType.Min;
                case "max": return OpType.Max;
                case "abs": return OpType.Abs;
                case "clamp": return OpType.Clamp;
                case "lerp": return OpType.Lerp;
                default: return OpType.Add;
            }
        }

        private static bool TryGetInputSource(Dictionary<string, GraphNode> nodes, List<GraphLink> links,
            string nodeId, string portName, out string sourceId)
        {
            sourceId = null;
            var link = links.FirstOrDefault(l => l.ToNodeId == nodeId && l.ToPort == portName);
            if (link == null)
            {
                return false;
            }

            if (!nodes.ContainsKey(link.FromNodeId))
            {
                return false;
            }

            if (nodes.TryGetValue(link.FromNodeId, out var fromNode) &&
                fromNode.Kind == GraphNodeKind.Include &&
                link.FromPort != null)
            {
                sourceId = BuildIncludeOutputId(link.FromNodeId, link.FromPort);
                return true;
            }

            sourceId = link.FromNodeId;
            return true;
        }

        private static string BuildIncludeOutputId(string nodeId, string portName)
        {
            return $"{nodeId}:{portName}";
        }
    }
}
