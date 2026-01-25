using DiyFfb.GraphTest;
using System;
using System.Collections.Generic;
using System.Linq;

namespace User.PluginSdkDemo.GraphEditor
{
    public static class GraphRuntimeConverter
    {
        /// <summary>
        /// Creates a GraphIncludeResolver configured with editor-format conversion support.
        /// Use this factory to ensure consistent resolver configuration across preview and runtime.
        /// </summary>
        public static GraphIncludeResolver CreateResolver(string baseDirectory)
        {
            return new GraphIncludeResolver(baseDirectory)
            {
                EditorFormatConverter = ConvertEditorJson
            };
        }

        /// <summary>
        /// Converts editor-format JSON to runtime GraphDefinition.
        /// Returns null if the JSON is not editor format or conversion fails.
        /// </summary>
        private static DiyFfb.GraphTest.GraphDefinition ConvertEditorJson(string json)
        {
            // Detect editor-format JSON (has "links" or "kind" fields)
            if (string.IsNullOrEmpty(json))
            {
                return null;
            }

            if (!json.Contains("\"links\"") && !json.Contains("\"kind\"") && !json.Contains("\"Kind\""))
            {
                return null;
            }

            var editorGraph = GraphSerializer.Deserialize(json, out var validation);
            if (editorGraph != null && validation != null && validation.IsValid)
            {
                return Convert(editorGraph);
            }

            return null;
        }

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
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, node.Id, port.Name, out var source))
                        {
                            // Build full signal name from SignalGroup + SignalSuffix
                            string signalName = BuildFullSignalName(node.SignalGroup, port.SignalSuffix, port.Name);
                            var outputNode = new DiyFfb.GraphTest.GraphNode
                            {
                                Id = BuildPortId(node.Id, port.Name),
                                Name = signalName,
                                Type = NodeType.Output,
                                Src = source
                            };
                            runtime.Nodes[outputNode.Id] = outputNode;
                        }
                    }
                    continue;
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
                else if (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Param)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        // Build full signal name from SignalGroup + SignalSuffix
                        string signalName = BuildFullSignalName(node.SignalGroup, port.SignalSuffix, port.Name);
                        var inputNode = new DiyFfb.GraphTest.GraphNode
                        {
                            Id = BuildPortId(node.Id, port.Name),
                            Name = signalName,
                            Type = node.Kind == GraphNodeKind.Input ? NodeType.Input : NodeType.Param
                        };
                        runtime.Nodes[inputNode.Id] = inputNode;
                    }
                    continue;
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

            if (nodes.TryGetValue(link.FromNodeId, out var sourceNode) &&
                (sourceNode.Kind == GraphNodeKind.Input || sourceNode.Kind == GraphNodeKind.Param) &&
                !string.IsNullOrWhiteSpace(link.FromPort))
            {
                sourceId = BuildPortId(link.FromNodeId, link.FromPort);
                return true;
            }

            sourceId = link.FromNodeId;
            return true;
        }

        private static string BuildIncludeOutputId(string nodeId, string portName)
        {
            return $"{nodeId}:{portName}";
        }

        private static string BuildPortId(string nodeId, string portName)
        {
            return $"{nodeId}:{portName}";
        }

        /// <summary>
        /// Builds full signal name from group and suffix, with fallback to legacy port name.
        /// </summary>
        private static string BuildFullSignalName(string group, string suffix, string legacyName)
        {
            // If we have both group and suffix, build the full name
            if (!string.IsNullOrEmpty(group) && !string.IsNullOrEmpty(suffix))
            {
                return group + "." + suffix;
            }

            // Fall back to legacy port name for backward compatibility
            return legacyName ?? "";
        }
    }
}
