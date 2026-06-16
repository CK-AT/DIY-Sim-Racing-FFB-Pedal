using DiyFfb.GraphTest;
using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.GraphEditor
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
        /// <param name="json">The JSON content to convert.</param>
        /// <param name="resolvedFilePath">The resolved file path, used for relative include resolution.</param>
        private static DiyFfb.GraphTest.GraphDefinition ConvertEditorJson(string json, string resolvedFilePath)
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
                // Populate nested Include ports using the sub-graph's directory.
                // This is critical for nested includes to resolve relative paths correctly.
                if (!string.IsNullOrEmpty(resolvedFilePath))
                {
                    string fileDir = System.IO.Path.GetDirectoryName(resolvedFilePath);
                    if (!string.IsNullOrEmpty(fileDir))
                    {
                        GraphSerializer.PopulateIncludePorts(editorGraph, fileDir);
                    }
                }
                return Convert(editorGraph);
            }

            return null;
        }

        public static DiyFfb.GraphTest.GraphDefinition Convert(GraphDefinition graph)
        {
            var runtime = new DiyFfb.GraphTest.GraphDefinition { Version = graph.Version };
            var nodes = graph.Nodes.ToDictionary(n => n.Id, n => n);

            // Resolve graph-local named buses: each LocalReceive maps to the
            // (FromNodeId, FromPort) that feeds the matching LocalSend. This
            // is consulted by TryGetInputSource so consumers wired from a
            // LocalReceive transparently read the bus source.
            var localBusReceiveMap = BuildLocalBusReceiveMap(graph);

            foreach (var node in graph.Nodes)
            {
                // LocalSend / LocalReceive nodes are pure editor sugar — they
                // collapse away here; consumers wired from them get redirected
                // via localBusReceiveMap in TryGetInputSource.
                if (node.Kind == GraphNodeKind.LocalSend || node.Kind == GraphNodeKind.LocalReceive)
                {
                    continue;
                }

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
                        if (TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
                        {
                            runtimeNode.Args.Add(source);
                            runtimeNode.ArgNegate.Add(IsNegateSupportedOp(node.Op) && port.Negate);
                        }
                    }
                }
                else if (node.Kind == GraphNodeKind.Func)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
                        {
                            runtimeNode.Args.Add(source);
                        }
                    }
                }
                else if (node.Kind == GraphNodeKind.Output)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
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
                else if (node.Kind == GraphNodeKind.ConfigOut)
                {
                    // ConfigOut nodes are like Output but write to config fields.
                    // In a sub-graph: converted normally using ConfigField as the name.
                    // In a parent graph with FunctionScope: converter creates scoped ConfigOut
                    // nodes from the Include's CachedInterface (see Include handling below).
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (!string.IsNullOrEmpty(port.ConfigField) &&
                            TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
                        {
                            // Explicit (unscoped) ConfigOut: FunctionScope names the single
                            // target function ("Function:field"). Empty = current behavior
                            // (parent-scoped in includes, or top-level fan-out by group).
                            string name = string.IsNullOrEmpty(node.FunctionScope)
                                ? port.ConfigField
                                : node.FunctionScope + ":" + port.ConfigField;
                            var configOutNode = new DiyFfb.GraphTest.GraphNode
                            {
                                Id = BuildPortId(node.Id, port.Name),
                                Name = name,
                                Type = NodeType.ConfigOut,
                                Src = source
                            };
                            runtime.Nodes[configOutNode.Id] = configOutNode;
                        }
                    }
                    continue;
                }
                else if (node.Kind == GraphNodeKind.Include)
                {
                    // Embedded sub-graph: recursively convert the inline definition.
                    // The runtime resolver checks InlineGraph before Path, so a
                    // path-less embedded include evaluates straight from memory.
                    if (node.InlineGraph != null)
                    {
                        runtimeNode.InlineGraph = Convert(node.InlineGraph);
                    }

                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
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

                    // FunctionScope: auto-register scoped outputs and config outputs
                    if (!string.IsNullOrEmpty(node.FunctionScope) && node.CachedInterface != null)
                    {
                        string scope = node.FunctionScope;

                        // Scoped outputs: sub-graph Scoped Output ports → top-level Output nodes
                        foreach (var scopedOut in node.CachedInterface.ScopedOutputs)
                        {
                            string includeOutputId = BuildIncludeOutputId(node.Id, scopedOut.Name);
                            // Ensure the OutputMap entry exists for the sub-graph evaluator
                            if (!runtimeNode.OutputMap.ContainsKey(scopedOut.Name))
                            {
                                runtimeNode.OutputMap[scopedOut.Name] = includeOutputId;
                            }
                            string scopedName = scope + "." + scopedOut.SignalSuffix;
                            var scopedOutput = new DiyFfb.GraphTest.GraphNode
                            {
                                Id = node.Id + ":scoped:" + scopedOut.Name,
                                Name = scopedName,
                                Type = NodeType.Output,
                                Src = includeOutputId
                            };
                            runtime.Nodes[scopedOutput.Id] = scopedOutput;
                        }

                        // Scoped config outputs: sub-graph ConfigOut ports → top-level ConfigOut nodes
                        foreach (var cfgOut in node.CachedInterface.ConfigOutputs)
                        {
                            string includeOutputId = BuildIncludeOutputId(node.Id, cfgOut.Name);
                            // Add OutputMap entry so the sub-graph's ConfigOut value flows through
                            if (!runtimeNode.OutputMap.ContainsKey(cfgOut.Name))
                            {
                                runtimeNode.OutputMap[cfgOut.Name] = includeOutputId;
                            }
                            string scopedName = scope + ":" + cfgOut.ConfigField;
                            var scopedConfigOut = new DiyFfb.GraphTest.GraphNode
                            {
                                Id = node.Id + ":scoped_cfg:" + cfgOut.Name,
                                Name = scopedName,
                                Type = NodeType.ConfigOut,
                                Src = includeOutputId
                            };
                            runtime.Nodes[scopedConfigOut.Id] = scopedConfigOut;
                        }

                        // Scoped config inputs (inverse of scoped config outputs): feed
                        // each sub-graph ConfigIn the scoped function's merged config value.
                        // A parent-level ConfigIn source node carries the scoped key
                        // "scope:ConfigField" (the plugin populates it); it is wired into
                        // the sub-graph via the Include's InputMap. The library ConfigIn
                        // node reads inputs["ConfigField"] (no scope prefix), so we key the
                        // InputMap entry by ConfigField — EvalInclude passes it through
                        // unmapped (it isn't an Input port) straight into subInputs.
                        foreach (var cfgIn in node.CachedInterface.ConfigInputs)
                        {
                            string scopedNodeId = node.Id + ":scoped_cfgin:" + cfgIn.Name;
                            var scopedConfigIn = new DiyFfb.GraphTest.GraphNode
                            {
                                Id = scopedNodeId,
                                Name = scope + ":" + cfgIn.ConfigField,
                                Type = NodeType.ConfigIn
                            };
                            runtime.Nodes[scopedConfigIn.Id] = scopedConfigIn;
                            runtimeNode.InputMap[cfgIn.ConfigField] = scopedNodeId;
                        }
                    }
                }
                else if (node.Kind == GraphNodeKind.Expr)
                {
                    // Expr is a single-output node (like Op/Func) but binds its
                    // inputs by name: each wired input port becomes an InputMap
                    // entry whose key is the identifier usable in the formula.
                    runtimeNode.Expr = node.Expr ?? "";
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
                    {
                        if (TryGetInputSource(nodes, graph.Links, localBusReceiveMap, node.Id, port.Name, out var source))
                        {
                            runtimeNode.InputMap[port.Name] = source;
                        }
                    }
                    // Falls through to add runtimeNode (id == node.Id); consumers
                    // wired from its output port resolve to this id.
                }
                else if (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Param)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        // Build full signal name from SignalGroup + SignalSuffix
                        string signalName = BuildFullSignalName(node.SignalGroup, port.SignalSuffix, port.Name);
                        // For Param nodes, get default value from graph-level Params dictionary
                        double constValue = 0.0;
                        if (node.Kind == GraphNodeKind.Param && graph.Params.TryGetValue(signalName, out var paramMeta))
                        {
                            constValue = paramMeta.DefaultValue;
                        }
                        var inputNode = new DiyFfb.GraphTest.GraphNode
                        {
                            Id = BuildPortId(node.Id, port.Name),
                            Name = signalName,
                            Type = node.Kind == GraphNodeKind.Input ? NodeType.Input : NodeType.Param,
                            ConstValue = constValue
                        };
                        runtime.Nodes[inputNode.Id] = inputNode;
                    }
                    continue;
                }
                else if (node.Kind == GraphNodeKind.ConfigIn)
                {
                    // ConfigIn nodes are sources (like Input) that read a merged config
                    // field value supplied by the plugin. One runtime node per output
                    // port; Name is "ConfigType:ConfigField" so the plugin can resolve
                    // the target function + field. Consumers wired from the output port
                    // resolve via TryGetInputSource → BuildPortId, same as Input/Param.
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        if (string.IsNullOrEmpty(port.ConfigField))
                        {
                            continue;
                        }
                        // Scoped: the parent Include's FunctionScope supplies the function,
                        // so the runtime key is the bare field (fed via the parent's InputMap).
                        // Unscoped: ConfigType names the function explicitly ("Function:Field").
                        string key = node.Scoped
                            ? port.ConfigField
                            : (string.IsNullOrEmpty(node.ConfigType) ? port.ConfigField : node.ConfigType + ":" + port.ConfigField);
                        var configInNode = new DiyFfb.GraphTest.GraphNode
                        {
                            Id = BuildPortId(node.Id, port.Name),
                            Name = key,
                            Type = NodeType.ConfigIn
                        };
                        runtime.Nodes[configInNode.Id] = configInNode;
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
                case GraphNodeKind.ConfigOut: return NodeType.ConfigOut;
                case GraphNodeKind.ConfigIn: return NodeType.ConfigIn;
                case GraphNodeKind.Expr: return NodeType.Expr;
                // LocalSend / LocalReceive collapse away at convert time and
                // never reach the runtime; MapNodeType shouldn't be called on
                // them, but if it is just return Const (harmless).
                case GraphNodeKind.LocalSend: return NodeType.Const;
                case GraphNodeKind.LocalReceive: return NodeType.Const;
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
                case "neg": return OpType.Neg;
                case "clamp": return OpType.Clamp;
                case "lerp": return OpType.Lerp;
                case "select": return OpType.Select;
                case "eq": return OpType.Eq;
                case "gt": return OpType.Gt;
                case "exp": return OpType.Exp;
                case "sqrt": return OpType.Sqrt;
                case "pow": return OpType.Pow;
                default: return OpType.Add;
            }
        }

        private static bool IsNegateSupportedOp(string op)
        {
            switch ((op ?? "").Trim().ToLowerInvariant())
            {
                case "add":
                case "+":
                case "mul":
                case "*":
                    return true;
                default:
                    return false;
            }
        }

        private static bool TryGetInputSource(Dictionary<string, GraphNode> nodes, List<GraphLink> links,
            Dictionary<(string NodeId, string PortName), (string FromNodeId, string FromPort)> localBusReceiveMap,
            string nodeId, string portName, out string sourceId)
        {
            sourceId = null;
            var link = links.FirstOrDefault(l => l.ToNodeId == nodeId && l.ToPort == portName);
            if (link == null)
            {
                return false;
            }

            string fromNodeId = link.FromNodeId;
            string fromPort = link.FromPort;

            // If the link originates at a LocalReceive node's port, follow the
            // bus back to the source that feeds the matching LocalSend port.
            if (localBusReceiveMap != null && localBusReceiveMap.TryGetValue((fromNodeId, fromPort), out var busSource))
            {
                fromNodeId = busSource.FromNodeId;
                fromPort = busSource.FromPort;
            }

            if (string.IsNullOrEmpty(fromNodeId) || !nodes.ContainsKey(fromNodeId))
            {
                return false;
            }

            if (nodes.TryGetValue(fromNodeId, out var fromNode) &&
                fromNode.Kind == GraphNodeKind.Include &&
                fromPort != null)
            {
                sourceId = BuildIncludeOutputId(fromNodeId, fromPort);
                return true;
            }

            if (nodes.TryGetValue(fromNodeId, out var sourceNode) &&
                (sourceNode.Kind == GraphNodeKind.Input || sourceNode.Kind == GraphNodeKind.Param ||
                 sourceNode.Kind == GraphNodeKind.ConfigIn) &&
                !string.IsNullOrWhiteSpace(fromPort))
            {
                sourceId = BuildPortId(fromNodeId, fromPort);
                return true;
            }

            sourceId = fromNodeId;
            return true;
        }

        /// <summary>
        /// Per-port bus resolution. Each entry maps (LocalReceive nodeId,
        /// receive output portName) → (FromNodeId, FromPort) of whatever
        /// feeds the matching LocalSend input port. A LocalSend/LocalReceive
        /// node can carry many bus ports; each port has its own BusName.
        /// Orphan receives (no matching send for the bus name) are omitted —
        /// TryGetInputSource will then fail to resolve them and the consumer
        /// port becomes unconnected (evaluates to 0).
        /// </summary>
        private static Dictionary<(string NodeId, string PortName), (string FromNodeId, string FromPort)> BuildLocalBusReceiveMap(GraphDefinition graph)
        {
            var result = new Dictionary<(string, string), (string, string)>();
            if (graph?.Nodes == null) return result;

            // bus name → (FromNodeId, FromPort) feeding the Send port
            var busSource = new Dictionary<string, (string, string)>(StringComparer.Ordinal);
            foreach (var sendNode in graph.Nodes)
            {
                if (sendNode.Kind != GraphNodeKind.LocalSend) continue;
                foreach (var port in sendNode.Ports)
                {
                    if (port.Kind != GraphPortKind.Input) continue;
                    if (string.IsNullOrEmpty(port.BusName)) continue;
                    var feeder = graph.Links.FirstOrDefault(l => l.ToNodeId == sendNode.Id && l.ToPort == port.Name);
                    if (feeder == null) continue;
                    busSource[port.BusName] = (feeder.FromNodeId, feeder.FromPort);
                }
            }

            foreach (var recvNode in graph.Nodes)
            {
                if (recvNode.Kind != GraphNodeKind.LocalReceive) continue;
                foreach (var port in recvNode.Ports)
                {
                    if (port.Kind != GraphPortKind.Output) continue;
                    if (string.IsNullOrEmpty(port.BusName)) continue;
                    if (busSource.TryGetValue(port.BusName, out var src))
                    {
                        result[(recvNode.Id, port.Name)] = src;
                    }
                }
            }
            return result;
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
