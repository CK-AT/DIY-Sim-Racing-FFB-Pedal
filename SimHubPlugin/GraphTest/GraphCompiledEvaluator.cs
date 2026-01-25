using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace DiyFfb.GraphTest
{
    public sealed class GraphCompiledEvaluator
    {
        private sealed class CompiledNode
        {
            public GraphNode Node;
            public int Index;
            public int[] ArgIndices = Array.Empty<int>();
            public bool[] ArgIsExtra = Array.Empty<bool>();
            public int SrcIndex = -1;
            public bool SrcIsExtra;
        }

        private readonly GraphDefinition _graph;
        private readonly IGraphResolver _resolver;
        private readonly IncludeContextCache _contextCache;
        private readonly string _baseDirectory;
        private readonly List<CompiledNode> _order = new List<CompiledNode>();
        private readonly Dictionary<string, int> _nodeIndexById = new Dictionary<string, int>();
        private readonly Dictionary<string, int> _extraIndexById = new Dictionary<string, int>();
        private readonly Dictionary<string, GraphCompiledEvaluator> _includeCache = new Dictionary<string, GraphCompiledEvaluator>();
        private readonly int[] _outputNodeIndices;
        private readonly double[] _values;
        private readonly double[] _extraValues;

        public GraphCompiledEvaluator(GraphDefinition graph, IGraphResolver resolver = null)
            : this(graph, resolver, null, null)
        {
        }

        public GraphCompiledEvaluator(GraphDefinition graph, IGraphResolver resolver, IncludeContextCache contextCache, string baseDirectory = null)
        {
            _graph = graph ?? throw new ArgumentNullException(nameof(graph));
            _resolver = resolver;
            _contextCache = contextCache;
            _baseDirectory = baseDirectory ?? "";

            foreach (var pair in _graph.Nodes)
            {
                _nodeIndexById[pair.Key] = _nodeIndexById.Count;
            }

            var includeOutputs = BuildIncludeOutputMap(_graph);
            foreach (var key in includeOutputs.Keys)
            {
                if (!_extraIndexById.ContainsKey(key))
                {
                    _extraIndexById[key] = _extraIndexById.Count;
                }
            }

            _values = new double[_nodeIndexById.Count];
            _extraValues = new double[_extraIndexById.Count];

            var ordered = TopoSort(_graph, includeOutputs);
            foreach (var node in ordered)
            {
                var compiled = new CompiledNode
                {
                    Node = node,
                    Index = _nodeIndexById[node.Id]
                };
                BuildArgs(compiled);
                BuildSrc(compiled);
                _order.Add(compiled);
            }

            _outputNodeIndices = _order
                .Where(n => n.Node.Type == NodeType.Output)
                .Select(n => n.Index)
                .ToArray();
        }

        public IReadOnlyDictionary<string, double> Evaluate(
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            return EvaluateWithTrace(inputs, parameters).Outputs;
        }

        public GraphEvaluationResult EvaluateWithTrace(
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            // NOTE: Context cache clearing moved to plugin level (before top-level evaluation)
            // to avoid sub-evaluators clearing parent context during Include evaluation.

            Array.Clear(_values, 0, _values.Length);
            if (_extraValues.Length > 0)
            {
                Array.Clear(_extraValues, 0, _extraValues.Length);
            }

            foreach (var compiled in _order)
            {
                switch (compiled.Node.Type)
                {
                    case NodeType.Input:
                        _values[compiled.Index] = inputs != null && inputs.TryGetValue(compiled.Node.Name, out var inVal) ? inVal : 0.0;
                        break;
                    case NodeType.Param:
                        _values[compiled.Index] = parameters != null && parameters.TryGetValue(compiled.Node.Name, out var pVal) ? pVal : 0.0;
                        break;
                    case NodeType.Const:
                        _values[compiled.Index] = compiled.Node.ConstValue;
                        break;
                    case NodeType.Op:
                        _values[compiled.Index] = EvalOp(compiled);
                        break;
                    case NodeType.Func:
                        _values[compiled.Index] = EvalFunc(compiled);
                        break;
                    case NodeType.Include:
                        EvalInclude(compiled, inputs, parameters);
                        break;
                    case NodeType.Output:
                        _values[compiled.Index] = Resolve(compiled.SrcIndex, compiled.SrcIsExtra);
                        break;
                }
            }

            var result = new GraphEvaluationResult();
            foreach (var pair in _nodeIndexById)
            {
                result.NodeValues[pair.Key] = _values[pair.Value];
            }
            foreach (var pair in _extraIndexById)
            {
                result.NodeValues[pair.Key] = _extraValues[pair.Value];
            }
            foreach (var index in _outputNodeIndices)
            {
                var node = _graph.Nodes.Values.First(n => _nodeIndexById[n.Id] == index);
                result.Outputs[node.Name] = _values[index];
            }

            return result;
        }

        private void BuildArgs(CompiledNode node)
        {
            if (node.Node.Args == null || node.Node.Args.Count == 0)
            {
                return;
            }

            var indices = new int[node.Node.Args.Count];
            var extras = new bool[node.Node.Args.Count];
            for (int i = 0; i < node.Node.Args.Count; i++)
            {
                BuildArgRef(node.Node.Args[i], out indices[i], out extras[i]);
            }
            node.ArgIndices = indices;
            node.ArgIsExtra = extras;
        }

        private void BuildSrc(CompiledNode node)
        {
            if (string.IsNullOrWhiteSpace(node.Node.Src))
            {
                node.SrcIndex = -1;
                node.SrcIsExtra = false;
                return;
            }
            BuildArgRef(node.Node.Src, out node.SrcIndex, out node.SrcIsExtra);
        }

        private void BuildArgRef(string id, out int index, out bool isExtra)
        {
            index = -1;
            isExtra = false;
            if (string.IsNullOrWhiteSpace(id))
            {
                return;
            }
            if (_nodeIndexById.TryGetValue(id, out index))
            {
                isExtra = false;
                return;
            }
            if (_extraIndexById.TryGetValue(id, out index))
            {
                isExtra = true;
            }
        }

        private double Resolve(int index, bool isExtra)
        {
            if (index < 0)
            {
                return 0.0;
            }
            return isExtra ? _extraValues[index] : _values[index];
        }

        private double ResolveById(string id)
        {
            BuildArgRef(id, out var index, out var isExtra);
            return Resolve(index, isExtra);
        }

        private string ResolveToAbsolutePath(string path)
        {
            if (string.IsNullOrEmpty(path))
            {
                return path;
            }
            if (Path.IsPathRooted(path))
            {
                return Path.GetFullPath(path);
            }
            if (!string.IsNullOrEmpty(_baseDirectory))
            {
                return Path.GetFullPath(Path.Combine(_baseDirectory, path));
            }
            return path;
        }

        private double EvalOp(CompiledNode node)
        {
            double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
            double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;

            switch (node.Node.Op)
            {
                case OpType.Add: return a + b;
                case OpType.Sub: return a - b;
                case OpType.Mul: return a * b;
                case OpType.Div: return Math.Abs(b) < 1e-9 ? 0.0 : a / b;
                case OpType.Min: return Math.Min(a, b);
                case OpType.Max: return Math.Max(a, b);
                case OpType.Abs: return Math.Abs(a);
                case OpType.Clamp:
                {
                    double min = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    double max = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 1.0;
                    return Math.Min(max, Math.Max(min, a));
                }
                case OpType.Lerp:
                {
                    double t = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 0.0;
                    return a + (b - a) * t;
                }
            }

            return 0.0;
        }

        private double EvalFunc(CompiledNode node)
        {
            switch (node.Node.Func)
            {
                case "qhat_eff":
                {
                    double iasKts = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double vref = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 60.0;
                    double iasMps = iasKts * 0.514444;
                    double vrefMps = vref * 0.514444;
                    double qHat = iasMps * iasMps;
                    double qHatRef = vrefMps * vrefMps;
                    return qHatRef <= 0.0 ? 0.0 : qHat / qHatRef;
                }
                case "torque_norm":
                {
                    double trq = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double trqRef = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 1.0;
                    return trqRef <= 0.0 ? 0.0 : trq / trqRef;
                }
                case "rpm_norm":
                {
                    double rpm = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double rpmRef = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 1.0;
                    return rpmRef <= 0.0 ? 0.0 : rpm / rpmRef;
                }
                case "assist_loss":
                {
                    double rpmNorm = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    return Math.Min(1.0, Math.Max(0.0, 1.0 - rpmNorm));
                }
            }

            return 0.0;
        }

        private void EvalInclude(CompiledNode node, IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            GraphDefinition subGraph = node.Node.InlineGraph;
            string key = null;
            if (subGraph == null && !string.IsNullOrWhiteSpace(node.Node.Path) && _resolver != null)
            {
                key = node.Node.Path;
                if (!_includeCache.TryGetValue(key, out var cached))
                {
                    subGraph = _resolver.GetGraph(node.Node.Path);
                    if (subGraph != null)
                    {
                        // Pass context cache and base directory to sub-evaluator for nested includes
                        cached = new GraphCompiledEvaluator(subGraph, _resolver, _contextCache, _baseDirectory);
                        _includeCache[key] = cached;
                    }
                }
                subGraph = cached?._graph ?? subGraph;
            }
            else if (subGraph != null)
            {
                key = "inline:" + node.Node.Id;
                if (!_includeCache.TryGetValue(key, out var cached))
                {
                    cached = new GraphCompiledEvaluator(subGraph, _resolver, _contextCache, _baseDirectory);
                    _includeCache[key] = cached;
                }
                subGraph = cached._graph;
            }

            if (subGraph == null)
            {
                return;
            }

            GraphCompiledEvaluator evaluator = _includeCache[key];

            // Build mapping from short names (InputMap keys) to full names (subGraph Input node names)
            // This handles the case where Include ports use SignalSuffix but runtime uses full names
            var shortToFullName = BuildShortToFullNameMap(subGraph, node.Node.InputMap.Keys);

            var subInputs = new Dictionary<string, double>();
            foreach (var mapping in node.Node.InputMap)
            {
                // Try to find the full name for this short name, otherwise use the key as-is
                string inputName = shortToFullName.TryGetValue(mapping.Key, out var fullName) ? fullName : mapping.Key;
                subInputs[inputName] = ResolveById(mapping.Value);
            }

            // Capture context for sub-graph preview
            if (_contextCache != null && !string.IsNullOrEmpty(key) && !key.StartsWith("inline:"))
            {
                string resolvedPath = ResolveToAbsolutePath(node.Node.Path);

                // Build parameters from sub-graph's perspective (keyed by sub-graph's Param node names)
                var paramsCopy = new Dictionary<string, double>();
                foreach (var subNode in subGraph.Nodes.Values)
                {
                    if (subNode.Type == NodeType.Param && !string.IsNullOrEmpty(subNode.Name))
                    {
                        // Use the value from parent's parameters if available, otherwise 0.0
                        double value = parameters != null && parameters.TryGetValue(subNode.Name, out var pVal) ? pVal : 0.0;
                        paramsCopy[subNode.Name] = value;
                    }
                }

                _contextCache.Add(resolvedPath, new IncludeCallContext
                {
                    IncludeNodeId = node.Node.Id,
                    IncludeNodeTitle = !string.IsNullOrEmpty(node.Node.Name) ? node.Node.Name : node.Node.Id,
                    IncludePath = resolvedPath,
                    Inputs = new Dictionary<string, double>(subInputs),
                    Parameters = paramsCopy
                });
            }

            var outputs = evaluator.Evaluate(subInputs, parameters);

            // Similarly map output names
            var shortToFullOutput = BuildShortToFullNameMap(subGraph, node.Node.OutputMap.Keys, NodeType.Output);

            foreach (var mapping in node.Node.OutputMap)
            {
                string outputName = shortToFullOutput.TryGetValue(mapping.Key, out var fullOutName) ? fullOutName : mapping.Key;
                if (outputs.TryGetValue(outputName, out var value) &&
                    _extraIndexById.TryGetValue(mapping.Value, out var extraIndex))
                {
                    _extraValues[extraIndex] = value;
                }
            }
        }

        /// <summary>
        /// Builds a mapping from short names (SignalSuffix) to full names (SignalGroup.SignalSuffix).
        /// </summary>
        private static Dictionary<string, string> BuildShortToFullNameMap(
            GraphDefinition graph,
            IEnumerable<string> shortNames,
            NodeType targetType = NodeType.Input)
        {
            var result = new Dictionary<string, string>();
            var shortNameSet = new HashSet<string>(shortNames);

            foreach (var graphNode in graph.Nodes.Values)
            {
                if (graphNode.Type != targetType)
                {
                    continue;
                }

                string fullName = graphNode.Name;
                if (string.IsNullOrEmpty(fullName))
                {
                    continue;
                }

                // Check if any short name matches the end of the full name
                // e.g., "Speed.IAS" matches "XPlane.Speed.IAS"
                foreach (var shortName in shortNameSet)
                {
                    if (fullName == shortName)
                    {
                        // Exact match (e.g., library graph)
                        result[shortName] = fullName;
                    }
                    else if (fullName.EndsWith("." + shortName, StringComparison.Ordinal))
                    {
                        // Suffix match (e.g., "XPlane.Speed.IAS" ends with ".Speed.IAS")
                        result[shortName] = fullName;
                    }
                }
            }

            return result;
        }

        private static Dictionary<string, string> BuildIncludeOutputMap(GraphDefinition graph)
        {
            var includeOutputs = new Dictionary<string, string>();
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
                        includeOutputs[mapping.Value] = node.Id;
                    }
                }
            }
            return includeOutputs;
        }

        private static List<GraphNode> TopoSort(GraphDefinition graph, Dictionary<string, string> includeOutputs)
        {
            var result = new List<GraphNode>();
            var visiting = new HashSet<string>();
            var visited = new HashSet<string>();

            void Visit(string id)
            {
                if (visited.Contains(id)) return;
                if (visiting.Contains(id)) throw new InvalidOperationException("Graph has a cycle.");
                visiting.Add(id);

                if (graph.Nodes.TryGetValue(id, out var node))
                {
                    foreach (var dep in node.Args ?? Enumerable.Empty<string>())
                    {
                        Visit(dep);
                    }
                    if (!string.IsNullOrEmpty(node.Src))
                    {
                        Visit(node.Src);
                    }
                    // Include nodes have dependencies via InputMap values
                    if (node.Type == NodeType.Include && node.InputMap != null)
                    {
                        foreach (var dep in node.InputMap.Values)
                        {
                            if (!string.IsNullOrEmpty(dep))
                            {
                                Visit(dep);
                            }
                        }
                    }
                    result.Add(node);
                }
                else if (includeOutputs.TryGetValue(id, out var includeId))
                {
                    Visit(includeId);
                }

                visiting.Remove(id);
                visited.Add(id);
            }

            foreach (var node in graph.Nodes.Values)
            {
                Visit(node.Id);
            }

            return result;
        }
    }
}
