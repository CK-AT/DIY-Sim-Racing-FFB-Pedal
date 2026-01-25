using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.GraphTest
{
    public enum NodeType
    {
        Input,
        Param,
        Const,
        Op,
        Func,
        Include,
        Output
    }

    public enum OpType
    {
        Add,
        Sub,
        Mul,
        Div,
        Min,
        Max,
        Abs,
        Clamp,
        Lerp
    }

    public sealed class GraphNode
    {
        public string Id = "";
        public NodeType Type;
        public string Name = "";
        public double ConstValue;
        public OpType Op;
        public string Func = "";
        public List<string> Args = new List<string>();
        public string Src = "";
        public string Path = "";
        public Dictionary<string, string> InputMap = new Dictionary<string, string>();
        public Dictionary<string, string> OutputMap = new Dictionary<string, string>();
        public GraphDefinition InlineGraph;
    }

    public sealed class GraphDefinition
    {
        public int Version = 1;
        public Dictionary<string, GraphNode> Nodes = new Dictionary<string, GraphNode>();
    }

    public sealed class GraphEvaluationResult
    {
        public Dictionary<string, double> Outputs { get; } = new Dictionary<string, double>();
        public Dictionary<string, double> NodeValues { get; } = new Dictionary<string, double>();
        public List<string> Warnings { get; } = new List<string>();
    }

    public interface IGraphResolver
    {
        GraphDefinition GetGraph(string path);
    }

    public sealed class GraphEvaluator
    {
        private readonly GraphDefinition _graph;
        private readonly List<GraphNode> _order;
        private readonly Dictionary<string, double> _values = new Dictionary<string, double>();
        private readonly IGraphResolver _resolver;

        public GraphEvaluator(GraphDefinition graph, IGraphResolver resolver = null)
        {
            _graph = graph ?? throw new ArgumentNullException(nameof(graph));
            _resolver = resolver;
            _order = TopoSort(graph);
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
            _values.Clear();

            foreach (var node in _order)
            {
                switch (node.Type)
                {
                    case NodeType.Input:
                        _values[node.Id] = inputs != null && inputs.TryGetValue(node.Name, out var inVal) ? inVal : 0.0;
                        break;
                    case NodeType.Param:
                        _values[node.Id] = parameters != null && parameters.TryGetValue(node.Name, out var pVal) ? pVal : 0.0;
                        break;
                    case NodeType.Const:
                        _values[node.Id] = node.ConstValue;
                        break;
                    case NodeType.Op:
                        _values[node.Id] = EvalOp(node);
                        break;
                    case NodeType.Func:
                        _values[node.Id] = EvalFunc(node);
                        break;
                    case NodeType.Include:
                        _values[node.Id] = EvalInclude(node, inputs, parameters);
                        break;
                    case NodeType.Output:
                        _values[node.Id] = Resolve(node.Src);
                        break;
                }
            }

            var result = new GraphEvaluationResult();
            foreach (var pair in _values)
            {
                result.NodeValues[pair.Key] = pair.Value;
            }

            foreach (var node in _order.Where(n => n.Type == NodeType.Output))
            {
                result.Outputs[node.Name] = _values[node.Id];
            }

            return result;
        }

        private double Resolve(string id)
        {
            return _values.TryGetValue(id, out var v) ? v : 0.0;
        }

        private double EvalOp(GraphNode node)
        {
            double a = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
            double b = node.Args.Count > 1 ? Resolve(node.Args[1]) : 0.0;

            switch (node.Op)
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
                    double min = node.Args.Count > 1 ? Resolve(node.Args[1]) : 0.0;
                    double max = node.Args.Count > 2 ? Resolve(node.Args[2]) : 1.0;
                    return Math.Min(max, Math.Max(min, a));
                }
                case OpType.Lerp:
                {
                    double t = node.Args.Count > 2 ? Resolve(node.Args[2]) : 0.0;
                    return a + (b - a) * t;
                }
            }

            return 0.0;
        }

        private double EvalFunc(GraphNode node)
        {
            switch (node.Func)
            {
                case "qhat_eff":
                {
                    double iasKts = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
                    double vref = node.Args.Count > 1 ? Resolve(node.Args[1]) : 60.0;
                    double iasMps = iasKts * 0.514444;
                    double vrefMps = vref * 0.514444;
                    double qHat = iasMps * iasMps;
                    double qHatRef = vrefMps * vrefMps;
                    return qHatRef <= 0.0 ? 0.0 : qHat / qHatRef;
                }
                case "torque_norm":
                {
                    double trq = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
                    double trqRef = node.Args.Count > 1 ? Resolve(node.Args[1]) : 1.0;
                    return trqRef <= 0.0 ? 0.0 : trq / trqRef;
                }
                case "rpm_norm":
                {
                    double rpm = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
                    double rpmRef = node.Args.Count > 1 ? Resolve(node.Args[1]) : 1.0;
                    return rpmRef <= 0.0 ? 0.0 : rpm / rpmRef;
                }
                case "assist_loss":
                {
                    double rpmNorm = node.Args.Count > 0 ? Resolve(node.Args[0]) : 0.0;
                    return Math.Min(1.0, Math.Max(0.0, 1.0 - rpmNorm));
                }
            }

            return 0.0;
        }

        private double EvalInclude(GraphNode node, IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            if (_resolver == null || string.IsNullOrWhiteSpace(node.Path))
            {
                var includeResolver = _resolver as GraphIncludeResolver;
                if (includeResolver == null)
                {
                    return 0.0;
                }

                var inlineGraph = includeResolver.ResolveInclude(node);
                if (inlineGraph == null)
                {
                    return 0.0;
                }

                return EvaluateIncludeGraph(node, inlineGraph, parameters);
            }

            var subGraph = _resolver.GetGraph(node.Path);
            if (subGraph == null)
            {
                var includeResolver = _resolver as GraphIncludeResolver;
                if (includeResolver == null)
                {
                    return 0.0;
                }

                var inlineGraph = includeResolver.ResolveInclude(node);
                if (inlineGraph == null)
                {
                    return 0.0;
                }

                return EvaluateIncludeGraph(node, inlineGraph, parameters);
            }

            return EvaluateIncludeGraph(node, subGraph, parameters);
        }

        private double EvaluateIncludeGraph(GraphNode node, GraphDefinition subGraph,
            IReadOnlyDictionary<string, double> parameters)
        {
            var subInputs = new Dictionary<string, double>();
            foreach (var mapping in node.InputMap)
            {
                subInputs[mapping.Key] = Resolve(mapping.Value);
            }

            var subOutputs = new GraphEvaluator(subGraph, _resolver).Evaluate(subInputs, parameters);
            foreach (var mapping in node.OutputMap)
            {
                if (subOutputs.TryGetValue(mapping.Key, out var value))
                {
                    _values[mapping.Value] = value;
                }
            }

            return 0.0;
        }

        private static List<GraphNode> TopoSort(GraphDefinition graph)
        {
            var result = new List<GraphNode>();
            var visiting = new HashSet<string>();
            var visited = new HashSet<string>();
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
