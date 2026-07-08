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
            public bool[] ArgNegate = Array.Empty<bool>();
            public int SrcIndex = -1;
            public bool SrcIsExtra;
            public string[] IncludeInputNames = Array.Empty<string>();
            public int[] IncludeInputIndices = Array.Empty<int>();
            public bool[] IncludeInputIsExtra = Array.Empty<bool>();
            public string[] IncludeOutputNames = Array.Empty<string>();
            public int[] IncludeOutputIndices = Array.Empty<int>();
            /// <summary>Starting index in _state for this node's persistent state slots. -1 if not stateful.</summary>
            public int StateBaseIndex = -1;

            // Expr nodes: formula parsed once at compile, plus the resolved array
            // slots for each inport identifier referenced by the formula.
            public NCalc.Expression CompiledExpr;
            public string[] ExprParamNames = Array.Empty<string>();
            public int[] ExprParamIndices = Array.Empty<int>();
            public bool[] ExprParamIsExtra = Array.Empty<bool>();
        }

        private readonly GraphDefinition _graph;
        private readonly IGraphResolver _resolver;
        private readonly IncludeContextCache _contextCache;
        private readonly string _baseDirectory;
        private readonly List<CompiledNode> _order = new List<CompiledNode>();
        private readonly Dictionary<string, int> _nodeIndexById = new Dictionary<string, int>();
        private readonly Dictionary<string, int> _extraIndexById = new Dictionary<string, int>();
        private readonly Dictionary<string, GraphCompiledEvaluator> _includeCache = new Dictionary<string, GraphCompiledEvaluator>();
        private readonly Dictionary<string, IncludeNameMap> _includeNameMapCache = new Dictionary<string, IncludeNameMap>();
        private readonly int[] _outputIndices;
        private readonly string[] _outputNames;
        private readonly int[] _configOutIndices;
        private readonly string[] _configOutNames;
        private readonly string[] _configInKeys;
        private readonly double[] _values;
        private readonly double[] _extraValues;
        private readonly double[] _state;  // persists across evaluations for stateful Func nodes
        private double _dt;  // seconds since last evaluation, set each cycle for stateful funcs

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
            int stateSlotCount = 0;
            foreach (var node in ordered)
            {
                var compiled = new CompiledNode
                {
                    Node = node,
                    Index = _nodeIndexById[node.Id]
                };
                BuildArgs(compiled);
                BuildSrc(compiled);
                BuildIncludeBindings(compiled);
                BuildExpr(compiled);

                // Assign persistent state slots for stateful Func nodes
                int slotsNeeded = GetStateSlotsNeeded(node);
                if (slotsNeeded > 0)
                {
                    compiled.StateBaseIndex = stateSlotCount;
                    stateSlotCount += slotsNeeded;
                }

                _order.Add(compiled);
            }

            _state = new double[stateSlotCount];

            var outputIndices = new List<int>();
            var outputNames = new List<string>();
            foreach (var node in _order)
            {
                if (node.Node.Type != NodeType.Output)
                {
                    continue;
                }

                outputIndices.Add(node.Index);
                outputNames.Add(node.Node.Name ?? "");
            }

            _outputIndices = outputIndices.ToArray();
            _outputNames = outputNames.ToArray();

            var configOutIndices = new List<int>();
            var configOutNames = new List<string>();
            foreach (var node in _order)
            {
                if (node.Node.Type != NodeType.ConfigOut)
                {
                    continue;
                }

                configOutIndices.Add(node.Index);
                configOutNames.Add(node.Node.Name ?? "");
            }

            _configOutIndices = configOutIndices.ToArray();
            _configOutNames = configOutNames.ToArray();

            var configInKeys = new List<string>();
            foreach (var node in _order)
            {
                if (node.Node.Type == NodeType.ConfigIn && !string.IsNullOrEmpty(node.Node.Name))
                {
                    configInKeys.Add(node.Node.Name);
                }
            }
            _configInKeys = configInKeys.ToArray();
        }

        /// <summary>
        /// The (scoped) config field keys this graph reads via ConfigIn nodes, in
        /// "ConfigType:FieldPath" form. The host resolves each to a function + merged
        /// config value and supplies it in the inputs dictionary before evaluation.
        /// </summary>
        public IReadOnlyList<string> ConfigInputKeys => _configInKeys;

        public IReadOnlyDictionary<string, double> Evaluate(
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters,
            double dt = 0.0)
        {
            return EvaluateWithTrace(inputs, parameters, dt).Outputs;
        }

        public GraphEvaluationResult EvaluateWithTrace(
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters,
            double dt = 0.0)
        {
            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.LogSection($"EvaluateWithTrace START (baseDir={_baseDirectory})");
                GraphDebugLogger.Log($"  _extraIndexById.Count={_extraIndexById.Count}, _extraValues.Length={_extraValues.Length}");
            }

            // NOTE: Context cache clearing moved to plugin level (before top-level evaluation)
            // to avoid sub-evaluators clearing parent context during Include evaluation.

            _dt = dt;
            Array.Clear(_values, 0, _values.Length);
            if (_extraValues.Length > 0)
            {
                Array.Clear(_extraValues, 0, _extraValues.Length);
            }

            var warnings = new List<string>();

            foreach (var compiled in _order)
            {
                switch (compiled.Node.Type)
                {
                    case NodeType.Input:
                    case NodeType.ConfigIn:
                        // ConfigIn is a source like Input: the plugin supplies the
                        // merged config value under the node's (scoped) field key.
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
                    case NodeType.Expr:
                        _values[compiled.Index] = EvalExpr(compiled);
                        break;
                    case NodeType.Include:
                        EvalInclude(compiled, inputs, parameters, warnings);
                        break;
                    case NodeType.Output:
                        _values[compiled.Index] = Resolve(compiled.SrcIndex, compiled.SrcIsExtra);
                        break;
                    case NodeType.ConfigOut:
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
                if (GraphDebugLogger.Enabled)
                {
                    GraphDebugLogger.Log($"  result.NodeValues[{pair.Key}] = _extraValues[{pair.Value}] = {_extraValues[pair.Value]:F4}");
                }
            }
            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.Log($"  Total NodeValues count: {result.NodeValues.Count}");
            }
            for (int i = 0; i < _outputIndices.Length; i++)
            {
                result.Outputs[_outputNames[i]] = _values[_outputIndices[i]];
            }
            for (int i = 0; i < _configOutIndices.Length; i++)
            {
                result.ConfigOutputs[_configOutNames[i]] = _values[_configOutIndices[i]];
            }
            foreach (var w in warnings)
            {
                result.Warnings.Add(w);
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
            var negates = new bool[node.Node.Args.Count];
            for (int i = 0; i < node.Node.Args.Count; i++)
            {
                BuildArgRef(node.Node.Args[i], out indices[i], out extras[i]);
                negates[i] = i < node.Node.ArgNegate.Count && node.Node.ArgNegate[i];
            }
            node.ArgIndices = indices;
            node.ArgIsExtra = extras;
            node.ArgNegate = negates;
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

        private void BuildIncludeBindings(CompiledNode node)
        {
            if (node.Node.Type != NodeType.Include)
            {
                return;
            }

            if (node.Node.InputMap != null && node.Node.InputMap.Count > 0)
            {
                int count = node.Node.InputMap.Count;
                var names = new string[count];
                var indices = new int[count];
                var extras = new bool[count];
                int i = 0;
                foreach (var mapping in node.Node.InputMap)
                {
                    names[i] = mapping.Key ?? "";
                    BuildArgRef(mapping.Value, out indices[i], out extras[i]);
                    i++;
                }
                node.IncludeInputNames = names;
                node.IncludeInputIndices = indices;
                node.IncludeInputIsExtra = extras;
            }

            if (node.Node.OutputMap != null && node.Node.OutputMap.Count > 0)
            {
                int count = node.Node.OutputMap.Count;
                var names = new string[count];
                var indices = new int[count];
                int i = 0;
                foreach (var mapping in node.Node.OutputMap)
                {
                    names[i] = mapping.Key ?? "";
                    if (string.IsNullOrWhiteSpace(mapping.Value) || !_extraIndexById.TryGetValue(mapping.Value, out var index))
                    {
                        indices[i] = -1;
                    }
                    else
                    {
                        indices[i] = index;
                    }
                    i++;
                }
                node.IncludeOutputNames = names;
                node.IncludeOutputIndices = indices;
            }
        }

        // Parses an Expr node's formula once and pre-resolves each referenced
        // inport identifier to its value-array slot. Enforces that the formula
        // only references wired inports (InputMap keys): any other free identifier
        // throws here, failing graph compile rather than a runtime tick.
        private void BuildExpr(CompiledNode node)
        {
            if (node.Node.Type != NodeType.Expr)
            {
                return;
            }

            var expr = GraphExprSupport.TryParse(node.Node.Expr, out string parseError);
            if (expr == null)
            {
                throw new InvalidOperationException(
                    $"Expr '{node.Node.Id}': {parseError}");
            }

            var inputMap = node.Node.InputMap ?? new Dictionary<string, string>();
            var used = GraphExprSupport.CollectIdentifiers(expr);

            var names = new List<string>();
            var indices = new List<int>();
            var extras = new List<bool>();
            foreach (var name in used)
            {
                if (GraphExprSupport.IsConstant(name))
                {
                    continue; // built-in constant (e.g. Pi), not an inport
                }
                if (!inputMap.TryGetValue(name, out var sourceId))
                {
                    throw new InvalidOperationException(
                        $"Expr '{node.Node.Id}': '{name}' is not a wired inport " +
                        $"(available: [{string.Join(", ", inputMap.Keys)}]).");
                }
                BuildArgRef(sourceId, out int idx, out bool isExtra);
                names.Add(name);
                indices.Add(idx);
                extras.Add(isExtra);
            }

            GraphExprSupport.SeedConstants(expr, used);
            node.CompiledExpr = expr;
            node.ExprParamNames = names.ToArray();
            node.ExprParamIndices = indices.ToArray();
            node.ExprParamIsExtra = extras.ToArray();
        }

        private double EvalExpr(CompiledNode node)
        {
            var expr = node.CompiledExpr;
            if (expr == null)
            {
                return 0.0;
            }

            // Feed only the wired inports the formula actually references, read
            // straight from the value arrays. (Boxing here is the known GC cost
            // of the tree-walking path — acceptable for a handful of Expr nodes.)
            for (int i = 0; i < node.ExprParamNames.Length; i++)
            {
                expr.Parameters[node.ExprParamNames[i]] =
                    Resolve(node.ExprParamIndices[i], node.ExprParamIsExtra[i]);
            }

            try
            {
                return GraphExprSupport.ToDouble(expr.Evaluate());
            }
            catch
            {
                return 0.0;
            }
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
            switch (node.Node.Op)
            {
                case OpType.Add:
                {
                    if (node.ArgIndices.Length == 0) return 0.0;
                    if (node.ArgIndices.Length == 1) return ResolveArg(node, 0);
                    double sum = 0.0;
                    for (int i = 0; i < node.ArgIndices.Length; i++)
                    {
                        sum += ResolveArg(node, i);
                    }
                    return sum;
                }
                case OpType.Sub:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    return a - b;
                }
                case OpType.Mul:
                {
                    if (node.ArgIndices.Length == 0) return 0.0;
                    if (node.ArgIndices.Length == 1) return 0.0;
                    double product = ResolveArg(node, 0);
                    for (int i = 1; i < node.ArgIndices.Length; i++)
                    {
                        product *= ResolveArg(node, i);
                    }
                    return product;
                }
                case OpType.Div:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    return Math.Abs(b) < 1e-9 ? 0.0 : a / b;
                }
                case OpType.Min:
                {
                    if (node.ArgIndices.Length == 0) return 0.0;
                    if (node.ArgIndices.Length == 1)
                    {
                        double a = Resolve(node.ArgIndices[0], node.ArgIsExtra[0]);
                        return Math.Min(a, 0.0);
                    }
                    double value = Resolve(node.ArgIndices[0], node.ArgIsExtra[0]);
                    for (int i = 1; i < node.ArgIndices.Length; i++)
                    {
                        value = Math.Min(value, Resolve(node.ArgIndices[i], node.ArgIsExtra[i]));
                    }
                    return value;
                }
                case OpType.Max:
                {
                    if (node.ArgIndices.Length == 0) return 0.0;
                    if (node.ArgIndices.Length == 1)
                    {
                        double a = Resolve(node.ArgIndices[0], node.ArgIsExtra[0]);
                        return Math.Max(a, 0.0);
                    }
                    double value = Resolve(node.ArgIndices[0], node.ArgIsExtra[0]);
                    for (int i = 1; i < node.ArgIndices.Length; i++)
                    {
                        value = Math.Max(value, Resolve(node.ArgIndices[i], node.ArgIsExtra[i]));
                    }
                    return value;
                }
                case OpType.Abs:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    return Math.Abs(a);
                }
                case OpType.Neg:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    return -a;
                }
                case OpType.Clamp:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double min = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    double max = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 1.0;
                    return Math.Min(max, Math.Max(min, a));
                }
                case OpType.Lerp:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    double t = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 0.0;
                    return a + (b - a) * t;
                }
                case OpType.Select:
                {
                    double cond = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double a = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    double b = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 0.0;
                    return cond > 0.5 ? a : b;
                }
                case OpType.Eq:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    return Math.Abs(a - b) < 0.001 ? 1.0 : 0.0;
                }
                case OpType.Gt:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    return a > b ? 1.0 : 0.0;
                }
                case OpType.Exp:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    return Math.Exp(a);
                }
                case OpType.Sqrt:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    return a <= 0.0 ? 0.0 : Math.Sqrt(a);
                }
                case OpType.Pow:
                {
                    double a = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double b = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 0.0;
                    return Math.Pow(a, b);
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
                case "buffet":
                {
                    double alpha = node.ArgIndices.Length > 0 ? Resolve(node.ArgIndices[0], node.ArgIsExtra[0]) : 0.0;
                    double start = node.ArgIndices.Length > 1 ? Resolve(node.ArgIndices[1], node.ArgIsExtra[1]) : 10.0;
                    double full = node.ArgIndices.Length > 2 ? Resolve(node.ArgIndices[2], node.ArgIsExtra[2]) : 18.0;
                    double gain = node.ArgIndices.Length > 3 ? Resolve(node.ArgIndices[3], node.ArgIsExtra[3]) : 0.05;
                    double qhatEff = node.ArgIndices.Length > 4 ? Resolve(node.ArgIndices[4], node.ArgIsExtra[4]) : 1.0;
                    if (full <= start || gain <= 0.0) return 0.0;
                    if (alpha <= start) return 0.0;
                    double t = Math.Max(0.0, Math.Min(1.0, (alpha - start) / (full - start)));
                    return t * gain * qhatEff;
                }

                // --- Stateful functions (use persistent _state slots) ---

                // accumulator(trigger, step, min, max [, reset])
                // While trigger > 0.5, adds step * dt each cycle (step is in units/sec).
                // Optional reset arg: if > 0.5, zeros the accumulator.
                // State slot 0: accumulated value.
                case "accumulator":
                {
                    int si = node.StateBaseIndex;
                    double trigger = node.ArgIndices.Length > 0 ? ResolveArg(node, 0) : 0.0;
                    double step = node.ArgIndices.Length > 1 ? ResolveArg(node, 1) : 0.0;
                    double min = node.ArgIndices.Length > 2 ? ResolveArg(node, 2) : double.MinValue;
                    double max = node.ArgIndices.Length > 3 ? ResolveArg(node, 3) : double.MaxValue;
                    double reset = node.ArgIndices.Length > 4 ? ResolveArg(node, 4) : 0.0;
                    if (reset > 0.5)
                    {
                        _state[si] = 0.0;
                    }
                    else if (trigger > 0.5)
                    {
                        _state[si] = Math.Min(max, Math.Max(min, _state[si] + step * _dt));
                    }
                    return _state[si];
                }

                // sample_hold(input, trigger)
                // Captures input on falling edge of trigger (1→0 transition).
                // State slot 0: previous trigger value. Slot 1: held value.
                case "sample_hold":
                {
                    int si = node.StateBaseIndex;
                    double input = node.ArgIndices.Length > 0 ? ResolveArg(node, 0) : 0.0;
                    double trigger = node.ArgIndices.Length > 1 ? ResolveArg(node, 1) : 0.0;
                    double prevTrigger = _state[si];
                    _state[si] = trigger;
                    if (prevTrigger > 0.5 && trigger <= 0.5)
                    {
                        _state[si + 1] = input;
                    }
                    return _state[si + 1];
                }

                // edge_detect(input)
                // Outputs 1.0 for one tick on rising edge (0→1), 0.0 otherwise.
                // State slot 0: previous input value.
                case "edge_detect":
                {
                    int si = node.StateBaseIndex;
                    double input = node.ArgIndices.Length > 0 ? ResolveArg(node, 0) : 0.0;
                    double prev = _state[si];
                    _state[si] = input;
                    return (prev <= 0.5 && input > 0.5) ? 1.0 : 0.0;
                }

                // lag_asym(input, tau_up_sec, tau_down_sec)
                // First-order lag with direction-dependent time constant.
                // tau_up applies when input > prev (rising); tau_down when input <= prev (falling).
                // State slot 0: prev_output.
                case "lag_asym":
                {
                    int si = node.StateBaseIndex;
                    double input = node.ArgIndices.Length > 0 ? ResolveArg(node, 0) : 0.0;
                    double tauUp = node.ArgIndices.Length > 1 ? ResolveArg(node, 1) : 0.25;
                    double tauDown = node.ArgIndices.Length > 2 ? ResolveArg(node, 2) : 2.0;
                    double prev = _state[si];
                    double tau = (input > prev) ? tauUp : tauDown;
                    if (tau <= 0.0 || _dt <= 0.0)
                    {
                        _state[si] = input;
                        return input;
                    }
                    double alpha = 1.0 - Math.Exp(-_dt / tau);
                    double output = prev + alpha * (input - prev);
                    _state[si] = output;
                    return output;
                }
            }

            return 0.0;
        }

        /// <summary>
        /// Returns the number of persistent state slots needed by a node, or 0 if stateless.
        /// </summary>
        private static int GetStateSlotsNeeded(GraphNode node)
        {
            if (node.Type != NodeType.Func) return 0;
            switch (node.Func)
            {
                case "accumulator":  return 1;  // accumulated value
                case "sample_hold":  return 2;  // previous trigger + held value
                case "edge_detect":  return 1;  // previous input value
                case "lag_asym":     return 1;  // previous output
                default: return 0;
            }
        }

        /// <summary>
        /// Resets all persistent state to zero. Call on profile/vehicle switch.
        /// Also resets state in cached sub-graph evaluators (Include nodes).
        /// </summary>
        public void ResetState()
        {
            if (_state.Length > 0)
            {
                Array.Clear(_state, 0, _state.Length);
            }
            foreach (var sub in _includeCache.Values)
            {
                sub.ResetState();
            }
        }

        /// <summary>
        /// Captures all persistent state (this evaluator + sub-evaluators) as a flat dictionary.
        /// Keys are scoped by include cache key to disambiguate sub-graph state.
        /// </summary>
        public Dictionary<string, double[]> GetStateSnapshot()
        {
            var snapshot = new Dictionary<string, double[]>();
            if (_state.Length > 0)
            {
                snapshot[""] = (double[])_state.Clone();
            }
            foreach (var kv in _includeCache)
            {
                var subSnapshot = kv.Value.GetStateSnapshot();
                foreach (var sub in subSnapshot)
                {
                    string key = string.IsNullOrEmpty(sub.Key)
                        ? kv.Key
                        : kv.Key + "|" + sub.Key;
                    snapshot[key] = sub.Value;
                }
            }
            return snapshot;
        }

        /// <summary>
        /// Restores persistent state from a snapshot previously captured by GetStateSnapshot().
        /// Mismatched keys or array lengths are silently skipped (graph structure may have changed).
        /// </summary>
        public void RestoreStateSnapshot(Dictionary<string, double[]> snapshot)
        {
            if (snapshot == null) return;

            if (snapshot.TryGetValue("", out var root) && root.Length == _state.Length)
            {
                Array.Copy(root, _state, _state.Length);
            }

            foreach (var kv in _includeCache)
            {
                // Build the sub-snapshot for this include by stripping our prefix
                var subSnapshot = new Dictionary<string, double[]>();
                string prefix = kv.Key + "|";
                foreach (var entry in snapshot)
                {
                    if (entry.Key == kv.Key)
                    {
                        subSnapshot[""] = entry.Value;
                    }
                    else if (entry.Key.StartsWith(prefix, StringComparison.Ordinal))
                    {
                        subSnapshot[entry.Key.Substring(prefix.Length)] = entry.Value;
                    }
                }
                if (subSnapshot.Count > 0)
                {
                    kv.Value.RestoreStateSnapshot(subSnapshot);
                }
            }
        }

        private void EvalInclude(CompiledNode node, IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters, List<string> warnings = null)
        {
            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.LogSection($"EvalInclude: {node.Node.Id} (path={node.Node.Path})");
                GraphDebugLogger.LogMap("OutputMap", node.Node.OutputMap);
            }

            GraphDefinition subGraph = node.Node.InlineGraph;
            string key = null;
            if (subGraph == null && !string.IsNullOrWhiteSpace(node.Node.Path) && _resolver != null)
            {
                // Resolve path relative to current graph's directory for nested include support.
                // This ensures inner/inner.json in sub/middle.json resolves to sub/inner/inner.json.
                string resolvedSubGraphPath = ResolveToAbsolutePath(node.Node.Path);
                // Key by node ID + path so each Include node gets its own evaluator
                // instance (and its own _state for stateful funcs like accumulator).
                key = node.Node.Id + ":" + resolvedSubGraphPath;
                if (!_includeCache.TryGetValue(key, out var cached))
                {
                    subGraph = _resolver.GetGraph(resolvedSubGraphPath);  // Pass absolute path to resolver
                    if (subGraph != null)
                    {
                        // Pass the sub-graph's directory as base for nested include resolution.
                        string subGraphDir = Path.GetDirectoryName(resolvedSubGraphPath) ?? _baseDirectory;
                        cached = new GraphCompiledEvaluator(subGraph, _resolver, _contextCache, subGraphDir);
                        _includeCache[key] = cached;
                    }
                    else
                    {
                        warnings?.Add($"Include '{node.Node.Id}': failed to load '{node.Node.Path}'");
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
                if (_resolver == null)
                {
                    warnings?.Add($"Include '{node.Node.Id}': no resolver configured");
                }
                return;
            }

            GraphCompiledEvaluator evaluator = _includeCache[key];

            var includeNameMap = GetIncludeNameMap(key, subGraph, node);
            var shortToFullName = includeNameMap?.Inputs;
            var shortToFullOutput = includeNameMap?.Outputs;

            var subInputs = new Dictionary<string, double>();
            for (int i = 0; i < node.IncludeInputNames.Length; i++)
            {
                // Try to find the full name for this short name, otherwise use the key as-is
                string shortName = node.IncludeInputNames[i];
                string inputName = shortToFullName != null && shortToFullName.TryGetValue(shortName, out var fullName) ? fullName : shortName;
                subInputs[inputName] = Resolve(node.IncludeInputIndices[i], node.IncludeInputIsExtra[i]);
            }

            // Build parameters from sub-graph's perspective (keyed by sub-graph's Param node names)
            // Use sub-graph's Param node default (ConstValue) if not overridden by parent
            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.Log($"  Building subParams from parent parameters ({parameters?.Count ?? 0} entries):");
                GraphDebugLogger.LogDict("parent parameters", parameters);
            }

            // Seed with the parent's parameters so resolved values/overrides
            // propagate THROUGH intermediate sub-graphs that don't have the param
            // node themselves (e.g. a cue param two includes deep:
            // template -> msfs_derivations -> cue). Then fill this sub-graph's own
            // param-node defaults for any the parent didn't supply.
            var subParams = new Dictionary<string, double>();
            if (parameters != null)
            {
                foreach (var kv in parameters)
                {
                    subParams[kv.Key] = kv.Value;
                }
            }
            foreach (var subNode in subGraph.Nodes.Values)
            {
                if (subNode.Type == NodeType.Param && !string.IsNullOrEmpty(subNode.Name)
                    && !subParams.ContainsKey(subNode.Name))
                {
                    subParams[subNode.Name] = subNode.ConstValue;
                    if (GraphDebugLogger.Enabled)
                    {
                        GraphDebugLogger.Log($"    Param '{subNode.Name}': default = {subNode.ConstValue}");
                    }
                }
            }

            // Use EvaluateWithTrace so we can bridge BOTH Outputs and ConfigOutputs.
            // The parent's Include OutputMap mixes both kinds — Evaluate(...) returns
            // only Outputs, which silently drops sub-graph ConfigOut values and leaves
            // the parent's scoped ConfigOut nodes reading 0.
            var subResult = evaluator.EvaluateWithTrace(subInputs, subParams, _dt);
            var outputs = subResult.Outputs;

            // Capture context for sub-graph preview (after evaluate so state is current).
            // File includes key by resolved path; embedded (inline) sub-graphs key by
            // "inline:<nodeId>" so their tabs can request live context too.
            if (_contextCache != null && !string.IsNullOrEmpty(key))
            {
                string resolvedPath = key.StartsWith("inline:")
                    ? key
                    : ResolveToAbsolutePath(node.Node.Path);

                _contextCache.Add(resolvedPath, new IncludeCallContext
                {
                    IncludeNodeId = node.Node.Id,
                    IncludeNodeTitle = !string.IsNullOrEmpty(node.Node.Name) ? node.Node.Name : node.Node.Id,
                    IncludePath = resolvedPath,
                    Inputs = new Dictionary<string, double>(subInputs),
                    Parameters = new Dictionary<string, double>(subParams),
                    StateSnapshot = evaluator.GetStateSnapshot()
                });
            }

            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.Log($"  Sub-graph evaluation complete");
                GraphDebugLogger.LogDict("subInputs", subInputs);
                GraphDebugLogger.LogDict("subParams", subParams);
                GraphDebugLogger.LogDict("outputs (from sub-graph)", outputs);
            }

            // Similarly map output names
            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.LogMap("shortToFullOutput", shortToFullOutput);
                GraphDebugLogger.Log($"  _extraIndexById count: {_extraIndexById.Count}");
            }

            for (int i = 0; i < node.IncludeOutputNames.Length; i++)
            {
                string shortName = node.IncludeOutputNames[i];
                string outputName = shortToFullOutput != null && shortToFullOutput.TryGetValue(shortName, out var fullOutName) ? fullOutName : shortName;
                // The OutputMap entry can refer to either an Output or a ConfigOut port
                // in the sub-graph (parent treats them uniformly when wiring scoped
                // outputs / config outputs from a FunctionScope Include).
                if (!outputs.TryGetValue(outputName, out var value) &&
                    !subResult.ConfigOutputs.TryGetValue(outputName, out value))
                {
                    warnings?.Add($"Include '{node.Node.Id}' output '{shortName}': no match for '{outputName}' in sub-graph outputs [{string.Join(", ", outputs.Keys)}] or configOutputs [{string.Join(", ", subResult.ConfigOutputs.Keys)}]");
                    continue;
                }
                int extraIndex = node.IncludeOutputIndices[i];
                if (extraIndex < 0)
                {
                    if (GraphDebugLogger.Enabled)
                    {
                        GraphDebugLogger.Log($"  WARNING: extra index not found for '{shortName}'");
                    }
                    warnings?.Add($"Include '{node.Node.Id}' output '{shortName}': extra index not found");
                    continue;
                }
                if (GraphDebugLogger.Enabled)
                {
                    GraphDebugLogger.Log($"  Storing: _extraValues[{extraIndex}] = {value:F4} (key={shortName}, outputName={outputName})");
                }
                _extraValues[extraIndex] = value;
            }
        }

        private IncludeNameMap GetIncludeNameMap(string key, GraphDefinition subGraph, CompiledNode node)
        {
            if (string.IsNullOrEmpty(key) || subGraph == null)
            {
                return null;
            }

            if (_includeNameMapCache.TryGetValue(key, out var cached))
            {
                return cached;
            }

            var map = new IncludeNameMap
            {
                Inputs = BuildShortToFullNameMap(subGraph, node.IncludeInputNames, NodeType.Input),
                Outputs = BuildShortToFullNameMap(subGraph, node.IncludeOutputNames, NodeType.Output)
            };
            _includeNameMapCache[key] = map;
            return map;
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

        private sealed class IncludeNameMap
        {
            public Dictionary<string, string> Inputs;
            public Dictionary<string, string> Outputs;
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
                    // Include and Expr nodes have dependencies via InputMap values
                    if ((node.Type == NodeType.Include || node.Type == NodeType.Expr) && node.InputMap != null)
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

        private double ResolveArg(CompiledNode node, int index)
        {
            double value = Resolve(node.ArgIndices[index], node.ArgIsExtra[index]);
            if (index < node.ArgNegate.Length && node.ArgNegate[index])
            {
                value = -value;
            }
            return value;
        }
    }
}
