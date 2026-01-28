using DiyFfb.GraphTest;
using System.Collections.Generic;
using System.IO;
using RuntimeGraphDefinition = DiyFfb.GraphTest.GraphDefinition;

namespace User.PluginSdkDemo.GraphEditor
{
    public sealed class GraphPreviewEvaluator
    {
        private IGraphResolver _resolver;
        private string _baseDirectory;
        private GraphDefinition _cachedGraph;
        private RuntimeGraphDefinition _cachedRuntime;
        private GraphCompiledEvaluator _cachedEvaluator;
        private IGraphResolver _cachedResolver;
        private string _cachedBaseDirectory;
        private bool _cacheDirty = true;

        /// <summary>
        /// Enable or disable debug logging for preview evaluation.
        /// Log file location: %LocalAppData%\DiyFfb\graph_debug.log
        /// </summary>
        public bool DebugLoggingEnabled
        {
            get => GraphDebugLogger.Enabled;
            set
            {
                GraphDebugLogger.Enabled = value;
                if (value)
                {
                    GraphDebugLogger.Clear();
                }
            }
        }

        public string DebugLogPath => GraphDebugLogger.LogPath;

        public void SetResolver(IGraphResolver resolver)
        {
            _resolver = resolver;
            InvalidateCache();
        }

        public void SetBaseDirectory(string baseDirectory)
        {
            _baseDirectory = baseDirectory;
            InvalidateCache();
        }

        public void InvalidateCache()
        {
            _cacheDirty = true;
            _cachedGraph = null;
            _cachedRuntime = null;
            _cachedEvaluator = null;
            _cachedResolver = null;
            _cachedBaseDirectory = null;
        }

        public GraphEvaluationResult Evaluate(GraphDefinition graph,
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            if (graph == null)
            {
                return new GraphEvaluationResult();
            }

            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.LogSection("GraphPreviewEvaluator.Evaluate");
                GraphDebugLogger.Log($"  _baseDirectory: {_baseDirectory}");
                GraphDebugLogger.Log($"  _resolver: {(_resolver != null ? "set" : "null")}");
                GraphDebugLogger.LogDict("inputs", inputs);
                GraphDebugLogger.LogDict("parameters", parameters);
            }

            bool cacheValid = !_cacheDirty
                              && ReferenceEquals(graph, _cachedGraph)
                              && ReferenceEquals(_resolver, _cachedResolver)
                              && string.Equals(_baseDirectory ?? "", _cachedBaseDirectory ?? "", System.StringComparison.Ordinal);

            if (!cacheValid)
            {
                // Populate Include ports so OutputMap is built correctly (matches runtime path)
                if (!string.IsNullOrEmpty(_baseDirectory))
                {
                    GraphSerializer.PopulateIncludePorts(graph, _baseDirectory);
                }

                _cachedRuntime = GraphRuntimeConverter.Convert(graph);
                _cachedEvaluator = new GraphCompiledEvaluator(_cachedRuntime, _resolver, null, _baseDirectory);
                _cachedGraph = graph;
                _cachedResolver = _resolver;
                _cachedBaseDirectory = _baseDirectory;
                _cacheDirty = false;

                if (GraphDebugLogger.Enabled)
                {
                    GraphDebugLogger.Log($"  Converted to runtime format: {_cachedRuntime.Nodes.Count} nodes");
                }
            }

            var result = _cachedEvaluator.EvaluateWithTrace(inputs, parameters);

            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.Log($"  Result: {result.NodeValues.Count} node values, {result.Outputs.Count} outputs");
                if (result.Warnings.Count > 0)
                {
                    GraphDebugLogger.Log($"  Warnings: {string.Join("; ", result.Warnings)}");
                }
            }

            return result;
        }
    }
}
