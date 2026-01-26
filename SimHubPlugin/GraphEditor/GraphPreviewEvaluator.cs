using DiyFfb.GraphTest;
using System.Collections.Generic;
using System.IO;

namespace User.PluginSdkDemo.GraphEditor
{
    public sealed class GraphPreviewEvaluator
    {
        private IGraphResolver _resolver;
        private string _baseDirectory;

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
        }

        public void SetBaseDirectory(string baseDirectory)
        {
            _baseDirectory = baseDirectory;
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

            // Populate Include ports so OutputMap is built correctly (matches runtime path)
            if (!string.IsNullOrEmpty(_baseDirectory))
            {
                GraphSerializer.PopulateIncludePorts(graph, _baseDirectory);
            }

            var runtime = GraphRuntimeConverter.Convert(graph);

            if (GraphDebugLogger.Enabled)
            {
                GraphDebugLogger.Log($"  Converted to runtime format: {runtime.Nodes.Count} nodes");
            }

            // Pass the base directory so nested includes can resolve relative paths
            var evaluator = new GraphCompiledEvaluator(runtime, _resolver, null, _baseDirectory);
            var result = evaluator.EvaluateWithTrace(inputs, parameters);

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
