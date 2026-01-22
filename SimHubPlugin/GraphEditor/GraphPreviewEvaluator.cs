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
}
