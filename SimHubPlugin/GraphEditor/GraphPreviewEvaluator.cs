using DiyFfb.GraphTest;
using System.Collections.Generic;

namespace User.PluginSdkDemo.GraphEditor
{
    public sealed class GraphPreviewEvaluator
    {
        private IGraphResolver _resolver;

        public void SetResolver(IGraphResolver resolver)
        {
            _resolver = resolver;
        }

        public GraphEvaluationResult Evaluate(GraphDefinition graph,
            IReadOnlyDictionary<string, double> inputs,
            IReadOnlyDictionary<string, double> parameters)
        {
            if (graph == null)
            {
                return new GraphEvaluationResult();
            }

            var runtime = GraphRuntimeConverter.Convert(graph);
            var evaluator = new GraphCompiledEvaluator(runtime, _resolver);
            return evaluator.EvaluateWithTrace(inputs, parameters);
        }
    }
}
