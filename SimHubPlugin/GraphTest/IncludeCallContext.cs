using System.Collections.Generic;

namespace DiyFfb.GraphTest
{
    /// <summary>
    /// Captures the inputs and parameters passed to an Include node during evaluation.
    /// Used to enable sub-graph previews with real parent context.
    /// </summary>
    public sealed class IncludeCallContext
    {
        public string IncludeNodeId { get; set; }
        public string IncludeNodeTitle { get; set; }
        public string IncludePath { get; set; }  // Resolved absolute path
        public IReadOnlyDictionary<string, double> Inputs { get; set; }
        public IReadOnlyDictionary<string, double> Parameters { get; set; }
        /// <summary>
        /// State snapshot from the sub-graph evaluator at capture time.
        /// Allows the preview evaluator to sync stateful nodes (accumulators, sample_holds).
        /// </summary>
        public Dictionary<string, double[]> StateSnapshot { get; set; }
    }
}
