using System;
using System.Collections.Generic;

namespace DiyFfb.GraphEditor
{
    public enum GraphNodeKind
    {
        Input,
        Param,
        Const,
        Op,
        Func,
        Include,
        Output
    }

    public enum GraphPortKind
    {
        Input,
        Output
    }

    public sealed class GraphDefinition
    {
        public int Version { get; set; } = 1;
        public List<GraphNode> Nodes { get; } = new List<GraphNode>();
        public List<GraphLink> Links { get; } = new List<GraphLink>();
        public Dictionary<string, GraphParam> Params { get; } = new Dictionary<string, GraphParam>();
        public Dictionary<string, double> ParamValues { get; set; } = new Dictionary<string, double>();

        /// <summary>
        /// When true, this graph is a reusable library block. Input/Output nodes use freeform
        /// port names instead of binding to the signal catalog. Included graphs should set this.
        /// </summary>
        public bool IsLibraryGraph { get; set; }
    }

    public sealed class GraphNode
    {
        public string Id { get; set; } = Guid.NewGuid().ToString("N");
        public string Title { get; set; } = "";
        public GraphNodeKind Kind { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public List<GraphPort> Ports { get; } = new List<GraphPort>();

        public string Op { get; set; } = "";
        public string Func { get; set; } = "";
        public string IncludePath { get; set; } = "";
        public double ConstValue { get; set; }

        /// <summary>
        /// Signal group for Input/Output/Param nodes (e.g., "XPlane", "FlightStickPitch", "Aircraft").
        /// </summary>
        public string SignalGroup { get; set; } = "";

        /// <summary>
        /// Cached interface from the included graph. Not serialized.
        /// Populated by SyncIncludePorts() when IncludePath changes.
        /// </summary>
        public IncludedGraphInterface CachedInterface { get; set; }
    }

    /// <summary>
    /// Describes the interface (inputs/outputs) of an included graph.
    /// Extracted from the included graph's Input/Output nodes.
    /// </summary>
    public sealed class IncludedGraphInterface
    {
        /// <summary>
        /// Names of Input node ports in the included graph (expected inputs).
        /// These become input ports on the Include node.
        /// </summary>
        public List<string> Inputs { get; } = new List<string>();

        /// <summary>
        /// Names of Output node ports in the included graph (provided outputs).
        /// These become output ports on the Include node.
        /// </summary>
        public List<string> Outputs { get; } = new List<string>();

        /// <summary>
        /// True if the interface was successfully extracted.
        /// </summary>
        public bool IsValid { get; set; }

        /// <summary>
        /// Error message if IsValid is false.
        /// </summary>
        public string Error { get; set; } = "";
    }

    public sealed class GraphPort
    {
        public string Name { get; set; } = "";
        public GraphPortKind Kind { get; set; }

        /// <summary>
        /// Signal suffix for Input/Output ports (e.g., "IAS_kts", "SpringGain").
        /// Combined with node's SignalGroup to form full signal name.
        /// For Param nodes, this is freeform.
        /// </summary>
        public string SignalSuffix { get; set; } = "";

        /// <summary>
        /// Per-input negation flag for Op nodes (only meaningful on Op input ports).
        /// </summary>
        public bool Negate { get; set; }
    }

    public sealed class GraphLink
    {
        public string FromNodeId { get; set; } = "";
        public string FromPort { get; set; } = "";
        public string ToNodeId { get; set; } = "";
        public string ToPort { get; set; } = "";
    }

    public sealed class GraphParam
    {
        public string Name { get; set; } = "";
        public double DefaultValue { get; set; }
        public double Min { get; set; }
        public double Max { get; set; }
        public GraphParamUi Ui { get; set; }
    }

    public sealed class GraphParamUi
    {
        public string Widget { get; set; } = "";
        public string Label { get; set; } = "";
        public string Group { get; set; } = "";
        public string Units { get; set; } = "";
        public double? Step { get; set; }
        public int? Precision { get; set; }
        public bool LogScale { get; set; }
        public List<GraphParamOption> Options { get; } = new List<GraphParamOption>();
    }

    public sealed class GraphParamOption
    {
        public string Value { get; set; } = "";
        public string Label { get; set; } = "";
    }
}
