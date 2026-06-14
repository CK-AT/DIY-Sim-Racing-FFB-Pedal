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
        Output,
        ConfigOut,

        /// <summary>
        /// "Send" end of a graph-local named bus. Has one input port; the value
        /// flowing in is published on the bus name (LocalBusName). One Send per
        /// name per graph. Runtime: collapsed into direct wiring by the
        /// editor→runtime converter (no runtime representation needed).
        /// </summary>
        LocalSend,

        /// <summary>
        /// "Receive" end of a graph-local named bus. Has one output port; emits
        /// the value of the matching LocalSend (by LocalBusName). Any number per
        /// graph. Orphan receives (no matching Send) evaluate to 0.
        /// </summary>
        LocalReceive,

        /// <summary>
        /// Evaluates a user-authored math formula (NCalc syntax). Has one output
        /// port and any number of named input ports; the formula may only reference
        /// those input port names as variables (built-in functions like Pow/Abs/if
        /// are allowed). The port name is the identifier used in the formula.
        /// </summary>
        Expr
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
        /// Formula text for Expr nodes (NCalc syntax). Variables must match this
        /// node's input port names.
        /// </summary>
        public string Expr { get; set; } = "";

        /// <summary>
        /// Signal group for Input/Output/Param nodes (e.g., "XPlane", "FlightStickPitch", "Aircraft").
        /// </summary>
        public string SignalGroup { get; set; } = "";

        /// <summary>
        /// Function scope for Include nodes. When set, scoped Output and ConfigOut nodes
        /// inside the included sub-graph are auto-registered using this as their function group.
        /// E.g., "FlightStickPitch", "FlightStickRoll", "FlightPedals".
        /// Empty string means unscoped (default, backward-compatible behavior).
        /// </summary>
        public string FunctionScope { get; set; } = "";

        /// <summary>
        /// Marks an Output node as scoped. When true, ports use SignalSuffix (dropdown)
        /// instead of freeform Name, and the output is auto-registered by the parent
        /// Include's FunctionScope. When false, the output appears as a normal port on
        /// the Include node. Only meaningful on Output nodes in library graphs.
        /// </summary>
        public bool Scoped { get; set; }

        /// <summary>
        /// Config type for ConfigOut nodes. Determines which OverrideFieldRegistry fields
        /// are available in the dropdown (e.g., "FlightStick" or "FlightPedals").
        /// Must match the function type implied by the parent Include's FunctionScope.
        /// </summary>
        public string ConfigType { get; set; } = "";

        /// <summary>
        /// Embedded sub-graph definition for Include nodes. When non-null the
        /// Include is *embedded* (self-contained in the parent, no file) rather
        /// than referencing IncludePath. Mutually exclusive with IncludePath:
        /// an embedded include has InlineGraph set and IncludePath empty.
        /// Ports are derived from this graph's Input/Output nodes, same as a
        /// file include. Nesting is allowed (recursion).
        /// </summary>
        public GraphDefinition InlineGraph { get; set; }

        /// <summary>True if this Include node is embedded (inline) rather than file-backed.</summary>
        public bool IsEmbeddedInclude => Kind == GraphNodeKind.Include && InlineGraph != null;

        /// <summary>
        /// Optional display order for an Include node's input/output ports, as a
        /// list of port names. Include ports are derived from the sub-graph each
        /// load, so this override (applied after derivation) lets the parent fix a
        /// port order for tidy wiring independent of the sub-graph's node order.
        /// Purely cosmetic — links/maps are name-keyed. Null = derived order.
        /// Unknown names are ignored; new ports append in derived order.
        /// </summary>
        public List<string> InputPortOrder { get; set; }
        public List<string> OutputPortOrder { get; set; }

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
        /// Names of unscoped Output node ports (provided outputs).
        /// These become output ports on the Include node.
        /// </summary>
        public List<string> Outputs { get; } = new List<string>();

        /// <summary>
        /// Scoped Output node ports. These do NOT become ports on the Include node —
        /// they are auto-registered by the converter when FunctionScope is set.
        /// Name is the internal port name (for OutputMap), SignalSuffix is the catalog suffix.
        /// </summary>
        public List<ScopedOutputPort> ScopedOutputs { get; } = new List<ScopedOutputPort>();

        /// <summary>
        /// ConfigOut node ports in the included graph.
        /// These do NOT become ports on the Include node — they are only used
        /// when the Include has a FunctionScope, to register scoped config outputs.
        /// </summary>
        public List<ConfigOutputPort> ConfigOutputs { get; } = new List<ConfigOutputPort>();

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

        /// <summary>
        /// Config field path for ConfigOut input ports. Stores the OverrideFieldRegistry
        /// FieldPath (e.g., "flight_stick.damping", "flight_stick.vib_harmonic_ratios.0").
        /// Only meaningful on ConfigOut node input ports.
        /// </summary>
        public string ConfigField { get; set; } = "";

        /// <summary>
        /// Graph-local bus name for LocalSend/LocalReceive ports. Each port on a
        /// Send/Receive node has its own bus name, allowing one node to carry
        /// multiple buses. One Send port per bus name per graph; any number of
        /// Receive ports may match a Send by this string. Empty on all other
        /// port kinds.
        /// </summary>
        public string BusName { get; set; } = "";
    }

    /// <summary>
    /// Describes a scoped Output port extracted from an included sub-graph.
    /// </summary>
    public sealed class ScopedOutputPort
    {
        /// <summary>Port name in the sub-graph (used for OutputMap lookup).</summary>
        public string Name { get; set; } = "";

        /// <summary>Signal suffix (e.g., "SpringGain") used to build the scoped name.</summary>
        public string SignalSuffix { get; set; } = "";
    }

    /// <summary>
    /// Describes a ConfigOut port extracted from an included sub-graph.
    /// </summary>
    public sealed class ConfigOutputPort
    {
        /// <summary>Port name in the sub-graph (used for OutputMap lookup).</summary>
        public string Name { get; set; } = "";

        /// <summary>OverrideFieldRegistry FieldPath (e.g., "flight_stick.damping").</summary>
        public string ConfigField { get; set; } = "";

        /// <summary>Config type from the ConfigOut node (e.g., "FlightStick").</summary>
        public string ConfigType { get; set; } = "";
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

        /// <summary>
        /// Optional "muted" value. When set, the editor renders a mute checkbox
        /// next to the slider; toggling it on substitutes this value for the
        /// param's normal value in the runtime parameters dict. Used for the
        /// tuning-workflow "solo a cue" pattern. Null = no mute available.
        /// </summary>
        public double? MuteValue { get; set; }
    }

    public sealed class GraphParamOption
    {
        public string Value { get; set; } = "";
        public string Label { get; set; } = "";
    }
}
