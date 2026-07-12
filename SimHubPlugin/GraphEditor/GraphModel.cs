using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;

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
        /// One-tick delay (z^-1). A dedicated stateful node whose ports are drawn
        /// mirrored — INPUT on the right, OUTPUT on the left — because its output
        /// is the PREVIOUS tick's input, so a downstream value typically feeds
        /// back leftward into it. It breaks feedback cycles: the input is not a
        /// topological dependency (the runtime samples it at end-of-tick). The
        /// editor→runtime converter collapses it to a Func node with
        /// Func="unit_delay" (like the LocalSend/LocalReceive sugar). One input
        /// port ("in") and one output port ("out").
        /// </summary>
        Delay,

        /// <summary>
        /// Reads a config field value INTO the graph as a source (mirror of
        /// ConfigOut). Has one or more output ports, each bound to an
        /// OverrideFieldRegistry field path via ConfigField. The value is the
        /// current MERGED config value of that field for the scoped function,
        /// supplied by the plugin at eval time. Scoped via the parent Include's
        /// FunctionScope, exactly like ConfigOut.
        /// </summary>
        ConfigIn,

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
        Expr,

        /// <summary>
        /// Plan 23: declares custom MSFS SimConnect variables (SimVars / LVARs)
        /// on top of the fixed defaults. Each output port carries the raw datum
        /// name (SimVar) + unit (Unit) to register, and its SignalSuffix is the
        /// graph-facing alias — the port emits MSFS.&lt;alias&gt; like an Input node
        /// port. Top-level graphs only. The plugin scans these nodes to build the
        /// dynamic registration list; the runtime converter emits the ports as
        /// ordinary MSFS input signals (raw name/unit are registration-only).
        /// </summary>
        MsfsVarDef,

        /// <summary>
        /// Plan 24: writes graph values back to MSFS (write-side mirror of
        /// MsfsVarDef). Each INPUT port names a target via SimVar — prefix selects
        /// the transport (A:/L: over SetDataOnSimObject, B: over the Input Event
        /// API) — plus a Unit and an integrated linear range map
        /// (InMin/InMax → OutMin/OutMax) applied at the write boundary. Ports are
        /// input sinks (like ConfigOut): they resolve their value by wiring and
        /// feed the eval result's MsfsVarOutputs channel keyed by SignalSuffix
        /// (alias). Top-level graphs only. Registration/map metadata never enters
        /// the runtime graph.
        /// </summary>
        MsfsVarOut
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

    public sealed class GraphNode : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private void OnPropertyChanged([CallerMemberName] string name = null) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

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
        public string FunctionScope
        {
            get => _functionScope;
            set { if (_functionScope != value) { _functionScope = value ?? ""; OnPropertyChanged(); } }
        }
        private string _functionScope = "";

        /// <summary>
        /// Marks an Output node as scoped. When true, ports use SignalSuffix (dropdown)
        /// instead of freeform Name, and the output is auto-registered by the parent
        /// Include's FunctionScope. When false, the output appears as a normal port on
        /// the Include node. Only meaningful on Output nodes in library graphs.
        /// </summary>
        public bool Scoped
        {
            get => _scoped;
            set { if (_scoped != value) { _scoped = value; OnPropertyChanged(); } }
        }
        private bool _scoped;

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
        /// ConfigIn node ports in the included graph. Mirror of ConfigOutputs:
        /// they do NOT become ports on the Include node — when the Include has a
        /// FunctionScope, the converter feeds each one a scoped merged-config value.
        /// </summary>
        public List<ConfigInputPort> ConfigInputs { get; } = new List<ConfigInputPort>();

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
        /// Config field path for ConfigOut input ports / ConfigIn output ports.
        /// Stores the OverrideFieldRegistry FieldPath (e.g., "FlightControl.PosMin").
        /// Only meaningful on ConfigOut input ports and ConfigIn output ports.
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

        /// <summary>
        /// Plan 23: raw SimConnect datum name to register for MsfsVarDef output
        /// ports (e.g. "L:HELI_COLL_TRIM_TGT", "GENERAL ENG RPM:1"). Registration
        /// metadata only — never enters the runtime graph. Empty on other kinds.
        /// </summary>
        public string SimVar { get; set; } = "";

        /// <summary>
        /// Plan 23: SimConnect unit string for MsfsVarDef output ports (e.g.
        /// "number", "percent", "radians"). Registration metadata only. Empty on
        /// other kinds.
        /// </summary>
        public string Unit { get; set; } = "";

        /// <summary>
        /// Plan 24: integrated linear range map for MsfsVarOut input ports. The
        /// incoming graph value is mapped InMin/InMax → OutMin/OutMax and always
        /// clamped to the output range at the write boundary (never in graph
        /// eval). Identity default (0..1 → 0..1) = passthrough. Metadata only.
        /// </summary>
        public double InMin { get; set; } = 0.0;
        public double InMax { get; set; } = 1.0;
        public double OutMin { get; set; } = 0.0;
        public double OutMax { get; set; } = 1.0;
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

    /// <summary>
    /// Describes a ConfigIn port extracted from an included sub-graph (mirror of
    /// ConfigOutputPort). The parent Include feeds it a scoped merged-config value.
    /// </summary>
    public sealed class ConfigInputPort
    {
        /// <summary>Port name in the sub-graph (used for InputMap lookup).</summary>
        public string Name { get; set; } = "";

        /// <summary>OverrideFieldRegistry FieldPath being read (e.g., "FlightControl.PosMin").</summary>
        public string ConfigField { get; set; } = "";

        /// <summary>Config type from the ConfigIn node (e.g., "FlightStick").</summary>
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
