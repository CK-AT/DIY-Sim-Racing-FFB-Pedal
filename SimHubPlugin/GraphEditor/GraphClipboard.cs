using System.Collections.Generic;

namespace DiyFfb.GraphEditor
{
    /// <summary>
    /// Holds copied graph content for clipboard operations.
    /// Contains nodes, their interconnecting links, and positioning metadata.
    /// </summary>
    public sealed class GraphClipboardData
    {
        /// <summary>
        /// Custom clipboard format identifier for graph data.
        /// </summary>
        public const string ClipboardFormat = "DiyFfbGraphClipboard";

        /// <summary>
        /// Copied nodes with original IDs (remapped on paste).
        /// </summary>
        public List<GraphNode> Nodes { get; set; } = new List<GraphNode>();

        /// <summary>
        /// Links between copied nodes (only internal links where both endpoints are in selection).
        /// </summary>
        public List<GraphLink> Links { get; set; } = new List<GraphLink>();

        /// <summary>
        /// Parameter definitions for copied Param nodes.
        /// Key is parameter name, value is the full GraphParam definition.
        /// </summary>
        public Dictionary<string, GraphParam> Params { get; set; } = new Dictionary<string, GraphParam>();

        /// <summary>
        /// X coordinate of selection center (for paste offset calculation).
        /// </summary>
        public double CenterX { get; set; }

        /// <summary>
        /// Y coordinate of selection center (for paste offset calculation).
        /// </summary>
        public double CenterY { get; set; }
    }
}
