using System;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;

namespace DiyFfb.GraphEditor
{
    /// <summary>
    /// Represents a single tab in the graph editor, containing its own graph and editor control instance.
    /// </summary>
    public sealed class GraphEditorTab : INotifyPropertyChanged
    {
        private string filePath;
        private bool isDirty;
        private bool isPinned;
        private bool isActiveGraph;
        private string contextSuffix;
        private GraphEditorTab parentTab;
        private string embeddedNodeId;
        private string embeddedNodeTitle;

        public GraphEditorTab()
        {
            Id = Guid.NewGuid().ToString("N");
            EditorControl = new GraphEditorControl();
            UndoStack = new GraphUndoStack();
            EditorControl.SetUndoStack(UndoStack);
            Graph = new GraphDefinition();
        }

        /// <summary>
        /// Unique identifier for this tab.
        /// </summary>
        public string Id { get; }

        /// <summary>
        /// The editor control instance for this tab.
        /// </summary>
        public GraphEditorControl EditorControl { get; }
        public GraphUndoStack UndoStack { get; }

        /// <summary>
        /// Absolute file path for this graph, or null if unsaved.
        /// </summary>
        public string FilePath
        {
            get => filePath;
            set
            {
                if (filePath != value)
                {
                    filePath = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(DisplayName));
                }
            }
        }

        /// <summary>
        /// Display name shown in the tab header.
        /// </summary>
        public string DisplayName
        {
            get
            {
                // Embedded sub-graph tabs show their chain: parent/node[/node...]
                if (IsEmbedded)
                {
                    return ChainTitle;
                }

                string name = string.IsNullOrWhiteSpace(filePath)
                    ? "Untitled"
                    : Path.GetFileName(filePath);

                if (isActiveGraph)
                {
                    return $"Active: {name}";
                }

                // Append context suffix if previewing with parent context
                if (!string.IsNullOrEmpty(contextSuffix))
                {
                    return name + contextSuffix;
                }

                return name;
            }
        }

        /// <summary>
        /// The parent tab whose graph contains this embedded sub-graph's Include node,
        /// or null for normal file/active tabs.
        /// </summary>
        public GraphEditorTab ParentTab => parentTab;

        /// <summary>Id of the embedded Include node in the parent graph (embedded tabs only).</summary>
        public string EmbeddedNodeId => embeddedNodeId;

        /// <summary>True if this tab edits an embedded (inline, path-less) sub-graph.</summary>
        public bool IsEmbedded => parentTab != null && !string.IsNullOrEmpty(embeddedNodeId);

        /// <summary>
        /// Title chain for the tab header. Root is the parent file name; embedded
        /// tabs append "/nodeName", so nested embeds read "file/outer/inner".
        /// </summary>
        public string ChainTitle
        {
            get
            {
                if (IsEmbedded)
                {
                    string parentChain = parentTab?.ChainTitle ?? "?";
                    string leaf = string.IsNullOrEmpty(embeddedNodeTitle) ? embeddedNodeId : embeddedNodeTitle;
                    return parentChain + "/" + leaf;
                }
                return string.IsNullOrWhiteSpace(filePath) ? "Untitled" : Path.GetFileNameWithoutExtension(filePath);
            }
        }

        /// <summary>
        /// Whether this tab has unsaved changes.
        /// </summary>
        public bool IsDirty
        {
            get => isDirty;
            set
            {
                if (isDirty != value)
                {
                    isDirty = value;
                    OnPropertyChanged();
                }
            }
        }

        /// <summary>
        /// Whether this tab is pinned (non-closeable). True for active graph tab.
        /// </summary>
        public bool IsPinned
        {
            get => isPinned;
            set
            {
                if (isPinned != value)
                {
                    isPinned = value;
                    OnPropertyChanged();
                }
            }
        }

        /// <summary>
        /// Whether this tab represents the active vehicle graph.
        /// </summary>
        public bool IsActiveGraph
        {
            get => isActiveGraph;
            set
            {
                if (isActiveGraph != value)
                {
                    isActiveGraph = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(DisplayName));
                }
            }
        }

        /// <summary>
        /// Optional suffix shown after the filename when previewing with parent context.
        /// E.g., " (via MyInclude)"
        /// </summary>
        public string ContextSuffix
        {
            get => contextSuffix;
            set
            {
                if (contextSuffix != value)
                {
                    contextSuffix = value;
                    OnPropertyChanged();
                    OnPropertyChanged(nameof(DisplayName));
                }
            }
        }

        /// <summary>
        /// The graph definition for this tab. Getter returns the live graph from the EditorControl.
        /// </summary>
        public GraphDefinition Graph
        {
            get => EditorControl.GetGraph();
            set
            {
                var graphToSet = value ?? new GraphDefinition();
                EditorControl.SetGraph(graphToSet);
                EditorControl.InitializeUndoStack();
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Gets the directory containing this graph file, or null if unsaved.
        /// </summary>
        public string BaseDirectory => string.IsNullOrWhiteSpace(filePath)
            ? null
            : Path.GetDirectoryName(filePath);

        /// <summary>
        /// Loads a graph from the specified file path.
        /// </summary>
        public bool LoadFromFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path) || !File.Exists(path))
            {
                return false;
            }

            try
            {
                string json = File.ReadAllText(path);
                var loadedGraph = GraphSerializer.Deserialize(json, out _);
                FilePath = path;
                EditorControl.BaseDirectory = BaseDirectory;  // Must be set BEFORE Graph (SetGraph triggers SyncIncludePorts)
                EditorControl.FilePath = path;  // Set file path for context lookup
                Graph = loadedGraph;
                IsDirty = false;
                return true;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Loads an embedded sub-graph into this tab. The tab edits an independent
        /// clone; FlushToParent() writes it back into the parent node's InlineGraph.
        /// Base directory / file path mirror the parent so nested file-includes
        /// inside the embedded graph resolve relative to the parent's location.
        /// </summary>
        public void LoadFromEmbedded(GraphEditorTab parent, GraphNode node)
        {
            if (parent == null || node == null)
            {
                return;
            }

            parentTab = parent;
            embeddedNodeId = node.Id;
            embeddedNodeTitle = string.IsNullOrWhiteSpace(node.Title) ? node.Id : node.Title;

            EditorControl.BaseDirectory = parent.BaseDirectory;
            EditorControl.FilePath = parent.FilePath;
            // Live-preview context for an embedded sub-graph is keyed by the
            // include node id, not a file path (it has none).
            EditorControl.ContextKeyOverride = "inline:" + node.Id;

            var source = node.InlineGraph ?? new GraphDefinition { IsLibraryGraph = true };
            Graph = CloneGraph(source);
            IsDirty = false;
            OnPropertyChanged(nameof(DisplayName));
        }

        /// <summary>
        /// Writes this embedded tab's current graph back into the parent node's
        /// InlineGraph and marks the parent dirty. The embedded sub-graph has no
        /// file of its own — saving the parent file persists it. No-op if the
        /// parent node no longer exists (e.g. removed in the parent).
        /// </summary>
        public void FlushToParent()
        {
            if (!IsEmbedded || parentTab?.Graph == null)
            {
                return;
            }

            var node = parentTab.Graph.Nodes.FirstOrDefault(n => n.Id == embeddedNodeId);
            if (node == null)
            {
                return;
            }

            node.InlineGraph = Graph;
            parentTab.IsDirty = true;
        }

        private static GraphDefinition CloneGraph(GraphDefinition g)
        {
            try
            {
                return GraphSerializer.Deserialize(GraphSerializer.Serialize(g), out _);
            }
            catch
            {
                return new GraphDefinition { IsLibraryGraph = true };
            }
        }

        /// <summary>
        /// Saves the graph to the current file path.
        /// </summary>
        public bool Save()
        {
            if (string.IsNullOrWhiteSpace(filePath))
            {
                return false;
            }

            return SaveAs(filePath);
        }

        /// <summary>
        /// Saves the graph to the specified file path.
        /// </summary>
        public bool SaveAs(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return false;
            }

            try
            {
                // Adjust Include paths when saving to a new location
                string oldBase = BaseDirectory;
                string newBase = Path.GetDirectoryName(path);
                AdjustIncludePaths(oldBase, newBase);

                string json = GraphSerializer.Serialize(Graph);
                File.WriteAllText(path, json);
                FilePath = path;
                EditorControl.FilePath = path;  // Update file path for context lookup
                EditorControl.BaseDirectory = BaseDirectory;  // Update base directory too
                IsDirty = false;
                return true;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Adjusts all Include node paths when saving to a new location.
        /// Makes paths absolute (using old base), then relative to new base.
        /// </summary>
        private void AdjustIncludePaths(string oldBase, string newBase)
        {
            if (Graph?.Nodes == null)
            {
                return;
            }

            foreach (var node in Graph.Nodes)
            {
                if (node.Kind != GraphNodeKind.Include || string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    continue;
                }

                // Step 1: Make absolute (if relative and old base exists)
                string absolutePath = node.IncludePath;
                if (!Path.IsPathRooted(absolutePath) && !string.IsNullOrWhiteSpace(oldBase))
                {
                    try
                    {
                        absolutePath = Path.GetFullPath(Path.Combine(oldBase, absolutePath));
                    }
                    catch
                    {
                        // Keep original if resolution fails
                    }
                }

                // Step 2: Make relative to new base
                if (!string.IsNullOrWhiteSpace(newBase))
                {
                    node.IncludePath = MakeRelativePath(newBase, absolutePath);
                }
                else
                {
                    // No new base - keep absolute
                    node.IncludePath = absolutePath;
                }
            }
        }

        /// <summary>
        /// Creates a relative path from a base directory to a target path.
        /// </summary>
        private static string MakeRelativePath(string baseDir, string fullPath)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(baseDir) || string.IsNullOrWhiteSpace(fullPath))
                {
                    return fullPath;
                }

                var baseUri = new Uri(AppendDirectorySeparator(Path.GetFullPath(baseDir)));
                var fullUri = new Uri(Path.GetFullPath(fullPath));
                if (baseUri.Scheme != fullUri.Scheme)
                {
                    return fullPath;
                }

                string relative = Uri.UnescapeDataString(baseUri.MakeRelativeUri(fullUri).ToString());
                return relative.Replace('/', Path.DirectorySeparatorChar);
            }
            catch
            {
                return fullPath;
            }
        }

        private static string AppendDirectorySeparator(string path)
        {
            if (string.IsNullOrEmpty(path))
            {
                return path;
            }

            char lastChar = path[path.Length - 1];
            if (lastChar == Path.DirectorySeparatorChar || lastChar == Path.AltDirectorySeparatorChar)
            {
                return path;
            }

            return path + Path.DirectorySeparatorChar;
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
