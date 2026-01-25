using System;
using System.ComponentModel;
using System.IO;
using System.Runtime.CompilerServices;

namespace User.PluginSdkDemo.GraphEditor
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

        public GraphEditorTab()
        {
            Id = Guid.NewGuid().ToString("N");
            EditorControl = new GraphEditorControl();
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
                string name = string.IsNullOrWhiteSpace(filePath)
                    ? "Untitled"
                    : Path.GetFileName(filePath);

                if (isActiveGraph)
                {
                    return $"Active: {name}";
                }

                return name;
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
        /// The graph definition for this tab. Getter returns the live graph from the EditorControl.
        /// </summary>
        public GraphDefinition Graph
        {
            get => EditorControl.GetGraph();
            set
            {
                var graphToSet = value ?? new GraphDefinition();
                EditorControl.SetGraph(graphToSet);
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
                string json = GraphSerializer.Serialize(Graph);
                File.WriteAllText(path, json);
                FilePath = path;
                IsDirty = false;
                return true;
            }
            catch
            {
                return false;
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
