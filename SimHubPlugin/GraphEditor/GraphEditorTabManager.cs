using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;

namespace User.PluginSdkDemo.GraphEditor
{
    /// <summary>
    /// Manages multiple graph editor tabs, including the pinned active graph tab.
    /// </summary>
    public sealed class GraphEditorTabManager : INotifyPropertyChanged
    {
        private GraphEditorTab selectedTab;
        private GraphEditorTab activeGraphTab;
        private readonly Dictionary<string, GraphDefinition> includeCache = new Dictionary<string, GraphDefinition>(StringComparer.OrdinalIgnoreCase);

        public GraphEditorTabManager()
        {
            Tabs = new ObservableCollection<GraphEditorTab>();
        }

        /// <summary>
        /// Collection of all open tabs.
        /// </summary>
        public ObservableCollection<GraphEditorTab> Tabs { get; }

        /// <summary>
        /// The currently selected (visible) tab.
        /// </summary>
        public GraphEditorTab SelectedTab
        {
            get => selectedTab;
            set
            {
                if (selectedTab != value)
                {
                    selectedTab = value;
                    OnPropertyChanged();
                    SelectedTabChanged?.Invoke(this, EventArgs.Empty);
                }
            }
        }

        /// <summary>
        /// The pinned active graph tab (auto-loaded from vehicle selection).
        /// </summary>
        public GraphEditorTab ActiveGraphTab => activeGraphTab;

        /// <summary>
        /// Shared include cache across all tabs.
        /// </summary>
        public Dictionary<string, GraphDefinition> IncludeCache => includeCache;

        /// <summary>
        /// Event raised when the selected tab changes.
        /// </summary>
        public event EventHandler SelectedTabChanged;

        /// <summary>
        /// Event raised when a tab is added.
        /// </summary>
        public event EventHandler<GraphEditorTab> TabAdded;

        /// <summary>
        /// Event raised when a tab is removed.
        /// </summary>
        public event EventHandler<GraphEditorTab> TabRemoved;

        /// <summary>
        /// Creates the pinned active graph tab. Called once during initialization.
        /// </summary>
        public GraphEditorTab CreateActiveGraphTab()
        {
            if (activeGraphTab != null)
            {
                return activeGraphTab;
            }

            activeGraphTab = new GraphEditorTab
            {
                IsPinned = true,
                IsActiveGraph = true
            };

            // Initialize with default graph so it's not empty
            activeGraphTab.Graph = BuildDefaultGraph();

            Tabs.Insert(0, activeGraphTab);
            SelectedTab = activeGraphTab;
            TabAdded?.Invoke(this, activeGraphTab);
            return activeGraphTab;
        }

        /// <summary>
        /// Sets the active graph from a file path. Called when vehicle selection changes.
        /// </summary>
        public void SetActiveGraph(string path)
        {
            if (activeGraphTab == null)
            {
                CreateActiveGraphTab();
            }

            if (string.IsNullOrWhiteSpace(path))
            {
                activeGraphTab.FilePath = null;
                activeGraphTab.Graph = BuildDefaultGraph();
                activeGraphTab.IsDirty = false;
                return;
            }

            string absolutePath = path;
            if (!Path.IsPathRooted(path))
            {
                absolutePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, path);
            }

            if (!File.Exists(absolutePath))
            {
                activeGraphTab.FilePath = null;
                activeGraphTab.Graph = BuildDefaultGraph();
                activeGraphTab.IsDirty = false;
                return;
            }

            activeGraphTab.LoadFromFile(absolutePath);
        }

        /// <summary>
        /// Opens a graph from a file path, or switches to existing tab if already open.
        /// </summary>
        public GraphEditorTab OpenGraph(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return null;
            }

            string absolutePath = path;
            if (!Path.IsPathRooted(path))
            {
                absolutePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, path);
            }

            // Check if already open
            var existing = FindTabByPath(absolutePath);
            if (existing != null)
            {
                SelectedTab = existing;
                return existing;
            }

            // Create new tab
            var tab = new GraphEditorTab();
            if (!tab.LoadFromFile(absolutePath))
            {
                return null;
            }

            Tabs.Add(tab);
            SelectedTab = tab;
            TabAdded?.Invoke(this, tab);
            return tab;
        }

        /// <summary>
        /// Opens a graph resolved relative to the specified base directory.
        /// </summary>
        public GraphEditorTab OpenGraphRelative(string relativePath, string baseDirectory)
        {
            if (string.IsNullOrWhiteSpace(relativePath))
            {
                return null;
            }

            string absolutePath = relativePath;
            if (!Path.IsPathRooted(relativePath) && !string.IsNullOrWhiteSpace(baseDirectory))
            {
                absolutePath = Path.Combine(baseDirectory, relativePath);
            }

            return OpenGraph(absolutePath);
        }

        /// <summary>
        /// Creates a new untitled tab.
        /// </summary>
        public GraphEditorTab CreateNewTab()
        {
            var tab = new GraphEditorTab
            {
                Graph = BuildDefaultGraph()
            };

            Tabs.Add(tab);
            SelectedTab = tab;
            TabAdded?.Invoke(this, tab);
            return tab;
        }

        /// <summary>
        /// Closes a tab. Pinned tabs cannot be closed.
        /// </summary>
        /// <returns>True if the tab was closed, false if cancelled or pinned.</returns>
        public bool CloseTab(GraphEditorTab tab)
        {
            if (tab == null || tab.IsPinned)
            {
                return false;
            }

            int index = Tabs.IndexOf(tab);
            if (index < 0)
            {
                return false;
            }

            Tabs.RemoveAt(index);
            TabRemoved?.Invoke(this, tab);

            // Select adjacent tab
            if (SelectedTab == tab)
            {
                if (index > 0)
                {
                    SelectedTab = Tabs[index - 1];
                }
                else if (Tabs.Count > 0)
                {
                    SelectedTab = Tabs[0];
                }
                else
                {
                    SelectedTab = null;
                }
            }

            return true;
        }

        /// <summary>
        /// Finds an existing tab by its absolute file path.
        /// </summary>
        public GraphEditorTab FindTabByPath(string absolutePath)
        {
            if (string.IsNullOrWhiteSpace(absolutePath))
            {
                return null;
            }

            return Tabs.FirstOrDefault(t =>
                !string.IsNullOrWhiteSpace(t.FilePath) &&
                string.Equals(t.FilePath, absolutePath, StringComparison.OrdinalIgnoreCase));
        }

        /// <summary>
        /// Checks if any tabs have unsaved changes.
        /// </summary>
        public bool HasUnsavedChanges()
        {
            return Tabs.Any(t => t.IsDirty);
        }

        /// <summary>
        /// Gets all tabs with unsaved changes.
        /// </summary>
        public IEnumerable<GraphEditorTab> GetDirtyTabs()
        {
            return Tabs.Where(t => t.IsDirty);
        }

        /// <summary>
        /// Clears the shared include cache.
        /// </summary>
        public void ClearIncludeCache()
        {
            includeCache.Clear();
        }

        /// <summary>
        /// Loads an include graph from cache or disk.
        /// </summary>
        public GraphDefinition LoadInclude(string path, string baseDirectory)
        {
            string resolved = ResolvePath(path, baseDirectory);
            if (resolved == null)
            {
                return null;
            }

            if (includeCache.TryGetValue(resolved, out var cached))
            {
                return cached;
            }

            if (!File.Exists(resolved))
            {
                return null;
            }

            try
            {
                string json = File.ReadAllText(resolved);
                var graph = GraphSerializer.Deserialize(json, out _);
                includeCache[resolved] = graph;
                return graph;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// Resolves a path relative to a base directory.
        /// </summary>
        public static string ResolvePath(string path, string baseDirectory)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return null;
            }

            if (Path.IsPathRooted(path))
            {
                return path;
            }

            if (!string.IsNullOrWhiteSpace(baseDirectory))
            {
                return Path.Combine(baseDirectory, path);
            }

            return path;
        }

        private static GraphDefinition BuildDefaultGraph()
        {
            var graph = new GraphDefinition();
            var input = new GraphNode
            {
                Title = "Input",
                Kind = GraphNodeKind.Input,
                X = 40,
                Y = 40
            };
            input.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            graph.Nodes.Add(input);

            var output = new GraphNode
            {
                Title = "Output",
                Kind = GraphNodeKind.Output,
                X = 320,
                Y = 40
            };
            output.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
            graph.Nodes.Add(output);

            graph.Links.Add(new GraphLink
            {
                FromNodeId = input.Id,
                FromPort = "out",
                ToNodeId = output.Id,
                ToPort = "in"
            });

            return graph;
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
