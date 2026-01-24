using System;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using Microsoft.Win32;

namespace User.PluginSdkDemo.GraphEditor
{
    public partial class GraphEditorWindow : Window
    {
        private GraphDefinition rootGraph;
        private string rootGraphPath;
        private string currentGraphPath;
        private readonly Dictionary<string, GraphDefinition> includeCache = new Dictionary<string, GraphDefinition>();
        private bool suppressTreeSelection;
        private DiyFfbPlugin plugin;

        public GraphEditorWindow()
        {
            InitializeComponent();
            GraphEditor.SetGraph(BuildDefaultGraph());
            rootGraph = GraphEditor.GetGraph();
            currentGraphPath = null;
            GraphEditor.IncludeOpenRequested += OnIncludeOpenRequested;
            GraphEditor.GraphChanged += RefreshHierarchy;
            GraphEditor.GraphChanged += OnGraphChanged;
            GraphEditor.BaseDirectory = GetRootDirectory();
            RefreshHierarchy();
        }

        public void SetLiveInputProvider(Func<IDictionary<string, double>> provider)
        {
            GraphEditor.LiveInputProvider = provider;
        }

        public void SetPlugin(DiyFfbPlugin pluginInstance)
        {
            // Unsubscribe from old plugin if any
            if (plugin != null)
            {
                plugin.GraphParamChanged -= OnPluginGraphParamChanged;
            }

            plugin = pluginInstance;

            // Subscribe to new plugin parameter changes
            if (plugin != null)
            {
                plugin.GraphParamChanged += OnPluginGraphParamChanged;
            }

            // Wire up graph editor parameter changes to plugin
            GraphEditor.ParamValueChanged = (paramName, value) =>
            {
                plugin?.SetGraphParamValue(paramName, value);
            };
        }

        private void OnPluginGraphParamChanged(object sender, GraphParamChangedEventArgs e)
        {
            // Update graph editor when plugin parameters change externally
            Dispatcher.Invoke(() =>
            {
                GraphEditor.UpdateParamValue(e.ParamName, e.Value);
            });
        }

        private void OnGraphChanged()
        {
            // Notify plugin that graph content changed (for parameter UI refresh)
            plugin?.OnGraphContentChanged();
        }

        public void LoadGraphFromPath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            // Convert to absolute path if needed
            string absolutePath = path;
            if (!Path.IsPathRooted(path))
            {
                absolutePath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, path);
            }

            if (!File.Exists(absolutePath))
            {
                return;
            }

            rootGraphPath = absolutePath;
            GraphEditor.LoadGraphFromFile(absolutePath);
            rootGraph = GraphEditor.GetGraph();
            currentGraphPath = null;
            GraphEditor.BaseDirectory = GetRootDirectory();
            RefreshHierarchy();
        }

        private void ButtonLoad_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new OpenFileDialog
            {
                Filter = "Graph JSON (*.json)|*.json",
                DefaultExt = "json"
            };

            if (dialog.ShowDialog() == true)
            {
                rootGraphPath = dialog.FileName;
                GraphEditor.LoadGraphFromFile(dialog.FileName);
                rootGraph = GraphEditor.GetGraph();
                currentGraphPath = null;
                GraphEditor.BaseDirectory = GetRootDirectory();
                RefreshHierarchy();
            }
        }

        private void ButtonSave_Click(object sender, RoutedEventArgs e)
        {
            var dialog = new SaveFileDialog
            {
                Filter = "Graph JSON (*.json)|*.json",
                DefaultExt = "json",
                FileName = "ffb_graph.json"
            };

            if (dialog.ShowDialog() == true)
            {
                if (string.IsNullOrWhiteSpace(rootGraphPath))
                {
                    rootGraphPath = dialog.FileName;
                }
                GraphEditor.SaveGraphToFile(dialog.FileName);
            }
        }

        private void ButtonReset_Click(object sender, RoutedEventArgs e)
        {
            GraphEditor.SetGraph(BuildDefaultGraph());
            rootGraph = GraphEditor.GetGraph();
            rootGraphPath = null;
            currentGraphPath = null;
            includeCache.Clear();
            GraphEditor.BaseDirectory = GetRootDirectory();
            RefreshHierarchy();
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

        private void OnIncludeOpenRequested(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            string resolved = ResolvePath(path);
            var graph = LoadIncludeGraph(path);
            if (graph == null)
            {
                MessageBox.Show(this, $"Include not found:\n{path}", "Include not found",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            GraphEditor.SetGraph(graph);
            currentGraphPath = resolved;
            RefreshHierarchy(selectPath: resolved);
        }

        private void TreeHierarchy_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            if (suppressTreeSelection)
            {
                return;
            }

            if (TreeHierarchy.SelectedItem is GraphHierarchyItem item)
            {
                if (item.IsRoot)
                {
                    GraphEditor.SetGraph(rootGraph);
                    currentGraphPath = null;
                    return;
                }

                if (!string.IsNullOrWhiteSpace(item.Path))
                {
                    var graph = LoadIncludeGraph(item.Path);
                    if (graph != null)
                    {
                        GraphEditor.SetGraph(graph);
                        currentGraphPath = ResolvePath(item.Path);
                    }
                }
            }
        }

        private void RefreshHierarchy()
        {
            RefreshHierarchy(selectPath: currentGraphPath);
        }

        private void RefreshHierarchy(string selectPath)
        {
            suppressTreeSelection = true;
            TreeHierarchy.Items.Clear();
            ListLibrary.Items.Clear();

            var rootItem = new GraphHierarchyItem
            {
                Label = "Root Graph",
                Path = rootGraphPath ?? "(unsaved)",
                IsRoot = true
            };
            TreeHierarchy.Items.Add(rootItem);

            var includePaths = CollectIncludePaths(rootGraph);
            foreach (var path in includePaths)
            {
                rootItem.Children.Add(new GraphHierarchyItem
                {
                    Label = Path.GetFileName(path),
                    Path = path
                });
            }

            rootItem.IsExpanded = true;
            if (selectPath != null)
            {
                SelectTreeItem(rootItem, selectPath);
            }
            else
            {
                var rootContainer = TreeHierarchy.ItemContainerGenerator.ContainerFromItem(rootItem) as System.Windows.Controls.TreeViewItem;
                if (rootContainer != null)
                {
                    rootContainer.IsSelected = true;
                }
            }

            foreach (var item in BuildLibraryItems())
            {
                ListLibrary.Items.Add(item);
            }
            suppressTreeSelection = false;
        }

        private GraphDefinition LoadIncludeGraph(string path)
        {
            string resolved = ResolvePath(path);
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

            string json = File.ReadAllText(resolved);
            var graph = GraphSerializer.Deserialize(json, out _);
            includeCache[resolved] = graph;
            return graph;
        }

        private string ResolvePath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return null;
            }

            if (Path.IsPathRooted(path))
            {
                return path;
            }

            if (!string.IsNullOrWhiteSpace(rootGraphPath))
            {
                string rootDir = Path.GetDirectoryName(rootGraphPath);
                if (!string.IsNullOrWhiteSpace(rootDir))
                {
                    return Path.Combine(rootDir, path);
                }
            }

            return path;
        }

        private string GetRootDirectory()
        {
            if (string.IsNullOrWhiteSpace(rootGraphPath))
            {
                return null;
            }

            return Path.GetDirectoryName(rootGraphPath);
        }

        private static HashSet<string> CollectIncludePaths(GraphDefinition graph)
        {
            var paths = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            if (graph == null)
            {
                return paths;
            }

            foreach (var node in graph.Nodes)
            {
                if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    paths.Add(node.IncludePath);
                }
            }

            return paths;
        }

        private void ButtonOpenLibrary_Click(object sender, RoutedEventArgs e)
        {
            OpenLibrarySelection();
        }

        private void ListLibrary_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            OpenLibrarySelection();
        }

        private void OpenLibrarySelection()
        {
            if (ListLibrary.SelectedItem is GraphLibraryItem item && !string.IsNullOrWhiteSpace(item.Path))
            {
                var graph = LoadIncludeGraph(item.Path);
                if (graph != null)
                {
                    GraphEditor.SetGraph(graph);
                    currentGraphPath = ResolvePath(item.Path);
                    RefreshHierarchy(selectPath: currentGraphPath);
                }
            }
        }

        private IEnumerable<GraphLibraryItem> BuildLibraryItems()
        {
            var items = new List<GraphLibraryItem>();

            foreach (var pair in includeCache)
            {
                items.Add(new GraphLibraryItem
                {
                    Label = Path.GetFileName(pair.Key),
                    Path = pair.Key
                });
            }

            string rootDir = GetRootDirectory();
            if (!string.IsNullOrWhiteSpace(rootDir))
            {
                string embeddedDir = Path.Combine(rootDir, "graphs", "_embedded");
                string indexPath = Path.Combine(embeddedDir, "index.json");
                if (File.Exists(indexPath))
                {
                    try
                    {
                        string json = File.ReadAllText(indexPath);
                        var index = Newtonsoft.Json.JsonConvert.DeserializeObject<BlockLibraryIndex>(json);
                        if (index != null)
                        {
                            foreach (var entry in index.Entries)
                            {
                                items.Add(new GraphLibraryItem
                                {
                                    Label = Path.GetFileName(entry.Path),
                                    Path = entry.Path
                                });
                            }
                        }
                    }
                    catch
                    {
                    }
                }
            }

            items.Sort((a, b) => string.Compare(a.Label, b.Label, StringComparison.OrdinalIgnoreCase));
            return items;
        }

        private void SelectTreeItem(GraphHierarchyItem rootItem, string selectPath)
        {
            if (rootItem == null || string.IsNullOrWhiteSpace(selectPath))
            {
                return;
            }

            var rootContainer = TreeHierarchy.ItemContainerGenerator.ContainerFromItem(rootItem) as System.Windows.Controls.TreeViewItem;
            if (rootContainer == null)
            {
                return;
            }

            rootContainer.IsExpanded = true;
            foreach (var child in rootItem.Children)
            {
                if (!string.Equals(child.Path, selectPath, StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                rootContainer.UpdateLayout();
                var childContainer = rootContainer.ItemContainerGenerator.ContainerFromItem(child) as System.Windows.Controls.TreeViewItem;
                if (childContainer != null)
                {
                    childContainer.IsSelected = true;
                }
                break;
            }
        }

        private sealed class GraphHierarchyItem
        {
            public string Label { get; set; }
            public string Path { get; set; }
            public bool IsRoot { get; set; }
            public List<GraphHierarchyItem> Children { get; } = new List<GraphHierarchyItem>();
            public bool IsExpanded { get; set; }
            public bool IsSelected { get; set; }
            public override string ToString() => Label;
        }

        private sealed class GraphLibraryItem
        {
            public string Label { get; set; }
            public string Path { get; set; }
            public override string ToString() => Label;
        }

        private sealed class BlockLibraryIndex
        {
            public List<BlockLibraryEntry> Entries { get; set; } = new List<BlockLibraryEntry>();
        }

        private sealed class BlockLibraryEntry
        {
            public string Path { get; set; }
        }
    }
}
