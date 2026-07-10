using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using Microsoft.Win32;
using DiyFfb.Controls;
using DiyFfb.Helpers;

namespace DiyFfb.GraphEditor
{
    /// <summary>
    /// Converts true to Collapsed, false to Visible.
    /// </summary>
    public sealed class InverseBooleanToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is bool b && b)
            {
                return Visibility.Collapsed;
            }
            return Visibility.Visible;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    /// <summary>
    /// Converts a null/empty string to true (e.g. "no explicit function set" → Scoped checked).
    /// </summary>
    public sealed class StringEmptyToBoolConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return string.IsNullOrEmpty(value as string);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public partial class GraphEditorWindow : Window
    {
        private readonly GraphEditorTabManager tabManager;
        private bool suppressTreeSelection;
        private DiyFfbPlugin plugin;
        private Func<IDictionary<string, double>> liveInputProvider;
        private Func<IReadOnlyDictionary<string, uint>> msfsFailedVarProvider;
        private Func<IReadOnlyDictionary<string, uint>> msfsWriteFailedVarProvider;
        private Func<IReadOnlyDictionary<string, double>> configInProvider;
        private readonly System.Windows.Threading.DispatcherTimer _globalLiveTimer;
        private bool _globalLiveInputsEnabled = true;  // Enabled by default
        private GraphEditorControl _lastEditor;
        private PreviewWindow _previewWindow;

        public GraphEditorWindow()
        {
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);

            tabManager = new GraphEditorTabManager();
            tabManager.SelectedTabChanged += OnSelectedTabChanged;
            tabManager.TabAdded += OnTabAdded;
            tabManager.TabRemoved += OnTabRemoved;

            // Create the pinned active graph tab
            tabManager.CreateActiveGraphTab();

            // Bind TabControl to tabs collection
            EditorTabs.ItemsSource = tabManager.Tabs;
            EditorTabs.SelectedItem = tabManager.SelectedTab;
            _lastEditor = CurrentEditor;

            // Initialize Apply button state
            ButtonApply.IsEnabled = CurrentTab?.IsActiveGraph == true;

            // Initialize global live inputs timer (enabled by default)
            _globalLiveTimer = new System.Windows.Threading.DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(200)
            };
            _globalLiveTimer.Tick += OnGlobalLiveTimerTick;
            _globalLiveTimer.Start();

            // Handle window closing to stop the timer
            Closing += OnWindowClosing;

            RefreshHierarchy();
            UpdateUndoRedoButtons();
        }

        /// <summary>
        /// Gets the currently selected tab's editor control.
        /// </summary>
        private GraphEditorControl CurrentEditor => tabManager.SelectedTab?.EditorControl;

        /// <summary>
        /// Gets the currently selected tab.
        /// </summary>
        private GraphEditorTab CurrentTab => tabManager.SelectedTab;

        public void SetLiveInputProvider(Func<IDictionary<string, double>> provider)
        {
            liveInputProvider = provider;
            // Apply to all tabs
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.LiveInputProvider = provider;
            }
        }

        public void SetMsfsFailedVarProvider(Func<IReadOnlyDictionary<string, uint>> provider)
        {
            msfsFailedVarProvider = provider;
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.MsfsFailedVarProvider = provider;
            }
        }

        public void SetMsfsWriteFailedVarProvider(Func<IReadOnlyDictionary<string, uint>> provider)
        {
            msfsWriteFailedVarProvider = provider;
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.MsfsWriteFailedVarProvider = provider;
            }
        }

        public void SetConfigInProvider(Func<IReadOnlyDictionary<string, double>> provider)
        {
            configInProvider = provider;
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.ConfigInProvider = provider;
            }
        }

        public void SetPlugin(DiyFfbPlugin pluginInstance)
        {
            // Unsubscribe from old plugin if any
            if (plugin != null)
            {
                plugin.GraphParamChanged -= OnPluginGraphParamChanged;
                plugin.ActiveGraphChanged -= OnPluginActiveGraphChanged;
            }

            plugin = pluginInstance;

            // Subscribe to new plugin events
            if (plugin != null)
            {
                plugin.GraphParamChanged += OnPluginGraphParamChanged;
                plugin.ActiveGraphChanged += OnPluginActiveGraphChanged;
            }

            // Wire up parameter changes for all tabs
            foreach (var tab in tabManager.Tabs)
            {
                WireTabParamChanges(tab);

                // Set/update context provider (plugin reference may have changed)
                tab.EditorControl.ContextProvider = path => plugin?.ActiveIncludeContextCache?.GetContexts(path);
            }
        }

        private void OnPluginActiveGraphChanged(object sender, EventArgs e)
        {
            // When the plugin's active graph changes (vehicle selection), update the editor
            Dispatcher.Invoke(() =>
            {
                string path = plugin?.GetActiveGraphPath();
                if (string.IsNullOrWhiteSpace(path))
                {
                    return;
                }

                // Guard against re-entrant calls - skip if already showing this graph
                string currentPath = tabManager.ActiveGraphTab?.FilePath;
                if (!string.IsNullOrWhiteSpace(currentPath) &&
                    string.Equals(currentPath, path, StringComparison.OrdinalIgnoreCase))
                {
                    return;
                }

                LoadGraphFromPath(path);
            });
        }

        private void WireTabParamChanges(GraphEditorTab tab)
        {
            // Unsubscribe first to avoid duplicate handlers
            tab.EditorControl.IncludeOpenRequested -= OnIncludeOpenRequested;
            tab.EditorControl.EmbeddedOpenRequested -= OnEmbeddedOpenRequested;

            tab.EditorControl.ParamValueChanged = (paramName, value) =>
            {
                // Only propagate from active graph tab to plugin
                if (tab.IsActiveGraph)
                {
                    plugin?.SetGraphParamValue(paramName, value);
                }
            };

            // GraphChanged uses a closure over 'tab', so we can't easily unsubscribe.
            // Only subscribe if not already done (check via a tag or just accept one-time wiring).
            // Since ParamValueChanged is assigned (not +=), we use it as a proxy for "already wired".
            // Actually, we need a better approach - use a HashSet to track wired tabs.
            if (!_wiredTabs.Contains(tab.Id))
            {
                _wiredTabs.Add(tab.Id);
                tab.EditorControl.GraphChanged += () =>
                {
                    // Note: We intentionally do NOT call plugin?.OnGraphContentChanged() here.
                    // That fires ActiveGraphChanged which is meant for graph FILE changes,
                    // not for every edit. Calling it here causes feedback loops with sliders.
                    RefreshHierarchy();
                };
            }

            tab.EditorControl.IncludeOpenRequested += OnIncludeOpenRequested;
            tab.EditorControl.EmbeddedOpenRequested += OnEmbeddedOpenRequested;
        }

        private readonly HashSet<string> _wiredTabs = new HashSet<string>();
        private readonly HashSet<string> _wiredUndoTabs = new HashSet<string>();

        private void OnPluginGraphParamChanged(object sender, GraphParamChangedEventArgs e)
        {
            // Update ALL open tabs when plugin parameters change externally
            // (params are global across the graph tree, so include tabs need updates too)
            Dispatcher.Invoke(() =>
            {
                foreach (var tab in tabManager.Tabs)
                {
                    tab.EditorControl.UpdateParamValue(e.ParamName, e.Value);
                }
                // Note: We don't mark the graph tab as dirty here because param overrides
                // are stored in the profile, not the graph file.
            });
        }

        private void OnSelectedTabChanged(object sender, EventArgs e)
        {
            var previousEditor = _lastEditor;
            var currentEditor = CurrentEditor;
            EditorTabs.SelectedItem = tabManager.SelectedTab;
            RefreshHierarchy();
            ButtonApply.IsEnabled = CurrentTab?.IsActiveGraph == true;
            UpdateUndoRedoButtons();
            // Note: We do NOT sync params from plugin on tab switch - the graph's in-memory
            // state should persist. SyncParamsFromPlugin is only called when loading a new graph.
            if (_previewWindow != null && _previewWindow.IsVisible && previousEditor != currentEditor)
            {
                previousEditor?.DetachPreviewWindow();
                currentEditor?.AttachPreviewWindow(_previewWindow);
            }

            _lastEditor = currentEditor;
        }

        private void UpdateUndoState(GraphEditorTab tab)
        {
            if (tab == null)
            {
                return;
            }

            bool dirty = tab.UndoStack?.IsDirty == true;
            tab.IsDirty = dirty;
            tab.EditorControl.SetDirtyState(dirty);
            // Embedded sub-graph edits flush back into the parent node's InlineGraph
            // and dirty the parent, so saving the parent file persists them.
            if (dirty && tab.IsEmbedded)
            {
                tab.FlushToParent();
                // Keep the parent's Include node ports in sync if the sub-graph's
                // interface (Input/Output nodes) changed.
                tab.ParentTab?.EditorControl.RefreshIncludeNode(tab.EmbeddedNodeId);
            }
            UpdateUndoRedoButtons();
        }

        private void UpdateUndoRedoButtons()
        {
            ButtonUndo.IsEnabled = CurrentTab?.EditorControl.CanUndo == true;
            ButtonRedo.IsEnabled = CurrentTab?.EditorControl.CanRedo == true;
        }

        private void SyncParamsFromPlugin()
        {
            var activeTab = tabManager.ActiveGraphTab;
            if (activeTab == null || plugin == null)
            {
                return;
            }

            // Get all collected param names (includes params from Include nodes)
            var paramNames = activeTab.EditorControl.GetCollectedParamNames();
            foreach (var paramName in paramNames)
            {
                double pluginValue = plugin.GetGraphParamValue(paramName);
                activeTab.EditorControl.UpdateParamValue(paramName, pluginValue);
            }
        }

        private void OnWindowClosing(object sender, CancelEventArgs e)
        {
            // Stop the global live inputs timer to prevent evaluation after close
            _globalLiveTimer.Stop();

            if (_previewWindow != null)
            {
                _previewWindow.Close();
            }
        }

        private void OnGlobalLiveTimerTick(object sender, EventArgs e)
        {
            if (!_globalLiveInputsEnabled)
            {
                return;
            }

            // Tick live inputs for all tabs
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.TickLiveInputs();
            }
        }

        private void OnTabLiveInputsStateChanged(object sender, bool enabled)
        {
            // One tab's checkbox was toggled - sync the global state to all tabs
            _globalLiveInputsEnabled = enabled;

            if (_globalLiveInputsEnabled)
            {
                _globalLiveTimer.Start();
            }
            else
            {
                _globalLiveTimer.Stop();
            }

            // Sync state to all tabs (without raising events to avoid loops)
            foreach (var tab in tabManager.Tabs)
            {
                tab.EditorControl.SetLiveInputsState(enabled, raiseEvent: false);
            }
        }

        private void OnTabAdded(object sender, GraphEditorTab tab)
        {
            WireTabParamChanges(tab);

            // Apply live input provider if set
            if (liveInputProvider != null)
            {
                tab.EditorControl.LiveInputProvider = liveInputProvider;
            }
            if (msfsFailedVarProvider != null)
            {
                tab.EditorControl.MsfsFailedVarProvider = msfsFailedVarProvider;
            }
            if (msfsWriteFailedVarProvider != null)
            {
                tab.EditorControl.MsfsWriteFailedVarProvider = msfsWriteFailedVarProvider;
            }
            if (configInProvider != null)
            {
                tab.EditorControl.ConfigInProvider = configInProvider;
            }

            // Provide runtime state snapshot for active graph tabs (top-level state sync)
            if (tab.IsActiveGraph)
            {
                tab.EditorControl.LiveStateProvider = () => plugin?.GetActiveGraphStateSnapshot();
            }

            // Set context provider for include context preview
            tab.EditorControl.ContextProvider = path => plugin?.ActiveIncludeContextCache?.GetContexts(path);

            // Subscribe to context changes to update tab label
            tab.EditorControl.ContextChanged += (s, contextId) => UpdateTabContextLabel(tab, contextId);

            // Subscribe to live inputs state change and sync initial state
            tab.EditorControl.LiveInputsStateChanged += OnTabLiveInputsStateChanged;
            tab.EditorControl.SetLiveInputsState(_globalLiveInputsEnabled, raiseEvent: false);

            if (!_wiredUndoTabs.Contains(tab.Id))
            {
                _wiredUndoTabs.Add(tab.Id);
                tab.UndoStack.Changed += (_, __) => UpdateUndoState(tab);
            }

            UpdateUndoState(tab);
        }

        private void OnTabRemoved(object sender, GraphEditorTab tab)
        {
            // Clean up event handlers
            tab.EditorControl.IncludeOpenRequested -= OnIncludeOpenRequested;
            tab.EditorControl.EmbeddedOpenRequested -= OnEmbeddedOpenRequested;
            tab.EditorControl.LiveInputsStateChanged -= OnTabLiveInputsStateChanged;
            UpdateUndoRedoButtons();
        }

        private void UpdateTabContextLabel(GraphEditorTab tab, string contextId)
        {
            if (contextId != null && plugin?.ActiveIncludeContextCache != null)
            {
                // Normalize path for cache lookup (cache uses absolute paths)
                string normalizedPath = tab.FilePath;
                try
                {
                    if (!string.IsNullOrEmpty(tab.FilePath))
                        normalizedPath = System.IO.Path.GetFullPath(tab.FilePath);
                }
                catch { }

                var contexts = plugin.ActiveIncludeContextCache.GetContexts(normalizedPath);
                DiyFfb.GraphTest.IncludeCallContext ctx = null;
                if (contexts != null)
                {
                    foreach (var c in contexts)
                    {
                        if (c.IncludeNodeId == contextId)
                        {
                            ctx = c;
                            break;
                        }
                    }
                }

                if (ctx != null)
                {
                    tab.ContextSuffix = $" (via {ctx.IncludeNodeTitle})";
                    return;
                }
            }
            tab.ContextSuffix = null;
        }

        public void LoadGraphFromPath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            // Set the active graph
            tabManager.SetActiveGraph(path);
            EditorTabs.SelectedItem = tabManager.ActiveGraphTab;
            RefreshHierarchy();

            // Sync params from plugin to ensure graph editor shows current runtime values
            SyncParamsFromPlugin();
        }

        private void ButtonNew_Click(object sender, RoutedEventArgs e)
        {
            // Get available templates
            string baseDir = CurrentTab?.BaseDirectory ?? AppDomain.CurrentDomain.BaseDirectory;
            var templates = GraphTemplateRegistry.GetTemplates(null, baseDir);

            if (templates.Any())
            {
                // Show template selector for new graphs
                var templateDialog = new GraphTemplateSelectorDialog("Any", "(new graph)", templates)
                {
                    Owner = this
                };

                if (templateDialog.ShowDialog() == true)
                {
                    if (templateDialog.SelectedTemplate != null)
                    {
                        string templatePath = GraphTemplateRegistry.ResolveTemplatePath(
                            templateDialog.SelectedTemplate.TemplatePath, baseDir);

                        if (!string.IsNullOrWhiteSpace(templatePath))
                        {
                            var tab = tabManager.CreateNewTab();
                            tab.LoadFromFile(templatePath);
                            tab.FilePath = null; // Clear path so it's "untitled"
                            tab.IsDirty = true;
                            EditorTabs.SelectedItem = tab;
                            RefreshHierarchy();
                            return;
                        }
                    }

                    // Skipped template selection, create empty graph
                    tabManager.CreateNewTab();
                    RefreshHierarchy();
                }
            }
            else
            {
                // No templates available, create empty graph
                tabManager.CreateNewTab();
                RefreshHierarchy();
            }
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
                var tab = tabManager.OpenGraph(dialog.FileName);
                if (tab != null)
                {
                    EditorTabs.SelectedItem = tab;
                    RefreshHierarchy();
                }
            }
        }

        private void ButtonSave_Click(object sender, RoutedEventArgs e)
        {
            if (CurrentTab == null)
            {
                return;
            }

            // Embedded sub-graphs have no file of their own. Flush the edit
            // chain up into the parent node(s) and save the root file tab,
            // which serializes the inline graphs. The embedded chain is then
            // marked clean since its content is now persisted.
            GraphEditorTab target = CurrentTab;
            if (CurrentTab.IsEmbedded)
            {
                target = FlushEmbeddedChainToRoot(CurrentTab);
                if (target == null)
                {
                    return;
                }
            }

            if (string.IsNullOrWhiteSpace(target.FilePath))
            {
                // No path yet, do Save As (operates on the selected tab; only
                // reached for a genuinely unsaved root, never for embedded).
                ButtonSaveAs_Click(sender, e);
                return;
            }

            // Check if graph is shared before saving
            if (plugin != null)
            {
                var scanner = new GraphUsageScanner(plugin);
                var report = scanner.GetUsageReport(target.FilePath, plugin.GetActiveProfileKey());

                if (report.IsShared)
                {
                    var result = ShowSharedGraphSaveDialog(report);
                    switch (result)
                    {
                        case SharedGraphSaveResult.Cancel:
                            return;
                        case SharedGraphSaveResult.SaveAsCopy:
                            SaveAsCopyForCurrentVehicle();
                            return;
                        case SharedGraphSaveResult.SaveAnyway:
                            // Fall through to normal save
                            break;
                    }
                }
            }

            if (target.Save())
            {
                // Auto-apply to runtime when saving the active graph
                if (target.IsActiveGraph)
                {
                    plugin?.ApplyGraphToRuntime(target.Graph, target.FilePath);
                }

                target.EditorControl.MarkUndoClean();
                UpdateUndoState(target);
                if (CurrentTab.IsEmbedded)
                {
                    MarkEmbeddedChainClean(CurrentTab);
                }
            }
            else
            {
                ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        /// <summary>
        /// Flushes an embedded tab's edits up through every ancestor embedded tab
        /// into the parent node InlineGraphs, returning the root file-backed tab.
        /// </summary>
        private GraphEditorTab FlushEmbeddedChainToRoot(GraphEditorTab tab)
        {
            var t = tab;
            while (t != null && t.IsEmbedded)
            {
                t.FlushToParent();
                t = t.ParentTab;
            }
            return t;
        }

        /// <summary>Clears the dirty state on an embedded tab and its ancestors after a save.</summary>
        private void MarkEmbeddedChainClean(GraphEditorTab tab)
        {
            for (var t = tab; t != null && t.IsEmbedded; t = t.ParentTab)
            {
                t.EditorControl.MarkUndoClean();
                UpdateUndoState(t);
            }
        }

        private void ButtonSaveAs_Click(object sender, RoutedEventArgs e)
        {
            if (CurrentTab == null)
            {
                return;
            }

            var dialog = new SaveFileDialog
            {
                Filter = "Graph JSON (*.json)|*.json",
                DefaultExt = "json",
                FileName = string.IsNullOrWhiteSpace(CurrentTab.FilePath)
                    ? "ffb_graph.json"
                    : Path.GetFileName(CurrentTab.FilePath)
            };

            if (dialog.ShowDialog() == true)
            {
                if (CurrentTab.SaveAs(dialog.FileName))
                {
                    RefreshHierarchy();

                    // Auto-apply to runtime when saving the active graph
                    if (CurrentTab.IsActiveGraph)
                    {
                        plugin?.ApplyGraphToRuntime(CurrentTab.Graph, CurrentTab.FilePath);
                    }

                    CurrentTab.EditorControl.MarkUndoClean();
                    UpdateUndoState(CurrentTab);
                }
                else
                {
                    ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
        }

        /// <summary>
        /// Shows the shared graph save dialog and returns the user's choice.
        /// </summary>
        private SharedGraphSaveResult ShowSharedGraphSaveDialog(GraphUsageReport report)
        {
            var dialog = new SharedGraphSaveDialog(report)
            {
                Owner = this
            };
            dialog.ShowDialog();
            return dialog.Result;
        }

        /// <summary>
        /// Saves the current graph as a copy and updates the current vehicle's GraphPath.
        /// </summary>
        private void SaveAsCopyForCurrentVehicle()
        {
            if (CurrentTab == null)
                return;

            // Generate default filename: original_copy.json or original_vehicleId.json
            string originalName = Path.GetFileNameWithoutExtension(CurrentTab.FilePath);
            string vehicleKey = plugin?.GetActiveProfileKey();
            string suffix = "_copy";
            if (!string.IsNullOrEmpty(vehicleKey))
            {
                // Extract car ID from "gameId::carId"
                int sep = vehicleKey.IndexOf("::");
                if (sep > 0 && sep + 2 < vehicleKey.Length)
                {
                    suffix = "_" + SanitizeFileName(vehicleKey.Substring(sep + 2));
                }
            }

            string defaultDir = Path.GetDirectoryName(CurrentTab.FilePath);
            string defaultName = originalName + suffix + ".json";

            var dialog = new SaveFileDialog
            {
                Filter = "Graph JSON (*.json)|*.json",
                DefaultExt = "json",
                FileName = defaultName,
                InitialDirectory = defaultDir
            };

            if (dialog.ShowDialog() == true)
            {
                string newPath = dialog.FileName;

                if (CurrentTab.SaveAs(newPath))
                {
                    RefreshHierarchy();

                    // Update current vehicle's GraphPath to point to the new copy
                    if (plugin != null)
                    {
                        string gameId = plugin.GetActiveGameId();
                        string carId = plugin.GetActiveCarId();
                        if (!string.IsNullOrEmpty(gameId) && !string.IsNullOrEmpty(carId))
                        {
                            // Convert to relative path if within base directory
                            string relativePath = MakeRelativeGraphPath(newPath);
                            plugin.SetVehicleGraphPath(gameId, carId, relativePath);
                        }
                    }

                    // Auto-apply to runtime
                    if (CurrentTab.IsActiveGraph)
                    {
                        plugin?.ApplyGraphToRuntime(CurrentTab.Graph, CurrentTab.FilePath);
                    }

                    CurrentTab.EditorControl.MarkUndoClean();
                    UpdateUndoState(CurrentTab);
                }
                else
                {
                    ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
        }

        /// <summary>
        /// Makes a graph path relative to the app base directory if possible.
        /// </summary>
        private string MakeRelativeGraphPath(string fullPath)
        {
            if (string.IsNullOrEmpty(fullPath))
                return fullPath;

            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            if (fullPath.StartsWith(baseDir, StringComparison.OrdinalIgnoreCase))
            {
                string relative = fullPath.Substring(baseDir.Length);
                // Normalize to forward slashes for consistency
                return relative.TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
                               .Replace(Path.DirectorySeparatorChar, '/');
            }

            return fullPath;
        }

        /// <summary>
        /// Sanitizes a string for use in a filename.
        /// </summary>
        private static string SanitizeFileName(string name)
        {
            if (string.IsNullOrEmpty(name))
                return "copy";

            var invalid = Path.GetInvalidFileNameChars();
            var sanitized = new System.Text.StringBuilder();
            foreach (char c in name)
            {
                if (Array.IndexOf(invalid, c) < 0 && c != ' ')
                    sanitized.Append(c);
                else if (c == ' ')
                    sanitized.Append('_');
            }
            return sanitized.Length > 0 ? sanitized.ToString() : "copy";
        }

        private void ButtonApply_Click(object sender, RoutedEventArgs e)
        {
            if (CurrentTab == null || !CurrentTab.IsActiveGraph)
            {
                return;
            }

            plugin?.ApplyGraphToRuntime(CurrentTab.Graph, CurrentTab.FilePath);
        }

        private void ButtonPreviewInputs_Click(object sender, RoutedEventArgs e)
        {
            if (_previewWindow != null && _previewWindow.IsVisible)
            {
                _previewWindow.Close();
                return;
            }

            EnsurePreviewWindow();
            CurrentEditor?.AttachPreviewWindow(_previewWindow);
            _previewWindow.Show();
            _previewWindow.Activate();
        }

        private void EnsurePreviewWindow()
        {
            if (_previewWindow != null)
            {
                return;
            }

            _previewWindow = new PreviewWindow
            {
                Owner = this
            };
            _previewWindow.Closed += OnPreviewWindowClosed;
        }

        private void OnPreviewWindowClosed(object sender, EventArgs e)
        {
            if (_previewWindow != null)
            {
                _previewWindow.Closed -= OnPreviewWindowClosed;
            }
            CurrentEditor?.DetachPreviewWindow();
            _previewWindow = null;
        }

        private void ButtonUndo_Click(object sender, RoutedEventArgs e)
        {
            if (CurrentTab?.EditorControl.Undo() == true)
            {
                UpdateUndoState(CurrentTab);
            }
        }

        private void ButtonRedo_Click(object sender, RoutedEventArgs e)
        {
            if (CurrentTab?.EditorControl.Redo() == true)
            {
                UpdateUndoState(CurrentTab);
            }
        }

        private void ButtonCloseTab_Click(object sender, RoutedEventArgs e)
        {
            CloseCurrentTab();
        }

        private void TabCloseButton_Click(object sender, RoutedEventArgs e)
        {
            if (sender is FrameworkElement element && element.Tag is GraphEditorTab tab)
            {
                CloseTab(tab);
            }
        }

        private void CloseCurrentTab()
        {
            if (CurrentTab != null)
            {
                CloseTab(CurrentTab);
            }
        }

        private void CloseTab(GraphEditorTab tab)
        {
            if (tab == null || tab.IsPinned)
            {
                return;
            }

            // Embedded sub-graph tabs have no file and their edits are already
            // flushed into the parent (which carries the dirty state). Closing
            // loses nothing — flush once more and close without prompting.
            if (tab.IsEmbedded)
            {
                tab.FlushToParent();
                tabManager.CloseTab(tab);
                RefreshHierarchy();
                return;
            }

            if (tab.IsDirty)
            {
                var result = ThemedMessageBox.Show(
                    this,
                    $"Save changes to {tab.DisplayName}?",
                    "Unsaved Changes",
                    MessageBoxButton.YesNoCancel,
                    MessageBoxImage.Question);

                switch (result)
                {
                    case MessageBoxResult.Yes:
                        if (string.IsNullOrWhiteSpace(tab.FilePath))
                        {
                            var dialog = new SaveFileDialog
                            {
                                Filter = "Graph JSON (*.json)|*.json",
                                DefaultExt = "json",
                                FileName = "ffb_graph.json"
                            };

                            if (dialog.ShowDialog() != true)
                            {
                                return; // Cancelled
                            }

                            if (!tab.SaveAs(dialog.FileName))
                            {
                                ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                                    MessageBoxButton.OK, MessageBoxImage.Error);
                                return;
                            }
                        }
                        else
                        {
                            // Check if graph is shared before saving
                            if (plugin != null)
                            {
                                var scanner = new GraphUsageScanner(plugin);
                                var report = scanner.GetUsageReport(tab.FilePath, plugin.GetActiveProfileKey());

                                if (report.IsShared)
                                {
                                    var sharedResult = ShowSharedGraphSaveDialog(report);
                                    switch (sharedResult)
                                    {
                                        case SharedGraphSaveResult.Cancel:
                                            return; // Cancelled
                                        case SharedGraphSaveResult.SaveAsCopy:
                                            // Show SaveAs dialog for the tab
                                            var saveAsDialog = new SaveFileDialog
                                            {
                                                Filter = "Graph JSON (*.json)|*.json",
                                                DefaultExt = "json",
                                                FileName = Path.GetFileName(tab.FilePath),
                                                InitialDirectory = Path.GetDirectoryName(tab.FilePath)
                                            };
                                            if (saveAsDialog.ShowDialog() != true)
                                            {
                                                return; // Cancelled
                                            }
                                            if (!tab.SaveAs(saveAsDialog.FileName))
                                            {
                                                ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                                                    MessageBoxButton.OK, MessageBoxImage.Error);
                                                return;
                                            }
                                            break;
                                        case SharedGraphSaveResult.SaveAnyway:
                                            // Fall through to normal save
                                            if (!tab.Save())
                                            {
                                                ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                                                    MessageBoxButton.OK, MessageBoxImage.Error);
                                                return;
                                            }
                                            break;
                                    }
                                }
                                else if (!tab.Save())
                                {
                                    ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                                        MessageBoxButton.OK, MessageBoxImage.Error);
                                    return;
                                }
                            }
                            else if (!tab.Save())
                            {
                                ThemedMessageBox.Show(this, "Failed to save graph.", "Save Error",
                                    MessageBoxButton.OK, MessageBoxImage.Error);
                                return;
                            }
                        }
                        break;

                    case MessageBoxResult.Cancel:
                        return; // Cancelled

                    case MessageBoxResult.No:
                        // Don't save, proceed to close
                        break;
                }
            }

            tabManager.CloseTab(tab);
            RefreshHierarchy();
        }

        private void EditorTabs_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (EditorTabs.SelectedItem is GraphEditorTab tab && tab != tabManager.SelectedTab)
            {
                tabManager.SelectedTab = tab;
            }

            // Update Apply button state based on whether current tab is the active graph
            ButtonApply.IsEnabled = CurrentTab?.IsActiveGraph == true;
            UpdateUndoRedoButtons();
        }

        private void GraphEditorWindow_PreviewKeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if ((Keyboard.Modifiers & ModifierKeys.Control) != ModifierKeys.Control)
            {
                return;
            }

            if (e.Key == Key.Z && (Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift)
            {
                if (CurrentTab?.EditorControl.Redo() == true)
                {
                    UpdateUndoState(CurrentTab);
                }
                e.Handled = true;
                return;
            }

            if (e.Key == Key.Z)
            {
                if (CurrentTab?.EditorControl.Undo() == true)
                {
                    UpdateUndoState(CurrentTab);
                }
                e.Handled = true;
                return;
            }

            if (e.Key == Key.Y)
            {
                if (CurrentTab?.EditorControl.Redo() == true)
                {
                    UpdateUndoState(CurrentTab);
                }
                e.Handled = true;
                return;
            }

            if (e.Key == Key.S)
            {
                ButtonSave_Click(sender, e);
                e.Handled = true;
                return;
            }

            if (e.Key == Key.O)
            {
                ButtonLoad_Click(sender, e);
                e.Handled = true;
            }
        }

        private void OnEmbeddedOpenRequested(string nodeId, string contextId)
        {
            var parent = CurrentTab;
            if (parent?.Graph == null || string.IsNullOrEmpty(nodeId))
            {
                return;
            }

            var node = parent.Graph.Nodes.FirstOrDefault(n => n.Id == nodeId && n.IsEmbeddedInclude);
            if (node == null)
            {
                return;
            }

            var tab = tabManager.OpenEmbedded(parent, node);
            if (tab == null)
            {
                return;
            }

            EditorTabs.SelectedItem = tab;
            RefreshHierarchy();

            if (!string.IsNullOrEmpty(contextId))
            {
                tab.EditorControl.SetSelectedContext(contextId);
            }
        }

        private void OnIncludeOpenRequested(string path, string contextId)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            // Resolve relative to current tab's base directory
            string baseDir = CurrentTab?.BaseDirectory;
            string resolved = GraphEditorTabManager.ResolvePath(path, baseDir);

            // Try to open or switch to existing tab
            var tab = tabManager.OpenGraph(resolved);
            if (tab == null)
            {
                ThemedMessageBox.Show(this, $"Include not found:\n{path}", "Include not found",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            EditorTabs.SelectedItem = tab;
            RefreshHierarchy();

            // Auto-select context if provided (from double-click with live mode active)
            if (!string.IsNullOrEmpty(contextId))
            {
                tab.EditorControl.SetSelectedContext(contextId);
            }
        }

        private void TreeHierarchy_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            if (suppressTreeSelection)
            {
                return;
            }

            if (TreeHierarchy.SelectedItem is GraphHierarchyItem item)
            {
                if (item.IsRoot && CurrentTab != null)
                {
                    // Stay on current tab at root level
                    return;
                }

                if (!string.IsNullOrWhiteSpace(item.Path))
                {
                    // Open include in new tab or switch to existing
                    string resolved = GraphEditorTabManager.ResolvePath(item.Path, CurrentTab?.BaseDirectory);
                    var tab = tabManager.OpenGraph(resolved);
                    if (tab != null)
                    {
                        EditorTabs.SelectedItem = tab;
                    }
                }
            }
        }

        private void RefreshHierarchy()
        {
            RefreshHierarchy(selectPath: null);
        }

        private void RefreshHierarchy(string selectPath)
        {
            suppressTreeSelection = true;
            TreeHierarchy.Items.Clear();
            ListLibrary.Items.Clear();

            if (CurrentTab == null)
            {
                suppressTreeSelection = false;
                return;
            }

            var rootItem = new GraphHierarchyItem
            {
                Label = CurrentTab.DisplayName,
                Path = CurrentTab.FilePath ?? "(unsaved)",
                IsRoot = true
            };
            TreeHierarchy.Items.Add(rootItem);

            var includePaths = CollectIncludePaths(CurrentTab.Graph);
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
                var tab = tabManager.OpenGraph(item.Path);
                if (tab != null)
                {
                    EditorTabs.SelectedItem = tab;
                    RefreshHierarchy();
                }
            }
        }

        private IEnumerable<GraphLibraryItem> BuildLibraryItems()
        {
            var items = new List<GraphLibraryItem>();

            foreach (var pair in tabManager.IncludeCache)
            {
                items.Add(new GraphLibraryItem
                {
                    Label = Path.GetFileName(pair.Key),
                    Path = pair.Key
                });
            }

            string rootDir = CurrentTab?.BaseDirectory;
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
