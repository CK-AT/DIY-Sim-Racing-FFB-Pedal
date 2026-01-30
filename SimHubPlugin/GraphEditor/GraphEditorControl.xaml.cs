using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Globalization;
using Microsoft.Win32;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using DiyFfb.GraphTest;
using User.PluginSdkDemo;

namespace User.PluginSdkDemo.GraphEditor
{
    public partial class GraphEditorControl : UserControl
    {
        private GraphDefinition _graph;
        private readonly Dictionary<string, NodeVisual> _nodeVisuals = new Dictionary<string, NodeVisual>();
        private readonly List<LinkVisual> _linkVisuals = new List<LinkVisual>();
        private readonly Dictionary<string, double> _nodeValues = new Dictionary<string, double>();
        private readonly ObservableCollection<PreviewEntry> _previewInputEntries = new ObservableCollection<PreviewEntry>();
        private readonly ObservableCollection<PreviewEntry> _previewParamEntries = new ObservableCollection<PreviewEntry>();
        private readonly Dictionary<string, PreviewEntry> _previewInputLookup = new Dictionary<string, PreviewEntry>();
        private readonly Dictionary<string, PreviewEntry> _previewParamLookup = new Dictionary<string, PreviewEntry>();
        private readonly HashSet<string> _explicitlyUpdatedParams = new HashSet<string>();
        private readonly DispatcherTimer _includePathDebounceTimer;
        private readonly DispatcherTimer _undoDebounceTimer;
        private readonly DispatcherTimer _previewRefreshTimer;
        private bool _liveInputsEnabled = true;  // Enabled by default (global setting synced from Window)
        private PreviewWindow _previewWindow;
        private string _previewStatusText = "";
        private readonly GraphPreviewEvaluator _previewEvaluator = new GraphPreviewEvaluator();
        private readonly ObservableCollection<PortEditEntry> _portEntries = new ObservableCollection<PortEditEntry>();
        private bool _isPanning;
        private bool _panWasDragged;
        private bool _hasUserPanned;
        private bool _isSelecting;
        private Point _lastPanPoint;
        private Point _dragStartPoint;
        private NodeVisual _dragNode;
        private List<NodeVisual> _dragNodes;
        private readonly Dictionary<NodeVisual, Point> _dragNodeStartPositions = new Dictionary<NodeVisual, Point>();
        private Point _dragOffset;
        private PortVisual _pendingPort;
        private NodeVisual _selectedNode;
        private readonly HashSet<NodeVisual> _selectedNodes = new HashSet<NodeVisual>();
        private readonly HashSet<LinkVisual> _selectedLinks = new HashSet<LinkVisual>();
        private LinkVisual _hoveredLink;
        private NodeVisual _hoverPortNode;
        private LinkVisual _edgeDragLink;
        private bool _edgeDragFromSource;
        private System.Windows.Shapes.Path _edgePreview;
        private PortVisual _edgePreviewSource;
        private Point _selectionStart;
        private Rectangle _selectionRect;
        private static readonly SolidColorBrush NodeFillBrush = new SolidColorBrush(Color.FromRgb(45, 45, 45));
        private static readonly SolidColorBrush NodeSelectedBrush = new SolidColorBrush(Color.FromRgb(58, 58, 90));
        private static readonly SolidColorBrush LinkBrush = new SolidColorBrush(Color.FromRgb(120, 120, 120));
        private static readonly SolidColorBrush LinkSelectedBrush = new SolidColorBrush(Color.FromRgb(240, 180, 70));
        private static readonly SolidColorBrush LinkHoverBrush = new SolidColorBrush(Color.FromRgb(160, 200, 255));
        private static readonly SolidColorBrush LinkHandleBrush = new SolidColorBrush(Color.FromRgb(80, 140, 220));
        private static readonly SolidColorBrush LinkHandleSelectedBrush = new SolidColorBrush(Color.FromRgb(240, 180, 70));

        // Title bar colors for different node types
        private static readonly SolidColorBrush TitleBarInput = new SolidColorBrush(Color.FromRgb(60, 120, 180));      // Blue
        private static readonly SolidColorBrush TitleBarOutput = new SolidColorBrush(Color.FromRgb(200, 120, 40));     // Orange
        private static readonly SolidColorBrush TitleBarParam = new SolidColorBrush(Color.FromRgb(140, 80, 180));      // Purple
        private static readonly SolidColorBrush TitleBarConst = new SolidColorBrush(Color.FromRgb(100, 100, 100));     // Gray
        private static readonly SolidColorBrush TitleBarOp = new SolidColorBrush(Color.FromRgb(80, 150, 80));          // Green
        private static readonly SolidColorBrush TitleBarFunc = new SolidColorBrush(Color.FromRgb(60, 140, 160));       // Teal
        private static readonly SolidColorBrush TitleBarInclude = new SolidColorBrush(Color.FromRgb(180, 80, 140));    // Magenta

        private static readonly FontFamily NodeFontFamily = new FontFamily("Segoe UI");
        private const double TitleFontSize = 11.0;
        private const double PortFontSize = 10.0;
        private const double NodeMinWidth = 80.0;
        private const double NodePadding = 4.0;
        private const double PortLabelPadding = 6.0;
        private const double PortRowSpacing = 25.0;
        private const double EdgeRewirePickRadius = 28.0;
        private const double PortHoverPadding = 12.0;
        private const double GridSize = 10.0;
        private const double ParamControlHeight = 18.0;
        private const double ParamControlWidth = 120.0;
        private const int PreviewRefreshThrottleMs = 500;
        private bool _isInspectorUpdating;
        private readonly string[] _opChoices = { "add", "sub", "mul", "div", "min", "max", "abs", "neg", "clamp", "lerp" };
        private readonly string[] _funcChoices = { "qhat_eff", "torque_norm", "rpm_norm", "assist_loss", "buffet" };
        private readonly string[] _paramWidgetChoices = { "slider", "knob", "checkbox", "enum", "text" };
        private double _curveTension = 0.5;
        private const double HandleSize = 10.0;
        private LinkVisual _draggingHandle;
        private Point _handleDragOffset;
        private bool _updatingParamValue;
        private GraphUndoStack _undoStack;
        private bool _pendingUndoDebounce;
        private bool _suppressUndoCapture;
        private bool _isRestoringUndo;
        private bool _suppressPreviewRefresh;
        private bool _previewRefreshPending;

        public event Action<string, string> IncludeOpenRequested;  // (path, includeNodeId)
        public event Action GraphChanged;
        public event Action<bool> DirtyChanged;
        public event EventHandler<string> ContextChanged;  // string = contextId or null
        public event EventHandler<bool> LiveInputsStateChanged;  // Raised when user toggles the live inputs checkbox

        public IReadOnlyList<string> OpChoices => _opChoices;
        public IReadOnlyList<string> FuncChoices => _funcChoices;
        public IReadOnlyList<string> ParamWidgetChoices => _paramWidgetChoices;
        public ObservableCollection<PortEditEntry> PortEntries => _portEntries;

        public ObservableCollection<string> IncludeInputNames { get; } = new ObservableCollection<string>();
        public ObservableCollection<string> IncludeOutputNames { get; } = new ObservableCollection<string>();

        public static readonly DependencyProperty IncludeErrorMessageProperty =
            DependencyProperty.Register(nameof(IncludeErrorMessage), typeof(string), typeof(GraphEditorControl),
                new PropertyMetadata(""));

        public string IncludeErrorMessage
        {
            get => (string)GetValue(IncludeErrorMessageProperty);
            private set => SetValue(IncludeErrorMessageProperty, value ?? "");
        }

        public static readonly DependencyProperty IncludeErrorVisibleProperty =
            DependencyProperty.Register(nameof(IncludeErrorVisible), typeof(bool), typeof(GraphEditorControl),
                new PropertyMetadata(false));

        public bool IncludeErrorVisible
        {
            get => (bool)GetValue(IncludeErrorVisibleProperty);
            private set => SetValue(IncludeErrorVisibleProperty, value);
        }

        public static readonly DependencyProperty IsLibraryGraphProperty =
            DependencyProperty.Register(nameof(IsLibraryGraph), typeof(bool), typeof(GraphEditorControl),
                new PropertyMetadata(false));

        public bool IsLibraryGraph
        {
            get => (bool)GetValue(IsLibraryGraphProperty);
            private set => SetValue(IsLibraryGraphProperty, value);
        }

        public static readonly DependencyProperty SelectedPortEntryProperty =
            DependencyProperty.Register(nameof(SelectedPortEntry), typeof(PortEditEntry), typeof(GraphEditorControl),
                new PropertyMetadata(null));

        public PortEditEntry SelectedPortEntry
        {
            get => (PortEditEntry)GetValue(SelectedPortEntryProperty);
            set => SetValue(SelectedPortEntryProperty, value);
        }

        private string _baseDirectory;
        private string _filePath;
        private string _selectedContextId;
        private bool _contextIsUserSelected;  // True if user explicitly selected a context (sticky)
        private HashSet<string> _lastContextIds = new HashSet<string>();
        private bool _hadLiveContext;
        private Func<string, IReadOnlyList<IncludeCallContext>> _contextProvider;
        public string BaseDirectory
        {
            get => _baseDirectory;
            set
            {
                _baseDirectory = value;
                UpdatePreviewResolver();
            }
        }

        /// <summary>
        /// Absolute file path for the graph being edited, or null if unsaved.
        /// Used for include context lookup.
        /// </summary>
        public string FilePath
        {
            get => _filePath;
            set
            {
                _filePath = value;
                RefreshContextDropdown(force: true);
            }
        }

        /// <summary>
        /// Provider function to get include contexts for a given file path.
        /// Set by the plugin to enable context-aware preview.
        /// </summary>
        public Func<string, IReadOnlyList<IncludeCallContext>> ContextProvider
        {
            get => _contextProvider;
            set
            {
                _contextProvider = value;
                RefreshContextDropdown(force: true);
            }
        }

        private bool _isDirty;
        public bool IsDirty
        {
            get => _isDirty;
            private set
            {
                if (_isDirty != value)
                {
                    _isDirty = value;
                    DirtyIndicator.Visibility = value ? Visibility.Visible : Visibility.Collapsed;
                    DirtyChanged?.Invoke(value);
                }
            }
        }
        public Func<IDictionary<string, double>> LiveInputProvider { get; set; }
        public Action<string, double> ParamValueChanged { get; set; }

        /// <summary>
        /// Enable debug logging to trace include evaluation flow.
        /// Log file: %LocalAppData%\DiyFfb\graph_debug.log
        /// </summary>
        public bool DebugLoggingEnabled
        {
            get => _previewEvaluator.DebugLoggingEnabled;
            set => _previewEvaluator.DebugLoggingEnabled = value;
        }

        /// <summary>
        /// Gets the path to the debug log file when debug logging is enabled.
        /// </summary>
        public string DebugLogPath => _previewEvaluator.DebugLogPath;

        public GraphEditorControl()
        {
            InitializeComponent();
            _graph = new GraphDefinition();
            _includePathDebounceTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(300) };
            _includePathDebounceTimer.Tick += OnIncludePathDebounce;
            _undoDebounceTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(300) };
            _undoDebounceTimer.Tick += OnUndoDebounce;
            _previewRefreshTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(PreviewRefreshThrottleMs) };
            _previewRefreshTimer.Tick += OnPreviewRefreshTimer;
            CanvasSurface.SizeChanged += (_, __) => UpdateCanvasExtent();
            GraphChanged += OnGraphChanged;
            UpdatePreviewResolver();  // Ensure resolver exists even for unsaved graphs
        }

        public void SetGraph(GraphDefinition graph)
        {
            _graph = graph ?? new GraphDefinition();
            _previewEvaluator.InvalidateCache();
            _hasUserPanned = false;
            IsDirty = false;
            _pendingUndoDebounce = false;
            _undoDebounceTimer.Stop();

            // Reset context selection when loading a new graph
            _contextIsUserSelected = false;
            _selectedContextId = null;
            _hadLiveContext = false;

            // Update the Library Graph checkbox to match the graph's flag
            CheckLibraryGraph.IsChecked = _graph.IsLibraryGraph;
            IsLibraryGraph = _graph.IsLibraryGraph;

            foreach (var node in _graph.Nodes)
            {
                if (node.Kind == GraphNodeKind.Func)
                {
                    EnsureFuncPorts(node);
                }
                else if (node.Kind == GraphNodeKind.Op)
                {
                    EnsureOpPorts(node, node.Op);
                }
                else if (node.Kind == GraphNodeKind.Include)
                {
                    // Sync Include node ports from included graph interface
                    SyncIncludePorts(node);
                }
            }
            RebuildSurface();
        }

        public GraphDefinition GetGraph()
        {
            return _graph;
        }

        public bool CanUndo => _undoStack?.CanUndo == true;
        public bool CanRedo => _undoStack?.CanRedo == true;

        public void SetUndoStack(GraphUndoStack undoStack)
        {
            _undoStack = undoStack;
        }

        public void InitializeUndoStack()
        {
            if (_undoStack == null)
            {
                return;
            }

            _undoStack.Reset(CreateUndoSnapshot());
            SetDirtyState(false);
        }

        public void MarkUndoClean()
        {
            _undoStack?.MarkClean();
            SetDirtyState(_undoStack?.IsDirty == true);
        }

        public bool Undo()
        {
            if (_undoStack == null)
            {
                return false;
            }

            var snapshot = _undoStack.Undo();
            if (snapshot == null)
            {
                return false;
            }

            RestoreUndoSnapshot(snapshot);
            return true;
        }

        public bool Redo()
        {
            if (_undoStack == null)
            {
                return false;
            }

            var snapshot = _undoStack.Redo();
            if (snapshot == null)
            {
                return false;
            }

            RestoreUndoSnapshot(snapshot);
            return true;
        }

        public void SetDirtyState(bool dirty)
        {
            IsDirty = dirty;
        }

        /// <summary>
        /// Returns the names of all collected parameters (local and from includes).
        /// Used to sync preview values with runtime plugin values.
        /// </summary>
        public IEnumerable<string> GetCollectedParamNames()
        {
            return _previewParamLookup.Keys;
        }

        public void UpdateNodeValues(Dictionary<string, double> values)
        {
            _nodeValues.Clear();
            if (values != null)
            {
                foreach (var pair in values)
                {
                    _nodeValues[pair.Key] = pair.Value;
                }
            }

            UpdateOutputValueLabels();
        }


        public void LoadGraphFromFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path) || !File.Exists(path))
            {
                return;
            }

            string json = File.ReadAllText(path);
            var graph = GraphSerializer.Deserialize(json, out _);
            SetGraph(graph);
        }

        private GraphUndoSnapshot CreateUndoSnapshot()
        {
            return new GraphUndoSnapshot
            {
                GraphJson = GraphSerializer.Serialize(_graph),
                View = GetViewState(),
                SelectedNodeId = _selectedNode?.Node?.Id
            };
        }

        private GraphEditorViewState GetViewState()
        {
            return new GraphEditorViewState
            {
                ScaleX = SurfaceScale.ScaleX,
                ScaleY = SurfaceScale.ScaleY,
                TranslateX = SurfaceTranslate.X,
                TranslateY = SurfaceTranslate.Y
            };
        }

        private void ApplyViewState(GraphEditorViewState view)
        {
            if (view == null)
            {
                return;
            }

            SurfaceScale.ScaleX = view.ScaleX;
            SurfaceScale.ScaleY = view.ScaleY;
            SurfaceTranslate.X = view.TranslateX;
            SurfaceTranslate.Y = view.TranslateY;
        }

        private void RestoreUndoSnapshot(GraphUndoSnapshot snapshot)
        {
            if (snapshot == null)
            {
                return;
            }

            _isRestoringUndo = true;
            _suppressUndoCapture = true;
            _isInspectorUpdating = true;
            _undoDebounceTimer.Stop();
            try
            {
                var graph = GraphSerializer.Deserialize(snapshot.GraphJson, out _);
                SetGraph(graph);
                ApplyViewState(snapshot.View);
                RestoreSelection(snapshot.SelectedNodeId);
                UpdateSelectionVisuals();
                UpdateInspector();
                RefreshPreview();
            }
            finally
            {
                _isInspectorUpdating = false;
                _suppressUndoCapture = false;
                _isRestoringUndo = false;
            }
        }

        private void RestoreSelection(string nodeId)
        {
            _selectedLinks.Clear();
            _selectedNodes.Clear();
            _selectedNode = null;

            if (!string.IsNullOrWhiteSpace(nodeId) && _nodeVisuals.TryGetValue(nodeId, out var visual))
            {
                _selectedNode = visual;
                _selectedNodes.Add(visual);
            }
        }

        private void OnGraphChanged()
        {
            _previewEvaluator.InvalidateCache();

            if (_suppressUndoCapture || _undoStack == null)
            {
                return;
            }

            if (_pendingUndoDebounce)
            {
                _pendingUndoDebounce = false;
                ScheduleUndoSnapshot();
                return;
            }

            PushUndoSnapshot();
        }

        private void ScheduleUndoSnapshot()
        {
            if (_undoStack == null)
            {
                return;
            }

            _undoDebounceTimer.Stop();
            _undoDebounceTimer.Start();
        }

        private void OnUndoDebounce(object sender, EventArgs e)
        {
            _undoDebounceTimer.Stop();
            PushUndoSnapshot();
        }

        private void PushUndoSnapshot()
        {
            if (_undoStack == null || _isRestoringUndo)
            {
                return;
            }

            var snapshot = CreateUndoSnapshot();
            var current = _undoStack.Current;
            if (current != null && string.Equals(current.GraphJson, snapshot.GraphJson, StringComparison.Ordinal))
            {
                return;
            }

            _undoStack.Push(snapshot);
        }

        public void SaveGraphToFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            string json = GraphSerializer.Serialize(_graph);
            File.WriteAllText(path, json);
            IsDirty = false;
        }

        /// <summary>
        /// Clears the dirty flag. Call after saving via external mechanism.
        /// </summary>
        public void ClearDirty()
        {
            IsDirty = false;
        }

        private void RebuildSurface()
        {
            CanvasSurface.Children.Clear();
            _nodeVisuals.Clear();
            _linkVisuals.Clear();
            _edgePreview = null;
            _selectionRect = null;

            foreach (var link in _graph.Links)
            {
                var path = new System.Windows.Shapes.Path
                {
                    Stroke = LinkBrush,
                    StrokeThickness = 2,
                    IsHitTestVisible = true,
                    ToolTip = $"{link.FromNodeId}.{link.FromPort} -> {link.ToNodeId}.{link.ToPort}"
                };
                var handle = new Ellipse
                {
                    Width = HandleSize,
                    Height = HandleSize,
                    Stroke = Brushes.Black,
                    StrokeThickness = 1,
                    Fill = LinkHandleBrush,
                    Visibility = Visibility.Collapsed,
                    Tag = link
                };
                handle.MouseLeftButtonDown += Handle_MouseLeftButtonDown;
                handle.MouseLeftButtonUp += Handle_MouseLeftButtonUp;
                handle.MouseMove += Handle_MouseMove;

                var visual = new LinkVisual { Path = path, Link = link, Handle = handle };
                _linkVisuals.Add(visual);
                path.MouseLeftButtonDown += Link_MouseLeftButtonDown;
                path.MouseEnter += Link_MouseEnter;
                path.MouseLeave += Link_MouseLeave;
                path.ContextMenuOpening += Link_ContextMenuOpening;
                CanvasSurface.Children.Add(path);
                CanvasSurface.Children.Add(handle);
            }

            foreach (var node in _graph.Nodes)
            {
                var nodeVisual = BuildNodeVisual(node);
                _nodeVisuals[node.Id] = nodeVisual;
                CanvasSurface.Children.Add(nodeVisual.Container);
            }

            EnsureSelectionRectangle();
            EnsureEdgePreview();
            UpdateAllLinkGeometry();
            UpdateCanvasExtent();
            SyncPreviewEntries();
            RefreshPreview();
            UpdateInspector();
            GraphChanged?.Invoke();
        }

        private void UpdateCanvasExtent()
        {
            const double padding = 300.0;
            double width = CanvasSurface.ActualWidth;
            double height = CanvasSurface.ActualHeight;

            double minX = 0.0;
            double minY = 0.0;
            double maxX = 0.0;
            double maxY = 0.0;
            bool hasBounds = false;

            if (_graph?.Nodes != null && _graph.Nodes.Count > 0)
            {
                foreach (var node in _graph.Nodes)
                {
                    double nodeWidth = ComputeNodeWidth(node);
                    double nodeHeight = ComputeNodeHeight(node);
                    double left = node.X;
                    double top = node.Y;
                    double right = node.X + nodeWidth;
                    double bottom = node.Y + nodeHeight;

                    if (!hasBounds)
                    {
                        minX = left;
                        minY = top;
                        maxX = right;
                        maxY = bottom;
                        hasBounds = true;
                    }
                    else
                    {
                        minX = Math.Min(minX, left);
                        minY = Math.Min(minY, top);
                        maxX = Math.Max(maxX, right);
                        maxY = Math.Max(maxY, bottom);
                    }
                }
            }

            if (hasBounds)
            {
                width = Math.Max(width, (maxX - minX) + padding * 2.0);
                height = Math.Max(height, (maxY - minY) + padding * 2.0);
            }

            CanvasSurface.Width = Math.Max(width, 800.0);
            CanvasSurface.Height = Math.Max(height, 600.0);
        }

        private NodeVisual BuildNodeVisual(GraphNode node)
        {
            var container = new Border
            {
                Background = NodeFillBrush,
                Tag = node,
                CornerRadius = new CornerRadius(6)
            };

            double width = ComputeNodeWidth(node);
            double height = ComputeNodeHeight(node);

            var nodeCanvas = new Canvas
            {
                Width = width,
                Height = height
            };

            // Add colored title bar background
            var titleBar = new Rectangle
            {
                Width = width,
                Height = 24,
                Fill = GetTitleBarColor(node.Kind),
                RadiusX = 6,
                RadiusY = 6
            };
            Canvas.SetLeft(titleBar, 0);
            Canvas.SetTop(titleBar, 0);
            nodeCanvas.Children.Add(titleBar);

            // Add clip to prevent title bar from showing outside rounded corners at bottom
            var titleBarClip = new Rectangle
            {
                Width = width,
                Height = 12,
                Fill = GetTitleBarColor(node.Kind)
            };
            Canvas.SetLeft(titleBarClip, 0);
            Canvas.SetTop(titleBarClip, 12);
            nodeCanvas.Children.Add(titleBarClip);

            var title = new TextBlock
            {
                Text = BuildNodeTitle(node),
                Foreground = Brushes.White,
                FontSize = TitleFontSize,
                FontFamily = NodeFontFamily,
                Margin = new Thickness(6, 4, 6, 4)
            };
            nodeCanvas.Children.Add(title);

            container.MouseLeftButtonDown += Node_MouseLeftButtonDown;
            container.MouseLeftButtonUp += Node_MouseLeftButtonUp;
            container.MouseMove += Node_MouseMove;
            container.MouseRightButtonDown += Node_MouseRightButtonDown;

            int inputIndex = 0;
            int outputIndex = 0;
            var outputValueLabels = new List<PortValueVisual>();
            foreach (var port in node.Ports)
            {
                var portVisual = new PortVisual
                {
                    NodeId = node.Id,
                    PortName = port.Name,
                    Kind = port.Kind
                };

                var portEllipse = new Ellipse
                {
                    Width = 10,
                    Height = 10,
                    Fill = port.Kind == GraphPortKind.Input ? Brushes.DeepSkyBlue : Brushes.Orange,
                    Stroke = Brushes.Black,
                    StrokeThickness = 1,
                    Tag = portVisual
                };

                portEllipse.MouseLeftButtonDown += Port_MouseLeftButtonDown;
                portEllipse.MouseLeftButtonUp += Port_MouseLeftButtonUp;
                portEllipse.MouseRightButtonDown += Port_MouseRightButtonDown;

                int portIndex = port.Kind == GraphPortKind.Input ? inputIndex : outputIndex;
                double y = 34 + portIndex * PortRowSpacing;
                double x = port.Kind == GraphPortKind.Input ? -5 : nodeCanvas.Width - 5;
                Canvas.SetLeft(portEllipse, x);
                Canvas.SetTop(portEllipse, y);
                nodeCanvas.Children.Add(portEllipse);

                string portLabel = GetPortDisplayLabel(node, port);
                var label = new TextBlock
                {
                    Text = portLabel,
                    Foreground = Brushes.LightGray,
                    FontSize = PortFontSize,
                    FontFamily = NodeFontFamily,
                    Tag = portVisual
                };
                double labelWidth = MeasureTextWidth(portLabel, PortFontSize);
                double labelX;
                if (port.Kind == GraphPortKind.Input)
                {
                    labelX = PortLabelPadding;
                }
                else if (node.Kind == GraphNodeKind.Param)
                {
                    labelX = width - ParamControlWidth - PortLabelPadding - labelWidth - 6;
                }
                else
                {
                    labelX = width - PortLabelPadding - labelWidth;
                }
                Canvas.SetLeft(label, labelX);
                Canvas.SetTop(label, y - 2);
                nodeCanvas.Children.Add(label);

                if (node.Kind == GraphNodeKind.Param && port.Kind == GraphPortKind.Output)
                {
                    string paramName = GetPortSignalName(node, port);
                    var param = GetOrCreateParam(node, paramName);
                    var ui = param.Ui ?? new GraphParamUi();
                    var controlInfo = BuildParamControl(param, ui, paramName, ParamControlWidth);
                    controlInfo.Control.Height = ParamControlHeight;
                    controlInfo.Control.Tag = new ParamControlTag(port.Name, controlInfo.YOffset);
                    Canvas.SetLeft(controlInfo.Control, width - ParamControlWidth - PortLabelPadding);
                    Canvas.SetTop(controlInfo.Control, y + controlInfo.YOffset);
                    nodeCanvas.Children.Add(controlInfo.Control);
                }

                if (port.Kind == GraphPortKind.Output)
                {
                    var valueLabel = new TextBlock
                    {
                        Text = "",
                        Foreground = Brushes.LightGreen,
                        FontSize = 9,
                        FontFamily = NodeFontFamily,
                        Width = 60,
                        TextTrimming = TextTrimming.CharacterEllipsis,
                        Tag = portVisual
                    };
                    Canvas.SetLeft(valueLabel, width + 6);
                    Canvas.SetTop(valueLabel, y - 2);
                    nodeCanvas.Children.Add(valueLabel);
                    outputValueLabels.Add(new PortValueVisual(port.Name, valueLabel));
                }

                if (port.Kind == GraphPortKind.Input)
                {
                    inputIndex++;
                }
                else
                {
                    outputIndex++;
                }
            }

            container.Child = nodeCanvas;
            container.Width = width;
            container.Height = height;
            Canvas.SetLeft(container, node.X);
            Canvas.SetTop(container, node.Y);
            return new NodeVisual
            {
                Container = container,
                Node = node,
                TitleBlock = title,
                InnerCanvas = nodeCanvas,
                TitleBar = titleBar,
                TitleBarClip = titleBarClip,
                OutputValues = outputValueLabels
            };
        }

        private ParamControlInfo BuildParamControl(GraphParam param, GraphParamUi ui, string paramName, double width)
        {
            string widget = (ui?.Widget ?? "").Trim().ToLowerInvariant();
            if (string.IsNullOrWhiteSpace(widget))
            {
                widget = "slider";
            }

            double yOffset = widget == "text" ? -7 : -4;
            if (widget == "checkbox")
            {
                var check = new CheckBox
                {
                    Content = string.IsNullOrWhiteSpace(ui.Label) ? "Enabled" : ui.Label,
                    Foreground = Brushes.LightGray,
                    VerticalAlignment = VerticalAlignment.Center,
                    IsChecked = param.DefaultValue > 0.5
                };
                check.Checked += (_, __) => SetParamDefault(paramName, 1.0);
                check.Unchecked += (_, __) => SetParamDefault(paramName, 0.0);
                return new ParamControlInfo(check, yOffset);
            }

            if (widget == "enum")
            {
                var combo = new ComboBox
                {
                    Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                    Foreground = Brushes.White,
                    BorderBrush = new SolidColorBrush(Color.FromRgb(74, 74, 74)),
                    BorderThickness = new Thickness(1),
                    ItemsSource = ui.Options,
                    DisplayMemberPath = "Label",
                    SelectedValuePath = "Value",
                    Width = width
                };
                combo.SelectedItem = FindOptionForValue(ui.Options, param.DefaultValue);
                combo.SelectionChanged += (_, __) =>
                {
                    if (combo.SelectedItem is GraphParamOption option)
                    {
                        double value = ParseOptionValue(option.Value, param.DefaultValue);
                        SetParamDefault(paramName, value);
                    }
                };
                return new ParamControlInfo(combo, yOffset);
            }

            if (widget == "slider" || widget == "knob")
            {
                double min;
                double max;
                GetParamRange(param, out min, out max);
                var slider = new Slider
                {
                    Minimum = min,
                    Maximum = max,
                    Value = param.DefaultValue,
                    Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                    Foreground = Brushes.White,
                    Width = width,
                    Margin = new Thickness(0, 0, 0, 0)
                };
                if (ui.Step.HasValue && ui.Step.Value > 0)
                {
                    slider.SmallChange = ui.Step.Value;
                }
                if (widget == "knob")
                {
                    slider.IsSnapToTickEnabled = ui.Step.HasValue && ui.Step.Value > 0;
                    slider.TickFrequency = ui.Step.HasValue && ui.Step.Value > 0 ? ui.Step.Value : (max - min) / 10.0;
                    slider.BorderBrush = new SolidColorBrush(Color.FromRgb(90, 90, 120));
                    slider.BorderThickness = new Thickness(1);
                }
                slider.ValueChanged += (_, __) => SetParamDefault(paramName, slider.Value);
                return new ParamControlInfo(slider, yOffset);
            }

            var text = new TextBox
            {
                Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                Foreground = Brushes.White,
                BorderBrush = new SolidColorBrush(Color.FromRgb(74, 74, 74)),
                BorderThickness = new Thickness(1),
                Padding = new Thickness(0, 0, 0, 0),
                Width = width,
                Height = ParamControlHeight,
                Text = param.DefaultValue.ToString("F3", CultureInfo.InvariantCulture)
            };
            text.TextChanged += (_, __) =>
            {
                if (TryParseDouble(text.Text, out var value))
                {
                    SetParamDefault(paramName, value);
                }
            };
            return new ParamControlInfo(text, yOffset);
        }

        private static GraphParamOption FindOptionForValue(List<GraphParamOption> options, double value)
        {
            if (options == null)
            {
                return null;
            }

            foreach (var option in options)
            {
                double parsed = ParseOptionValue(option.Value, double.NaN);
                if (!double.IsNaN(parsed) && Math.Abs(parsed - value) < 1e-6)
                {
                    return option;
                }
            }
            return options.Count > 0 ? options[0] : null;
        }

        private static double ParseOptionValue(string text, double fallback)
        {
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }
            return fallback;
        }

        private void SetParamDefault(string name, double value)
        {
            // Prevent recursion when updating from external source
            if (_updatingParamValue)
            {
                return;
            }

            if (_graph.Params.TryGetValue(name, out var param))
            {
                param.DefaultValue = value;
            }
            else
            {
                _graph.Params[name] = new GraphParam { Name = name, DefaultValue = value };
            }

            SyncPreviewEntries();
            if (_previewParamLookup.TryGetValue(name, out var entry))
            {
                entry.Value = value;
            }

            // Notify external listeners (e.g., plugin)
            ParamValueChanged?.Invoke(name, value);

            // Mark graph as changed (dirty)
            GraphChanged?.Invoke();
        }

        public void UpdateParamValue(string name, double value)
        {
            _updatingParamValue = true;
            try
            {
                // Track this param as explicitly updated (for context preview overlay)
                _explicitlyUpdatedParams.Add(name);

                // Update graph param
                if (_graph.Params.TryGetValue(name, out var param))
                {
                    param.DefaultValue = value;
                }

                // Update preview entry
                if (_previewParamLookup.TryGetValue(name, out var entry))
                {
                    entry.Value = value;
                }

                // Update the control for this parameter (find and update slider/textbox/etc)
                foreach (var nodeVisual in _nodeVisuals.Values)
                {
                    if (nodeVisual.Node.Kind == GraphNodeKind.Param && nodeVisual.Node.Ports.Count > 0)
                    {
                        // Check ALL ports, not just the first one (param nodes can have multiple output ports)
                        foreach (var port in nodeVisual.Node.Ports)
                        {
                            // Use full hierarchical signal name for matching (e.g., "XPlane.IAS_kts")
                            string portSignalName = GetPortSignalName(nodeVisual.Node, port);
                            if (portSignalName == name && port.Kind == GraphPortKind.Output)
                            {
                                // Find the control in the node's inner canvas children
                                foreach (var child in nodeVisual.InnerCanvas.Children)
                                {
                                    if (child is FrameworkElement element && element.Tag is ParamControlTag tag && tag.PortName == port.Name)
                                    {
                                        if (element is Slider slider)
                                        {
                                            slider.Value = value;
                                            slider.UpdateLayout();
                                        }
                                        else if (element is TextBox textBox)
                                        {
                                            int precision = param.Ui?.Precision ?? 3;
                                            textBox.Text = value.ToString($"F{precision}", CultureInfo.InvariantCulture);
                                        }
                                        else if (element is CheckBox checkBox)
                                        {
                                            checkBox.IsChecked = value > 0.5;
                                        }
                                        else if (element is ComboBox comboBox)
                                        {
                                            comboBox.SelectedItem = FindOptionForValue(param.Ui?.Options, value);
                                        }
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }
            finally
            {
                _updatingParamValue = false;
            }

            // Refresh preview to show updated param values on node outputs
            RefreshPreview();
        }

        private static int GetPortRowCount(GraphNode node)
        {
            int inputCount = 0;
            int outputCount = 0;
            foreach (var port in node.Ports)
            {
                if (port.Kind == GraphPortKind.Input)
                {
                    inputCount++;
                }
                else
                {
                    outputCount++;
                }
            }
            return Math.Max(1, Math.Max(inputCount, outputCount));
        }

        private sealed class ParamControlTag
        {
            public string PortName { get; }
            public double YOffset { get; }

            public ParamControlTag(string portName, double yOffset)
            {
                PortName = portName;
                YOffset = yOffset;
            }
        }

        private sealed class ParamControlInfo
        {
            public FrameworkElement Control { get; }
            public double YOffset { get; }

            public ParamControlInfo(FrameworkElement control, double yOffset)
            {
                Control = control;
                YOffset = yOffset;
            }
        }

        private static int GetOutputPortIndex(GraphNode node, string portName)
        {
            if (node == null)
            {
                return -1;
            }

            int index = 0;
            foreach (var port in node.Ports)
            {
                if (port.Kind == GraphPortKind.Output)
                {
                    if (port.Name == portName)
                    {
                        return index;
                    }
                    index++;
                }
            }
            return -1;
        }

        private static void GetParamRange(GraphParam param, out double min, out double max)
        {
            min = param.Min;
            max = param.Max;
            if (Math.Abs(max - min) < 1e-9)
            {
                max = min + 1.0;
            }
        }

        private void UpdateAllLinkGeometry()
        {
            foreach (var linkVisual in _linkVisuals)
            {
                if (!_nodeVisuals.TryGetValue(linkVisual.Link.FromNodeId, out var fromNode) ||
                    !_nodeVisuals.TryGetValue(linkVisual.Link.ToNodeId, out var toNode))
                {
                    continue;
                }

                Point from = GetPortAnchor(fromNode, linkVisual.Link.FromPort, GraphPortKind.Output);
                Point to = GetPortAnchor(toNode, linkVisual.Link.ToPort, GraphPortKind.Input);
                linkVisual.Path.Data = BuildLinkGeometry(from, to, linkVisual);
                UpdateHandlePosition(linkVisual, from, to);
            }
        }

        private Point GetPortAnchor(NodeVisual nodeVisual, string portName, GraphPortKind kind)
        {
            var node = nodeVisual.Node;
            int index = 0;
            foreach (var port in node.Ports)
            {
                if (port.Kind == kind && port.Name == portName)
                {
                    double x = node.X + (kind == GraphPortKind.Input ? 0 : nodeVisual.Container.Width);
                    double y = node.Y + 34 + index * PortRowSpacing + 5;
                    return new Point(x, y);
                }
                if (port.Kind == kind)
                {
                    index++;
                }
            }

            return new Point(node.X, node.Y);
        }

        private void CanvasSurface_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Focus();
            _isSelecting = true;
            _selectionStart = e.GetPosition(CanvasSurface);
            UpdateSelectionRectangle(_selectionStart, _selectionStart);
            _selectionRect.Visibility = Visibility.Visible;
            CanvasSurface.CaptureMouse();
            e.Handled = true;
        }

        private void CanvasSurface_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_isSelecting)
            {
                _isSelecting = false;
                CanvasSurface.ReleaseMouseCapture();
                _selectionRect.Visibility = Visibility.Collapsed;
                SelectNodesInRectangle();
                UpdateInspector();
                return;
            }
            CancelEdgeDrag();
            CancelEdgePreview();
        }

        private void CanvasSurface_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isSelecting)
            {
                Point selectionPoint = e.GetPosition(CanvasSurface);
                UpdateSelectionRectangle(_selectionStart, selectionPoint);
                return;
            }

            if (_edgeDragLink != null)
            {
                UpdateEdgePreview(e.GetPosition(CanvasSurface));
                return;
            }

            if (_pendingPort != null)
            {
                UpdateEdgePreview(e.GetPosition(CanvasSurface));
                return;
            }

            UpdateHoverPortVisibility(e.GetPosition(CanvasSurface));

            if (_isPanning && _dragNode == null)
            {
                Point panPoint = e.GetPosition(this);
                Vector delta = panPoint - _lastPanPoint;
                if (Math.Abs(delta.X) > 1.0 || Math.Abs(delta.Y) > 1.0)
                {
                    _panWasDragged = true;
                    _hasUserPanned = true;
                }
                SurfaceTranslate.X += delta.X;
                SurfaceTranslate.Y += delta.Y;
                _lastPanPoint = panPoint;
            }
        }

        private void CanvasSurface_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            double zoom = e.Delta > 0 ? 1.1 : 0.9;
            SurfaceScale.ScaleX *= zoom;
            SurfaceScale.ScaleY *= zoom;
        }

        private void Node_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is Border border && border.Tag is GraphNode node)
            {
                // Handle double-click on Include nodes
                if (e.ClickCount == 2 && node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    // Pass node.Id so the new tab can auto-select this context when live mode is active
                    string contextId = _liveInputsEnabled ? node.Id : null;
                    IncludeOpenRequested?.Invoke(node.IncludePath, contextId);
                    e.Handled = true;
                    return;
                }

                _dragNode = _nodeVisuals[node.Id];
                bool shift = (Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift;
                bool clickedSelected = _selectedNodes.Contains(_dragNode);
                if (!shift)
                {
                    _selectedLinks.Clear();
                    UpdateLinkSelectionVisuals();
                }
                if (shift)
                {
                    if (_selectedNodes.Contains(_dragNode))
                    {
                        _selectedNodes.Remove(_dragNode);
                    }
                    else
                    {
                        _selectedNodes.Add(_dragNode);
                    }
                }
                else
                {
                    if (!clickedSelected)
                    {
                        _selectedNodes.Clear();
                        _selectedNodes.Add(_dragNode);
                    }
                }

                _selectedNode = _selectedNodes.Count == 1 ? _dragNode : null;
                UpdateSelectionVisuals();
                UpdateInspector();
                Point mousePos = e.GetPosition(CanvasSurface);
                _dragOffset = new Point(mousePos.X - node.X, mousePos.Y - node.Y);
                _dragStartPoint = mousePos;
                _dragNodes = null;
                _dragNodeStartPositions.Clear();
                if (_selectedNodes.Count > 1 && _selectedNodes.Contains(_dragNode))
                {
                    _dragNodes = _selectedNodes.ToList();
                    foreach (var selected in _dragNodes)
                    {
                        _dragNodeStartPositions[selected] = new Point(selected.Node.X, selected.Node.Y);
                    }
                }
                border.CaptureMouse();
                e.Handled = true;
            }
        }

        private void Node_MouseMove(object sender, MouseEventArgs e)
        {
            if (_dragNode == null)
            {
                return;
            }

            Point mousePos = e.GetPosition(CanvasSurface);
            if (_dragNodes != null && _dragNodes.Count > 1)
            {
                Vector delta = mousePos - _dragStartPoint;
                foreach (var selected in _dragNodes)
                {
                    if (!_dragNodeStartPositions.TryGetValue(selected, out var start))
                    {
                        continue;
                    }
                    selected.Node.X = SnapToGrid(start.X + delta.X);
                    selected.Node.Y = SnapToGrid(start.Y + delta.Y);
                    Canvas.SetLeft(selected.Container, selected.Node.X);
                    Canvas.SetTop(selected.Container, selected.Node.Y);
                }
            }
            else
            {
                _dragNode.Node.X = SnapToGrid(mousePos.X - _dragOffset.X);
                _dragNode.Node.Y = SnapToGrid(mousePos.Y - _dragOffset.Y);
                Canvas.SetLeft(_dragNode.Container, _dragNode.Node.X);
                Canvas.SetTop(_dragNode.Container, _dragNode.Node.Y);
            }
            UpdateAllLinkGeometry();
        }

        private void Node_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_dragNode != null)
            {
                // Check if any node actually moved
                bool moved = false;
                if (_dragNodes != null && _dragNodes.Count > 0)
                {
                    foreach (var node in _dragNodes)
                    {
                        if (_dragNodeStartPositions.TryGetValue(node, out var start))
                        {
                            if (Math.Abs(node.Node.X - start.X) > 0.1 || Math.Abs(node.Node.Y - start.Y) > 0.1)
                            {
                                moved = true;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    // Single node drag - check against drag start (compare snapped positions)
                    var currentSnapped = new Point(SnapToGrid(_dragNode.Node.X), SnapToGrid(_dragNode.Node.Y));
                    var expectedSnapped = new Point(SnapToGrid(_dragStartPoint.X - _dragOffset.X), SnapToGrid(_dragStartPoint.Y - _dragOffset.Y));
                    if (Math.Abs(currentSnapped.X - expectedSnapped.X) > 0.1 || Math.Abs(currentSnapped.Y - expectedSnapped.Y) > 0.1)
                    {
                        moved = true;
                    }
                }

                _dragNode.Container.ReleaseMouseCapture();
                _dragNode = null;
                _dragNodes = null;
                _dragNodeStartPositions.Clear();

                if (moved)
                {
                    GraphChanged?.Invoke();
                }
            }
        }

        private void Port_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is Ellipse ellipse && ellipse.Tag is PortVisual port)
            {
                _pendingPort = port;
                _edgePreviewSource = port;
                StartEdgePreview(port);
                e.Handled = true;
            }
        }

        private void Port_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_edgeDragLink != null)
            {
                if (sender is Ellipse dragEllipse && dragEllipse.Tag is PortVisual dragTarget &&
                    dragTarget.Kind == GraphPortKind.Input)
                {
                    _graph.Links.RemoveAll(link => link.ToNodeId == dragTarget.NodeId && link.ToPort == dragTarget.PortName);
                    _edgeDragLink.Link.ToNodeId = dragTarget.NodeId;
                    _edgeDragLink.Link.ToPort = dragTarget.PortName;
                    _edgeDragLink = null;
                    _edgeDragFromSource = false;
                    HideEdgePreview();
                    RebuildSurface();
                    GraphChanged?.Invoke();
                    return;
                }
                if (_edgeDragFromSource &&
                    sender is Ellipse sourceEllipse && sourceEllipse.Tag is PortVisual sourceTarget &&
                    sourceTarget.Kind == GraphPortKind.Output)
                {
                    _edgeDragLink.Link.FromNodeId = sourceTarget.NodeId;
                    _edgeDragLink.Link.FromPort = sourceTarget.PortName;
                    _edgeDragLink = null;
                    _edgeDragFromSource = false;
                    HideEdgePreview();
                    RebuildSurface();
                    GraphChanged?.Invoke();
                    return;
                }
            }

            if (_pendingPort == null)
            {
                CancelEdgePreview();
                return;
            }

            if (sender is Ellipse ellipse && ellipse.Tag is PortVisual targetPort)
            {
                if (_pendingPort.Kind != targetPort.Kind)
                {
                    var from = _pendingPort.Kind == GraphPortKind.Output ? _pendingPort : targetPort;
                    var to = _pendingPort.Kind == GraphPortKind.Output ? targetPort : _pendingPort;
                    _graph.Links.RemoveAll(existing => existing.ToNodeId == to.NodeId && existing.ToPort == to.PortName);
                    var newLink = new GraphLink
                    {
                        FromNodeId = from.NodeId,
                        FromPort = from.PortName,
                        ToNodeId = to.NodeId,
                        ToPort = to.PortName
                    };
                    _graph.Links.Add(newLink);
                    RebuildSurface();
                    GraphChanged?.Invoke();
                }
            }

            _pendingPort = null;
            CancelEdgePreview();
        }

        private void Port_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is Ellipse ellipse && ellipse.Tag is PortVisual port)
            {
                if (port.Kind != GraphPortKind.Output)
                {
                    return;
                }

                if (_nodeVisuals.TryGetValue(port.NodeId, out var node))
                {
                    _selectedNodes.Clear();
                    _selectedNodes.Add(node);
                    UpdateSelectionVisuals();
                }

                _pendingPort = port;
                _edgePreviewSource = port;
                StartEdgePreview(port);
                e.Handled = true;
            }
        }

        private void CanvasSurface_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            _isPanning = true;
            _panWasDragged = false;
            _lastPanPoint = e.GetPosition(this);
            CanvasSurface.CaptureMouse();
            e.Handled = true;
        }

        private void CanvasSurface_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_isPanning)
            {
                _isPanning = false;
                CanvasSurface.ReleaseMouseCapture();
            }

            if (!_panWasDragged)
            {
                var menu = BuildContextMenu(e.GetPosition(CanvasSurface));
                menu.IsOpen = true;
            }
            e.Handled = true;
        }

        private void CanvasSurface_ContextMenuOpening(object sender, ContextMenuEventArgs e)
        {
            if (CanvasSurface.ContextMenu == null)
            {
                CanvasSurface.ContextMenu = BuildContextMenu(Mouse.GetPosition(CanvasSurface));
            }
        }

        private ContextMenu BuildContextMenu(Point position)
        {
            var menu = new ContextMenu();

            // Edit operations
            var copyItem = BuildMenuItem("Copy", CopySelectedToClipboard);
            copyItem.InputGestureText = "Ctrl+C";
            copyItem.IsEnabled = _selectedNodes.Count > 0;
            menu.Items.Add(copyItem);

            var pasteItem = BuildMenuItem("Paste", () => PasteFromClipboard(position));
            pasteItem.InputGestureText = "Ctrl+V";
            pasteItem.IsEnabled = Clipboard.ContainsData(GraphClipboardData.ClipboardFormat) || Clipboard.ContainsText();
            menu.Items.Add(pasteItem);

            var cutItem = BuildMenuItem("Cut", CutSelectedToClipboard);
            cutItem.InputGestureText = "Ctrl+X";
            cutItem.IsEnabled = _selectedNodes.Count > 0;
            menu.Items.Add(cutItem);

            var deleteItem = BuildMenuItem("Delete", () => { DeleteSelectedLinks(); DeleteSelectedNodes(); });
            deleteItem.InputGestureText = "Del";
            deleteItem.IsEnabled = _selectedNodes.Count > 0 || _selectedLinks.Count > 0;
            menu.Items.Add(deleteItem);

            menu.Items.Add(new Separator());

            // Add nodes
            menu.Items.Add(BuildMenuItem("Add Input", () => AddNode(GraphNodeKind.Input, position)));
            menu.Items.Add(BuildMenuItem("Add Param", () => AddNode(GraphNodeKind.Param, position)));
            menu.Items.Add(BuildMenuItem("Add Const", () => AddNode(GraphNodeKind.Const, position)));
            menu.Items.Add(BuildMenuItem("Add Op", () => AddNode(GraphNodeKind.Op, position)));
            menu.Items.Add(BuildMenuItem("Add Func", () => AddNode(GraphNodeKind.Func, position)));
            menu.Items.Add(BuildMenuItem("Add Include", () => AddNode(GraphNodeKind.Include, position)));
            menu.Items.Add(BuildMenuItem("Add Output", () => AddNode(GraphNodeKind.Output, position)));
            menu.Items.Add(new Separator());
            menu.Items.Add(BuildMenuItem("Zoom to Fit", ZoomToFit));
            menu.Items.Add(BuildMenuItem("Align Left", AlignSelectedLeft));
            menu.Items.Add(BuildMenuItem("Align Top", AlignSelectedTop));
            menu.Items.Add(BuildMenuItem("Distribute Horizontally", DistributeSelectedHorizontally));
            menu.Items.Add(BuildMenuItem("Distribute Vertically", DistributeSelectedVertically));
            menu.Items.Add(new Separator());
            menu.Items.Add(BuildMenuItem("Increase Edge Curvature", () => AdjustCurveTension(0.1)));
            menu.Items.Add(BuildMenuItem("Decrease Edge Curvature", () => AdjustCurveTension(-0.1)));
            return menu;
        }

        private MenuItem BuildMenuItem(string header, Action action)
        {
            var item = new MenuItem { Header = header };
            item.Click += (_, __) => action();
            return item;
        }

        private void AddNode(GraphNodeKind kind, Point position)
        {
            var node = new GraphNode
            {
                Kind = kind,
                Title = kind.ToString(),
                X = position.X,
                Y = position.Y
            };

            if (kind == GraphNodeKind.Input)
            {
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Param)
            {
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Op)
            {
                node.Op = "mul";
                node.Ports.Add(new GraphPort { Name = "a", Kind = GraphPortKind.Input });
                node.Ports.Add(new GraphPort { Name = "b", Kind = GraphPortKind.Input });
                node.Ports.Add(new GraphPort { Name = GetOpOutputName(node.Op), Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Func)
            {
                node.Func = _funcChoices[0];
                EnsureFuncPorts(node);
            }
            else if (kind == GraphNodeKind.Const)
            {
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Include)
            {
                node.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Output)
            {
                node.Ports.Add(new GraphPort { Name = "in", Kind = GraphPortKind.Input });
            }

            _graph.Nodes.Add(node);
            var visual = BuildNodeVisual(node);
            _nodeVisuals[node.Id] = visual;
            CanvasSurface.Children.Add(visual.Container);
            _selectedNode = visual;
            _selectedNodes.Clear();
            _selectedNodes.Add(visual);
            UpdateSelectionVisuals();
            SyncPreviewEntries();
            RefreshPreview();
            UpdateInspector();
            GraphChanged?.Invoke();
        }

        private void UpdateInspector()
        {
            if (_selectedNode == null)
            {
                TextNodeTitle.Text = _selectedNodes.Count > 1 ? "(multiple)" : "(none)";
                InspectorContent.Content = null;
                InspectorContent.Visibility = Visibility.Collapsed;
                TextNoSelection.Visibility = _selectedNodes.Count > 0 ? Visibility.Collapsed : Visibility.Visible;
                return;
            }

            var node = _selectedNode.Node;
            TextNodeTitle.Text = BuildSelectedNodeLabel(node);

            InspectorContent.Content = node;
            InspectorContent.Visibility = UsesTemplateInspector(node) ? Visibility.Visible : Visibility.Collapsed;
            TextNoSelection.Visibility = Visibility.Collapsed;
            if (UsesTemplateInspector(node) && (node.Kind == GraphNodeKind.Input ||
                                                node.Kind == GraphNodeKind.Output ||
                                                node.Kind == GraphNodeKind.Param ||
                                                node.Kind == GraphNodeKind.Op))
            {
                SyncPortEntries(node);
            }
            if (UsesTemplateInspector(node) && node.Kind == GraphNodeKind.Include)
            {
                RebuildIncludePortEditors(node);
            }
        }

        private void GraphEditorControl_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.OriginalSource is TextBoxBase || e.OriginalSource is ComboBox)
            {
                return;
            }

            if (e.Key == Key.Delete)
            {
                DeleteSelectedLinks();
                DeleteSelectedNodes();
                e.Handled = true;
            }
            else if (e.Key == Key.F)
            {
                ZoomToFit();
                e.Handled = true;
            }
            else if (e.Key == Key.L)
            {
                AlignSelectedLeft();
                e.Handled = true;
            }
            else if (e.Key == Key.T)
            {
                AlignSelectedTop();
                e.Handled = true;
            }
            else if (e.Key == Key.C && Keyboard.Modifiers == ModifierKeys.Control)
            {
                CopySelectedToClipboard();
                e.Handled = true;
            }
            else if (e.Key == Key.V && Keyboard.Modifiers == ModifierKeys.Control)
            {
                // Paste at mouse position if over canvas
                var mousePos = Mouse.GetPosition(CanvasSurface);
                bool inCanvas = mousePos.X >= 0 && mousePos.Y >= 0
                             && mousePos.X <= CanvasSurface.ActualWidth
                             && mousePos.Y <= CanvasSurface.ActualHeight;
                PasteFromClipboard(inCanvas ? (Point?)mousePos : null);
                e.Handled = true;
            }
            else if (e.Key == Key.X && Keyboard.Modifiers == ModifierKeys.Control)
            {
                CutSelectedToClipboard();
                e.Handled = true;
            }
        }

        private void DeleteSelectedNodes()
        {
            if (_selectedNodes.Count == 0)
            {
                return;
            }

            var removeIds = new HashSet<string>(_selectedNodes.Select(n => n.Node.Id));
            _graph.Nodes.RemoveAll(node => removeIds.Contains(node.Id));
            _graph.Links.RemoveAll(link => removeIds.Contains(link.FromNodeId) || removeIds.Contains(link.ToNodeId));
            _selectedNodes.Clear();
            _selectedNode = null;
            RebuildSurface();
        }

        private void DeleteSelectedLinks()
        {
            if (_selectedLinks.Count == 0)
            {
                return;
            }

            foreach (var link in _selectedLinks)
            {
                _graph.Links.Remove(link.Link);
            }
            _selectedLinks.Clear();
            RebuildSurface();
        }

        #region Clipboard Operations

        private void CopySelectedToClipboard()
        {
            if (_selectedNodes.Count == 0)
                return;

            var selectedIds = new HashSet<string>(_selectedNodes.Select(nv => nv.Node.Id));

            // Clone selected nodes (keep original IDs; remap on paste)
            var copiedNodes = _selectedNodes.Select(nv => CloneNode(nv.Node)).ToList();

            // For Include nodes, convert relative paths to absolute for cross-graph paste
            foreach (var node in copiedNodes)
            {
                if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    node.IncludePath = ResolveToAbsolutePath(node.IncludePath);
                }
            }

            // Only copy links where both endpoints are in selection
            var copiedLinks = _graph.Links
                .Where(link => selectedIds.Contains(link.FromNodeId) && selectedIds.Contains(link.ToNodeId))
                .Select(CloneLink)
                .ToList();

            // Copy params for Param nodes
            var copiedParams = new Dictionary<string, GraphParam>();
            foreach (var nv in _selectedNodes)
            {
                if (nv.Node.Kind == GraphNodeKind.Param)
                {
                    foreach (var port in nv.Node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        string paramName = GetPortSignalName(nv.Node, port);
                        if (_graph.Params.TryGetValue(paramName, out var param) && !copiedParams.ContainsKey(paramName))
                        {
                            copiedParams[paramName] = CloneParam(param);
                        }
                    }
                }
            }

            // Calculate selection center for paste positioning
            var bounds = GetSelectionBounds();
            var clipboardData = new GraphClipboardData
            {
                Nodes = copiedNodes,
                Links = copiedLinks,
                Params = copiedParams,
                CenterX = bounds.X + bounds.Width / 2,
                CenterY = bounds.Y + bounds.Height / 2
            };

            // Serialize and set to Windows clipboard
            var json = GraphClipboardSerializer.Serialize(clipboardData);
            var dataObject = new DataObject();
            dataObject.SetData(GraphClipboardData.ClipboardFormat, json);
            dataObject.SetData(DataFormats.Text, json);
            Clipboard.SetDataObject(dataObject, true);
        }

        private string ResolveToAbsolutePath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                return path;

            // Already absolute
            if (System.IO.Path.IsPathRooted(path))
                return path;

            // Resolve relative to base directory
            if (!string.IsNullOrWhiteSpace(_baseDirectory))
            {
                try
                {
                    return System.IO.Path.GetFullPath(System.IO.Path.Combine(_baseDirectory, path));
                }
                catch
                {
                    return path;
                }
            }

            return path;
        }

        private void PasteFromClipboard(Point? targetPosition = null)
        {
            // Check for our clipboard format
            string json = null;
            if (Clipboard.ContainsData(GraphClipboardData.ClipboardFormat))
            {
                json = Clipboard.GetData(GraphClipboardData.ClipboardFormat) as string;
            }
            else if (Clipboard.ContainsText())
            {
                // Try to parse text as graph clipboard data
                json = Clipboard.GetText();
            }

            if (string.IsNullOrEmpty(json))
                return;

            GraphClipboardData clipboardData;
            try
            {
                clipboardData = GraphClipboardSerializer.Deserialize(json);
            }
            catch
            {
                return; // Invalid clipboard data
            }

            if (clipboardData == null || clipboardData.Nodes.Count == 0)
                return;

            // Generate new IDs and build remapping
            var idRemap = new Dictionary<string, string>();
            foreach (var node in clipboardData.Nodes)
            {
                var oldId = node.Id;
                var newId = Guid.NewGuid().ToString("N");
                idRemap[oldId] = newId;
                node.Id = newId;
            }

            // Remap link node references
            foreach (var link in clipboardData.Links)
            {
                if (idRemap.TryGetValue(link.FromNodeId, out var newFromId))
                    link.FromNodeId = newFromId;
                if (idRemap.TryGetValue(link.ToNodeId, out var newToId))
                    link.ToNodeId = newToId;
            }

            // For Include nodes, convert absolute paths back to relative and clear stale cache
            foreach (var node in clipboardData.Nodes)
            {
                if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    node.IncludePath = MakeRelativePath(_baseDirectory, node.IncludePath);
                    // Clear cached interface - it was from the source graph context and is now stale
                    node.CachedInterface = null;
                }
            }

            // Calculate position offset
            double offsetX, offsetY;
            if (targetPosition.HasValue)
            {
                offsetX = targetPosition.Value.X - clipboardData.CenterX;
                offsetY = targetPosition.Value.Y - clipboardData.CenterY;
            }
            else
            {
                // Default: offset by fixed amount
                offsetX = 50;
                offsetY = 50;
            }

            // Apply offset to all nodes
            foreach (var node in clipboardData.Nodes)
            {
                node.X += offsetX;
                node.Y += offsetY;
            }

            // Add nodes and links to graph
            foreach (var node in clipboardData.Nodes)
                _graph.Nodes.Add(node);

            foreach (var link in clipboardData.Links)
                _graph.Links.Add(link);

            // Sync ports for Include nodes (must be done after adding to graph)
            foreach (var node in clipboardData.Nodes)
            {
                if (node.Kind == GraphNodeKind.Include)
                {
                    SyncIncludePorts(node);
                }
            }

            // Merge params (don't overwrite existing params with same name)
            if (clipboardData.Params != null)
            {
                foreach (var kvp in clipboardData.Params)
                {
                    if (!_graph.Params.ContainsKey(kvp.Key))
                    {
                        _graph.Params[kvp.Key] = kvp.Value;
                    }
                }
            }

            // Rebuild visuals
            RebuildSurface();

            // Select newly pasted nodes
            _selectedNodes.Clear();
            _selectedLinks.Clear();
            foreach (var node in clipboardData.Nodes)
            {
                if (_nodeVisuals.TryGetValue(node.Id, out var visual))
                    _selectedNodes.Add(visual);
            }
            _selectedNode = _selectedNodes.Count == 1 ? _selectedNodes.First() : null;
            UpdateSelectionVisuals();
        }

        private void CutSelectedToClipboard()
        {
            CopySelectedToClipboard();
            DeleteSelectedLinks();
            DeleteSelectedNodes();
        }

        private Rect GetSelectionBounds()
        {
            if (_selectedNodes.Count == 0)
                return Rect.Empty;

            double minX = double.MaxValue, minY = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue;

            foreach (var nv in _selectedNodes)
            {
                minX = Math.Min(minX, nv.Node.X);
                minY = Math.Min(minY, nv.Node.Y);
                maxX = Math.Max(maxX, nv.Node.X + nv.Container.ActualWidth);
                maxY = Math.Max(maxY, nv.Node.Y + nv.Container.ActualHeight);
            }

            return new Rect(minX, minY, maxX - minX, maxY - minY);
        }

        private static GraphNode CloneNode(GraphNode source)
        {
            var clone = new GraphNode
            {
                Id = source.Id,
                Title = source.Title,
                Kind = source.Kind,
                X = source.X,
                Y = source.Y,
                Op = source.Op,
                Func = source.Func,
                IncludePath = source.IncludePath,
                ConstValue = source.ConstValue,
                SignalGroup = source.SignalGroup,
                CachedInterface = source.CachedInterface
            };

            foreach (var port in source.Ports)
            {
                clone.Ports.Add(new GraphPort
                {
                    Name = port.Name,
                    Kind = port.Kind,
                    SignalSuffix = port.SignalSuffix,
                    Negate = port.Negate
                });
            }

            return clone;
        }

        private static GraphLink CloneLink(GraphLink source)
        {
            return new GraphLink
            {
                FromNodeId = source.FromNodeId,
                FromPort = source.FromPort,
                ToNodeId = source.ToNodeId,
                ToPort = source.ToPort
            };
        }

        private static GraphParam CloneParam(GraphParam source)
        {
            var clone = new GraphParam
            {
                Name = source.Name,
                DefaultValue = source.DefaultValue,
                Min = source.Min,
                Max = source.Max
            };

            if (source.Ui != null)
            {
                clone.Ui = new GraphParamUi
                {
                    Widget = source.Ui.Widget,
                    Label = source.Ui.Label,
                    Group = source.Ui.Group,
                    Units = source.Ui.Units,
                    Step = source.Ui.Step,
                    Precision = source.Ui.Precision,
                    LogScale = source.Ui.LogScale
                };

                foreach (var option in source.Ui.Options)
                {
                    clone.Ui.Options.Add(new GraphParamOption
                    {
                        Value = option.Value,
                        Label = option.Label
                    });
                }
            }

            return clone;
        }

        #endregion

        private void EnsureSelectionRectangle()
        {
            if (_selectionRect != null)
            {
                CanvasSurface.Children.Add(_selectionRect);
                return;
            }

            _selectionRect = new Rectangle
            {
                Stroke = new SolidColorBrush(Color.FromRgb(120, 180, 255)),
                StrokeThickness = 1,
                StrokeDashArray = new DoubleCollection { 4, 2 },
                Fill = new SolidColorBrush(Color.FromArgb(48, 80, 120, 200)),
                Visibility = Visibility.Collapsed,
                IsHitTestVisible = false
            };
            CanvasSurface.Children.Add(_selectionRect);
        }

        private void UpdateSelectionRectangle(Point start, Point end)
        {
            double x = Math.Min(start.X, end.X);
            double y = Math.Min(start.Y, end.Y);
            double width = Math.Abs(start.X - end.X);
            double height = Math.Abs(start.Y - end.Y);
            Canvas.SetLeft(_selectionRect, x);
            Canvas.SetTop(_selectionRect, y);
            _selectionRect.Width = width;
            _selectionRect.Height = height;
        }

        private void SelectNodesInRectangle()
        {
            var rect = new Rect(Canvas.GetLeft(_selectionRect), Canvas.GetTop(_selectionRect),
                _selectionRect.Width, _selectionRect.Height);
            _selectedNodes.Clear();

            foreach (var node in _nodeVisuals.Values)
            {
                var bounds = new Rect(node.Node.X, node.Node.Y, node.Container.Width, node.Container.Height);
                if (rect.IntersectsWith(bounds))
                {
                    _selectedNodes.Add(node);
                }
            }

            _selectedNode = _selectedNodes.Count == 1 ? _selectedNodes.First() : null;
            UpdateSelectionVisuals();
        }

        private void UpdateSelectionVisuals()
        {
            foreach (var node in _nodeVisuals.Values)
            {
                node.Container.Background = _selectedNodes.Contains(node) ? NodeSelectedBrush : NodeFillBrush;
                UpdatePortHandleVisibility(node);
            }
            UpdateLinkSelectionVisuals();
        }

        private void UpdateLinkSelectionVisuals()
        {
            foreach (var link in _linkVisuals)
            {
                ApplyLinkStyle(link);
            }
        }

        private void Link_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is System.Windows.Shapes.Path path)
            {
                var link = _linkVisuals.FirstOrDefault(l => ReferenceEquals(l.Path, path));
                if (link == null)
                {
                    return;
                }

                if ((Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift)
                {
                    if (_selectedLinks.Contains(link))
                    {
                        _selectedLinks.Remove(link);
                    }
                    else
                    {
                        _selectedLinks.Add(link);
                    }
                }
                else
                {
                    _selectedLinks.Clear();
                    _selectedLinks.Add(link);
                    _selectedNodes.Clear();
                    _selectedNode = null;
                    UpdateInspector();
                }

                UpdateLinkSelectionVisuals();
                e.Handled = true;
                bool rewireSource = ShouldRewireSource(link, e.GetPosition(CanvasSurface));
                StartEdgeDrag(link, e.GetPosition(CanvasSurface), rewireSource);
            }
        }

        private bool ShouldRewireSource(LinkVisual link, Point pickPoint)
        {
            if (link == null)
            {
                return false;
            }

            if (!_nodeVisuals.TryGetValue(link.Link.FromNodeId, out var fromNode) ||
                !_nodeVisuals.TryGetValue(link.Link.ToNodeId, out var toNode))
            {
                return false;
            }

            Point from = GetPortAnchor(fromNode, link.Link.FromPort, GraphPortKind.Output);
            Point to = GetPortAnchor(toNode, link.Link.ToPort, GraphPortKind.Input);
            double distFrom = Distance(from, pickPoint);
            double distTo = Distance(to, pickPoint);

            if (distFrom <= EdgeRewirePickRadius || distTo <= EdgeRewirePickRadius)
            {
                return distFrom <= distTo;
            }

            return false;
        }

        private static double Distance(Point a, Point b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        private void Link_MouseEnter(object sender, MouseEventArgs e)
        {
            if (sender is System.Windows.Shapes.Path path)
            {
                var link = _linkVisuals.FirstOrDefault(l => ReferenceEquals(l.Path, path));
                if (link == null)
                {
                    return;
                }
                _hoveredLink = link;
                ApplyLinkStyle(link);
            }
        }

        private void Link_MouseLeave(object sender, MouseEventArgs e)
        {
            if (sender is System.Windows.Shapes.Path path)
            {
                var link = _linkVisuals.FirstOrDefault(l => ReferenceEquals(l.Path, path));
                if (link == null)
                {
                    return;
                }
                if (ReferenceEquals(_hoveredLink, link))
                {
                    _hoveredLink = null;
                }
                ApplyLinkStyle(link);
            }
        }

        private void Link_ContextMenuOpening(object sender, ContextMenuEventArgs e)
        {
            if (sender is System.Windows.Shapes.Path path)
            {
                var link = _linkVisuals.FirstOrDefault(l => ReferenceEquals(l.Path, path));
                if (link == null)
                {
                    return;
                }

                if (!_selectedLinks.Contains(link))
                {
                    _selectedLinks.Clear();
                    _selectedLinks.Add(link);
                    UpdateLinkSelectionVisuals();
                }

                var menu = new ContextMenu();
                menu.Items.Add(BuildMenuItem("Delete Edge", () =>
                {
                    _selectedLinks.Clear();
                    _selectedLinks.Add(link);
                    DeleteSelectedLinks();
                }));
                menu.Items.Add(BuildMenuItem("Rewire Edge", () =>
                {
                    StartEdgeDrag(link, Mouse.GetPosition(CanvasSurface), false);
                }));
                menu.Items.Add(BuildMenuItem("Rewire Source", () =>
                {
                    StartEdgeDrag(link, Mouse.GetPosition(CanvasSurface), true);
                }));
                menu.Items.Add(BuildMenuItem("Reset Reroute", () =>
                {
                    link.HasManualControls = false;
                    UpdateAllLinkGeometry();
                }));
                path.ContextMenu = menu;
            }
        }

        private void ApplyLinkStyle(LinkVisual link)
        {
            bool selected = _selectedLinks.Contains(link);
            bool hovered = ReferenceEquals(_hoveredLink, link);
            link.Path.Stroke = selected ? LinkSelectedBrush : hovered ? LinkHoverBrush : LinkBrush;
            link.Path.StrokeThickness = selected ? 3.0 : hovered ? 2.6 : 2.0;
            if (link.Handle != null)
            {
                link.Handle.Visibility = selected || hovered || ReferenceEquals(_draggingHandle, link)
                    ? Visibility.Visible
                    : Visibility.Collapsed;
                link.Handle.Fill = selected ? LinkHandleSelectedBrush : LinkHandleBrush;
            }
        }

        private Geometry BuildLinkGeometry(Point from, Point to, LinkVisual link)
        {
            double dx = Math.Max(40.0, Math.Abs(to.X - from.X) * 0.5);
            double scaled = dx * _curveTension;
            var control1 = link != null && link.HasManualControls ? link.Control1 : new Point(from.X + scaled, from.Y);
            var control2 = link != null && link.HasManualControls ? link.Control2 : new Point(to.X - scaled, to.Y);

            var figure = new PathFigure { StartPoint = from, IsClosed = false };
            figure.Segments.Add(new BezierSegment(control1, control2, to, true));

            var geometry = new PathGeometry();
            geometry.Figures.Add(figure);
            return geometry;
        }

        private void UpdateHandlePosition(LinkVisual link, Point from, Point to)
        {
            if (link == null || link.Handle == null)
            {
                return;
            }

            Point handlePoint;
            if (link.HasManualControls)
            {
                handlePoint = new Point(
                    (link.Control1.X + link.Control2.X) * 0.5,
                    (link.Control1.Y + link.Control2.Y) * 0.5);
            }
            else
            {
                handlePoint = new Point((from.X + to.X) * 0.5, (from.Y + to.Y) * 0.5);
            }

            Canvas.SetLeft(link.Handle, handlePoint.X - HandleSize * 0.5);
            Canvas.SetTop(link.Handle, handlePoint.Y - HandleSize * 0.5);
        }

        private void Handle_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is Ellipse ellipse)
            {
                var link = _linkVisuals.FirstOrDefault(l => ReferenceEquals(l.Handle, ellipse));
                if (link == null)
                {
                    return;
                }

                _draggingHandle = link;
                Point mouse = e.GetPosition(CanvasSurface);
                Point handlePos = new Point(Canvas.GetLeft(ellipse) + HandleSize * 0.5,
                    Canvas.GetTop(ellipse) + HandleSize * 0.5);
                _handleDragOffset = new Point(mouse.X - handlePos.X, mouse.Y - handlePos.Y);
                ellipse.CaptureMouse();
                e.Handled = true;
            }
        }

        private void Handle_MouseMove(object sender, MouseEventArgs e)
        {
            if (_draggingHandle == null)
            {
                return;
            }

            if (!(sender is Ellipse ellipse))
            {
                return;
            }

            Point mouse = e.GetPosition(CanvasSurface);
            Point handlePoint = new Point(mouse.X - _handleDragOffset.X, mouse.Y - _handleDragOffset.Y);

            if (!_nodeVisuals.TryGetValue(_draggingHandle.Link.FromNodeId, out var fromNode) ||
                !_nodeVisuals.TryGetValue(_draggingHandle.Link.ToNodeId, out var toNode))
            {
                return;
            }

            Point from = GetPortAnchor(fromNode, _draggingHandle.Link.FromPort, GraphPortKind.Output);
            Point to = GetPortAnchor(toNode, _draggingHandle.Link.ToPort, GraphPortKind.Input);
            double dx = Math.Max(40.0, Math.Abs(to.X - from.X) * 0.5);
            double scaled = dx * _curveTension;
            double t = Math.Abs(to.X - from.X) < 1e-3 ? 0.5 : Math.Max(0.0, Math.Min(1.0, (handlePoint.X - from.X) / (to.X - from.X)));
            double y = handlePoint.Y;

            _draggingHandle.Control1 = new Point(from.X + scaled * t, y);
            _draggingHandle.Control2 = new Point(to.X - scaled * (1.0 - t), y);
            _draggingHandle.HasManualControls = true;

            _draggingHandle.Path.Data = BuildLinkGeometry(from, to, _draggingHandle);
            UpdateHandlePosition(_draggingHandle, from, to);
        }

        private void Handle_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_draggingHandle != null && sender is Ellipse ellipse)
            {
                ellipse.ReleaseMouseCapture();
                _draggingHandle = null;
                e.Handled = true;
                GraphChanged?.Invoke();
            }
        }

        private void ZoomToFit()
        {
            if (_graph.Nodes.Count == 0 || CanvasSurface.ActualWidth < 10 || CanvasSurface.ActualHeight < 10)
            {
                return;
            }

            double minX = _graph.Nodes.Min(n => n.X);
            double minY = _graph.Nodes.Min(n => n.Y);
            double maxX = _graph.Nodes.Max(n => n.X + 160);
            double maxY = _graph.Nodes.Max(n => n.Y + 80);
            double width = Math.Max(1.0, maxX - minX);
            double height = Math.Max(1.0, maxY - minY);

            double pad = 40.0;
            double scaleX = (CanvasSurface.ActualWidth - pad) / width;
            double scaleY = (CanvasSurface.ActualHeight - pad) / height;
            double scale = Math.Max(0.1, Math.Min(scaleX, scaleY));

            SurfaceScale.ScaleX = scale;
            SurfaceScale.ScaleY = scale;
            SurfaceTranslate.X = -minX * scale + pad * 0.5;
            SurfaceTranslate.Y = -minY * scale + pad * 0.5;
        }

        private void AlignSelectedLeft()
        {
            if (_selectedNodes.Count < 2)
            {
                return;
            }

            double left = _selectedNodes.Min(n => n.Node.X);
            foreach (var node in _selectedNodes)
            {
                node.Node.X = SnapToGrid(left);
                Canvas.SetLeft(node.Container, node.Node.X);
            }
            UpdateAllLinkGeometry();
            GraphChanged?.Invoke();
        }

        private void AlignSelectedTop()
        {
            if (_selectedNodes.Count < 2)
            {
                return;
            }

            double top = _selectedNodes.Min(n => n.Node.Y);
            foreach (var node in _selectedNodes)
            {
                node.Node.Y = SnapToGrid(top);
                Canvas.SetTop(node.Container, node.Node.Y);
            }
            UpdateAllLinkGeometry();
            GraphChanged?.Invoke();
        }

        private void DistributeSelectedHorizontally()
        {
            if (_selectedNodes.Count < 3)
            {
                return;
            }

            var ordered = _selectedNodes.OrderBy(n => n.Node.X).ToList();
            double left = ordered.First().Node.X;
            double right = ordered.Last().Node.X;
            double span = right - left;
            if (span <= 0.0)
            {
                return;
            }

            double step = span / (ordered.Count - 1);
            for (int i = 0; i < ordered.Count; i++)
            {
                ordered[i].Node.X = SnapToGrid(left + step * i);
                Canvas.SetLeft(ordered[i].Container, ordered[i].Node.X);
            }
            UpdateAllLinkGeometry();
            GraphChanged?.Invoke();
        }

        private void DistributeSelectedVertically()
        {
            if (_selectedNodes.Count < 3)
            {
                return;
            }

            var ordered = _selectedNodes.OrderBy(n => n.Node.Y).ToList();
            double top = ordered.First().Node.Y;
            double bottom = ordered.Last().Node.Y;
            double span = bottom - top;
            if (span <= 0.0)
            {
                return;
            }

            double step = span / (ordered.Count - 1);
            for (int i = 0; i < ordered.Count; i++)
            {
                ordered[i].Node.Y = SnapToGrid(top + step * i);
                Canvas.SetTop(ordered[i].Container, ordered[i].Node.Y);
            }
            UpdateAllLinkGeometry();
            GraphChanged?.Invoke();
        }

        private void AdjustCurveTension(double delta)
        {
            _curveTension = Math.Max(0.1, Math.Min(0.9, _curveTension + delta));
            UpdateAllLinkGeometry();
        }

        private static double SnapToGrid(double value)
        {
            return Math.Round(value / GridSize) * GridSize;
        }

        private static double Clamp(double value, double min, double max)
        {
            if (min > max)
            {
                double swap = min;
                min = max;
                max = swap;
            }

            if (value < min)
            {
                return min;
            }
            if (value > max)
            {
                return max;
            }
            return value;
        }

        private void SyncPreviewEntries()
        {
            var inputNames = new HashSet<string>();
            foreach (var node in _graph.Nodes.Where(n => n.Kind == GraphNodeKind.Input))
            {
                foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                {
                    string name = GetPortSignalName(node, port);
                    inputNames.Add(name);
                    if (!_previewInputLookup.ContainsKey(name))
                    {
                        AddPreviewEntry(_previewInputEntries, _previewInputLookup, name, 0.0);
                    }
                }
            }
            RemoveMissingEntries(_previewInputEntries, _previewInputLookup, inputNames);

            var paramNames = new HashSet<string>();
            foreach (var node in _graph.Nodes.Where(n => n.Kind == GraphNodeKind.Param))
            {
                foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                {
                    string name = GetPortSignalName(node, port);
                    paramNames.Add(name);
                    if (!_previewParamLookup.ContainsKey(name))
                    {
                        double defaultValue = node.ConstValue;
                        if (_graph.Params != null && _graph.Params.TryGetValue(name, out var param))
                        {
                            defaultValue = param.DefaultValue;
                        }
                        AddPreviewEntry(_previewParamEntries, _previewParamLookup, name, defaultValue);
                    }
                }
            }

            // Also add entries for parameters defined in graph.Params but without Param nodes
            if (_graph.Params != null)
            {
                foreach (var kvp in _graph.Params)
                {
                    string name = kvp.Key;
                    if (!paramNames.Contains(name))
                    {
                        paramNames.Add(name);
                        if (!_previewParamLookup.ContainsKey(name))
                        {
                            AddPreviewEntry(_previewParamEntries, _previewParamLookup, name, kvp.Value.DefaultValue);
                        }
                    }
                }
            }

            // Collect params from included sub-graphs (recursive)
            CollectIncludeParams(_graph, _baseDirectory, paramNames);

            RemoveMissingEntries(_previewParamEntries, _previewParamLookup, paramNames);
        }

        private void AddPreviewEntry(ObservableCollection<PreviewEntry> list,
            Dictionary<string, PreviewEntry> lookup, string name, double value)
        {
            var entry = new PreviewEntry(name, value);
            entry.ValueChanged += (_, __) => RequestPreviewRefresh();
            list.Add(entry);
            lookup[name] = entry;
        }

        private void RemoveMissingEntries(ObservableCollection<PreviewEntry> list,
            Dictionary<string, PreviewEntry> lookup, HashSet<string> keepNames)
        {
            for (int i = list.Count - 1; i >= 0; i--)
            {
                var entry = list[i];
                if (!keepNames.Contains(entry.Name))
                {
                    lookup.Remove(entry.Name);
                    list.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// Recursively collects parameters from included sub-graphs.
        /// </summary>
        private void CollectIncludeParams(GraphDefinition graph, string baseDir, HashSet<string> paramNames)
        {
            if (graph?.Nodes == null || string.IsNullOrEmpty(baseDir))
            {
                return;
            }

            foreach (var node in graph.Nodes.Where(n => n.Kind == GraphNodeKind.Include))
            {
                if (string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    continue;
                }

                // Resolve the include path relative to the current graph's directory
                string includePath = node.IncludePath;
                string resolvedPath;
                try
                {
                    resolvedPath = System.IO.Path.IsPathRooted(includePath)
                        ? includePath
                        : System.IO.Path.GetFullPath(System.IO.Path.Combine(baseDir, includePath));
                }
                catch
                {
                    continue;
                }

                if (!File.Exists(resolvedPath))
                {
                    continue;
                }

                // Load the included graph
                GraphDefinition includedGraph;
                try
                {
                    string json = File.ReadAllText(resolvedPath);
                    includedGraph = GraphSerializer.Deserialize(json, out _);
                }
                catch
                {
                    continue;
                }

                if (includedGraph == null)
                {
                    continue;
                }

                // Add params from this include (defaults, can be overridden by parent)
                if (includedGraph.Params != null)
                {
                    foreach (var kvp in includedGraph.Params)
                    {
                        string name = kvp.Key;
                        if (!paramNames.Contains(name))
                        {
                            paramNames.Add(name);
                            if (!_previewParamLookup.ContainsKey(name))
                            {
                                AddPreviewEntry(_previewParamEntries, _previewParamLookup, name, kvp.Value.DefaultValue);
                            }
                        }
                    }
                }

                // Recurse into nested includes
                string includeDir = System.IO.Path.GetDirectoryName(resolvedPath);
                CollectIncludeParams(includedGraph, includeDir, paramNames);
            }
        }

        private void UpdatePreviewResolver()
        {
            if (!string.IsNullOrWhiteSpace(_baseDirectory) && Directory.Exists(_baseDirectory))
            {
                _previewEvaluator.SetResolver(GraphRuntimeConverter.CreateResolver(_baseDirectory));
                _previewEvaluator.SetBaseDirectory(_baseDirectory);
            }
            else
            {
                // Always create a resolver - it can still resolve absolute Include paths
                // even without a valid base directory (e.g., for unsaved graphs with pasted Includes)
                _previewEvaluator.SetResolver(GraphRuntimeConverter.CreateResolver(""));
                _previewEvaluator.SetBaseDirectory(null);
            }
        }

        private void RefreshPreview()
        {
            try
            {
                Dictionary<string, double> inputs;
                Dictionary<string, double> parameters;

                // Check if we should use context-provided inputs
                if (_selectedContextId != null && _contextProvider != null)
                {
                    // Normalize path for cache lookup (same as RefreshContextDropdown)
                    string normalizedPath = _filePath;
                    try
                    {
                        normalizedPath = System.IO.Path.GetFullPath(_filePath);
                    }
                    catch
                    {
                        // Keep original if normalization fails
                    }

                    var contexts = _contextProvider(normalizedPath);
                    IncludeCallContext ctx = null;
                    if (contexts != null)
                    {
                        foreach (var c in contexts)
                        {
                            if (c.IncludeNodeId == _selectedContextId)
                            {
                                ctx = c;
                                break;
                            }
                        }
                    }

                    if (ctx != null)
                    {
                        inputs = new Dictionary<string, double>();
                        foreach (var kvp in ctx.Inputs)
                        {
                            inputs[kvp.Key] = kvp.Value;
                        }
                        parameters = new Dictionary<string, double>();
                        if (ctx.Parameters != null)
                        {
                            foreach (var kvp in ctx.Parameters)
                            {
                                parameters[kvp.Key] = kvp.Value;
                            }
                        }
                        // Overlay only explicitly updated param values to ensure param changes
                        // propagate to include context preview immediately, while preserving
                        // profile overrides for params that haven't been changed this session
                        foreach (var entry in _previewParamEntries)
                        {
                            if (_explicitlyUpdatedParams.Contains(entry.Name))
                            {
                                parameters[entry.Name] = entry.Value;
                            }
                        }
                    }
                    else
                    {
                        // Context not found in cache - could be temporary dropout
                        if (_contextIsUserSelected)
                        {
                            // Don't fall back to standalone immediately; wait for cache to repopulate
                            // Skip this refresh cycle
                            return;
                        }

                        // Not user-selected, safe to fall back
                        _selectedContextId = null;
                        RefreshContextDropdown();
                        return;
                    }
                }
                else
                {
                    // Standalone mode - use manual entries
                    inputs = new Dictionary<string, double>();
                    foreach (var entry in _previewInputEntries)
                    {
                        inputs[entry.Name] = entry.Value;
                    }

                    parameters = new Dictionary<string, double>();
                    foreach (var entry in _previewParamEntries)
                    {
                        parameters[entry.Name] = entry.Value;
                    }
                }

                var result = _previewEvaluator.Evaluate(_graph, inputs, parameters);
                UpdateNodeValues(result.NodeValues);
                if (result.Warnings != null && result.Warnings.Count > 0)
                {
                    SetPreviewStatusText(string.Join("; ", result.Warnings));
                }
                else
                {
                    SetPreviewStatusText("");
                }
            }
            catch (Exception ex)
            {
                UpdateNodeValues(null);
                SetPreviewStatusText($"Preview error: {ex.Message}");
            }
        }

        private void CheckLibraryGraph_Changed(object sender, RoutedEventArgs e)
        {
            if (_graph == null)
            {
                return;
            }

            bool isLibrary = CheckLibraryGraph.IsChecked == true;
            if (_graph.IsLibraryGraph == isLibrary)
            {
                return;
            }

            _graph.IsLibraryGraph = isLibrary;
            IsLibraryGraph = isLibrary;

            // Rebuild the surface and inspector to reflect the new mode
            RebuildSurface();
            UpdateInspector();
        }

        private void CheckDebugLogging_Changed(object sender, RoutedEventArgs e)
        {
            bool enabled = CheckDebugLogging.IsChecked == true;
            _previewEvaluator.DebugLoggingEnabled = enabled;

            // Update the log path display
            if (enabled && !string.IsNullOrEmpty(_previewEvaluator.DebugLogPath))
            {
                TextDebugLogPath.Text = System.IO.Path.GetFileName(_previewEvaluator.DebugLogPath);
                TextDebugLogPath.ToolTip = _previewEvaluator.DebugLogPath + "\nClick to open log file location";
            }
            else
            {
                TextDebugLogPath.Text = "";
            }
        }

        private void TextDebugLogPath_Click(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            string logPath = _previewEvaluator.DebugLogPath;
            if (!string.IsNullOrEmpty(logPath) && File.Exists(logPath))
            {
                // Open Explorer and select the file
                System.Diagnostics.Process.Start("explorer.exe", $"/select,\"{logPath}\"");
            }
        }

        /// <summary>
        /// Sets the live inputs state. Called by the window to sync global state across all tabs.
        /// </summary>
        /// <param name="enabled">Whether live inputs are enabled</param>
        /// <param name="raiseEvent">If true, raises LiveInputsStateChanged (used when user toggles checkbox)</param>
        public void SetLiveInputsState(bool enabled, bool raiseEvent)
        {
            _liveInputsEnabled = enabled;
            UpdatePreviewWindowLiveInputs(enabled);

            if (raiseEvent)
            {
                LiveInputsStateChanged?.Invoke(this, enabled);
            }

            if (_liveInputsEnabled)
            {
                ApplyLiveInputs();
            }
        }

        public void AttachPreviewWindow(PreviewWindow window)
        {
            if (ReferenceEquals(_previewWindow, window))
            {
                return;
            }

            DetachPreviewWindow();
            _previewWindow = window;
            if (_previewWindow == null)
            {
                return;
            }

            _previewWindow.SetSources(_previewInputEntries, _previewParamEntries);
            _previewWindow.SetLiveInputsState(_liveInputsEnabled);
            _previewWindow.SetInputsEnabled(_selectedContextId == null);
            _previewWindow.SetStatusText(_previewStatusText);
            _previewWindow.LiveInputsToggled += OnPreviewWindowLiveInputsToggled;
            _previewWindow.Closed += OnPreviewWindowClosed;
        }

        public void DetachPreviewWindow()
        {
            if (_previewWindow == null)
            {
                return;
            }

            _previewWindow.LiveInputsToggled -= OnPreviewWindowLiveInputsToggled;
            _previewWindow.Closed -= OnPreviewWindowClosed;
            _previewWindow = null;
        }

        private void OnPreviewWindowLiveInputsToggled(object sender, bool enabled)
        {
            SetLiveInputsState(enabled, raiseEvent: true);
        }

        private void OnPreviewWindowClosed(object sender, EventArgs e)
        {
            DetachPreviewWindow();
        }

        private void UpdatePreviewWindowLiveInputs(bool enabled)
        {
            _previewWindow?.SetLiveInputsState(enabled);
        }

        private void UpdatePreviewWindowInputsEnabled(bool enabled)
        {
            _previewWindow?.SetInputsEnabled(enabled);
        }

        private void SetPreviewStatusText(string text)
        {
            _previewStatusText = text ?? "";
            _previewWindow?.SetStatusText(_previewStatusText);
        }


        /// <summary>
        /// Called by the window's global timer to tick live inputs for this tab.
        /// </summary>
        public void TickLiveInputs()
        {
            if (!_liveInputsEnabled)
            {
                return;
            }

            ApplyLiveInputs();

            // Refresh context dropdown during live mode, but skip if dropdown is open
            // (active graph may have evaluated with new inputs, making contexts available)
            if (_contextProvider != null && !string.IsNullOrEmpty(_filePath) && !ComboEvalContext.IsDropDownOpen)
            {
                RefreshContextDropdown();

                // If a context is selected, also refresh preview with new context values
                if (_selectedContextId != null)
                {
                    RequestPreviewRefresh();
                }
            }
        }

        private void ApplyLiveInputs()
        {
            if (!_liveInputsEnabled || LiveInputProvider == null)
            {
                return;
            }

            var liveInputs = LiveInputProvider();
            if (liveInputs == null)
            {
                return;
            }

            SyncPreviewEntries();
            bool hadChanges = false;
            _suppressPreviewRefresh = true;
            foreach (var entry in _previewInputEntries)
            {
                if (liveInputs.TryGetValue(entry.Name, out var value))
                {
                    if (Math.Abs(entry.Value - value) >= 1e-9)
                    {
                        entry.Value = value;
                        hadChanges = true;
                    }
                }
            }
            _suppressPreviewRefresh = false;
            if (hadChanges && _previewRefreshPending)
            {
                RequestPreviewRefresh();
            }
        }

        private void RequestPreviewRefresh()
        {
            if (_suppressPreviewRefresh)
            {
                _previewRefreshPending = true;
                return;
            }

            _previewRefreshPending = true;
            _previewRefreshTimer.Stop();
            _previewRefreshTimer.Start();
        }

        private void OnPreviewRefreshTimer(object sender, EventArgs e)
        {
            _previewRefreshTimer.Stop();
            if (!_previewRefreshPending)
            {
                return;
            }

            _previewRefreshPending = false;
            RefreshPreview();
        }

        private void RefreshContextDropdown(bool force = false)
        {
            if (_contextProvider == null || string.IsNullOrEmpty(_filePath))
            {
                // Only clear selection if not user-selected (sticky behavior)
                if (!_contextIsUserSelected)
                {
                    if (_graph != null && _graph.IsLibraryGraph)
                    {
                        EnsureStandaloneContextVisible();
                    }
                    else if (PanelEvalContext.Visibility != Visibility.Collapsed)
                    {
                        ComboEvalContext.Items.Clear();
                        ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
                        PanelEvalContext.Visibility = Visibility.Collapsed;
                        _selectedContextId = null;
                        _lastContextIds.Clear();
                    }
                }
                return;
            }

            // Normalize path for cache lookup (use GetFullPath for consistent format)
            string normalizedPath = _filePath;
            try
            {
                normalizedPath = System.IO.Path.GetFullPath(_filePath);
            }
            catch
            {
                // Keep original if normalization fails
            }

            var contexts = _contextProvider(normalizedPath);

            if (contexts == null || contexts.Count == 0)
            {
                // Cache is temporarily empty - DON'T reset if user had selected a context
                // The cache will be repopulated on next evaluation cycle
                ComboEvalContext.Items.Clear();
                ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
                if (_hadLiveContext && !string.IsNullOrEmpty(_selectedContextId))
                {
                    ComboEvalContext.Items.Add(new ComboBoxItem
                    {
                        Content = "(last context)",
                        Tag = _selectedContextId
                    });
                }
                PanelEvalContext.Visibility = Visibility.Visible;
                if (_hadLiveContext && _contextIsUserSelected && _selectedContextId != null)
                {
                    ComboEvalContext.SelectedItem = ComboEvalContext.Items[1];
                }
                else
                {
                    _selectedContextId = null;
                    _contextIsUserSelected = false;
                    ComboEvalContext.SelectedItem = ComboEvalContext.Items[0];
                }
                _lastContextIds.Clear();
                return;
            }

            _hadLiveContext = true;

            // Check if contexts have changed (by comparing IDs)
            var currentIds = new HashSet<string>(contexts.Select(c => c.IncludeNodeId));
            if (!force && currentIds.SetEquals(_lastContextIds))
            {
                // Contexts haven't changed, skip rebuild
                return;
            }

            // Contexts changed, rebuild dropdown
            _lastContextIds = currentIds;

            ComboEvalContext.Items.Clear();
            ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });

            foreach (var ctx in contexts)
            {
                ComboEvalContext.Items.Add(new ComboBoxItem
                {
                    Content = ctx.IncludeNodeTitle,
                    Tag = ctx.IncludeNodeId
                });
            }

            PanelEvalContext.Visibility = Visibility.Visible;

            // Restore selection or default to standalone
            ComboBoxItem selected = null;
            foreach (ComboBoxItem item in ComboEvalContext.Items)
            {
                if ((string)item.Tag == _selectedContextId)
                {
                    selected = item;
                    break;
                }
            }
            ComboEvalContext.SelectedItem = selected ?? ComboEvalContext.Items[0];
        }

        private void EnsureStandaloneContextVisible()
        {
            ComboEvalContext.Items.Clear();
            ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });
            PanelEvalContext.Visibility = Visibility.Visible;
            _selectedContextId = null;
            _lastContextIds.Clear();
            ComboEvalContext.SelectedItem = ComboEvalContext.Items[0];
        }

        private void ComboEvalContext_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ComboEvalContext.SelectedItem is ComboBoxItem item)
            {
                _selectedContextId = item.Tag as string;
                _contextIsUserSelected = (_selectedContextId != null);  // Track that user made a choice
                UpdateParamControlsEnabled();
                RefreshPreview();
                ContextChanged?.Invoke(this, _selectedContextId);
            }
        }

        /// <summary>
        /// Programmatically selects a context by Include node ID.
        /// Called when navigating to an include graph via double-click while live mode is active.
        /// </summary>
        public void SetSelectedContext(string contextId)
        {
            _selectedContextId = contextId;
            _contextIsUserSelected = (contextId != null);
            RefreshContextDropdown(force: true);
            RefreshPreview();
            ContextChanged?.Invoke(this, _selectedContextId);
        }

        /// <summary>
        /// Updates the enabled state of preview controls based on context selection.
        /// When a non-standalone context is selected, input/param values come from the context
        /// so the controls should be disabled.
        /// </summary>
        private void UpdateParamControlsEnabled()
        {
            bool enabled = _selectedContextId == null;

            // Disable preview inputs and params lists
            UpdatePreviewWindowInputsEnabled(enabled);

            // Disable param controls on Param nodes in the canvas
            foreach (var nodeVisual in _nodeVisuals.Values)
            {
                if (nodeVisual.Node.Kind != GraphNodeKind.Param)
                    continue;

                foreach (var child in nodeVisual.InnerCanvas.Children)
                {
                    if (child is FrameworkElement element && element.Tag is ParamControlTag)
                    {
                        element.IsEnabled = enabled;
                    }
                }
            }
        }

        private static string GetNodeName(GraphNode node)
        {
            return string.IsNullOrWhiteSpace(node.Title) ? node.Id : node.Title;
        }

        /// <summary>
        /// Gets the display label for a port. For Input/Output nodes in top-level graphs, shows SignalSuffix.
        /// For library graph Input/Output nodes, shows Name (freeform).
        /// </summary>
        private string GetPortDisplayLabel(GraphNode node, GraphPort port)
        {
            bool isLibraryGraph = _graph != null && _graph.IsLibraryGraph;
            bool isNegatedOpInput = node != null &&
                node.Kind == GraphNodeKind.Op &&
                port?.Kind == GraphPortKind.Input &&
                port.Negate &&
                IsNegateSupportedOp(node.Op);

            // In library graphs, Input/Output nodes use Name directly (freeform)
            if (isLibraryGraph && (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Output))
            {
                return isNegatedOpInput ? "-" + port.Name : port.Name;
            }

            // In top-level graphs, Input/Output nodes use SignalSuffix if set
            if ((node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Output) &&
                !string.IsNullOrEmpty(port.SignalSuffix))
            {
                return isNegatedOpInput ? "-" + port.SignalSuffix : port.SignalSuffix;
            }
            return isNegatedOpInput ? "-" + port.Name : port.Name;
        }

        /// <summary>
        /// Builds the full signal name for a port (e.g., "XPlane.IAS_kts").
        /// Uses SignalGroup + SignalSuffix if both are set, otherwise falls back to port.Name.
        /// </summary>
        private static string GetPortSignalName(GraphNode node, GraphPort port)
        {
            if (!string.IsNullOrEmpty(node.SignalGroup) && !string.IsNullOrEmpty(port.SignalSuffix))
            {
                return node.SignalGroup + "." + port.SignalSuffix;
            }
            return port.Name ?? "";
        }

        private string BuildSelectedNodeLabel(GraphNode node)
        {
            if (node == null)
            {
                return "(none)";
            }

            string label;
            switch (node.Kind)
            {
                case GraphNodeKind.Const:
                    label = $"Const (value={node.ConstValue:F3})";
                    break;
                case GraphNodeKind.Op:
                    label = $"Op ({node.Op ?? "mul"})";
                    break;
                case GraphNodeKind.Func:
                    label = $"Func ({node.Func ?? "?"})";
                    break;
                case GraphNodeKind.Input:
                    label = BuildSignalNodeHeader("Input", node);
                    break;
                case GraphNodeKind.Output:
                    label = BuildSignalNodeHeader("Output", node);
                    break;
                case GraphNodeKind.Param:
                    label = BuildSignalNodeHeader("Param", node);
                    break;
                case GraphNodeKind.Include:
                    label = BuildIncludeHeader(node);
                    break;
                default:
                    label = node.Kind.ToString();
                    break;
            }

            if (!string.IsNullOrWhiteSpace(node.Title))
            {
                string title = node.Title.Trim();
                if (!string.IsNullOrWhiteSpace(title))
                {
                    label = $"{label} ({title})";
                }
            }

            return label;
        }

        private string BuildSignalNodeHeader(string kindLabel, GraphNode node)
        {
            string group = string.IsNullOrWhiteSpace(node.SignalGroup) ? GetEffectiveSignalGroup(node) : node.SignalGroup;
            int portCount = node.Ports?.Count ?? 0;
            if (!string.IsNullOrWhiteSpace(group))
            {
                return $"{kindLabel} (Group: {group}, Ports: {portCount})";
            }

            return $"{kindLabel} (Ports: {portCount})";
        }

        private static string BuildIncludeHeader(GraphNode node)
        {
            if (node == null || string.IsNullOrWhiteSpace(node.IncludePath))
            {
                return "Include";
            }

            string fileName = System.IO.Path.GetFileName(node.IncludePath);
            if (!string.IsNullOrWhiteSpace(fileName))
            {
                return $"Include ({fileName})";
            }

            return $"Include ({node.IncludePath})";
        }

        private static bool UsesTemplateInspector(GraphNode node)
        {
            if (node == null)
            {
                return false;
            }

            return node.Kind == GraphNodeKind.Const
                   || node.Kind == GraphNodeKind.Op
                   || node.Kind == GraphNodeKind.Func
                   || node.Kind == GraphNodeKind.Input
                   || node.Kind == GraphNodeKind.Output
                   || node.Kind == GraphNodeKind.Param
                   || node.Kind == GraphNodeKind.Include;
        }


        private static IReadOnlyList<string> GetSignalGroups(GraphNode node)
        {
            switch (node.Kind)
            {
                case GraphNodeKind.Input:
                    return GraphSignalCatalog.InputGroups;
                case GraphNodeKind.Output:
                    return GraphSignalCatalog.OutputGroups;
                case GraphNodeKind.Param:
                    return GraphSignalCatalog.ParamGroups;
                default:
                    return Array.Empty<string>();
            }
        }

        private void PopulateSignalGroupDropdown(ComboBox comboBox, GraphNode node)
        {
            if (comboBox == null || node == null)
            {
                return;
            }

            var groups = GetSignalGroups(node);
            comboBox.IsEditable = false;
            comboBox.ItemsSource = groups;

            // Select current group or default to first (display only, don't modify node)
            string currentGroup = node.SignalGroup;
            if (!string.IsNullOrEmpty(currentGroup) && groups.Contains(currentGroup))
            {
                comboBox.SelectedItem = currentGroup;
            }
            else if (groups.Count > 0)
            {
                comboBox.SelectedIndex = 0;
                // Don't set node.SignalGroup here - that would mark the graph dirty on selection
            }
        }

        private static string GetEffectiveSignalGroup(GraphNode node)
        {
            if (node == null || !string.IsNullOrWhiteSpace(node.SignalGroup))
            {
                return node?.SignalGroup ?? "";
            }

            switch (node.Kind)
            {
                case GraphNodeKind.Input:
                    return GraphSignalCatalog.InputGroups.Count > 0 ? GraphSignalCatalog.InputGroups[0] : "";
                case GraphNodeKind.Output:
                    return GraphSignalCatalog.OutputGroups.Count > 0 ? GraphSignalCatalog.OutputGroups[0] : "";
                case GraphNodeKind.Param:
                    return GraphSignalCatalog.ParamGroups.Count > 0 ? GraphSignalCatalog.ParamGroups[0] : "";
                default:
                    return "";
            }
        }

        private void InspectorSignalGroup_Loaded(object sender, RoutedEventArgs e)
        {
            if (sender is ComboBox comboBox && comboBox.DataContext is GraphNode node)
            {
                _isInspectorUpdating = true;
                PopulateSignalGroupDropdown(comboBox, node);
                _isInspectorUpdating = false;
            }
        }

        private void InspectorSignalGroup_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is ComboBox comboBox && comboBox.DataContext is GraphNode node)
            {
                string selectedGroup = comboBox.SelectedItem as string ?? "";
                ApplySignalGroupSelection(node, selectedGroup);
            }
        }

        private void ApplySignalGroupSelection(GraphNode node, string selectedGroup)
        {
            if (node == null)
            {
                return;
            }

            // Ignore if selection was cleared (happens when ItemsSource is reassigned)
            if (string.IsNullOrEmpty(selectedGroup))
            {
                return;
            }
            if (node.SignalGroup == selectedGroup)
            {
                return;
            }

            node.SignalGroup = selectedGroup;

            // Rebuild node visual to show new group and port labels
            RebuildSurface();
            if (_nodeVisuals.TryGetValue(node.Id, out var visual))
            {
                _selectedNode = visual;
                _selectedNodes.Clear();
                _selectedNodes.Add(visual);
            }
            UpdateSelectionVisuals();

            // Re-sync port entries to update signal options based on new group
            SyncPortEntries(node);

            GraphChanged?.Invoke();
        }

        private void ButtonAddInputPort_Click(object sender, RoutedEventArgs e)
        {
            AddPort(GraphPortKind.Input, "in");
        }

        private void ButtonAddOutputPort_Click(object sender, RoutedEventArgs e)
        {
            AddPort(GraphPortKind.Output, "out");
        }

        private void ButtonAddOpInput_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null)
            {
                return;
            }

            var node = _selectedNode.Node;
            if (node.Kind != GraphNodeKind.Op || !IsVariadicOp(node.Op))
            {
                return;
            }

            int inputCount = node.Ports.Count(p => p.Kind == GraphPortKind.Input);
            string name = GetVariadicOpInputName(inputCount);
            var port = new GraphPort { Name = name, Kind = GraphPortKind.Input };
            port.Name = EnsureUniquePortName(node, port, port.Name);
            node.Ports.Add(port);

            EnsureOpPorts(node, node.Op);
            RebuildSurface();
            if (_nodeVisuals.TryGetValue(node.Id, out var visual))
            {
                _selectedNode = visual;
                _selectedNodes.Clear();
                _selectedNodes.Add(visual);
            }
            UpdateSelectionVisuals();
            SyncPortEntries(node);
            RefreshPreview();
            GraphChanged?.Invoke();
        }

        private void AddPort(GraphPortKind kind, string baseName)
        {
            if (_selectedNode == null)
            {
                return;
            }

            var node = _selectedNode.Node;
            var port = new GraphPort { Name = baseName, Kind = kind };
            port.Name = EnsureUniquePortName(node, port, port.Name);
            node.Ports.Add(port);
            RebuildSurface();
            // Restore selection after rebuild (RebuildSurface clears visuals)
            if (_nodeVisuals.TryGetValue(node.Id, out var visual))
            {
                _selectedNode = visual;
                _selectedNodes.Clear();
                _selectedNodes.Add(visual);
            }
            UpdateSelectionVisuals();
            SyncPortEntries(node);
            GraphChanged?.Invoke();
        }

        private void ButtonRemovePort_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || !(sender is Button button))
            {
                return;
            }

            if (!(button.DataContext is PortEditEntry entry))
            {
                return;
            }

            var node = _selectedNode.Node;

            // Include node ports are auto-derived from included graph - don't allow removal
            if (node.Kind == GraphNodeKind.Include)
            {
                return;
            }

            if (node.Kind == GraphNodeKind.Op && !CanRemoveOpPort(node, entry.Port))
            {
                return;
            }

            RemovePort(node, entry.Port);
            if (node.Kind == GraphNodeKind.Op)
            {
                EnsureOpPorts(node, node.Op);
            }
            RebuildSurface();
            // Restore selection after rebuild
            if (_nodeVisuals.TryGetValue(node.Id, out var visual))
            {
                _selectedNode = visual;
                _selectedNodes.Clear();
                _selectedNodes.Add(visual);
            }
            UpdateSelectionVisuals();
            SyncPortEntries(node);
            RefreshPreview();
            GraphChanged?.Invoke();
        }

        private void EnsureFuncPorts(GraphNode node)
        {
            if (node == null || node.Kind != GraphNodeKind.Func)
            {
                return;
            }

            string func = (node.Func ?? "").Trim().ToLowerInvariant();
            string[] desiredInputs = GetFuncInputNames(func);

            EnsurePort(node, GraphPortKind.Output, "out");

            var inputPorts = new List<GraphPort>();
            foreach (var port in node.Ports)
            {
                if (port.Kind == GraphPortKind.Input)
                {
                    inputPorts.Add(port);
                }
            }

            for (int idx = inputPorts.Count - 1; idx >= desiredInputs.Length; idx--)
            {
                RemovePort(node, inputPorts[idx]);
                inputPorts.RemoveAt(idx);
            }

            for (int idx = 0; idx < desiredInputs.Length; idx++)
            {
                if (idx < inputPorts.Count)
                {
                    RenamePort(node, inputPorts[idx].Name, desiredInputs[idx]);
                    inputPorts[idx].Name = desiredInputs[idx];
                }
                else
                {
                    node.Ports.Add(new GraphPort { Name = desiredInputs[idx], Kind = GraphPortKind.Input });
                }
            }
        }

        private bool EnsureOpPorts(GraphNode node, string op)
        {
            if (node == null || node.Kind != GraphNodeKind.Op)
            {
                return false;
            }

            bool changed = false;
            string[] desiredInputs = GetOpInputNames(op);
            bool isVariadic = IsVariadicOp(op);
            int minInputs = GetOpMinInputCount(op);
            var inputPorts = node.Ports.Where(p => p.Kind == GraphPortKind.Input).ToList();
            var outputPorts = node.Ports.Where(p => p.Kind == GraphPortKind.Output).ToList();

            if (!IsNegateSupportedOp(op))
            {
                foreach (var port in inputPorts)
                {
                    if (port.Negate)
                    {
                        port.Negate = false;
                        changed = true;
                    }
                }
            }

            string outputName = isVariadic
                ? GetOpOutputName(op, inputPorts)
                : GetOpOutputName(op);
            GraphPort outPort = outputPorts.FirstOrDefault(p => p.Name == outputName);
            if (outPort == null)
            {
                if (outputPorts.Count > 0)
                {
                    outPort = outputPorts[0];
                    RenamePort(node, outPort.Name, outputName);
                    outPort.Name = outputName;
                    changed = true;
                }
                else
                {
                    node.Ports.Add(new GraphPort { Name = outputName, Kind = GraphPortKind.Output });
                    changed = true;
                }
            }

            foreach (var port in outputPorts.Where(p => !ReferenceEquals(p, outPort)).ToList())
            {
                RemovePort(node, port);
                changed = true;
            }

            if (!isVariadic)
            {
                for (int idx = inputPorts.Count - 1; idx >= desiredInputs.Length; idx--)
                {
                    RemovePort(node, inputPorts[idx]);
                    inputPorts.RemoveAt(idx);
                    changed = true;
                }

                for (int idx = 0; idx < desiredInputs.Length; idx++)
                {
                    if (idx < inputPorts.Count)
                    {
                        if (!string.Equals(inputPorts[idx].Name, desiredInputs[idx], StringComparison.Ordinal))
                        {
                            RenamePort(node, inputPorts[idx].Name, desiredInputs[idx]);
                            inputPorts[idx].Name = desiredInputs[idx];
                            changed = true;
                        }
                    }
                    else
                    {
                        node.Ports.Add(new GraphPort { Name = desiredInputs[idx], Kind = GraphPortKind.Input });
                        changed = true;
                    }
                }
            }
            else
            {
                for (int idx = inputPorts.Count; idx < minInputs; idx++)
                {
                    node.Ports.Add(new GraphPort { Name = GetVariadicOpInputName(idx), Kind = GraphPortKind.Input });
                    changed = true;
                }

                inputPorts = node.Ports.Where(p => p.Kind == GraphPortKind.Input).ToList();
                for (int idx = 0; idx < inputPorts.Count; idx++)
                {
                    string desiredName = GetVariadicOpInputName(idx);
                    if (!string.Equals(inputPorts[idx].Name, desiredName, StringComparison.Ordinal))
                    {
                        RenamePort(node, inputPorts[idx].Name, desiredName);
                        inputPorts[idx].Name = desiredName;
                        changed = true;
                    }
                }
            }

            if (isVariadic)
            {
                string desiredOutputName = GetOpOutputName(op, node.Ports.Where(p => p.Kind == GraphPortKind.Input));
                if (!string.Equals(outPort.Name, desiredOutputName, StringComparison.Ordinal))
                {
                    RenamePort(node, outPort.Name, desiredOutputName);
                    outPort.Name = desiredOutputName;
                    changed = true;
                }
            }

            return changed;
        }

        private static string[] GetFuncInputNames(string func)
        {
            switch (func)
            {
                case "qhat_eff":
                    return new[] { "ias_kts", "vref_kts" };
                case "torque_norm":
                    return new[] { "trq", "trq_ref" };
                case "rpm_norm":
                    return new[] { "rpm", "rpm_ref" };
                case "assist_loss":
                    return new[] { "rpm_norm" };
                case "buffet":
                    return new[] { "alpha", "start", "full", "gain", "qhat_eff" };
                default:
                    return new[] { "a", "b" };
            }
        }

        internal static bool IsVariadicOp(string op)
        {
            switch (NormalizeOp(op))
            {
                case "add":
                case "mul":
                case "min":
                case "max":
                    return true;
                default:
                    return false;
            }
        }

        private static bool IsNegateSupportedOp(string op)
        {
            switch (NormalizeOp(op))
            {
                case "add":
                case "mul":
                    return true;
                default:
                    return false;
            }
        }

        private static int GetOpMinInputCount(string op)
        {
            switch (NormalizeOp(op))
            {
                case "abs":
                case "neg":
                    return 1;
                case "clamp":
                case "lerp":
                    return 3;
                default:
                    return 2;
            }
        }

        private static string GetVariadicOpInputName(int index)
        {
            if (index < 26)
            {
                return ((char)('a' + index)).ToString();
            }
            return $"a{index + 1}";
        }

        private static string NormalizeOp(string op)
        {
            return (op ?? "").Trim().ToLowerInvariant();
        }

        private static string GetOpOutputName(string op)
        {
            switch (NormalizeOp(op))
            {
                case "add": return "a+b";
                case "sub": return "a-b";
                case "mul": return "a*b";
                case "div": return "a/b";
                case "min": return "min(a,b)";
                case "max": return "max(a,b)";
                case "abs": return "abs(a)";
                case "neg": return "-a";
                case "clamp": return "clamp(a,min,max)";
                case "lerp": return "lerp(a,b,t)";
                default: return "out";
            }
        }

        private static string GetOpOutputName(string op, IEnumerable<GraphPort> inputs)
        {
            string opKey = NormalizeOp(op);
            var inputList = inputs?.Where(port => port != null && !string.IsNullOrWhiteSpace(port.Name)).ToList()
                ?? new List<GraphPort>();
            if (inputList.Count == 0)
            {
                return GetOpOutputName(op);
            }

            if (opKey == "add")
            {
                var parts = new List<string>(inputList.Count);
                for (int i = 0; i < inputList.Count; i++)
                {
                    string name = inputList[i].Name;
                    bool negate = inputList[i].Negate;
                    if (i == 0)
                    {
                        parts.Add(negate ? "-" + name : name);
                    }
                    else
                    {
                        parts.Add((negate ? "-" : "+") + name);
                    }
                }
                return string.Join("", parts);
            }

            if (opKey == "mul")
            {
                var parts = new List<string>(inputList.Count);
                for (int i = 0; i < inputList.Count; i++)
                {
                    string name = inputList[i].Name;
                    bool negate = inputList[i].Negate;
                    if (i == 0)
                    {
                        parts.Add(negate ? "-" + name : name);
                    }
                    else
                    {
                        parts.Add((negate ? "*-" : "*") + name);
                    }
                }
                return string.Join("", parts);
            }

            if (opKey == "min" || opKey == "max")
            {
                string args = string.Join(",", inputList.Select(port => port.Name));
                return $"{opKey}({args})";
            }

            return GetOpOutputName(op);
        }

        private static string[] GetOpInputNames(string op)
        {
            switch (NormalizeOp(op))
            {
                case "abs":
                case "neg":
                    return new[] { "a" };
                case "clamp":
                    return new[] { "a", "min", "max" };
                case "lerp":
                    return new[] { "a", "b", "t" };
                default:
                    return new[] { "a", "b" };
            }
        }

        private static bool CanRemoveOpPort(GraphNode node, GraphPort port)
        {
            if (node == null || node.Kind != GraphNodeKind.Op || port == null)
            {
                return false;
            }

            if (!IsVariadicOp(node.Op))
            {
                return false;
            }

            if (port.Kind != GraphPortKind.Input)
            {
                return false;
            }

            int minInputs = GetOpMinInputCount(node.Op);
            int inputIndex = node.Ports.Where(p => p.Kind == GraphPortKind.Input).ToList().IndexOf(port);
            return inputIndex >= minInputs;
        }

        private void EnsurePort(GraphNode node, GraphPortKind kind, string name)
        {
            foreach (var port in node.Ports)
            {
                if (port.Kind == kind && string.Equals(port.Name, name, StringComparison.Ordinal))
                {
                    return;
                }
            }

            node.Ports.Add(new GraphPort { Name = name, Kind = kind });
        }

        private void RemovePort(GraphNode node, GraphPort port)
        {
            if (node == null || port == null)
            {
                return;
            }

            _graph.Links.RemoveAll(link =>
                (link.FromNodeId == node.Id && link.FromPort == port.Name) ||
                (link.ToNodeId == node.Id && link.ToPort == port.Name));
            node.Ports.Remove(port);
        }

        private void InspectorTitle_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is TextBox textBox &&
                textBox.DataContext is GraphNode node &&
                ReferenceEquals(node, _selectedNode.Node))
            {
                string desired = textBox.Text?.Trim() ?? "";
                node.Title = string.IsNullOrWhiteSpace(desired) ? null : desired;
                UpdateNodeTitleVisual(node);
                SyncPreviewEntries();
                RefreshPreview();
                _pendingUndoDebounce = true;
                GraphChanged?.Invoke();
            }
        }

        private void InspectorConst_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is TextBox textBox &&
                textBox.DataContext is GraphNode node &&
                ReferenceEquals(node, _selectedNode.Node) &&
                TryParseDouble(textBox.Text, out var value))
            {
                node.ConstValue = value;
                RefreshPreview();
                _pendingUndoDebounce = true;
                GraphChanged?.Invoke();
            }
        }

        private void InspectorOp_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is ComboBox comboBox &&
                comboBox.DataContext is GraphNode node &&
                ReferenceEquals(node, _selectedNode.Node) &&
                comboBox.SelectedItem is string op)
            {
                node.Op = op;
                bool portsChanged = EnsureOpPorts(node, op);
                UpdateNodeTitleVisual(node);
                if (portsChanged)
                {
                    RebuildSurface();
                    if (_nodeVisuals.TryGetValue(node.Id, out var visual))
                    {
                        _selectedNode = visual;
                        _selectedNodes.Clear();
                        _selectedNodes.Add(visual);
                    }
                    UpdateSelectionVisuals();
                }
                UpdateInspector();
                RefreshPreview();
                GraphChanged?.Invoke();
            }
        }

        private void InspectorFunc_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is ComboBox comboBox &&
                comboBox.DataContext is GraphNode node &&
                ReferenceEquals(node, _selectedNode.Node) &&
                comboBox.SelectedItem is string func)
            {
                node.Func = func;
                EnsureFuncPorts(node);
                RebuildSurface();
                if (_nodeVisuals.TryGetValue(node.Id, out var visual))
                {
                    _selectedNode = visual;
                    _selectedNodes.Clear();
                    _selectedNodes.Add(visual);
                }
                UpdateSelectionVisuals();
                UpdateInspector();
                RefreshPreview();
            }
        }

        private void EditIncludePath_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is TextBox textBox)
            {
                _selectedNode.Node.IncludePath = textBox.Text?.Trim() ?? "";
            }

            // Debounce the file I/O for SyncIncludePorts - restart timer on each keystroke
            _includePathDebounceTimer.Stop();
            _includePathDebounceTimer.Start();
        }

        private void OnIncludePathDebounce(object sender, EventArgs e)
        {
            _includePathDebounceTimer.Stop();

            if (_selectedNode == null)
            {
                return;
            }

            // Now do the file I/O to sync ports from the included graph
            SyncIncludePorts(_selectedNode.Node);
            RebuildSurface();
            UpdateInspector();

            GraphChanged?.Invoke();
        }


        private void SyncPortEntries(GraphNode node)
        {
            // Unsubscribe from old entries to prevent stale handlers firing
            foreach (var oldEntry in _portEntries)
            {
                oldEntry.NameChanged -= OnPortNameChanged;
                oldEntry.ParamChanged -= OnPortParamChanged;
                oldEntry.NegateChanged -= OnPortNegateChanged;
            }
            _portEntries.Clear();
            SelectedPortEntry = null;
            if (node == null)
            {
                return;
            }

            // In library graphs, Input/Output nodes use freeform naming (no signal catalog)
            bool isLibraryGraph = _graph != null && _graph.IsLibraryGraph;

            string effectiveSignalGroup = GetEffectiveSignalGroup(node);
            bool isOpNode = node.Kind == GraphNodeKind.Op;
            bool opVariadic = isOpNode && IsVariadicOp(node.Op);
            int opMinInputs = isOpNode ? GetOpMinInputCount(node.Op) : 0;
            var opInputPorts = isOpNode ? node.Ports.Where(p => p.Kind == GraphPortKind.Input).ToList() : null;
            foreach (var port in node.Ports)
            {
                if (isOpNode && port.Kind == GraphPortKind.Output)
                {
                    continue;
                }
                bool useSignalOptions = false;
                IReadOnlyList<string> signalOptions = null;
                bool showParamFields = false;
                GraphParam param = null;

                if (node.Kind == GraphNodeKind.Input && port.Kind == GraphPortKind.Output)
                {
                    // Library graphs use freeform port names; top-level graphs use signal catalog
                    if (!isLibraryGraph)
                    {
                        useSignalOptions = true;
                        signalOptions = GraphSignalCatalog.GetInputSignalsForGroup(effectiveSignalGroup);
                        MigratePortSignalSuffix(port, effectiveSignalGroup);
                    }
                }
                else if (node.Kind == GraphNodeKind.Output && port.Kind == GraphPortKind.Input)
                {
                    // Library graphs use freeform port names; top-level graphs use signal catalog
                    if (!isLibraryGraph)
                    {
                        useSignalOptions = true;
                        signalOptions = GraphSignalCatalog.GetOutputSignalsForGroup(effectiveSignalGroup);
                        MigratePortSignalSuffix(port, effectiveSignalGroup);
                    }
                }
                else if (node.Kind == GraphNodeKind.Param && port.Kind == GraphPortKind.Output)
                {
                    showParamFields = true;
                    string paramName = GetPortSignalName(node, port);
                    param = GetOrCreateParam(node, paramName);

                    // For Param nodes, SignalSuffix stores the freeform signal name
                    if (string.IsNullOrEmpty(port.SignalSuffix))
                    {
                        port.SignalSuffix = port.Name;
                    }
                }

                bool allowRemove = node.Kind != GraphNodeKind.Include;
                if (isOpNode)
                {
                    allowRemove = false;
                    if (opVariadic && port.Kind == GraphPortKind.Input)
                    {
                        int index = opInputPorts.IndexOf(port);
                        allowRemove = index >= opMinInputs;
                    }
                }
                bool hideParamUiButton = node.Kind == GraphNodeKind.Param;
                bool nameReadOnly = node.Kind == GraphNodeKind.Op;
                bool showNegate = isOpNode && port.Kind == GraphPortKind.Input && IsNegateSupportedOp(node.Op);
                var entry = new PortEditEntry(port, useSignalOptions, signalOptions, showParamFields, param, allowRemove,
                    hideParamUiButton, nameReadOnly, showNegate);
                entry.NameChanged += OnPortNameChanged;
                entry.ParamChanged += OnPortParamChanged;
                entry.NegateChanged += OnPortNegateChanged;
                _portEntries.Add(entry);
            }

            if (node.Kind == GraphNodeKind.Param && _portEntries.Count > 0)
            {
                SelectedPortEntry = _portEntries[0];
            }
        }

        /// <summary>
        /// Migrates old-style port names (full signal names) to the new SignalSuffix property.
        /// </summary>
        private static void MigratePortSignalSuffix(GraphPort port, string group)
        {
            if (!string.IsNullOrEmpty(port.SignalSuffix))
            {
                // Already migrated
                return;
            }

            string name = port.Name ?? "";
            string prefix = group + ".";

            if (name.StartsWith(prefix, StringComparison.Ordinal))
            {
                // Old-style full signal name, extract suffix
                port.SignalSuffix = name.Substring(prefix.Length);
                port.Name = port.SignalSuffix;
            }
            else if (!string.IsNullOrEmpty(name) && !name.Equals("out", StringComparison.OrdinalIgnoreCase) &&
                     !name.Equals("in", StringComparison.OrdinalIgnoreCase))
            {
                // Assume it's already a suffix or a valid signal name
                port.SignalSuffix = name;
            }
        }

        private void OnPortNameChanged(object sender, EventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is PortEditEntry entry)
            {
                var node = _selectedNode.Node;
                string desired = entry.Name?.Trim() ?? "";
                if (string.IsNullOrWhiteSpace(desired))
                {
                    return;
                }

                string unique = EnsureUniquePortName(node, entry.Port, desired);
                if (!string.Equals(unique, desired, StringComparison.Ordinal))
                {
                    _isInspectorUpdating = true;
                    entry.Name = unique;
                    _isInspectorUpdating = false;
                }

                string oldName = entry.Port.Name;
                RenamePort(node, oldName, unique);
                entry.Port.Name = unique;

                // For signal-bound Input/Output nodes, also update SignalSuffix
                // Library graph Input/Output nodes use freeform Name, not SignalSuffix
                bool isLibraryGraph = _graph != null && _graph.IsLibraryGraph;
                if ((node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Output) &&
                    entry.UseSignalOptions && !isLibraryGraph)
                {
                    entry.Port.SignalSuffix = unique;
                }
                else if (node.Kind == GraphNodeKind.Param && entry.Port.Kind == GraphPortKind.Output)
                {
                    // For Param nodes, validate that the full signal name doesn't collide with output signals
                    string oldFullName = GraphSignalCatalog.BuildSignalName(node.SignalGroup, oldName);
                    string newFullName = GraphSignalCatalog.BuildSignalName(node.SignalGroup, unique);
                    if (GraphSignalCatalog.IsValidOutputSignal(newFullName))
                    {
                        // Collision with output signal - revert and warn
                        _isInspectorUpdating = true;
                        entry.Name = oldName;
                        _isInspectorUpdating = false;
                        System.Windows.MessageBox.Show(
                            $"Parameter name '{newFullName}' conflicts with a reserved output signal name.",
                            "Invalid Parameter Name",
                            System.Windows.MessageBoxButton.OK,
                            System.Windows.MessageBoxImage.Warning);
                        return;
                    }

                    // For Param nodes, SignalSuffix stores the freeform signal name
                    entry.Port.SignalSuffix = unique;
                    RenameParam(oldFullName, newFullName);
                }

                // Use full signal name for param lookup
                string paramLookupName = node.Kind == GraphNodeKind.Param
                    ? GetPortSignalName(node, entry.Port)
                    : unique;
                entry.RefreshParamReference(GetParam(paramLookupName));

                // Rebuild surface to update port visuals reliably
                RebuildSurface();
                if (_nodeVisuals.TryGetValue(node.Id, out var visual))
                {
                    _selectedNode = visual;
                    _selectedNodes.Clear();
                    _selectedNodes.Add(visual);
                }
                UpdateSelectionVisuals();

                SyncPreviewEntries();
                RefreshPreview();
                _pendingUndoDebounce = true;
                GraphChanged?.Invoke();
            }
        }

        private void PortName_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Enter)
            {
                return;
            }

            if (sender is TextBox textBox)
            {
                var binding = BindingOperations.GetBindingExpression(textBox, TextBox.TextProperty);
                binding?.UpdateSource();
                e.Handled = true;
                return;
            }

            if (sender is ComboBox comboBox)
            {
                var binding = BindingOperations.GetBindingExpression(comboBox, ComboBox.TextProperty);
                binding?.UpdateSource();
                e.Handled = true;
            }
        }

        private void OnPortNegateChanged(object sender, EventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (sender is PortEditEntry entry)
            {
                var node = _selectedNode.Node;
                if (node.Kind != GraphNodeKind.Op || entry.Port.Kind != GraphPortKind.Input)
                {
                    return;
                }

                bool portsChanged = EnsureOpPorts(node, node.Op);
                if (portsChanged)
                {
                    RebuildSurface();
                    if (_nodeVisuals.TryGetValue(node.Id, out var visual))
                    {
                        _selectedNode = visual;
                        _selectedNodes.Clear();
                        _selectedNodes.Add(visual);
                    }
                    UpdateSelectionVisuals();
                }

                SyncPreviewEntries();
                RefreshPreview();
                _pendingUndoDebounce = true;
                GraphChanged?.Invoke();
            }
        }

        private void UpdatePortVisual(string nodeId, string oldName, string newName)
        {
            if (!_nodeVisuals.TryGetValue(nodeId, out var visual))
            {
                return;
            }

            foreach (var child in visual.InnerCanvas.Children)
            {
                // Update TextBlock labels
                if (child is TextBlock label && label.Tag is PortVisual portVisual)
                {
                    if (portVisual.PortName == oldName)
                    {
                        label.Text = newName;
                        portVisual.PortName = newName;
                    }
                }

                // Update param control tags
                if (child is FrameworkElement element && element.Tag is ParamControlTag tag)
                {
                    if (tag.PortName == oldName)
                    {
                        element.Tag = new ParamControlTag(newName, tag.YOffset);
                    }
                }

                // Update ellipse port visuals
                if (child is Ellipse ellipse && ellipse.Tag is PortVisual ellipsePort)
                {
                    if (ellipsePort.PortName == oldName)
                    {
                        ellipsePort.PortName = newName;
                    }
                }
            }

            // Update output value visuals
            if (visual.OutputValues != null)
            {
                foreach (var output in visual.OutputValues)
                {
                    if (output.PortName == oldName)
                    {
                        output.PortName = newName;
                    }
                }
            }

            UpdateNodeSize(visual);
        }

        private void OnPortParamChanged(object sender, EventArgs e)
        {
            if (_isInspectorUpdating)
            {
                return;
            }

            if (sender is PortEditEntry entry && _selectedNode != null)
            {
                string paramName = GetPortSignalName(_selectedNode.Node, entry.Port);
                if (_previewParamLookup.TryGetValue(paramName, out var previewEntry) && entry.Param != null)
                {
                    double nextValue = entry.Param.DefaultValue;
                    // Only overwrite preview value if it was still at the previous default.
                    if (Math.Abs(previewEntry.Value - entry.LastParamDefaultValue) < 1e-9)
                    {
                        previewEntry.Value = nextValue;
                    }
                }
            }

            RefreshPreview();
            _pendingUndoDebounce = true;
            GraphChanged?.Invoke();
        }

        private void EditParamUi_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null)
            {
                return;
            }

            var button = sender as Button;
            var entry = button?.DataContext as PortEditEntry;
            var param = entry?.Param;
            if (param == null)
            {
                return;
            }

            var dialog = new GraphParamUiDialog(param, _paramWidgetChoices)
            {
                Owner = Window.GetWindow(this)
            };
            if (dialog.ShowDialog() == true)
            {
                entry.RefreshParamReference(param);
                RebuildSurface();
                if (_nodeVisuals.TryGetValue(_selectedNode.Node.Id, out var visual))
                {
                    _selectedNode = visual;
                    _selectedNodes.Clear();
                    _selectedNodes.Add(visual);
                    UpdateSelectionVisuals();
                }
                SyncPreviewEntries();
                RefreshPreview();
                GraphChanged?.Invoke();
            }
        }

        private void SignalTree_SelectedItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            var tree = sender as TreeView;
            var entry = tree?.Tag as PortEditEntry;
            var node = e.NewValue as SignalTreeNode;
            if (entry == null || node == null || !node.IsLeaf)
            {
                return;
            }

            entry.Name = node.FullName;
            entry.IsSignalPopupOpen = false;
        }

        private string EnsureUniquePortName(GraphNode node, GraphPort port, string desired)
        {
            if (node == null)
            {
                return desired;
            }

            string candidate = desired;
            int suffix = 1;
            while (node.Ports.Any(p => !ReferenceEquals(p, port) &&
                                       string.Equals(p.Name, candidate, StringComparison.OrdinalIgnoreCase)))
            {
                candidate = $"{desired}_{suffix}";
                suffix++;
            }

            return candidate;
        }

        private void RenamePort(GraphNode node, string oldName, string newName)
        {
            if (node == null || oldName == newName)
            {
                return;
            }

            foreach (var link in _graph.Links)
            {
                if (link.FromNodeId == node.Id && link.FromPort == oldName)
                {
                    link.FromPort = newName;
                }
                if (link.ToNodeId == node.Id && link.ToPort == oldName)
                {
                    link.ToPort = newName;
                }
            }
        }

        private GraphParam GetOrCreateParam(GraphNode node, string name)
        {
            if (node == null)
            {
                return null;
            }

            if (!_graph.Params.TryGetValue(name, out var param))
            {
                param = new GraphParam
                {
                    Name = name,
                    DefaultValue = 0.0,
                    Min = 0.0,
                    Max = 1.0
                };
                _graph.Params[name] = param;
            }

            return param;
        }

        private void UpdateParamName(string oldName, string newName, GraphNode node)
        {
            if (oldName == newName)
            {
                return;
            }

            if (_graph.Params.TryGetValue(oldName, out var param))
            {
                _graph.Params.Remove(oldName);
                param.Name = newName;
                _graph.Params[newName] = param;
            }
            else
            {
                _graph.Params[newName] = new GraphParam { Name = newName };
            }
        }

        private void RenameParam(string oldName, string newName)
        {
            if (string.IsNullOrWhiteSpace(oldName) || string.IsNullOrWhiteSpace(newName) || oldName == newName)
            {
                return;
            }

            if (_graph.Params.TryGetValue(oldName, out var param))
            {
                _graph.Params.Remove(oldName);
                param.Name = newName;
                _graph.Params[newName] = param;
            }
            else
            {
                _graph.Params[newName] = new GraphParam { Name = newName };
            }
        }

        private GraphParam GetParam(string name)
        {
            if (string.IsNullOrWhiteSpace(name))
            {
                return null;
            }

            _graph.Params.TryGetValue(name, out var param);
            return param;
        }

        private string EnsureUniqueNodeName(GraphNode node, string desired)
        {
            string candidate = desired;
            int suffix = 1;
            while (_graph.Nodes.Any(n => !ReferenceEquals(n, node) &&
                                         (n.Kind == GraphNodeKind.Input || n.Kind == GraphNodeKind.Param) &&
                                         string.Equals(GetNodeName(n), candidate, StringComparison.OrdinalIgnoreCase)))
            {
                candidate = $"{desired}_{suffix}";
                suffix++;
            }

            if (!string.Equals(candidate, desired, StringComparison.Ordinal))
            {
                node.Title = candidate;
            }

            return candidate;
        }

        private void UpdateNodeTitleVisual(GraphNode node)
        {
            if (node == null)
            {
                return;
            }

            if (_nodeVisuals.TryGetValue(node.Id, out var visual))
            {
                visual.TitleBlock.Text = BuildNodeTitle(node);
                UpdateNodeSize(visual);
            }
        }

        private string BuildNodeTitle(GraphNode node)
        {
            if (node == null)
            {
                return "";
            }

            // In library graphs, Input/Output nodes use Title (freeform), not SignalGroup
            // Param nodes always use SignalGroup
            bool isLibraryGraph = _graph != null && _graph.IsLibraryGraph;
            bool usesSignalBinding = (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Output || node.Kind == GraphNodeKind.Param)
                                     && (!isLibraryGraph || node.Kind == GraphNodeKind.Param);

            if (usesSignalBinding && !string.IsNullOrWhiteSpace(node.SignalGroup))
            {
                return node.SignalGroup;
            }

            string title = string.IsNullOrWhiteSpace(node.Title) ? node.Kind.ToString() : node.Title;
            if (node.Kind == GraphNodeKind.Op && !string.IsNullOrWhiteSpace(node.Op))
            {
                return $"{title} ({node.Op})";
            }
            if (node.Kind == GraphNodeKind.Func && !string.IsNullOrWhiteSpace(node.Func))
            {
                return $"{title} ({node.Func})";
            }

            return title;
        }

        private double MeasureTextWidth(string text, double fontSize)
        {
            if (string.IsNullOrEmpty(text))
            {
                return 0.0;
            }

            double pixelsPerDip = 1.0;
            try
            {
                pixelsPerDip = VisualTreeHelper.GetDpi(this).PixelsPerDip;
            }
            catch
            {
                pixelsPerDip = 1.0;
            }

            var typeface = new Typeface(NodeFontFamily, FontStyles.Normal, FontWeights.Normal, FontStretches.Normal);
            var formatted = new FormattedText(
                text,
                CultureInfo.CurrentCulture,
                FlowDirection.LeftToRight,
                typeface,
                fontSize,
                Brushes.White,
                pixelsPerDip);
            return formatted.WidthIncludingTrailingWhitespace;
        }

        private SolidColorBrush GetTitleBarColor(GraphNodeKind kind)
        {
            switch (kind)
            {
                case GraphNodeKind.Input: return TitleBarInput;
                case GraphNodeKind.Output: return TitleBarOutput;
                case GraphNodeKind.Param: return TitleBarParam;
                case GraphNodeKind.Const: return TitleBarConst;
                case GraphNodeKind.Op: return TitleBarOp;
                case GraphNodeKind.Func: return TitleBarFunc;
                case GraphNodeKind.Include: return TitleBarInclude;
                default: return TitleBarConst;
            }
        }

        private double ComputeNodeWidth(GraphNode node)
        {
            if (node == null)
            {
                return NodeMinWidth;
            }

            double titleWidth = MeasureTextWidth(BuildNodeTitle(node), TitleFontSize);
            double maxInput = 0.0;
            double maxOutput = 0.0;
            foreach (var port in node.Ports)
            {
                if (!string.IsNullOrWhiteSpace(port.Name))
                {
                    double labelWidth = MeasureTextWidth(port.Name, PortFontSize);
                    if (port.Kind == GraphPortKind.Input)
                    {
                        maxInput = Math.Max(maxInput, labelWidth);
                    }
                    else
                    {
                        maxOutput = Math.Max(maxOutput, labelWidth);
                    }
                }
            }

            double contentWidth = Math.Max(titleWidth, maxInput + maxOutput + 18.0);
            if (node != null && node.Kind == GraphNodeKind.Param && node.Ports.Any(p => p.Kind == GraphPortKind.Output))
            {
                contentWidth = Math.Max(contentWidth, maxInput + maxOutput + ParamControlWidth + 18.0);
            }
            return Math.Max(NodeMinWidth, contentWidth + NodePadding * 2.0);
        }

        private double ComputeNodeHeight(GraphNode node)
        {
            int inputCount = 0;
            int outputCount = 0;
            if (node != null)
            {
                foreach (var port in node.Ports)
                {
                    if (port.Kind == GraphPortKind.Input)
                    {
                        inputCount++;
                    }
                    else
                    {
                        outputCount++;
                    }
                }
            }

            int portCount = Math.Max(1, Math.Max(inputCount, outputCount));
            return 42 + portCount * PortRowSpacing;
        }

        private void UpdateNodeSize(NodeVisual visual)
        {
            if (visual == null)
            {
                return;
            }

            double width = ComputeNodeWidth(visual.Node);
            double height = ComputeNodeHeight(visual.Node);
            visual.InnerCanvas.Width = width;
            visual.InnerCanvas.Height = height;
            visual.Container.Width = width;
            visual.Container.Height = height;
            if (visual.TitleBar != null)
            {
                visual.TitleBar.Width = width;
            }
            if (visual.TitleBarClip != null)
            {
                visual.TitleBarClip.Width = width;
            }

            foreach (var child in visual.InnerCanvas.Children)
            {
                var ellipse = child as Ellipse;
                if (ellipse?.Tag is PortVisual port && port.Kind == GraphPortKind.Output)
                {
                    Canvas.SetLeft(ellipse, width - 5);
                }

                var label = child as TextBlock;
                if (label?.Tag is PortVisual labelPort)
                {
                    double labelWidth = MeasureTextWidth(label.Text ?? labelPort.PortName, PortFontSize);
                    double x;
                    if (labelPort.Kind == GraphPortKind.Input)
                    {
                        x = PortLabelPadding;
                    }
                    else if (visual.Node.Kind == GraphNodeKind.Param)
                    {
                        x = width - ParamControlWidth - PortLabelPadding - labelWidth - 6;
                    }
                    else
                    {
                        x = width - PortLabelPadding - labelWidth;
                    }
                    Canvas.SetLeft(label, x);
                }

                var element = child as FrameworkElement;
                var tag = element?.Tag as ParamControlTag;
                if (tag != null && visual.Node.Kind == GraphNodeKind.Param)
                {
                    int index = GetOutputPortIndex(visual.Node, tag.PortName);
                    if (index >= 0)
                    {
                        double y = 26 + index * PortRowSpacing;
                        Canvas.SetLeft(element, width - ParamControlWidth - PortLabelPadding);
                        Canvas.SetTop(element, y + tag.YOffset);
                        element.Width = ParamControlWidth;
                        element.Height = ParamControlHeight;
                    }
                }
            }

            UpdateOutputValuePositions(visual, width);
        }

        private void UpdateOutputValueLabels()
        {
            foreach (var visual in _nodeVisuals.Values)
            {
                if (visual.OutputValues == null || visual.OutputValues.Count == 0)
                {
                    continue;
                }

                foreach (var output in visual.OutputValues)
                {
                    string valueKey = GetOutputValueKey(visual.Node, output.PortName);
                    if (string.IsNullOrWhiteSpace(valueKey))
                    {
                        output.Label.Text = "";
                        continue;
                    }

                    if (_nodeValues.TryGetValue(valueKey, out var value))
                    {
                        output.Label.Text = value.ToString("F3", CultureInfo.InvariantCulture);
                    }
                    else
                    {
                        output.Label.Text = "";
                    }
                }
            }
        }

        private void UpdateOutputValuePositions(NodeVisual visual, double width)
        {
            if (visual.OutputValues == null || visual.OutputValues.Count == 0)
            {
                return;
            }

            foreach (var output in visual.OutputValues)
            {
                int outputIndex = GetOutputPortIndex(visual.Node, output.PortName);
                double y = 26 + outputIndex * PortRowSpacing;
                Canvas.SetLeft(output.Label, width + 6);
                Canvas.SetTop(output.Label, y - 2);
            }
        }

        private string GetOutputValueKey(GraphNode node, string portName)
        {
            if (node == null)
            {
                return "";
            }

            if (node.Kind == GraphNodeKind.Include ||
                node.Kind == GraphNodeKind.Output ||
                node.Kind == GraphNodeKind.Input ||
                node.Kind == GraphNodeKind.Param)
            {
                return $"{node.Id}:{portName}";
            }

            return node.Id;
        }

        private void UpdatePortHandleVisibility(NodeVisual node)
        {
            if (node == null)
            {
                return;
            }

            bool forceVisible = _pendingPort != null || _edgePreviewSource != null || _edgeDragLink != null;
            bool selected = _selectedNodes.Contains(node);
            bool hovered = ReferenceEquals(node, _hoverPortNode);
            foreach (var child in node.InnerCanvas.Children)
            {
                var ellipse = child as Ellipse;
                if (ellipse?.Tag is PortVisual port)
                {
                    ellipse.Visibility = forceVisible || selected || hovered ? Visibility.Visible : Visibility.Collapsed;
                    if (port.Kind == GraphPortKind.Output)
                    {
                        Canvas.SetLeft(ellipse, node.InnerCanvas.Width - 5);
                    }
                }
            }
        }

        private void UpdateAllPortHandleVisibility()
        {
            foreach (var node in _nodeVisuals.Values)
            {
                UpdatePortHandleVisibility(node);
            }
        }

        private void UpdateHoverPortVisibility(Point cursor)
        {
            if (_pendingPort != null || _edgePreviewSource != null || _edgeDragLink != null)
            {
                return;
            }

            NodeVisual closest = null;
            double bestDistance = double.MaxValue;
            foreach (var node in _nodeVisuals.Values)
            {
                var bounds = new Rect(
                    node.Node.X - PortHoverPadding,
                    node.Node.Y - PortHoverPadding,
                    node.Container.Width + PortHoverPadding * 2.0,
                    node.Container.Height + PortHoverPadding * 2.0);
                if (!bounds.Contains(cursor))
                {
                    continue;
                }

                double dx = 0.0;
                if (cursor.X < node.Node.X)
                {
                    dx = node.Node.X - cursor.X;
                }
                else if (cursor.X > node.Node.X + node.Container.Width)
                {
                    dx = cursor.X - (node.Node.X + node.Container.Width);
                }

                double dy = 0.0;
                if (cursor.Y < node.Node.Y)
                {
                    dy = node.Node.Y - cursor.Y;
                }
                else if (cursor.Y > node.Node.Y + node.Container.Height)
                {
                    dy = cursor.Y - (node.Node.Y + node.Container.Height);
                }

                double distance = Math.Sqrt(dx * dx + dy * dy);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    closest = node;
                }
            }

            if (!ReferenceEquals(_hoverPortNode, closest))
            {
                _hoverPortNode = closest;
                UpdateAllPortHandleVisibility();
            }
        }

        private static bool TryParseDouble(string text, out double value)
        {
            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) ||
                   double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        }

        private static string MakeRelativePath(string baseDir, string fullPath)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(baseDir) || string.IsNullOrWhiteSpace(fullPath))
                {
                    return fullPath;
                }

                var baseUri = new Uri(AppendDirectorySeparator(System.IO.Path.GetFullPath(baseDir)));
                var fullUri = new Uri(System.IO.Path.GetFullPath(fullPath));
                if (baseUri.Scheme != fullUri.Scheme)
                {
                    return fullPath;
                }

                string relative = Uri.UnescapeDataString(baseUri.MakeRelativeUri(fullUri).ToString());
                return relative.Replace('/', System.IO.Path.DirectorySeparatorChar);
            }
            catch
            {
                return fullPath;
            }
        }

        private static string AppendDirectorySeparator(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return path;
            }

            if (!path.EndsWith(System.IO.Path.DirectorySeparatorChar.ToString(), StringComparison.Ordinal))
            {
                return path + System.IO.Path.DirectorySeparatorChar;
            }

            return path;
        }

        private void ButtonOpenInclude_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Include)
            {
                return;
            }

            string path = _selectedNode.Node.IncludePath;
            if (!string.IsNullOrWhiteSpace(path))
            {
                // From inspector button, don't auto-select context (pass null)
                IncludeOpenRequested?.Invoke(path, null);
            }
        }

        private void ButtonBrowseInclude_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Include)
            {
                return;
            }

            var dialog = new OpenFileDialog
            {
                Filter = "Graph JSON (*.json)|*.json",
                DefaultExt = "json"
            };

            if (!string.IsNullOrWhiteSpace(BaseDirectory) && Directory.Exists(BaseDirectory))
            {
                dialog.InitialDirectory = BaseDirectory;
            }

            if (dialog.ShowDialog() == true)
            {
                string path = dialog.FileName;
                if (!string.IsNullOrWhiteSpace(BaseDirectory))
                {
                    path = MakeRelativePath(BaseDirectory, dialog.FileName);
                }
                if (sender is Button button && button.Tag is TextBox textBox)
                {
                    textBox.Text = path;
                }
                else if (_selectedNode != null)
                {
                    _selectedNode.Node.IncludePath = path;
                    UpdateInspector();
                }
            }
        }

        private void RebuildIncludePortEditors(GraphNode node)
        {
            IncludeInputNames.Clear();
            IncludeOutputNames.Clear();

            // Show error if interface extraction failed
            var iface = node.CachedInterface;
            if (iface != null && !iface.IsValid)
            {
                IncludeErrorMessage = iface.Error;
                IncludeErrorVisible = true;
            }
            else
            {
                IncludeErrorMessage = "";
                IncludeErrorVisible = false;
            }

            foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
            {
                IncludeInputNames.Add(port.Name);
            }
            foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
            {
                IncludeOutputNames.Add(port.Name);
            }

            if (IncludeInputNames.Count == 0)
            {
                IncludeInputNames.Add("(none)");
            }
            if (IncludeOutputNames.Count == 0)
            {
                IncludeOutputNames.Add("(none)");
            }

            // Legacy panel removed; collections drive the template.
        }

        /// <summary>
        /// Synchronizes an Include node's ports with the interface of its included graph.
        /// Preserves existing links where port names match.
        /// </summary>
        private void SyncIncludePorts(GraphNode node)
        {
            if (node == null || node.Kind != GraphNodeKind.Include || string.IsNullOrWhiteSpace(node.IncludePath))
            {
                return;
            }

            // Extract interface from included graph
            var iface = GraphSerializer.ExtractInterfaceFromPath(node.IncludePath, BaseDirectory);
            node.CachedInterface = iface;

            if (!iface.IsValid)
            {
                // Show error but don't clear ports (user might fix path)
                return;
            }

            // Build sets of new port names
            var newInputNames = new HashSet<string>(iface.Inputs);
            var newOutputNames = new HashSet<string>(iface.Outputs);

            // Clear ports
            node.Ports.Clear();

            // Add input ports from interface
            foreach (var inputName in iface.Inputs)
            {
                node.Ports.Add(new GraphPort { Name = inputName, Kind = GraphPortKind.Input });
            }

            // Add output ports from interface
            foreach (var outputName in iface.Outputs)
            {
                node.Ports.Add(new GraphPort { Name = outputName, Kind = GraphPortKind.Output });
            }

            // Remove links to/from ports that no longer exist
            _graph.Links.RemoveAll(link =>
            {
                if (link.ToNodeId == node.Id && !newInputNames.Contains(link.ToPort))
                    return true;
                if (link.FromNodeId == node.Id && !newOutputNames.Contains(link.FromPort))
                    return true;
                return false;
            });
        }

        private void ButtonRefreshIncludePorts_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Include)
            {
                return;
            }

            SyncIncludePorts(_selectedNode.Node);
            RebuildSurface();
            UpdateInspector();
            GraphChanged?.Invoke();
        }

        private void EnsureEdgePreview()
        {
            if (_edgePreview != null)
            {
                return;
            }

            _edgePreview = new System.Windows.Shapes.Path
            {
                Stroke = new SolidColorBrush(Color.FromRgb(180, 180, 180)),
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 4, 3 },
                Visibility = Visibility.Collapsed,
                IsHitTestVisible = false
            };
            CanvasSurface.Children.Add(_edgePreview);
        }

        private void StartEdgePreview(PortVisual port)
        {
            if (port == null)
            {
                return;
            }

            EnsureEdgePreview();
            _edgePreviewSource = port;
            UpdateAllPortHandleVisibility();
            UpdateEdgePreview(Mouse.GetPosition(CanvasSurface));
        }

        private void StartEdgeDrag(LinkVisual link, Point startPoint, bool rewireSource)
        {
            if (link == null)
            {
                return;
            }

            _edgeDragLink = link;
            _edgeDragFromSource = rewireSource;
            UpdateAllPortHandleVisibility();
            UpdateEdgePreview(startPoint);
        }

        private void UpdateEdgePreview(Point currentPoint)
        {
            if (_edgeDragLink == null && _edgePreviewSource == null)
            {
                return;
            }

            if (_edgeDragLink != null && _edgeDragFromSource)
            {
                if (!_nodeVisuals.TryGetValue(_edgeDragLink.Link.ToNodeId, out var toNode))
                {
                    return;
                }

                Point to = GetPortAnchor(toNode, _edgeDragLink.Link.ToPort, GraphPortKind.Input);
                _edgePreview.Data = BuildLinkGeometry(currentPoint, to, null);
            }
            else
            {
                string fromNodeId = _edgeDragLink != null ? _edgeDragLink.Link.FromNodeId : _edgePreviewSource.NodeId;
                string fromPort = _edgeDragLink != null ? _edgeDragLink.Link.FromPort : _edgePreviewSource.PortName;
                var fromKind = _edgeDragLink != null ? GraphPortKind.Output : _edgePreviewSource.Kind;
                if (!_nodeVisuals.TryGetValue(fromNodeId, out var fromNode))
                {
                    return;
                }

                Point from = GetPortAnchor(fromNode, fromPort, fromKind);
                _edgePreview.Data = BuildLinkGeometry(from, currentPoint, null);
            }
            _edgePreview.Visibility = Visibility.Visible;
        }

        private void HideEdgePreview()
        {
            if (_edgePreview != null)
            {
                _edgePreview.Visibility = Visibility.Collapsed;
                _edgePreview.Data = null;
            }
        }

        private void CancelEdgeDrag()
        {
            if (_edgeDragLink != null)
            {
                _edgeDragLink = null;
                _edgeDragFromSource = false;
                HideEdgePreview();
                UpdateAllPortHandleVisibility();
            }
        }

        private void CancelEdgePreview()
        {
            _edgePreviewSource = null;
            HideEdgePreview();
            UpdateAllPortHandleVisibility();
        }

        private void Node_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (sender is Border border && border.Tag is GraphNode node)
            {
                var menu = new ContextMenu();
                menu.Items.Add(BuildMenuItem("Delete Node", () =>
                {
                    _selectedNodes.Clear();
                    _selectedNodes.Add(_nodeVisuals[node.Id]);
                    DeleteSelectedNodes();
                }));
                menu.Items.Add(BuildMenuItem("Duplicate Node", () =>
                {
                    DuplicateNode(node);
                }));

                if (node.Kind == GraphNodeKind.Include)
                {
                    var item = BuildMenuItem("Open Include", () =>
                    {
                        if (!string.IsNullOrWhiteSpace(node.IncludePath))
                        {
                            // From context menu, don't auto-select context (pass null)
                            IncludeOpenRequested?.Invoke(node.IncludePath, null);
                        }
                    });
                    item.IsEnabled = !string.IsNullOrWhiteSpace(node.IncludePath);
                    menu.Items.Add(new Separator());
                    menu.Items.Add(item);
                }

                border.ContextMenu = menu;
            }
        }

        private void DuplicateNode(GraphNode node)
        {
            if (node == null)
            {
                return;
            }

            var copy = new GraphNode
            {
                Title = node.Title,
                Kind = node.Kind,
                X = node.X + 20,
                Y = node.Y + 20,
                Op = node.Op,
                Func = node.Func,
                IncludePath = node.IncludePath,
                ConstValue = node.ConstValue,
                SignalGroup = node.SignalGroup
            };
            foreach (var port in node.Ports)
            {
                copy.Ports.Add(new GraphPort
                {
                    Name = port.Name,
                    Kind = port.Kind,
                    SignalSuffix = port.SignalSuffix,
                    Negate = port.Negate
                });
            }
            _graph.Nodes.Add(copy);
            RebuildSurface();
        }

        private sealed class PreviewEntry : INotifyPropertyChanged
        {
            private double _value;

            public PreviewEntry(string name, double value)
            {
                Name = name;
                _value = value;
            }

            public string Name { get; }

            public double Value
            {
                get => _value;
                set
                {
                    if (Math.Abs(_value - value) < 1e-9)
                    {
                        return;
                    }

                    _value = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Value)));
                    ValueChanged?.Invoke(this, EventArgs.Empty);
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;
            public event EventHandler ValueChanged;
        }

        private sealed class NodeVisual
        {
            public Border Container;
            public GraphNode Node;
            public TextBlock TitleBlock;
            public Canvas InnerCanvas;
            public Rectangle TitleBar;
            public Rectangle TitleBarClip;
            public List<PortValueVisual> OutputValues = new List<PortValueVisual>();
        }

        private sealed class PortValueVisual
        {
            public string PortName;
            public TextBlock Label;

            public PortValueVisual(string portName, TextBlock label)
            {
                PortName = portName;
                Label = label;
            }
        }


        public sealed class PortEditEntry : INotifyPropertyChanged
        {
            private string _name;
            private string _paramDefault;
            private string _paramMin;
            private string _paramMax;
            private double _lastParamDefaultValue;
            private string _uiWidget;
            private string _uiLabel;
            private string _uiGroup;
            private string _uiUnits;
            private string _uiStep;
            private string _uiPrecision;
            private bool _uiLogScale;
            private string _uiOptionsText;
            private GraphParam _param;
            private GraphParamUi _paramUi;
            private bool _isSignalPopupOpen;
            private bool _isNegated;

            public PortEditEntry(GraphPort port, bool useSignalOptions, IReadOnlyList<string> signalOptions,
                bool showParamFields, GraphParam param, bool allowRemove, bool hideParamUiButton, bool nameReadOnly, bool showNegate)
            {
                Port = port;
                // For Input/Output nodes with signal options, use SignalSuffix for display
                // For Param nodes or other cases, use Name
                _name = useSignalOptions && !string.IsNullOrEmpty(port.SignalSuffix)
                    ? port.SignalSuffix
                    : port.Name;
                _isNegated = port.Negate;
                UseSignalOptions = useSignalOptions;
                SignalOptions = signalOptions ?? Array.Empty<string>();
                SignalTree = BuildSignalTree(SignalOptions);
                ShowParamFields = showParamFields;
                HideParamUiButton = hideParamUiButton;
                AllowRemove = allowRemove;
                IsNameReadOnly = nameReadOnly;
                ShowNegate = showNegate;
                _param = param;
                _paramUi = EnsureParamUi();
                SyncParamText();
                SyncParamUiText();
            }

            public GraphPort Port { get; }
            public GraphParam Param => _param;
            public string KindLabel => Port.Kind == GraphPortKind.Input ? "Input" : "Output";
            public bool UseSignalOptions { get; }
            public IReadOnlyList<string> SignalOptions { get; }
            public ObservableCollection<SignalTreeNode> SignalTree { get; }
            public bool ShowParamFields { get; }
            public bool HideParamUiButton { get; }
            public bool AllowRemove { get; }
            public bool IsNameReadOnly { get; }
            public bool ShowNegate { get; }
            public bool IsSignalPopupOpen
            {
                get => _isSignalPopupOpen;
                set
                {
                    if (_isSignalPopupOpen == value)
                    {
                        return;
                    }
                    _isSignalPopupOpen = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsSignalPopupOpen)));
                }
            }

            public bool IsNegated
            {
                get => _isNegated;
                set
                {
                    if (_isNegated == value)
                    {
                        return;
                    }
                    _isNegated = value;
                    if (Port != null)
                    {
                        Port.Negate = value;
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsNegated)));
                    NegateChanged?.Invoke(this, EventArgs.Empty);
                }
            }

            public string Name
            {
                get => _name;
                set
                {
                    if (_name == value)
                    {
                        return;
                    }
                    _name = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Name)));
                    NameChanged?.Invoke(this, EventArgs.Empty);
                }
            }

            public string ParamDefault
            {
                get => _paramDefault;
                set
                {
                    if (_paramDefault == value)
                    {
                        return;
                    }
                    _paramDefault = value;
                    if (TryParse(value, out var parsed) && _param != null)
                    {
                        _lastParamDefaultValue = _param.DefaultValue;
                        _param.DefaultValue = parsed;
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamDefault)));
                }
            }

            public double LastParamDefaultValue => _lastParamDefaultValue;

            public string ParamMin
            {
                get => _paramMin;
                set
                {
                    if (_paramMin == value)
                    {
                        return;
                    }
                    _paramMin = value;
                    if (TryParse(value, out var parsed) && _param != null)
                    {
                        _param.Min = parsed;
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamMin)));
                }
            }

            public string ParamMax
            {
                get => _paramMax;
                set
                {
                    if (_paramMax == value)
                    {
                        return;
                    }
                    _paramMax = value;
                    if (TryParse(value, out var parsed) && _param != null)
                    {
                        _param.Max = parsed;
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamMax)));
                }
            }

            public string UiWidget
            {
                get => _uiWidget;
                set
                {
                    if (_uiWidget == value)
                    {
                        return;
                    }
                    _uiWidget = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Widget = value ?? "";
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiWidget)));
                }
            }

            public string UiLabel
            {
                get => _uiLabel;
                set
                {
                    if (_uiLabel == value)
                    {
                        return;
                    }
                    _uiLabel = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Label = value ?? "";
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiLabel)));
                }
            }

            public string UiGroup
            {
                get => _uiGroup;
                set
                {
                    if (_uiGroup == value)
                    {
                        return;
                    }
                    _uiGroup = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Group = value ?? "";
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiGroup)));
                }
            }

            public string UiUnits
            {
                get => _uiUnits;
                set
                {
                    if (_uiUnits == value)
                    {
                        return;
                    }
                    _uiUnits = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Units = value ?? "";
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiUnits)));
                }
            }

            public string UiStep
            {
                get => _uiStep;
                set
                {
                    if (_uiStep == value)
                    {
                        return;
                    }
                    _uiStep = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Step = ParseNullableDouble(value);
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiStep)));
                }
            }

            public string UiPrecision
            {
                get => _uiPrecision;
                set
                {
                    if (_uiPrecision == value)
                    {
                        return;
                    }
                    _uiPrecision = value;
                    if (_paramUi != null)
                    {
                        _paramUi.Precision = ParseNullableInt(value);
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiPrecision)));
                }
            }

            public bool UiLogScale
            {
                get => _uiLogScale;
                set
                {
                    if (_uiLogScale == value)
                    {
                        return;
                    }
                    _uiLogScale = value;
                    if (_paramUi != null)
                    {
                        _paramUi.LogScale = value;
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiLogScale)));
                }
            }

            public string UiOptionsText
            {
                get => _uiOptionsText;
                set
                {
                    if (_uiOptionsText == value)
                    {
                        return;
                    }
                    _uiOptionsText = value ?? "";
                    if (_paramUi != null)
                    {
                        _paramUi.Options.Clear();
                        foreach (var option in ParseOptionsText(_uiOptionsText))
                        {
                            _paramUi.Options.Add(option);
                        }
                        ParamChanged?.Invoke(this, EventArgs.Empty);
                    }
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiOptionsText)));
                }
            }

            public void RefreshParamReference(GraphParam param)
            {
                _param = param;
                _paramUi = EnsureParamUi();
                SyncParamText();
                SyncParamUiText();
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamDefault)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamMin)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(ParamMax)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiWidget)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiLabel)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiGroup)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiUnits)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiStep)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiPrecision)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiLogScale)));
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(UiOptionsText)));
            }

            private void SyncParamText()
            {
                if (_param == null)
                {
                    _paramDefault = "";
                    _paramMin = "";
                    _paramMax = "";
                    _lastParamDefaultValue = 0.0;
                    return;
                }

                _paramDefault = _param.DefaultValue.ToString("F3", CultureInfo.InvariantCulture);
                _paramMin = _param.Min.ToString("F3", CultureInfo.InvariantCulture);
                _paramMax = _param.Max.ToString("F3", CultureInfo.InvariantCulture);
                _lastParamDefaultValue = _param.DefaultValue;
            }

            private void SyncParamUiText()
            {
                if (_paramUi == null)
                {
                    _uiWidget = "";
                    _uiLabel = "";
                    _uiGroup = "";
                    _uiUnits = "";
                    _uiStep = "";
                    _uiPrecision = "";
                    _uiLogScale = false;
                    _uiOptionsText = "";
                    return;
                }

                _uiWidget = _paramUi.Widget ?? "";
                _uiLabel = _paramUi.Label ?? "";
                _uiGroup = _paramUi.Group ?? "";
                _uiUnits = _paramUi.Units ?? "";
                _uiStep = _paramUi.Step?.ToString("G", CultureInfo.InvariantCulture) ?? "";
                _uiPrecision = _paramUi.Precision?.ToString(CultureInfo.InvariantCulture) ?? "";
                _uiLogScale = _paramUi.LogScale;
                _uiOptionsText = FormatOptionsText(_paramUi.Options);
            }

            public event PropertyChangedEventHandler PropertyChanged;
            public event EventHandler NameChanged;
            public event EventHandler ParamChanged;
            public event EventHandler NegateChanged;

            private static bool TryParse(string text, out double value)
            {
                return double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
            }

            private GraphParamUi EnsureParamUi()
            {
                if (_param == null)
                {
                    return null;
                }

                if (_param.Ui == null)
                {
                    _param.Ui = new GraphParamUi();
                }
                return _param.Ui;
            }

            private static double? ParseNullableDouble(string text)
            {
                if (string.IsNullOrWhiteSpace(text))
                {
                    return null;
                }
                if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
                {
                    return value;
                }
                return null;
            }

            private static int? ParseNullableInt(string text)
            {
                if (string.IsNullOrWhiteSpace(text))
                {
                    return null;
                }
                if (int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var value))
                {
                    return value;
                }
                return null;
            }

            private static ObservableCollection<SignalTreeNode> BuildSignalTree(IReadOnlyList<string> options)
            {
                var roots = new List<SignalTreeNode>();
                var rootLookup = new Dictionary<string, SignalTreeNode>(StringComparer.OrdinalIgnoreCase);

                foreach (var option in options)
                {
                    if (string.IsNullOrWhiteSpace(option))
                    {
                        continue;
                    }

                    var parts = option.Split(new[] { '.' }, StringSplitOptions.RemoveEmptyEntries);
                    if (parts.Length == 0)
                    {
                        continue;
                    }

                    SignalTreeNode current = null;
                    for (int idx = 0; idx < parts.Length; idx++)
                    {
                        string part = parts[idx];
                        if (idx == 0)
                        {
                            if (!rootLookup.TryGetValue(part, out current))
                            {
                                current = new SignalTreeNode(part, "");
                                roots.Add(current);
                                rootLookup[part] = current;
                            }
                        }
                        else
                        {
                            current = current.GetOrAdd(part);
                        }

                        if (idx == parts.Length - 1)
                        {
                            current.FullName = option;
                        }
                    }
                }

                return new ObservableCollection<SignalTreeNode>(roots);
            }

            private static IEnumerable<GraphParamOption> ParseOptionsText(string text)
            {
                var lines = (text ?? "").Split(new[] { "\r\n", "\n" }, StringSplitOptions.RemoveEmptyEntries);
                foreach (var line in lines)
                {
                    var trimmed = line.Trim();
                    if (string.IsNullOrWhiteSpace(trimmed))
                    {
                        continue;
                    }

                    string value = trimmed;
                    string label = trimmed;
                    int idx = trimmed.IndexOf('=');
                    if (idx >= 0)
                    {
                        value = trimmed.Substring(0, idx).Trim();
                        label = trimmed.Substring(idx + 1).Trim();
                    }

                    yield return new GraphParamOption
                    {
                        Value = value,
                        Label = label
                    };
                }
            }

            private static string FormatOptionsText(List<GraphParamOption> options)
            {
                if (options == null || options.Count == 0)
                {
                    return "";
                }

                var lines = new List<string>();
                foreach (var option in options)
                {
                    if (string.IsNullOrWhiteSpace(option?.Value) && string.IsNullOrWhiteSpace(option?.Label))
                    {
                        continue;
                    }
                    string value = option?.Value ?? "";
                    string label = option?.Label ?? "";
                    lines.Add(string.IsNullOrWhiteSpace(label) || value == label
                        ? value
                        : $"{value}={label}");
                }

                return string.Join(Environment.NewLine, lines);
            }
        }

        public sealed class SignalTreeNode
        {
            public SignalTreeNode(string name, string fullName)
            {
                Name = name;
                FullName = fullName;
            }

            public string Name { get; }
            public string FullName { get; set; }
            public ObservableCollection<SignalTreeNode> Children { get; } = new ObservableCollection<SignalTreeNode>();
            public bool IsLeaf => Children.Count == 0;

            public SignalTreeNode GetOrAdd(string name)
            {
                foreach (var child in Children)
                {
                    if (string.Equals(child.Name, name, StringComparison.OrdinalIgnoreCase))
                    {
                        return child;
                    }
                }

                var node = new SignalTreeNode(name, "");
                Children.Add(node);
                return node;
            }
        }

        private sealed class LinkVisual
        {
            public System.Windows.Shapes.Path Path;
            public GraphLink Link;
            public Ellipse Handle;
            public bool HasManualControls;
            public Point Control1;
            public Point Control2;
        }

        private sealed class PortVisual
        {
            public string NodeId;
            public string PortName;
            public GraphPortKind Kind;
        }
    }

    public sealed class InspectorTemplateSelector : DataTemplateSelector
    {
        public DataTemplate ConstTemplate { get; set; }
        public DataTemplate OpTemplate { get; set; }
        public DataTemplate FuncTemplate { get; set; }
        public DataTemplate InputTemplate { get; set; }
        public DataTemplate OutputTemplate { get; set; }
        public DataTemplate ParamTemplate { get; set; }
        public DataTemplate IncludeTemplate { get; set; }

        public override DataTemplate SelectTemplate(object item, DependencyObject container)
        {
            if (item is GraphNode node)
            {
                switch (node.Kind)
                {
                    case GraphNodeKind.Const:
                        return ConstTemplate;
                    case GraphNodeKind.Op:
                        return OpTemplate;
                    case GraphNodeKind.Func:
                        return FuncTemplate;
                    case GraphNodeKind.Input:
                        return InputTemplate;
                    case GraphNodeKind.Output:
                        return OutputTemplate;
                    case GraphNodeKind.Param:
                        return ParamTemplate;
                    case GraphNodeKind.Include:
                        return IncludeTemplate;
                }
            }

            return base.SelectTemplate(item, container);
        }
    }

    public sealed class SignalGroupOptionsConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is GraphNodeKind kind)
            {
                switch (kind)
                {
                    case GraphNodeKind.Input:
                        return GraphSignalCatalog.InputGroups;
                    case GraphNodeKind.Output:
                        return GraphSignalCatalog.OutputGroups;
                    case GraphNodeKind.Param:
                        return GraphSignalCatalog.ParamGroups;
                }
            }

            return Array.Empty<string>();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public sealed class NullToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return value == null ? Visibility.Collapsed : Visibility.Visible;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public sealed class OpVariadicVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value is string op && GraphEditorControl.IsVariadicOp(op))
            {
                return Visibility.Visible;
            }
            return Visibility.Collapsed;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
