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
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;

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
        private readonly GraphPreviewEvaluator _previewEvaluator = new GraphPreviewEvaluator();
        private readonly ObservableCollection<PortEditEntry> _portEntries = new ObservableCollection<PortEditEntry>();
        private bool _isPanning;
        private bool _isSelecting;
        private Point _lastPanPoint;
        private NodeVisual _dragNode;
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
        private static readonly FontFamily NodeFontFamily = new FontFamily("Segoe UI");
        private const double TitleFontSize = 11.0;
        private const double PortFontSize = 10.0;
        private const double NodeMinWidth = 80.0;
        private const double NodePadding = 4.0;
        private const double PortLabelPadding = 6.0;
        private const double EdgeRewirePickRadius = 28.0;
        private const double PortHoverPadding = 12.0;
        private const double GridSize = 10.0;
        private bool _isInspectorUpdating;
        private readonly string[] _opChoices = { "add", "sub", "mul", "div", "min", "max", "abs", "clamp", "lerp" };
        private readonly string[] _funcChoices = { "qhat_eff", "torque_norm", "rpm_norm", "assist_loss" };
        private double _curveTension = 0.5;
        private const double HandleSize = 10.0;
        private LinkVisual _draggingHandle;
        private Point _handleDragOffset;

        public event Action<string> IncludeOpenRequested;
        public event Action GraphChanged;
        public string BaseDirectory { get; set; }

        public GraphEditorControl()
        {
            InitializeComponent();
            _graph = new GraphDefinition();
            PreviewInputsList.ItemsSource = _previewInputEntries;
            PreviewParamsList.ItemsSource = _previewParamEntries;
            PortsList.ItemsSource = _portEntries;
            EditOp.ItemsSource = _opChoices;
            EditFunc.ItemsSource = _funcChoices;
        }

        public void SetGraph(GraphDefinition graph)
        {
            _graph = graph ?? new GraphDefinition();
            RebuildSurface();
        }

        public GraphDefinition GetGraph()
        {
            return _graph;
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

            UpdateInspector();
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

        public void SaveGraphToFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            string json = GraphSerializer.Serialize(_graph);
            File.WriteAllText(path, json);
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
            SyncPreviewEntries();
            RefreshPreview();
            UpdateInspector();
            GraphChanged?.Invoke();
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
                double y = 26 + portIndex * 14;
                double x = port.Kind == GraphPortKind.Input ? -5 : nodeCanvas.Width - 5;
                Canvas.SetLeft(portEllipse, x);
                Canvas.SetTop(portEllipse, y);
                nodeCanvas.Children.Add(portEllipse);

                var label = new TextBlock
                {
                    Text = port.Name,
                    Foreground = Brushes.LightGray,
                    FontSize = PortFontSize,
                    FontFamily = NodeFontFamily,
                    Tag = portVisual
                };
                double labelWidth = MeasureTextWidth(port.Name, PortFontSize);
                Canvas.SetLeft(label, port.Kind == GraphPortKind.Input ? PortLabelPadding : width - PortLabelPadding - labelWidth);
                Canvas.SetTop(label, y - 2);
                nodeCanvas.Children.Add(label);

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
            return new NodeVisual { Container = container, Node = node, TitleBlock = title, InnerCanvas = nodeCanvas };
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
                    double y = node.Y + 26 + index * 14 + 5;
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
            if ((Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift)
            {
                _isSelecting = true;
                _selectionStart = e.GetPosition(CanvasSurface);
                UpdateSelectionRectangle(_selectionStart, _selectionStart);
                _selectionRect.Visibility = Visibility.Visible;
                CanvasSurface.CaptureMouse();
                e.Handled = true;
                return;
            }

            _isPanning = true;
            _lastPanPoint = e.GetPosition(this);
            CanvasSurface.CaptureMouse();
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
            _isPanning = false;
            CanvasSurface.ReleaseMouseCapture();
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
                _dragNode = _nodeVisuals[node.Id];
                if ((Keyboard.Modifiers & ModifierKeys.Shift) != ModifierKeys.Shift)
                {
                    _selectedLinks.Clear();
                    UpdateLinkSelectionVisuals();
                }
                if ((Keyboard.Modifiers & ModifierKeys.Shift) == ModifierKeys.Shift)
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
                    _selectedNodes.Clear();
                    _selectedNodes.Add(_dragNode);
                }

                _selectedNode = _selectedNodes.Count == 1 ? _dragNode : null;
                UpdateSelectionVisuals();
                UpdateInspector();
                Point mousePos = e.GetPosition(CanvasSurface);
                _dragOffset = new Point(mousePos.X - node.X, mousePos.Y - node.Y);
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
            _dragNode.Node.X = SnapToGrid(mousePos.X - _dragOffset.X);
            _dragNode.Node.Y = SnapToGrid(mousePos.Y - _dragOffset.Y);
            Canvas.SetLeft(_dragNode.Container, _dragNode.Node.X);
            Canvas.SetTop(_dragNode.Container, _dragNode.Node.Y);
            UpdateAllLinkGeometry();
        }

        private void Node_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (_dragNode != null)
            {
                _dragNode.Container.ReleaseMouseCapture();
                _dragNode = null;
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
            CanvasSurface.ContextMenu = BuildContextMenu(e.GetPosition(CanvasSurface));
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
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
            }
            else if (kind == GraphNodeKind.Func)
            {
                node.Func = _funcChoices[0];
                node.Ports.Add(new GraphPort { Name = "a", Kind = GraphPortKind.Input });
                node.Ports.Add(new GraphPort { Name = "b", Kind = GraphPortKind.Input });
                node.Ports.Add(new GraphPort { Name = "out", Kind = GraphPortKind.Output });
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
        }

        private void UpdateInspector()
        {
            if (_selectedNode == null)
            {
                TextNodeTitle.Text = _selectedNodes.Count > 1 ? "(multiple)" : "(none)";
                TextNodeKind.Text = _selectedNodes.Count > 1 ? "Multiple" : "";
                TextNodeValue.Text = "n/a";
                TextNodeInfo.Text = "";
                PanelEditFields.Visibility = _selectedNodes.Count > 0 ? Visibility.Collapsed : Visibility.Visible;
                return;
            }

            var node = _selectedNode.Node;
            TextNodeTitle.Text = string.IsNullOrWhiteSpace(node.Title) ? node.Id : node.Title;
            TextNodeKind.Text = node.Kind.ToString();

            if (_nodeValues.TryGetValue(node.Id, out var value))
            {
                TextNodeValue.Text = value.ToString("F4");
            }
            else
            {
                TextNodeValue.Text = "n/a";
            }

            TextNodeInfo.Text = BuildNodeInfo(node);
            UpdateInspectorEditFields(node);
        }

        private void GraphEditorControl_KeyDown(object sender, KeyEventArgs e)
        {
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

        private void SyncPreviewEntries()
        {
            var inputNames = new HashSet<string>();
            foreach (var node in _graph.Nodes.Where(n => n.Kind == GraphNodeKind.Input))
            {
                string name = GetNodeName(node);
                inputNames.Add(name);
                if (!_previewInputLookup.ContainsKey(name))
                {
                    AddPreviewEntry(_previewInputEntries, _previewInputLookup, name, 0.0);
                }
            }
            RemoveMissingEntries(_previewInputEntries, _previewInputLookup, inputNames);

            var paramNames = new HashSet<string>();
            foreach (var node in _graph.Nodes.Where(n => n.Kind == GraphNodeKind.Param))
            {
                string name = GetNodeName(node);
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
            RemoveMissingEntries(_previewParamEntries, _previewParamLookup, paramNames);
        }

        private void AddPreviewEntry(ObservableCollection<PreviewEntry> list,
            Dictionary<string, PreviewEntry> lookup, string name, double value)
        {
            var entry = new PreviewEntry(name, value);
            entry.ValueChanged += (_, __) => RefreshPreview();
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

        private void RefreshPreview()
        {
            try
            {
                var inputs = new Dictionary<string, double>();
                foreach (var entry in _previewInputEntries)
                {
                    inputs[entry.Name] = entry.Value;
                }

                var parameters = new Dictionary<string, double>();
                foreach (var entry in _previewParamEntries)
                {
                    parameters[entry.Name] = entry.Value;
                }

                var result = _previewEvaluator.Evaluate(_graph, inputs, parameters);
                UpdateNodeValues(result.NodeValues);
                TextPreviewStatus.Text = "";
            }
            catch (Exception ex)
            {
                UpdateNodeValues(null);
                TextPreviewStatus.Text = $"Preview error: {ex.Message}";
            }
        }

        private static string GetNodeName(GraphNode node)
        {
            return string.IsNullOrWhiteSpace(node.Title) ? node.Id : node.Title;
        }

        private string BuildNodeInfo(GraphNode node)
        {
            if (node.Kind == GraphNodeKind.Const)
            {
                return $"Const: {node.ConstValue}";
            }
            if (node.Kind == GraphNodeKind.Op)
            {
                return $"Op: {node.Op}";
            }
            if (node.Kind == GraphNodeKind.Func)
            {
                return $"Func: {node.Func}";
            }
            if (node.Kind == GraphNodeKind.Include)
            {
                return $"Include: {node.IncludePath}";
            }

            return "";
        }

        private void UpdateInspectorEditFields(GraphNode node)
        {
            _isInspectorUpdating = true;
            PanelEditFields.Visibility = Visibility.Visible;
            EditTitle.Text = GetNodeName(node);

            PanelConst.Visibility = node.Kind == GraphNodeKind.Const ? Visibility.Visible : Visibility.Collapsed;
            PanelOp.Visibility = node.Kind == GraphNodeKind.Op ? Visibility.Visible : Visibility.Collapsed;
            PanelFunc.Visibility = node.Kind == GraphNodeKind.Func ? Visibility.Visible : Visibility.Collapsed;
            PanelInclude.Visibility = node.Kind == GraphNodeKind.Include ? Visibility.Visible : Visibility.Collapsed;
            PanelParam.Visibility = node.Kind == GraphNodeKind.Param ? Visibility.Visible : Visibility.Collapsed;
            PanelPorts.Visibility = Visibility.Visible;
            ButtonAddInputPort.Visibility = node.Kind == GraphNodeKind.Output ? Visibility.Visible : Visibility.Collapsed;
            ButtonAddOutputPort.Visibility = (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Param)
                ? Visibility.Visible
                : Visibility.Collapsed;

            EditConst.Text = node.ConstValue.ToString("F3", CultureInfo.InvariantCulture);
            EditOp.SelectedItem = string.IsNullOrWhiteSpace(node.Op) ? "mul" : node.Op.ToLowerInvariant();
            EditFunc.SelectedItem = string.IsNullOrWhiteSpace(node.Func) ? _funcChoices[0] : node.Func;
            EditIncludePath.Text = node.IncludePath ?? "";
            ButtonOpenInclude.IsEnabled = node.Kind == GraphNodeKind.Include &&
                                          !string.IsNullOrWhiteSpace(node.IncludePath);

            if (node.Kind == GraphNodeKind.Param)
            {
                var param = GetOrCreateParam(node, GetNodeName(node));
                EditParamDefault.Text = param.DefaultValue.ToString("F3", CultureInfo.InvariantCulture);
                EditParamMin.Text = param.Min.ToString("F3", CultureInfo.InvariantCulture);
                EditParamMax.Text = param.Max.ToString("F3", CultureInfo.InvariantCulture);
            }

            SyncPortEntries(node);
            if (node.Kind == GraphNodeKind.Include)
            {
                RebuildIncludePortEditors(node);
            }

            _isInspectorUpdating = false;
        }

        private void ButtonAddInputPort_Click(object sender, RoutedEventArgs e)
        {
            AddPort(GraphPortKind.Input, "in");
        }

        private void ButtonAddOutputPort_Click(object sender, RoutedEventArgs e)
        {
            AddPort(GraphPortKind.Output, "out");
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
            SyncPortEntries(node);
            RebuildSurface();
        }

        private void EditTitle_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            string desired = EditTitle.Text?.Trim() ?? "";
            if (string.IsNullOrWhiteSpace(desired))
            {
                return;
            }

            var node = _selectedNode.Node;
            string oldName = GetNodeName(node);
            if (node.Kind == GraphNodeKind.Input || node.Kind == GraphNodeKind.Param)
            {
                desired = EnsureUniqueNodeName(node, desired);
            }

            node.Title = desired;
            UpdateNodeTitleVisual(node);

            if (node.Kind == GraphNodeKind.Param)
            {
                UpdateParamName(oldName, desired, node);
            }

            SyncPreviewEntries();
            RefreshPreview();
        }

        private void EditConst_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (TryParseDouble(EditConst.Text, out var value))
            {
                _selectedNode.Node.ConstValue = value;
                RefreshPreview();
            }
        }

        private void EditOp_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (EditOp.SelectedItem is string op)
            {
                _selectedNode.Node.Op = op;
                UpdateNodeTitleVisual(_selectedNode.Node);
                UpdateInspector();
                RefreshPreview();
            }
        }

        private void EditFunc_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_isInspectorUpdating || _selectedNode == null)
            {
                return;
            }

            if (EditFunc.SelectedItem is string func)
            {
                _selectedNode.Node.Func = func;
                UpdateNodeTitleVisual(_selectedNode.Node);
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

            _selectedNode.Node.IncludePath = EditIncludePath.Text?.Trim() ?? "";
            ButtonOpenInclude.IsEnabled = !string.IsNullOrWhiteSpace(_selectedNode.Node.IncludePath);
        }

        private void EditParamDefault_TextChanged(object sender, TextChangedEventArgs e)
        {
            UpdateParamValue(EditParamDefault.Text, (param, value) => param.DefaultValue = value);
        }

        private void EditParamMin_TextChanged(object sender, TextChangedEventArgs e)
        {
            UpdateParamValue(EditParamMin.Text, (param, value) => param.Min = value);
        }

        private void EditParamMax_TextChanged(object sender, TextChangedEventArgs e)
        {
            UpdateParamValue(EditParamMax.Text, (param, value) => param.Max = value);
        }

        private void UpdateParamValue(string text, Action<GraphParam, double> assign)
        {
            if (_isInspectorUpdating || _selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Param)
            {
                return;
            }

            if (!TryParseDouble(text, out var value))
            {
                return;
            }

            var param = GetOrCreateParam(_selectedNode.Node, GetNodeName(_selectedNode.Node));
            assign(param, value);
        }

        private void SyncPortEntries(GraphNode node)
        {
            _portEntries.Clear();
            if (node == null)
            {
                return;
            }

            foreach (var port in node.Ports)
            {
                var entry = new PortEditEntry(port);
                entry.NameChanged += OnPortNameChanged;
                _portEntries.Add(entry);
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

                RenamePort(node, entry.Port.Name, unique);
                entry.Port.Name = unique;
                RebuildSurface();
            }
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
                _isInspectorUpdating = true;
                EditTitle.Text = candidate;
                _isInspectorUpdating = false;
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
            return 34 + portCount * 14;
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
                    double labelWidth = MeasureTextWidth(labelPort.PortName, PortFontSize);
                    double x = labelPort.Kind == GraphPortKind.Input
                        ? PortLabelPadding
                        : width - PortLabelPadding - labelWidth;
                    Canvas.SetLeft(label, x);
                }
            }
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

        private void ButtonAddIncludeInput_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Include)
            {
                return;
            }

            AddIncludePort(_selectedNode.Node, GraphPortKind.Input);
        }

        private void ButtonAddIncludeOutput_Click(object sender, RoutedEventArgs e)
        {
            if (_selectedNode == null || _selectedNode.Node.Kind != GraphNodeKind.Include)
            {
                return;
            }

            AddIncludePort(_selectedNode.Node, GraphPortKind.Output);
        }

        private void AddIncludePort(GraphNode node, GraphPortKind kind)
        {
            string baseName = kind == GraphPortKind.Input ? "in" : "out";
            string name = EnsureUniquePortName(node, null, baseName);
            var port = new GraphPort { Name = name, Kind = kind };
            node.Ports.Add(port);
            RebuildSurface();
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
                IncludeOpenRequested?.Invoke(path);
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
                EditIncludePath.Text = path;
            }
        }

        private void RebuildIncludePortEditors(GraphNode node)
        {
            PanelIncludeInputs.Children.Clear();
            PanelIncludeOutputs.Children.Clear();

            foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Input))
            {
                PanelIncludeInputs.Children.Add(BuildIncludePortRow(node, port));
            }
            foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
            {
                PanelIncludeOutputs.Children.Add(BuildIncludePortRow(node, port));
            }
        }

        private UIElement BuildIncludePortRow(GraphNode node, GraphPort port)
        {
            var dock = new DockPanel { Margin = new Thickness(0, 2, 0, 2) };

            var box = new TextBox
            {
                Text = port.Name,
                Width = 132,
                Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                Foreground = Brushes.White,
                BorderBrush = new SolidColorBrush(Color.FromRgb(74, 74, 74)),
                BorderThickness = new Thickness(1),
                Padding = new Thickness(2, 1, 2, 1)
            };
            box.TextChanged += (_, __) =>
            {
                if (_isInspectorUpdating)
                {
                    return;
                }

                string desired = box.Text?.Trim() ?? "";
                if (string.IsNullOrWhiteSpace(desired))
                {
                    return;
                }

                string unique = EnsureUniquePortName(node, port, desired);
                if (!string.Equals(unique, desired, StringComparison.Ordinal))
                {
                    _isInspectorUpdating = true;
                    box.Text = unique;
                    _isInspectorUpdating = false;
                }

                RenamePort(node, port.Name, unique);
                port.Name = unique;
                RebuildSurface();
            };
            dock.Children.Add(box);

            var button = new Button
            {
                Content = "x",
                Width = 22,
                Height = 20,
                Margin = new Thickness(4, 0, 0, 0)
            };
            button.Click += (_, __) =>
            {
                node.Ports.Remove(port);
                _graph.Links.RemoveAll(link =>
                    (link.FromNodeId == node.Id && link.FromPort == port.Name) ||
                    (link.ToNodeId == node.Id && link.ToPort == port.Name));
                RebuildSurface();
            };
            DockPanel.SetDock(button, Dock.Right);
            dock.Children.Add(button);

            return dock;
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
                            IncludeOpenRequested?.Invoke(node.IncludePath);
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
                ConstValue = node.ConstValue
            };
            foreach (var port in node.Ports)
            {
                copy.Ports.Add(new GraphPort { Name = port.Name, Kind = port.Kind });
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
        }

        private sealed class PortEditEntry : INotifyPropertyChanged
        {
            private string _name;

            public PortEditEntry(GraphPort port)
            {
                Port = port;
                _name = port.Name;
            }

            public GraphPort Port { get; }
            public string KindLabel => Port.Kind == GraphPortKind.Input ? "Input" : "Output";

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

            public event PropertyChangedEventHandler PropertyChanged;
            public event EventHandler NameChanged;
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
}
