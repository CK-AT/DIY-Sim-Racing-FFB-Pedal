using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for GeneralKinematicsControl.xaml
    /// </summary>
    public partial class GeneralKinematicsControl : UserControl
    {
        private sealed class PinRow : INotifyPropertyChanged
        {
            private uint pinId;
            private double x;
            private double y;
            private bool grounded;
            private bool isContactPoint;
            private bool isRailInterface;

            public uint PinId
            {
                get => pinId;
                set
                {
                    if (pinId == value) return;
                    pinId = value;
                    OnPropertyChanged(nameof(PinId));
                }
            }

            public double X
            {
                get => x;
                set
                {
                    if (Math.Abs(x - value) < 1e-9) return;
                    x = value;
                    OnPropertyChanged(nameof(X));
                }
            }

            public double Y
            {
                get => y;
                set
                {
                    if (Math.Abs(y - value) < 1e-9) return;
                    y = value;
                    OnPropertyChanged(nameof(Y));
                }
            }

            public bool Grounded
            {
                get => grounded;
                set
                {
                    if (grounded == value) return;
                    grounded = value;
                    OnPropertyChanged(nameof(Grounded));
                }
            }

            public bool IsContactPoint
            {
                get => isContactPoint;
                set
                {
                    if (isContactPoint == value) return;
                    isContactPoint = value;
                    OnPropertyChanged(nameof(IsContactPoint));
                }
            }

            public bool IsRailInterface
            {
                get => isRailInterface;
                set
                {
                    if (isRailInterface == value) return;
                    isRailInterface = value;
                    OnPropertyChanged(nameof(IsRailInterface));
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;

            private void OnPropertyChanged(string propertyName)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        private sealed class BarRow : INotifyPropertyChanged
        {
            private readonly SortedSet<uint> pinIds = new SortedSet<uint>();
            private bool isMetering;

            public bool IsMetering
            {
                get => isMetering;
                set
                {
                    if (isMetering == value) return;
                    isMetering = value;
                    OnPropertyChanged(nameof(IsMetering));
                }
            }

            public string PinSummary => pinIds.Count == 0 ? "-" : string.Join(",", pinIds);

            public uint[] GetPinIds()
            {
                return pinIds.ToArray();
            }

            public void SetPins(IEnumerable<uint> pins)
            {
                pinIds.Clear();
                foreach (var id in pins)
                {
                    pinIds.Add(id);
                }
                OnPropertyChanged(nameof(PinSummary));
            }

            public void AddPin(uint id)
            {
                if (pinIds.Add(id))
                {
                    OnPropertyChanged(nameof(PinSummary));
                }
            }

            public void RemovePin(uint id)
            {
                if (pinIds.Remove(id))
                {
                    OnPropertyChanged(nameof(PinSummary));
                }
            }

            public bool RemovePinsNotIn(HashSet<uint> available)
            {
                bool changed = false;
                foreach (var id in pinIds.ToArray())
                {
                    if (!available.Contains(id))
                    {
                        pinIds.Remove(id);
                        changed = true;
                    }
                }
                if (changed)
                {
                    OnPropertyChanged(nameof(PinSummary));
                }
                return changed;
            }

            public event PropertyChangedEventHandler PropertyChanged;

            private void OnPropertyChanged(string propertyName)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        private sealed class BarVisual
        {
            public uint[] PinIds;
            public Line Line;
        }

        private GeneralKinematicConfig config;
        private GeneralKinematics.PoseCache poseCache;
        private DiyFfbPlugin plugin;
        private readonly ObservableCollection<PinRow> pinRows = new ObservableCollection<PinRow>();
        private readonly ObservableCollection<BarRow> barRows = new ObservableCollection<BarRow>();
        private readonly Dictionary<uint, Ellipse> pinShapes = new Dictionary<uint, Ellipse>();
        private readonly Dictionary<uint, TextBlock> pinLabels = new Dictionary<uint, TextBlock>();
        private readonly List<BarVisual> barVisuals = new List<BarVisual>();
        private readonly Dictionary<uint, int> pinIndexById = new Dictionary<uint, int>();
        private readonly DispatcherTimer rebuildTimer;
        private bool isLoading;
        private bool liveMode = true;
        private double lastAxisPosition;
        private double railTravelNegative;
        private double railTravelPositive;
        private double[] currentX;
        private double[] currentY;

        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;

        public GeneralKinematicsControl()
        {
            InitializeComponent();
            PinGrid.ItemsSource = pinRows;
            BarGrid.ItemsSource = barRows;
            pinRows.CollectionChanged += PinRows_CollectionChanged;
            barRows.CollectionChanged += BarRows_CollectionChanged;
            rebuildTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(200) };
            rebuildTimer.Tick += RebuildTimer_Tick;
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.plugin = plugin;
            BuildCanvas();
            RefreshPose();
        }

        public void UpdateConfig(GeneralKinematicConfig newConfig)
        {
            config = newConfig ?? new GeneralKinematicConfig();
            isLoading = true;
            LoadRowsFromConfig();
            railTravelNegative = config.RailTravelNegative;
            railTravelPositive = config.RailTravelPositive;
            TextRailNegative.Text = railTravelNegative.ToString("0.#", CultureInfo.CurrentCulture);
            TextRailPositive.Text = railTravelPositive.ToString("0.#", CultureInfo.CurrentCulture);
            isLoading = false;
            QueueRebuild();
        }

        public void OnAxisStateUpdate(AxisState axisState)
        {
            lastAxisPosition = axisState.Position;
            if (liveMode)
            {
                UpdatePose();
            }
        }

        private void PinRows_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.OldItems != null)
            {
                foreach (PinRow row in e.OldItems)
                {
                    row.PropertyChanged -= OnPinRowPropertyChanged;
                }
            }
            if (e.NewItems != null)
            {
                foreach (PinRow row in e.NewItems)
                {
                    row.PropertyChanged += OnPinRowPropertyChanged;
                }
            }
            if (isLoading) return;
            DropMissingBarPins();
            QueueRebuild();
        }

        private void BarRows_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.OldItems != null)
            {
                foreach (BarRow row in e.OldItems)
                {
                    row.PropertyChanged -= OnBarRowPropertyChanged;
                }
            }
            if (e.NewItems != null)
            {
                foreach (BarRow row in e.NewItems)
                {
                    row.PropertyChanged += OnBarRowPropertyChanged;
                }
            }
            if (isLoading) return;
            QueueRebuild();
        }

        private void OnPinRowPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (isLoading) return;
            var row = (PinRow)sender;
            switch (e.PropertyName)
            {
                case nameof(PinRow.Grounded):
                    if (row.Grounded)
                    {
                        ApplyExclusivePinFlags(row, PinFlag.Grounded);
                    }
                    break;
                case nameof(PinRow.IsContactPoint):
                    if (row.IsContactPoint)
                    {
                        ApplyExclusivePinFlags(row, PinFlag.Contact);
                    }
                    break;
                case nameof(PinRow.IsRailInterface):
                    if (row.IsRailInterface)
                    {
                        ApplyExclusivePinFlags(row, PinFlag.Rail);
                    }
                    break;
                case nameof(PinRow.PinId):
                    DropMissingBarPins();
                    break;
            }
            QueueRebuild();
        }

        private void OnBarRowPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (isLoading) return;
            var row = (BarRow)sender;
            if (e.PropertyName == nameof(BarRow.IsMetering) && row.IsMetering)
            {
                isLoading = true;
                foreach (var other in barRows)
                {
                    if (!ReferenceEquals(other, row) && other.IsMetering)
                    {
                        other.IsMetering = false;
                    }
                }
                isLoading = false;
            }
            QueueRebuild();
        }

        private enum PinFlag
        {
            Grounded,
            Contact,
            Rail
        }

        private void ApplyExclusivePinFlags(PinRow row, PinFlag flag)
        {
            isLoading = true;
            if (flag == PinFlag.Grounded)
            {
                row.IsContactPoint = false;
                row.IsRailInterface = false;
            }
            else if (flag == PinFlag.Contact)
            {
                row.Grounded = false;
                row.IsRailInterface = false;
                foreach (var other in pinRows)
                {
                    if (!ReferenceEquals(other, row))
                    {
                        other.IsContactPoint = false;
                    }
                }
            }
            else if (flag == PinFlag.Rail)
            {
                row.Grounded = false;
                row.IsContactPoint = false;
                foreach (var other in pinRows)
                {
                    if (!ReferenceEquals(other, row))
                    {
                        other.IsRailInterface = false;
                    }
                }
            }
            isLoading = false;
        }

        private void DropMissingBarPins()
        {
            var available = new HashSet<uint>(pinRows.Select(p => p.PinId));
            foreach (var bar in barRows)
            {
                bar.RemovePinsNotIn(available);
            }
        }

        private void LoadRowsFromConfig()
        {
            pinRows.Clear();
            foreach (var pin in config.Pins)
            {
                pinRows.Add(new PinRow
                {
                    PinId = pin.PinId,
                    X = pin.X,
                    Y = pin.Y,
                    Grounded = pin.Grounded,
                    IsContactPoint = pin.IsContactPoint,
                    IsRailInterface = pin.IsRailInterface
                });
            }

            barRows.Clear();
            foreach (var bar in config.Bars)
            {
                var row = new BarRow
                {
                    IsMetering = bar.IsMetering
                };
                row.SetPins(bar.PinIds);
                barRows.Add(row);
            }
        }

        private void RebuildTimer_Tick(object sender, EventArgs e)
        {
            rebuildTimer.Stop();
            RebuildCache();
        }

        private void QueueRebuild()
        {
            if (isLoading) return;
            rebuildTimer.Stop();
            rebuildTimer.Start();
        }

        private void RebuildConfigFromRows()
        {
            if (config == null)
            {
                config = new GeneralKinematicConfig();
            }
            config.Pins.Clear();
            foreach (var row in pinRows)
            {
                config.Pins.Add(new GeneralKinematicPin
                {
                    PinId = row.PinId,
                    X = (float)row.X,
                    Y = (float)row.Y,
                    Grounded = row.Grounded,
                    IsContactPoint = row.IsContactPoint,
                    IsRailInterface = row.IsRailInterface
                });
            }
            config.Bars.Clear();
            foreach (var row in barRows)
            {
                var bar = new GeneralKinematicBar
                {
                    IsMetering = row.IsMetering
                };
                bar.PinIds.Add(row.GetPinIds());
                config.Bars.Add(bar);
            }
            config.RailTravelNegative = (float)railTravelNegative;
            config.RailTravelPositive = (float)railTravelPositive;
        }

        private void RebuildCache()
        {
            if (config == null) return;
            try
            {
                RebuildConfigFromRows();
                poseCache = GeneralKinematics.BuildPoseCache(config);
                BuildCanvas();
                var parameters = GeneralKinematics.CalcKinematicParameters(config);
                KinematicParametersChanged?.Invoke(parameters);
                if (poseCache.ContactPositions.Length > 0)
                {
                    Label_status.Content = string.Format(CultureInfo.CurrentCulture,
                        "Contact range: {0:0.#} .. {1:0.#} mm",
                        poseCache.ContactPositions.First(),
                        poseCache.ContactPositions.Last());
                }
                else
                {
                    Label_status.Content = "";
                }
                RefreshPose();
            }
            catch (Exception ex)
            {
                poseCache = null;
                Label_status.Content = ex.Message;
                BuildCanvas();
            }
        }

        private void RefreshPose()
        {
            if (liveMode)
            {
                UpdatePose();
            }
            else
            {
                UpdateStaticPose();
            }
        }

        private void BuildCanvas()
        {
            canvas_kinematic.Children.Clear();
            pinShapes.Clear();
            pinLabels.Clear();
            barVisuals.Clear();
            pinIndexById.Clear();

            DrawGridLines();
            UpdateScaleLabel();

            if (config == null) return;

            BuildRailLine();
            BuildBars();
            BuildPins();

            if (poseCache == null)
            {
                UpdateStaticPose();
            }
        }

        private void BuildRailLine()
        {
            var railPin = pinRows.FirstOrDefault(p => p.IsRailInterface);
            if (railPin == null) return;

            var start = ToCanvas(railPin.X - railTravelNegative, railPin.Y);
            var end = ToCanvas(railPin.X + railTravelPositive, railPin.Y);
            var line = new Line
            {
                X1 = start.X,
                Y1 = start.Y,
                X2 = end.X,
                Y2 = end.Y,
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 4, 4 },
                Stroke = Brushes.LightSteelBlue,
                Opacity = 0.8
            };
            canvas_kinematic.Children.Add(line);
        }

        private void BuildBars()
        {
            foreach (var row in barRows)
            {
                var pinIds = row.GetPinIds();
                var line = new Line
                {
                    StrokeThickness = row.IsMetering ? 3 : 2,
                    Stroke = row.IsMetering ? Brushes.Gold : Brushes.White,
                    Opacity = 0.9,
                    Visibility = pinIds.Length < 2 ? Visibility.Hidden : Visibility.Visible
                };
                barVisuals.Add(new BarVisual { PinIds = pinIds, Line = line });
                canvas_kinematic.Children.Add(line);
            }
        }

        private void BuildPins()
        {
            foreach (var row in pinRows)
            {
                if (pinShapes.ContainsKey(row.PinId)) continue;
                var ellipse = new Ellipse
                {
                    Width = 8,
                    Height = 8,
                    Fill = GetPinBrush(row),
                    Stroke = Brushes.White,
                    StrokeThickness = 1
                };
                var label = new TextBlock
                {
                    Text = row.PinId.ToString(CultureInfo.CurrentCulture),
                    FontFamily = new FontFamily("Arial"),
                    FontSize = 9,
                    Foreground = Brushes.White
                };
                canvas_kinematic.Children.Add(ellipse);
                canvas_kinematic.Children.Add(label);
                pinShapes[row.PinId] = ellipse;
                pinLabels[row.PinId] = label;
            }
        }

        private void UpdateStaticPose()
        {
            foreach (var row in pinRows)
            {
                var point = ToCanvas(row.X, row.Y);
                if (pinShapes.TryGetValue(row.PinId, out var ellipse))
                {
                    Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2.0);
                    Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2.0);
                }
                if (pinLabels.TryGetValue(row.PinId, out var label))
                {
                    Canvas.SetLeft(label, point.X + 4);
                    Canvas.SetTop(label, point.Y + 4);
                }
            }

            foreach (var bar in barVisuals)
            {
                if (bar.PinIds.Length < 2) continue;
                if (!TryGetBarExtentsFromConfig(bar.PinIds, out double ax, out double ay, out double bx, out double by))
                {
                    bar.Line.Visibility = Visibility.Hidden;
                    continue;
                }
                var pa = ToCanvas(ax, ay);
                var pb = ToCanvas(bx, by);
                bar.Line.Visibility = Visibility.Visible;
                bar.Line.X1 = pa.X;
                bar.Line.Y1 = pa.Y;
                bar.Line.X2 = pb.X;
                bar.Line.Y2 = pb.Y;
            }
        }

        private bool TryGetPointFromConfig(uint pinId, out PinRow row)
        {
            row = pinRows.FirstOrDefault(p => p.PinId == pinId);
            return row != null;
        }

        private bool TryGetBarExtentsFromConfig(uint[] pinIds, out double ax, out double ay, out double bx, out double by)
        {
            ax = ay = bx = by = 0.0;
            double bestDist = -1.0;
            bool found = false;
            for (int i = 0; i < pinIds.Length; i++)
            {
                if (!TryGetPointFromConfig(pinIds[i], out var a)) continue;
                for (int j = i + 1; j < pinIds.Length; j++)
                {
                    if (!TryGetPointFromConfig(pinIds[j], out var b)) continue;
                    double dx = a.X - b.X;
                    double dy = a.Y - b.Y;
                    double dist = dx * dx + dy * dy;
                    if (dist > bestDist)
                    {
                        bestDist = dist;
                        ax = a.X;
                        ay = a.Y;
                        bx = b.X;
                        by = b.Y;
                        found = true;
                    }
                }
            }
            return found;
        }

        private Brush GetPinBrush(PinRow row)
        {
            if (row.IsContactPoint) return Brushes.OrangeRed;
            if (row.IsRailInterface) return Brushes.DeepSkyBlue;
            if (row.Grounded) return Brushes.Gray;
            return Brushes.White;
        }

        private void UpdatePose()
        {
            if (poseCache == null || poseCache.PinPositionsX.Length == 0) return;
            if (!TryGetSegment(lastAxisPosition, out int idx, out double t)) return;

            int lastIndex = poseCache.PinPositionsX.Length - 1;
            int next = Math.Min(idx + 1, lastIndex);
            int pinCount = poseCache.PinIds.Length;
            if (currentX == null || currentX.Length != pinCount)
            {
                currentX = new double[pinCount];
                currentY = new double[pinCount];
            }
            pinIndexById.Clear();
            for (int i = 0; i < pinCount; i++)
            {
                pinIndexById[poseCache.PinIds[i]] = i;
                currentX[i] = poseCache.PinPositionsX[idx][i] + t * (poseCache.PinPositionsX[next][i] - poseCache.PinPositionsX[idx][i]);
                currentY[i] = poseCache.PinPositionsY[idx][i] + t * (poseCache.PinPositionsY[next][i] - poseCache.PinPositionsY[idx][i]);
            }

            for (int i = 0; i < pinCount; i++)
            {
                var pinId = poseCache.PinIds[i];
                var point = ToCanvas(currentX[i], currentY[i]);
                if (pinShapes.TryGetValue(pinId, out var ellipse))
                {
                    Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2.0);
                    Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2.0);
                }
                if (pinLabels.TryGetValue(pinId, out var label))
                {
                    Canvas.SetLeft(label, point.X + 4);
                    Canvas.SetTop(label, point.Y + 4);
                }
            }

            foreach (var bar in barVisuals)
            {
                if (bar.PinIds.Length < 2)
                {
                    bar.Line.Visibility = Visibility.Hidden;
                    continue;
                }
                if (!TryGetBarExtents(bar.PinIds, out double ax, out double ay, out double bx, out double by))
                {
                    bar.Line.Visibility = Visibility.Hidden;
                    continue;
                }
                var p1 = ToCanvas(ax, ay);
                var p2 = ToCanvas(bx, by);
                bar.Line.Visibility = Visibility.Visible;
                bar.Line.X1 = p1.X;
                bar.Line.Y1 = p1.Y;
                bar.Line.X2 = p2.X;
                bar.Line.Y2 = p2.Y;
            }
        }

        private bool TryGetBarExtents(uint[] pinIds, out double ax, out double ay, out double bx, out double by)
        {
            ax = ay = bx = by = 0.0;
            double bestDist = -1.0;
            bool found = false;
            for (int i = 0; i < pinIds.Length; i++)
            {
                if (!pinIndexById.TryGetValue(pinIds[i], out int idxA)) continue;
                for (int j = i + 1; j < pinIds.Length; j++)
                {
                    if (!pinIndexById.TryGetValue(pinIds[j], out int idxB)) continue;
                    double dx = currentX[idxA] - currentX[idxB];
                    double dy = currentY[idxA] - currentY[idxB];
                    double dist = dx * dx + dy * dy;
                    if (dist > bestDist)
                    {
                        bestDist = dist;
                        ax = currentX[idxA];
                        ay = currentY[idxA];
                        bx = currentX[idxB];
                        by = currentY[idxB];
                        found = true;
                    }
                }
            }
            return found;
        }

        private bool TryGetSegment(double contactPos, out int idx, out double t)
        {
            idx = 0;
            t = 0.0;
            if (poseCache == null || poseCache.ContactPositions.Length < 2) return false;

            double bestDist = double.PositiveInfinity;
            int bestIdx = 0;
            double bestT = 0.0;
            for (int i = 0; i < poseCache.ContactPositions.Length - 1; i++)
            {
                double a = poseCache.ContactPositions[i];
                double b = poseCache.ContactPositions[i + 1];
                double delta = b - a;
                if (Math.Abs(delta) < 1e-9)
                {
                    continue;
                }
                double localT = (contactPos - a) / delta;
                double clampedT = Math.Max(0.0, Math.Min(1.0, localT));
                double proj = a + delta * clampedT;
                double dist = Math.Abs(contactPos - proj);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx = i;
                    bestT = clampedT;
                }
            }

            idx = bestIdx;
            t = bestT;
            return true;
        }

        private void DrawGridLines()
        {
            if (plugin == null) return;
            double scale = plugin.Settings.kinematicDiagram_zeroPos_scale;
            if (scale <= 0.0) scale = 1.0;
            double spacing = 50.0 / scale;
            double ox = plugin.Settings.kinematicDiagram_zeroPos_OX % spacing;
            double oy = plugin.Settings.kinematicDiagram_zeroPos_OY % spacing;

            int rowCount = (int)Math.Floor(canvas_kinematic.Height / spacing);
            int colCount = (int)Math.Floor(canvas_kinematic.Width / spacing);

            for (int i = 0; i <= rowCount; i++)
            {
                var line = new Line
                {
                    X1 = 0,
                    Y1 = canvas_kinematic.Height - (oy + i * spacing),
                    X2 = canvas_kinematic.Width,
                    Y2 = canvas_kinematic.Height - (oy + i * spacing),
                    Stroke = Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                canvas_kinematic.Children.Add(line);
            }

            for (int i = 0; i <= colCount; i++)
            {
                var line = new Line
                {
                    X1 = ox + i * spacing,
                    Y1 = 0,
                    X2 = ox + i * spacing,
                    Y2 = canvas_kinematic.Height,
                    Stroke = Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                canvas_kinematic.Children.Add(line);
            }
        }

        private void UpdateScaleLabel()
        {
            if (plugin == null) return;
            Label_kinematic_scale.Content = Math.Round(plugin.Settings.kinematicDiagram_zeroPos_scale, 1).ToString(CultureInfo.CurrentCulture);
        }

        private Point ToCanvas(double x, double y)
        {
            double scale = plugin?.Settings?.kinematicDiagram_zeroPos_scale ?? 1.0;
            if (scale <= 0.0) scale = 1.0;
            double ox = plugin?.Settings?.kinematicDiagram_zeroPos_OX ?? 0.0;
            double oy = plugin?.Settings?.kinematicDiagram_zeroPos_OY ?? 0.0;
            return new Point(x / scale + ox, canvas_kinematic.Height - y / scale - oy);
        }

        private void btn_plus_kinematic_scale_Click(object sender, RoutedEventArgs e)
        {
            if (plugin == null) return;
            if (plugin.Settings.kinematicDiagram_zeroPos_scale < 2.0)
            {
                plugin.Settings.kinematicDiagram_zeroPos_scale += 0.1;
                BuildCanvas();
                RefreshPose();
            }
        }

        private void btn_minus_kinematic_scale_Click(object sender, RoutedEventArgs e)
        {
            if (plugin == null) return;
            if (plugin.Settings.kinematicDiagram_zeroPos_scale > 0.7)
            {
                plugin.Settings.kinematicDiagram_zeroPos_scale -= 0.1;
                BuildCanvas();
                RefreshPose();
            }
        }

        private void btn_add_pin_Click(object sender, RoutedEventArgs e)
        {
            uint nextId = 1;
            if (pinRows.Count > 0)
            {
                nextId = pinRows.Max(p => p.PinId) + 1;
            }
            pinRows.Add(new PinRow { PinId = nextId });
        }

        private void btn_remove_pin_Click(object sender, RoutedEventArgs e)
        {
            if (PinGrid.SelectedItem is PinRow row)
            {
                pinRows.Remove(row);
            }
        }

        private void btn_add_bar_Click(object sender, RoutedEventArgs e)
        {
            barRows.Add(new BarRow());
        }

        private void btn_remove_bar_Click(object sender, RoutedEventArgs e)
        {
            if (BarGrid.SelectedItem is BarRow row)
            {
                barRows.Remove(row);
            }
        }

        private void btn_rebuild_Click(object sender, RoutedEventArgs e)
        {
            rebuildTimer.Stop();
            RebuildCache();
        }

        private void PoseModeCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PoseModeCombo.SelectedIndex == 0)
            {
                liveMode = true;
            }
            else
            {
                liveMode = false;
            }
            RefreshPose();
        }

        private void RailTravel_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (isLoading) return;
            if (TryParseNonNegative(TextRailNegative.Text, out double negative))
            {
                railTravelNegative = negative;
            }
            if (TryParseNonNegative(TextRailPositive.Text, out double positive))
            {
                railTravelPositive = positive;
            }
            QueueRebuild();
        }

        private void RailTravel_LostFocus(object sender, RoutedEventArgs e)
        {
            TextRailNegative.Text = railTravelNegative.ToString("0.###", CultureInfo.CurrentCulture);
            TextRailPositive.Text = railTravelPositive.ToString("0.###", CultureInfo.CurrentCulture);
        }

        private bool TryParseNonNegative(string text, out double value)
        {
            value = 0.0;
            if (!double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out double parsed))
            {
                return false;
            }
            if (parsed < 0.0)
            {
                parsed = 0.0;
            }
            value = parsed;
            return true;
        }

        private void OnPickPinsClicked(object sender, RoutedEventArgs e)
        {
            if (!(sender is Button button)) return;
            if (!(button.DataContext is BarRow row)) return;

            var menu = new ContextMenu();
            var selected = new HashSet<uint>(row.GetPinIds());
            if (pinRows.Count == 0)
            {
                menu.Items.Add(new MenuItem { Header = "No pins", IsEnabled = false });
            }
            else
            {
                foreach (var pin in pinRows.OrderBy(p => p.PinId))
                {
                    var item = new MenuItem
                    {
                        Header = pin.PinId.ToString(CultureInfo.CurrentCulture),
                        IsCheckable = true,
                        IsChecked = selected.Contains(pin.PinId)
                    };
                    item.Checked += (s, _) => row.AddPin(pin.PinId);
                    item.Unchecked += (s, _) => row.RemovePin(pin.PinId);
                    menu.Items.Add(item);
                }
            }

            menu.PlacementTarget = button;
            menu.IsOpen = true;
        }
    }
}
