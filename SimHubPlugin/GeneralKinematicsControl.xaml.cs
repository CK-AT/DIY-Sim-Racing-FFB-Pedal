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
            private Brush barBrush = Brushes.White;

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

            public Brush BarBrush
            {
                get => barBrush;
                set
                {
                    if (Equals(barBrush, value)) return;
                    barBrush = value;
                    OnPropertyChanged(nameof(BarBrush));
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
            public Polyline Outline;
            public Brush Brush;
        }

        private sealed class BarPinRing
        {
            public uint PinId;
            public Ellipse Ring;
        }

        private GeneralKinematicConfig config;
        private GeneralKinematics.PoseCache poseCache;
        private DiyFfbPlugin plugin;
        private readonly ObservableCollection<PinRow> pinRows = new ObservableCollection<PinRow>();
        private readonly ObservableCollection<BarRow> barRows = new ObservableCollection<BarRow>();
        private readonly Dictionary<uint, Ellipse> pinShapes = new Dictionary<uint, Ellipse>();
        private readonly Dictionary<uint, TextBlock> pinLabels = new Dictionary<uint, TextBlock>();
        private readonly List<BarPinRing> barPinRings = new List<BarPinRing>();
        private readonly List<BarVisual> barVisuals = new List<BarVisual>();
        private readonly Dictionary<uint, int> pinIndexById = new Dictionary<uint, int>();
        private readonly DispatcherTimer rebuildTimer;
        private bool isAutoFitting;
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
            canvas_kinematic.SizeChanged += CanvasKinematic_SizeChanged;
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
            UpdateBarBrushes();
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
            UpdateBarBrushes();
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
                AutoFitCanvasToPoses(false);
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
                AutoFitCanvasToPoses(false);
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
            barPinRings.Clear();
            barVisuals.Clear();
            pinIndexById.Clear();

            DrawGridLines();
            UpdateScaleLabel();

            if (config == null) return;

            BuildRailLine();
            BuildBars();
            BuildPins();
            BuildBarPinRings();

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
            int barIndex = 0;
            foreach (var row in barRows)
            {
                var pinIds = row.GetPinIds();
                var brush = row.BarBrush ?? GetBarBrush(barIndex++);
                var outline = new Polyline
                {
                    StrokeThickness = row.IsMetering ? 3 : 2,
                    Stroke = brush,
                    Opacity = 0.9,
                    Visibility = pinIds.Length < 2 ? Visibility.Hidden : Visibility.Visible
                };
                outline.StrokeLineJoin = PenLineJoin.Round;
                outline.StrokeStartLineCap = PenLineCap.Round;
                outline.StrokeEndLineCap = PenLineCap.Round;
                Panel.SetZIndex(outline, 0);
                barVisuals.Add(new BarVisual
                {
                    PinIds = pinIds,
                    Outline = outline,
                    Brush = brush
                });
                canvas_kinematic.Children.Add(outline);
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
                Panel.SetZIndex(ellipse, 2);
                Panel.SetZIndex(label, 3);
                canvas_kinematic.Children.Add(ellipse);
                canvas_kinematic.Children.Add(label);
                pinShapes[row.PinId] = ellipse;
                pinLabels[row.PinId] = label;
            }
        }

        private void BuildBarPinRings()
        {
            const double baseSize = 12.0;
            const double ringStep = 3.0;
            var ringIndexByPin = new Dictionary<uint, int>();

            foreach (var bar in barVisuals)
            {
                if (bar.PinIds == null || bar.PinIds.Length == 0) continue;
                foreach (var pinId in bar.PinIds)
                {
                    int ringIndex = 0;
                    if (ringIndexByPin.TryGetValue(pinId, out int current))
                    {
                        ringIndex = current;
                    }
                    ringIndexByPin[pinId] = ringIndex + 1;

                    double size = baseSize + ringStep * 2.0 * ringIndex;
                    var ellipse = new Ellipse
                    {
                        Width = size,
                        Height = size,
                        Stroke = bar.Brush,
                        StrokeThickness = 2,
                        Fill = Brushes.Transparent
                    };
                    Panel.SetZIndex(ellipse, 1);
                    canvas_kinematic.Children.Add(ellipse);
                    barPinRings.Add(new BarPinRing { PinId = pinId, Ring = ellipse });
                }
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

            UpdateBarPinRingsFromConfig();

            foreach (var bar in barVisuals)
            {
                if (bar.PinIds.Length < 2) continue;
                if (!TryGetBarPointsFromConfig(bar.PinIds, out PointCollection points))
                {
                    bar.Outline.Visibility = Visibility.Hidden;
                    continue;
                }
                bar.Outline.Visibility = Visibility.Visible;
                bar.Outline.Points = points;
            }
        }

        private bool TryGetPointFromConfig(uint pinId, out PinRow row)
        {
            row = pinRows.FirstOrDefault(p => p.PinId == pinId);
            return row != null;
        }

        private bool TryGetBarPointsFromConfig(uint[] pinIds, out PointCollection points)
        {
            points = new PointCollection();
            for (int i = 0; i < pinIds.Length; i++)
            {
                if (!TryGetPointFromConfig(pinIds[i], out var row))
                {
                    points = null;
                    return false;
                }
                points.Add(ToCanvas(row.X, row.Y));
            }
            if (points.Count < 2)
            {
                points = null;
                return false;
            }
            if (points.Count > 2)
            {
                points.Add(points[0]);
            }
            return true;
        }

        private Brush GetPinBrush(PinRow row)
        {
            if (row.IsContactPoint) return Brushes.OrangeRed;
            if (row.IsRailInterface) return Brushes.DeepSkyBlue;
            if (row.Grounded) return Brushes.Gray;
            return Brushes.White;
        }

        private Brush GetBarBrush(int index)
        {
            Brush[] palette =
            {
                Brushes.DeepSkyBlue,
                Brushes.MediumSeaGreen,
                Brushes.Goldenrod,
                Brushes.OrangeRed,
                Brushes.MediumTurquoise,
                Brushes.Coral
            };
            return palette[index % palette.Length];
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

            UpdateBarPinRingsFromLive();

            foreach (var bar in barVisuals)
            {
                if (bar.PinIds.Length < 2)
                {
                    bar.Outline.Visibility = Visibility.Hidden;
                    continue;
                }
                if (!TryGetBarPoints(bar.PinIds, out PointCollection points))
                {
                    bar.Outline.Visibility = Visibility.Hidden;
                    continue;
                }
                bar.Outline.Visibility = Visibility.Visible;
                bar.Outline.Points = points;
            }
        }

        private bool TryGetBarPoints(uint[] pinIds, out PointCollection points)
        {
            points = new PointCollection();
            for (int i = 0; i < pinIds.Length; i++)
            {
                if (!pinIndexById.TryGetValue(pinIds[i], out int idx))
                {
                    points = null;
                    return false;
                }
                points.Add(ToCanvas(currentX[idx], currentY[idx]));
            }
            if (points.Count < 2)
            {
                points = null;
                return false;
            }
            if (points.Count > 2)
            {
                points.Add(points[0]);
            }
            return true;
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

            var origin = ToCanvas(0.0, 0.0);
            var xAxis = new Line
            {
                X1 = 0,
                Y1 = origin.Y,
                X2 = canvas_kinematic.Width,
                Y2 = origin.Y,
                Stroke = Brushes.LightSteelBlue,
                StrokeThickness = 2,
                Opacity = 0.35
            };
            var yAxis = new Line
            {
                X1 = origin.X,
                Y1 = 0,
                X2 = origin.X,
                Y2 = canvas_kinematic.Height,
                Stroke = Brushes.LightSteelBlue,
                StrokeThickness = 2,
                Opacity = 0.35
            };
            canvas_kinematic.Children.Add(xAxis);
            canvas_kinematic.Children.Add(yAxis);
        }

        private void CanvasKinematic_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            AutoFitCanvasToPoses(true);
        }

        private void AutoFitCanvasToPoses(bool refresh)
        {
            if (plugin == null || isAutoFitting) return;
            if (!TryGetPoseBounds(out double minX, out double maxX, out double minY, out double maxY)) return;

            double width = canvas_kinematic.ActualWidth;
            double height = canvas_kinematic.ActualHeight;
            if (width <= 1.0) width = canvas_kinematic.Width;
            if (height <= 1.0) height = canvas_kinematic.Height;
            if (width <= 1.0 || height <= 1.0) return;

            const double padding = 20.0;
            double usableWidth = Math.Max(1.0, width - 2.0 * padding);
            double usableHeight = Math.Max(1.0, height - 2.0 * padding);
            double rangeX = Math.Max(1e-6, maxX - minX);
            double rangeY = Math.Max(1e-6, maxY - minY);
            double scale = Math.Max(rangeX / usableWidth, rangeY / usableHeight);
            if (double.IsNaN(scale) || double.IsInfinity(scale) || scale <= 0.0) scale = 1.0;

            double rangeXCanvas = rangeX / scale;
            double rangeYCanvas = rangeY / scale;
            double extraX = Math.Max(0.0, usableWidth - rangeXCanvas);
            double extraY = Math.Max(0.0, usableHeight - rangeYCanvas);

            double ox = padding + extraX / 2.0 - minX / scale;
            double top = padding + extraY / 2.0;
            double oy = height - maxY / scale - top;

            isAutoFitting = true;
            try
            {
                plugin.Settings.kinematicDiagram_zeroPos_scale = scale;
                plugin.Settings.kinematicDiagram_zeroPos_OX = ox;
                plugin.Settings.kinematicDiagram_zeroPos_OY = oy;
                if (refresh)
                {
                    BuildCanvas();
                    RefreshPose();
                }
            }
            finally
            {
                isAutoFitting = false;
            }
        }

        private bool TryGetPoseBounds(out double minX, out double maxX, out double minY, out double maxY)
        {
            minX = double.PositiveInfinity;
            maxX = double.NegativeInfinity;
            minY = double.PositiveInfinity;
            maxY = double.NegativeInfinity;

            bool found = false;
            if (poseCache != null && poseCache.PinPositionsX.Length > 0)
            {
                int poseCount = poseCache.PinPositionsX.Length;
                int pinCount = poseCache.PinIds.Length;
                for (int i = 0; i < poseCount; i++)
                {
                    var xs = poseCache.PinPositionsX[i];
                    var ys = poseCache.PinPositionsY[i];
                    for (int j = 0; j < pinCount; j++)
                    {
                        double x = xs[j];
                        double y = ys[j];
                        if (double.IsNaN(x) || double.IsInfinity(x) || double.IsNaN(y) || double.IsInfinity(y)) continue;
                        minX = Math.Min(minX, x);
                        maxX = Math.Max(maxX, x);
                        minY = Math.Min(minY, y);
                        maxY = Math.Max(maxY, y);
                        found = true;
                    }
                }
            }
            else if (config != null && config.Pins.Count > 0)
            {
                foreach (var pin in config.Pins)
                {
                    double x = pin.X;
                    double y = pin.Y;
                    minX = Math.Min(minX, x);
                    maxX = Math.Max(maxX, x);
                    minY = Math.Min(minY, y);
                    maxY = Math.Max(maxY, y);
                    found = true;
                }

                var railPin = config.Pins.FirstOrDefault(p => p.IsRailInterface);
                if (railPin != null)
                {
                    minX = Math.Min(minX, railPin.X - railTravelNegative);
                    maxX = Math.Max(maxX, railPin.X + railTravelPositive);
                    minY = Math.Min(minY, railPin.Y);
                    maxY = Math.Max(maxY, railPin.Y);
                    found = true;
                }
            }

            return found;
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

        private void UpdateBarPinRingsFromConfig()
        {
            foreach (var entry in barPinRings)
            {
                if (!TryGetPointFromConfig(entry.PinId, out var row)) continue;
                var point = ToCanvas(row.X, row.Y);
                Canvas.SetLeft(entry.Ring, point.X - entry.Ring.Width / 2.0);
                Canvas.SetTop(entry.Ring, point.Y - entry.Ring.Height / 2.0);
            }
        }

        private void UpdateBarPinRingsFromLive()
        {
            foreach (var entry in barPinRings)
            {
                if (!pinIndexById.TryGetValue(entry.PinId, out int idx)) continue;
                var point = ToCanvas(currentX[idx], currentY[idx]);
                Canvas.SetLeft(entry.Ring, point.X - entry.Ring.Width / 2.0);
                Canvas.SetTop(entry.Ring, point.Y - entry.Ring.Height / 2.0);
            }
        }

        private void UpdateBarBrushes()
        {
            bool wasLoading = isLoading;
            isLoading = true;
            for (int i = 0; i < barRows.Count; i++)
            {
                barRows[i].BarBrush = GetBarBrush(i);
            }
            isLoading = wasLoading;
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
