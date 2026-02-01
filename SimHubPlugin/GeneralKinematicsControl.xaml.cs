using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace DiyFfb
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
                    double rounded = RoundPinCoordinate(value);
                    if (Math.Abs(x - rounded) < 1e-9) return;
                    x = rounded;
                    OnPropertyChanged(nameof(X));
                }
            }

            public double Y
            {
                get => y;
                set
                {
                    double rounded = RoundPinCoordinate(value);
                    if (Math.Abs(y - rounded) < 1e-9) return;
                    y = rounded;
                    OnPropertyChanged(nameof(Y));
                }
            }

            private static double RoundPinCoordinate(double value)
            {
                return Math.Round(value, 1, MidpointRounding.AwayFromZero);
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
            public Line SegmentA;
            public Line SegmentB;
            public Brush Brush;
            public bool IsMetering;
        }

        private sealed class BarPinRing
        {
            public uint PinId;
            public Ellipse Ring;
        }

        private enum PoseDisplayMode
        {
            Live,
            Test
        }

        private sealed class ContactForceVisual
        {
            public Line Shaft;
            public Polygon Head;
            public TextBlock Label;
        }

        private sealed class BarSensorVisual
        {
            public uint[] PinIds;
            public FrameworkElement Icon;
            public TextBlock Label;
        }

        private GeneralKinematicConfig config;
        private GeneralKinematics.PoseCache poseCache;
        private DiyFfbPlugin plugin;
        private readonly ObservableCollection<PinRow> pinRows = new ObservableCollection<PinRow>();
        private readonly ObservableCollection<BarRow> barRows = new ObservableCollection<BarRow>();
        private readonly Dictionary<uint, Ellipse> pinShapes = new Dictionary<uint, Ellipse>();
        private readonly Dictionary<uint, TextBlock> pinLabels = new Dictionary<uint, TextBlock>();
        private readonly Dictionary<uint, FrameworkElement> groundedMarkers = new Dictionary<uint, FrameworkElement>();
        private readonly List<BarPinRing> barPinRings = new List<BarPinRing>();
        private readonly List<BarVisual> barVisuals = new List<BarVisual>();
        private readonly List<BarSensorVisual> barSensorVisuals = new List<BarSensorVisual>();
        private readonly Dictionary<uint, int> pinIndexById = new Dictionary<uint, int>();
        private readonly DispatcherTimer rebuildTimer;
        private ContactForceVisual contactForceVisual;
        private KinematicParameters currentParameters;
        private bool isAutoFitting;
        private bool isLoading;
        private PoseDisplayMode poseMode = PoseDisplayMode.Live;
        private double lastAxisPosition;
        private double lastAxisForce;
        private double testAxisPosition;
        private double testAxisForce;
        private bool hasAxisState;
        private double railTravelNegative;
        private double railTravelPositive;
        private double[] currentX;
        private double[] currentY;
        private bool isPanning;
        private Point panStart;
        private double panStartOx;
        private double panStartOy;
        private const double MinZoomScale = 0.2;
        private const double MaxZoomScale = 5.0;
        private const double ZoomStep = 1.1;
        private const double ContactArrowLength = 26.0;
        private const double ContactArrowHeadLength = 8.0;
        private const double ContactArrowHeadWidth = 8.0;
        private const double MeteringIconWidth = 20.0;
        private const double MeteringIconHeight = 12.0;
        private const double MeteringIconWorldWidth = 35.0;
        private const double MeteringIconWorldHeight = 25.0;
        private const double TestPosePositionStep = 1.0;
        private const double TestPosePositionFineStep = 0.1;
        private const double TestPoseForceStep = 1.0;
        private const double PinPositionStep = 1.0;
        private const double PinPositionFineStep = 0.1;
        private const double RailTravelStep = 1.0;
        private const double RailTravelFineStep = 0.1;
        private const double GroundedMarkerWidth = 28.0;
        private const double GroundedMarkerHeight = 24.0;
        private const double GroundedMarkerOffsetY = 0.0;

        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;

        public GeneralKinematicsControl()
        {
            isLoading = true;
            InitializeComponent();
            testAxisPosition = 0.0;
            testAxisForce = 0.0;
            TextTestPosition.Text = testAxisPosition.ToString("0.###", CultureInfo.CurrentCulture);
            TextTestForce.Text = testAxisForce.ToString("0.0", CultureInfo.CurrentCulture);
            TestPosePanel.Visibility = Visibility.Collapsed;
            isLoading = false;
            PinGrid.ItemsSource = pinRows;
            BarGrid.ItemsSource = barRows;
            pinRows.CollectionChanged += PinRows_CollectionChanged;
            barRows.CollectionChanged += BarRows_CollectionChanged;
            rebuildTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(200) };
            rebuildTimer.Tick += RebuildTimer_Tick;
            canvas_kinematic.SizeChanged += CanvasKinematic_SizeChanged;
            canvas_kinematic.MouseWheel += CanvasKinematic_MouseWheel;
            canvas_kinematic.MouseLeftButtonDown += CanvasKinematic_MouseLeftButtonDown;
            canvas_kinematic.MouseLeftButtonUp += CanvasKinematic_MouseLeftButtonUp;
            canvas_kinematic.MouseMove += CanvasKinematic_MouseMove;
            canvas_kinematic.MouseLeave += CanvasKinematic_MouseLeave;
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
            railTravelNegative = NormalizeNonNegativeMillimeters(config.RailTravelNegative);
            railTravelPositive = NormalizeNonNegativeMillimeters(config.RailTravelPositive);
            TextRailNegative.Text = railTravelNegative.ToString("0.#", CultureInfo.CurrentCulture);
            TextRailPositive.Text = railTravelPositive.ToString("0.#", CultureInfo.CurrentCulture);
            isLoading = false;
            QueueRebuild();
        }

        public void OnAxisStateUpdate(AxisState axisState)
        {
            lastAxisPosition = axisState.Position;
            lastAxisForce = axisState.Force;
            hasAxisState = true;
            if (poseMode == PoseDisplayMode.Live)
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
                SetTestAxisPosition(testAxisPosition, poseMode == PoseDisplayMode.Test);
                AutoFitCanvasToPoses(false);
                BuildCanvas();
                var parameters = GeneralKinematics.CalcKinematicParameters(config);
                currentParameters = parameters;
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
            switch (poseMode)
            {
                case PoseDisplayMode.Live:
                    UpdatePose();
                    break;
                case PoseDisplayMode.Test:
                    UpdateTestPose();
                    break;
                default:
                    UpdateStaticPose();
                    break;
            }
        }

        private void BuildCanvas()
        {
            canvas_kinematic.Children.Clear();
            pinShapes.Clear();
            pinLabels.Clear();
            groundedMarkers.Clear();
            barPinRings.Clear();
            barVisuals.Clear();
            barSensorVisuals.Clear();
            pinIndexById.Clear();
            contactForceVisual = null;

            DrawGridLines();

            if (config == null) return;

            BuildRailLine();
            BuildBars();
            BuildPins();
            BuildBarPinRings();
            BuildContactForceArrow();
            BuildMeteringSensorIcons();

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
            var baseLine = new Line
            {
                X1 = start.X,
                Y1 = start.Y,
                X2 = end.X,
                Y2 = end.Y,
                StrokeThickness = 6,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = new SolidColorBrush(Color.FromArgb(80, 120, 170, 210))
            };
            var guideLine = new Line
            {
                X1 = start.X,
                Y1 = start.Y,
                X2 = end.X,
                Y2 = end.Y,
                StrokeThickness = 2,
                StrokeDashArray = new DoubleCollection { 6, 4 },
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = Brushes.DeepSkyBlue,
                Opacity = 0.9
            };

            const double capLength = 16.0;
            var capStroke = Brushes.DeepSkyBlue;
            var capHaloStroke = new SolidColorBrush(Color.FromArgb(70, 120, 190, 240));
            const double capHaloThickness = 6.0;
            var startHalo = new Line
            {
                X1 = start.X,
                Y1 = start.Y - capLength / 2.0,
                X2 = start.X,
                Y2 = start.Y + capLength / 2.0,
                StrokeThickness = capHaloThickness,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = capHaloStroke
            };
            var endHalo = new Line
            {
                X1 = end.X,
                Y1 = end.Y - capLength / 2.0,
                X2 = end.X,
                Y2 = end.Y + capLength / 2.0,
                StrokeThickness = capHaloThickness,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = capHaloStroke
            };
            var startCap = new Line
            {
                X1 = start.X,
                Y1 = start.Y - capLength / 2.0,
                X2 = start.X,
                Y2 = start.Y + capLength / 2.0,
                StrokeThickness = 2.0,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = capStroke,
                Opacity = 0.95
            };
            var endCap = new Line
            {
                X1 = end.X,
                Y1 = end.Y - capLength / 2.0,
                X2 = end.X,
                Y2 = end.Y + capLength / 2.0,
                StrokeThickness = 2.0,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Stroke = capStroke,
                Opacity = 0.95
            };

            canvas_kinematic.Children.Add(baseLine);
            canvas_kinematic.Children.Add(guideLine);
            canvas_kinematic.Children.Add(startHalo);
            canvas_kinematic.Children.Add(endHalo);
            canvas_kinematic.Children.Add(startCap);
            canvas_kinematic.Children.Add(endCap);
        }

        private void BuildBars()
        {
            int barIndex = 0;
            foreach (var row in barRows)
            {
                var pinIds = row.GetPinIds();
                var brush = row.BarBrush ?? GetBarBrush(barIndex++);
                if (row.IsMetering)
                {
                    var segmentA = new Line
                    {
                        StrokeThickness = 3,
                        Stroke = brush,
                        Opacity = 0.9,
                        Visibility = pinIds.Length < 2 ? Visibility.Hidden : Visibility.Visible,
                        StrokeStartLineCap = PenLineCap.Round,
                        StrokeEndLineCap = PenLineCap.Round
                    };
                    var segmentB = new Line
                    {
                        StrokeThickness = 3,
                        Stroke = brush,
                        Opacity = 0.9,
                        Visibility = pinIds.Length < 2 ? Visibility.Hidden : Visibility.Visible,
                        StrokeStartLineCap = PenLineCap.Round,
                        StrokeEndLineCap = PenLineCap.Round
                    };
                    Panel.SetZIndex(segmentA, 0);
                    Panel.SetZIndex(segmentB, 0);
                    barVisuals.Add(new BarVisual
                    {
                        PinIds = pinIds,
                        SegmentA = segmentA,
                        SegmentB = segmentB,
                        Brush = brush,
                        IsMetering = true
                    });
                    canvas_kinematic.Children.Add(segmentA);
                    canvas_kinematic.Children.Add(segmentB);
                }
                else
                {
                    var outline = new Polyline
                    {
                        StrokeThickness = 2,
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
                        Brush = brush,
                        IsMetering = false
                    });
                    canvas_kinematic.Children.Add(outline);
                }
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
                if (row.Grounded)
                {
                    var marker = CreateGroundedMarker();
                    Panel.SetZIndex(marker, 1);
                    canvas_kinematic.Children.Add(marker);
                    groundedMarkers[row.PinId] = marker;
                }
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

        private void BuildContactForceArrow()
        {
            if (!pinRows.Any(p => p.IsContactPoint)) return;

            var shaft = new Line
            {
                Stroke = Brushes.OrangeRed,
                StrokeThickness = 2,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round,
                Opacity = 0.9,
                IsHitTestVisible = false
            };
            var head = new Polygon
            {
                Fill = Brushes.OrangeRed,
                Stroke = Brushes.OrangeRed,
                StrokeThickness = 1,
                Opacity = 0.9,
                IsHitTestVisible = false
            };
            var label = new TextBlock
            {
                Text = "",
                FontFamily = new FontFamily("Arial"),
                FontSize = 9,
                Foreground = Brushes.OrangeRed,
                IsHitTestVisible = false,
                Visibility = Visibility.Hidden
            };
            Panel.SetZIndex(shaft, 4);
            Panel.SetZIndex(head, 4);
            Panel.SetZIndex(label, 5);
            canvas_kinematic.Children.Add(shaft);
            canvas_kinematic.Children.Add(head);
            canvas_kinematic.Children.Add(label);
            contactForceVisual = new ContactForceVisual { Shaft = shaft, Head = head, Label = label };
        }

        private void BuildMeteringSensorIcons()
        {
            int count = Math.Min(barRows.Count, barVisuals.Count);
            for (int i = 0; i < count; i++)
            {
                if (!barRows[i].IsMetering) continue;
                var icon = CreateMeteringSensorIcon(barVisuals[i].Brush ?? Brushes.White);
                var label = new TextBlock
                {
                    Text = "",
                    FontFamily = new FontFamily("Arial"),
                    FontSize = 9,
                    Foreground = barVisuals[i].Brush ?? Brushes.White,
                    IsHitTestVisible = false,
                    Visibility = Visibility.Hidden
                };
                Panel.SetZIndex(icon, 2);
                Panel.SetZIndex(label, 3);
                canvas_kinematic.Children.Add(icon);
                canvas_kinematic.Children.Add(label);
                barSensorVisuals.Add(new BarSensorVisual
                {
                    PinIds = barVisuals[i].PinIds,
                    Icon = icon,
                    Label = label
                });
            }
        }

        private FrameworkElement CreateMeteringSensorIcon(Brush brush)
        {
            var icon = new Canvas
            {
                Width = MeteringIconWidth,
                Height = MeteringIconHeight,
                IsHitTestVisible = false
            };

            var fill = CloneBrushWithOpacity(brush, 0.2);
            var body = new Rectangle
            {
                Width = MeteringIconWidth,
                Height = MeteringIconHeight,
                RadiusX = 2,
                RadiusY = 2,
                Stroke = brush,
                StrokeThickness = 1.5,
                Fill = fill
            };
            var center = new Ellipse
            {
                Width = 4,
                Height = 4,
                Stroke = brush,
                StrokeThickness = 1.2,
                Fill = Brushes.Transparent
            };
            var tabLeft = new Rectangle
            {
                Width = 4,
                Height = 2,
                Fill = brush
            };
            var tabRight = new Rectangle
            {
                Width = 4,
                Height = 2,
                Fill = brush
            };

            Canvas.SetLeft(center, (MeteringIconWidth - center.Width) / 2.0);
            Canvas.SetTop(center, (MeteringIconHeight - center.Height) / 2.0);
            Canvas.SetLeft(tabLeft, 1);
            Canvas.SetTop(tabLeft, (MeteringIconHeight - tabLeft.Height) / 2.0);
            Canvas.SetLeft(tabRight, MeteringIconWidth - tabRight.Width - 1);
            Canvas.SetTop(tabRight, (MeteringIconHeight - tabRight.Height) / 2.0);

            icon.Children.Add(body);
            icon.Children.Add(center);
            icon.Children.Add(tabLeft);
            icon.Children.Add(tabRight);
            return new Viewbox
            {
                Width = MeteringIconWidth,
                Height = MeteringIconHeight,
                Stretch = Stretch.Fill,
                RenderTransformOrigin = new Point(0.5, 0.5),
                IsHitTestVisible = false,
                Child = icon
            };
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
                if (groundedMarkers.TryGetValue(row.PinId, out var marker))
                {
                    PositionGroundedMarker(marker, point);
                }
            }

            UpdateBarPinRingsFromConfig();

            foreach (var bar in barVisuals)
            {
                if (bar.IsMetering)
                {
                    UpdateMeteringBarSegmentsFromConfig(bar);
                    continue;
                }
                if (bar.PinIds.Length < 2) continue;
                if (!TryGetBarPointsFromConfig(bar.PinIds, out PointCollection points))
                {
                    if (bar.Outline != null)
                    {
                        bar.Outline.Visibility = Visibility.Hidden;
                    }
                    continue;
                }
                if (bar.Outline != null)
                {
                    bar.Outline.Visibility = Visibility.Visible;
                    bar.Outline.Points = points;
                }
            }

            UpdateContactForceArrowFromConfig();
            UpdateMeteringSensorsFromConfig();
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
            UpdatePoseAtPosition(lastAxisPosition);
        }

        private void UpdateTestPose()
        {
            if (poseCache == null || poseCache.PinPositionsX.Length == 0) return;
            SetTestAxisPosition(testAxisPosition, false);
            UpdatePoseAtPosition(testAxisPosition);
        }

        private void UpdatePoseAtPosition(double contactPos)
        {
            if (!TryGetSegment(contactPos, out int idx, out double t)) return;

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
                if (groundedMarkers.TryGetValue(pinId, out var marker))
                {
                    PositionGroundedMarker(marker, point);
                }
            }

            UpdateBarPinRingsFromLive();

            foreach (var bar in barVisuals)
            {
                if (bar.IsMetering)
                {
                    UpdateMeteringBarSegmentsFromLive(bar);
                    continue;
                }
                if (bar.PinIds.Length < 2)
                {
                    if (bar.Outline != null)
                    {
                        bar.Outline.Visibility = Visibility.Hidden;
                    }
                    continue;
                }
                if (!TryGetBarPoints(bar.PinIds, out PointCollection points))
                {
                    if (bar.Outline != null)
                    {
                        bar.Outline.Visibility = Visibility.Hidden;
                    }
                    continue;
                }
                if (bar.Outline != null)
                {
                    bar.Outline.Visibility = Visibility.Visible;
                    bar.Outline.Points = points;
                }
            }

            UpdateContactForceArrowFromPose(contactPos);
            UpdateMeteringSensorsFromLive();
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

        private void CanvasKinematic_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (plugin == null) return;
            double scale = plugin.Settings.kinematicDiagram_zeroPos_scale;
            if (scale <= 0.0) scale = 1.0;

            double factor = e.Delta > 0 ? 1.0 / ZoomStep : ZoomStep;
            double nextScale = Clamp(scale * factor, MinZoomScale, MaxZoomScale);
            if (Math.Abs(nextScale - scale) < 1e-9) return;

            Point position = e.GetPosition(canvas_kinematic);
            ApplyZoom(position, nextScale);
            e.Handled = true;
        }

        private void CanvasKinematic_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (plugin == null) return;
            isPanning = true;
            panStart = e.GetPosition(canvas_kinematic);
            panStartOx = plugin.Settings.kinematicDiagram_zeroPos_OX;
            panStartOy = plugin.Settings.kinematicDiagram_zeroPos_OY;
            canvas_kinematic.CaptureMouse();
            canvas_kinematic.Cursor = Cursors.Hand;
            e.Handled = true;
        }

        private void CanvasKinematic_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isPanning || plugin == null) return;
            Point current = e.GetPosition(canvas_kinematic);
            Vector delta = current - panStart;
            plugin.Settings.kinematicDiagram_zeroPos_OX = panStartOx + delta.X;
            plugin.Settings.kinematicDiagram_zeroPos_OY = panStartOy - delta.Y;
            BuildCanvas();
            RefreshPose();
        }

        private void CanvasKinematic_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (!isPanning) return;
            isPanning = false;
            canvas_kinematic.ReleaseMouseCapture();
            canvas_kinematic.Cursor = Cursors.Arrow;
            e.Handled = true;
        }

        private void CanvasKinematic_MouseLeave(object sender, MouseEventArgs e)
        {
            if (!isPanning) return;
            isPanning = false;
            canvas_kinematic.ReleaseMouseCapture();
            canvas_kinematic.Cursor = Cursors.Arrow;
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

            const double basePadding = 20.0;
            double padLeft = basePadding;
            double padRight = basePadding;
            double padTop = basePadding;
            double padBottom = basePadding;
            if (pinRows.Any(p => p.Grounded))
            {
                padLeft += GroundedMarkerWidth / 2.0;
                padRight += GroundedMarkerWidth / 2.0;
                padBottom += GroundedMarkerOffsetY + GroundedMarkerHeight;
            }

            double usableWidth = Math.Max(1.0, width - padLeft - padRight);
            double usableHeight = Math.Max(1.0, height - padTop - padBottom);
            double rangeX = Math.Max(1e-6, maxX - minX);
            double rangeY = Math.Max(1e-6, maxY - minY);
            double scale = Math.Max(rangeX / usableWidth, rangeY / usableHeight);
            if (double.IsNaN(scale) || double.IsInfinity(scale) || scale <= 0.0) scale = 1.0;

            double rangeXCanvas = rangeX / scale;
            double rangeYCanvas = rangeY / scale;
            double extraX = Math.Max(0.0, usableWidth - rangeXCanvas);
            double extraY = Math.Max(0.0, usableHeight - rangeYCanvas);

            double ox = padLeft + extraX / 2.0 - minX / scale;
            double top = padTop + extraY / 2.0;
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

        private Point ToCanvas(double x, double y)
        {
            double scale = plugin?.Settings?.kinematicDiagram_zeroPos_scale ?? 1.0;
            if (scale <= 0.0) scale = 1.0;
            double ox = plugin?.Settings?.kinematicDiagram_zeroPos_OX ?? 0.0;
            double oy = plugin?.Settings?.kinematicDiagram_zeroPos_OY ?? 0.0;
            return new Point(x / scale + ox, canvas_kinematic.Height - y / scale - oy);
        }

        private void ApplyZoom(Point canvasPoint, double newScale)
        {
            if (plugin == null) return;
            double scale = plugin.Settings.kinematicDiagram_zeroPos_scale;
            if (scale <= 0.0) scale = 1.0;
            double ox = plugin.Settings.kinematicDiagram_zeroPos_OX;
            double oy = plugin.Settings.kinematicDiagram_zeroPos_OY;
            double height = canvas_kinematic.ActualHeight;
            if (height <= 1.0) height = canvas_kinematic.Height;

            double worldX = (canvasPoint.X - ox) * scale;
            double worldY = (height - canvasPoint.Y - oy) * scale;

            plugin.Settings.kinematicDiagram_zeroPos_scale = newScale;
            plugin.Settings.kinematicDiagram_zeroPos_OX = canvasPoint.X - worldX / newScale;
            plugin.Settings.kinematicDiagram_zeroPos_OY = height - canvasPoint.Y - worldY / newScale;

            BuildCanvas();
            RefreshPose();
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
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

        private void UpdateContactForceArrowFromConfig()
        {
            if (contactForceVisual == null) return;
            var contactPin = pinRows.FirstOrDefault(p => p.IsContactPoint);
            if (contactPin == null)
            {
                SetContactForceVisibility(false);
                return;
            }

            if (!TryGetContactPathDirection(0.0, contactPin.PinId, out var direction))
            {
                SetContactForceVisibility(false);
                return;
            }
            string labelText = TryGetContactForceLabel(out var contactLabel) ? contactLabel : null;
            UpdateContactForceArrow(contactPin.X, contactPin.Y, direction, labelText);
        }

        private void UpdateContactForceArrowFromPose(double contactPos)
        {
            if (contactForceVisual == null) return;
            var contactPin = pinRows.FirstOrDefault(p => p.IsContactPoint);
            if (contactPin == null || !pinIndexById.TryGetValue(contactPin.PinId, out int idx))
            {
                SetContactForceVisibility(false);
                return;
            }

            double x = currentX[idx];
            double y = currentY[idx];
            if (!TryGetContactPathDirection(contactPos, contactPin.PinId, out var direction))
            {
                SetContactForceVisibility(false);
                return;
            }
            string labelText = TryGetContactForceLabel(out var contactLabel) ? contactLabel : null;
            UpdateContactForceArrow(x, y, direction, labelText);
        }

        private void UpdateContactForceArrow(double pinX, double pinY, Vector directionWorld, string labelText)
        {
            if (contactForceVisual == null) return;

            var contactCanvas = ToCanvas(pinX, pinY);
            var dirCanvas = ToCanvas(pinX + directionWorld.X, pinY + directionWorld.Y) - contactCanvas;
            if (dirCanvas.Length < 1e-6)
            {
                dirCanvas = new Vector(0.0, -1.0);
            }
            dirCanvas.Normalize();

            var head = contactCanvas;
            var tail = head - dirCanvas * ContactArrowLength;
            contactForceVisual.Shaft.X1 = tail.X;
            contactForceVisual.Shaft.Y1 = tail.Y;
            contactForceVisual.Shaft.X2 = head.X;
            contactForceVisual.Shaft.Y2 = head.Y;

            var basePoint = head - dirCanvas * ContactArrowHeadLength;
            var perp = new Vector(-dirCanvas.Y, dirCanvas.X);
            var left = basePoint + perp * (ContactArrowHeadWidth / 2.0);
            var right = basePoint - perp * (ContactArrowHeadWidth / 2.0);
            contactForceVisual.Head.Points = new PointCollection { head, left, right };
            UpdateContactForceLabel(head, labelText);
            SetContactForceVisibility(true);
        }

        private void SetContactForceVisibility(bool visible)
        {
            if (contactForceVisual == null) return;
            var state = visible ? Visibility.Visible : Visibility.Hidden;
            contactForceVisual.Shaft.Visibility = state;
            contactForceVisual.Head.Visibility = state;
            if (contactForceVisual.Label != null)
            {
                contactForceVisual.Label.Visibility = state;
            }
        }

        private void UpdateContactForceLabel(Point head, string labelText)
        {
            if (contactForceVisual?.Label == null) return;
            if (string.IsNullOrWhiteSpace(labelText))
            {
                contactForceVisual.Label.Visibility = Visibility.Hidden;
                return;
            }

            contactForceVisual.Label.Text = labelText;
            contactForceVisual.Label.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
            var size = contactForceVisual.Label.DesiredSize;
            var anchor = GetContactForceLabelAnchor(head);
            Canvas.SetLeft(contactForceVisual.Label, anchor.X - size.Width / 2.0);
            Canvas.SetTop(contactForceVisual.Label, anchor.Y - size.Height - 6.0);
            contactForceVisual.Label.Visibility = Visibility.Visible;
        }

        private Point GetContactForceLabelAnchor(Point fallback)
        {
            var shaft = contactForceVisual?.Shaft;
            if (shaft == null) return fallback;
            return new Point((shaft.X1 + shaft.X2) / 2.0, (shaft.Y1 + shaft.Y2) / 2.0);
        }

        private bool TryGetContactPathDirection(double contactPos, uint contactPinId, out Vector direction)
        {
            direction = new Vector();
            if (poseCache == null || poseCache.ContactPositions.Length < 2) return false;
            if (!TryGetPosePinIndex(contactPinId, out int pinIdx)) return false;
            if (!TryGetSegment(contactPos, out int idx, out _)) return false;

            int lastIndex = poseCache.PinPositionsX.Length - 1;
            int next = Math.Min(idx + 1, lastIndex);
            double x1 = poseCache.PinPositionsX[idx][pinIdx];
            double y1 = poseCache.PinPositionsY[idx][pinIdx];
            double x2 = poseCache.PinPositionsX[next][pinIdx];
            double y2 = poseCache.PinPositionsY[next][pinIdx];
            direction = new Vector(x2 - x1, y2 - y1);

            if (direction.Length <= 1e-6 && idx > 0)
            {
                double xp = poseCache.PinPositionsX[idx - 1][pinIdx];
                double yp = poseCache.PinPositionsY[idx - 1][pinIdx];
                direction = new Vector(x1 - xp, y1 - yp);
            }

            return direction.Length > 1e-6;
        }

        private bool TryGetPosePinIndex(uint pinId, out int index)
        {
            index = -1;
            if (poseCache == null) return false;
            for (int i = 0; i < poseCache.PinIds.Length; i++)
            {
                if (poseCache.PinIds[i] == pinId)
                {
                    index = i;
                    return true;
                }
            }
            return false;
        }

        private bool TryGetDisplayInputs(out double contactPos, out double contactForce)
        {
            contactPos = 0.0;
            contactForce = 0.0;
            switch (poseMode)
            {
                case PoseDisplayMode.Live:
                    if (!hasAxisState) return false;
                    contactPos = lastAxisPosition;
                    contactForce = lastAxisForce;
                    return true;
                case PoseDisplayMode.Test:
                    contactPos = testAxisPosition;
                    contactForce = testAxisForce;
                    return true;
                default:
                    return false;
            }
        }

        private bool TryGetContactForceLabel(out string label)
        {
            label = null;
            if (!TryGetDisplayInputs(out _, out double force)) return false;
            label = FormatForceLabel(force);
            return true;
        }

        private bool TryGetMeasuredForceLabel(out string label)
        {
            label = null;
            if (!TryGetDisplayInputs(out double contactPos, out double contactForce)) return false;
            if (!TryGetForceConversionFactor(contactPos, out double factor)) return false;
            double measured = contactForce / factor;
            if (double.IsNaN(measured) || double.IsInfinity(measured)) return false;
            label = FormatForceLabel(measured);
            return true;
        }

        private string FormatForceLabel(double forceValue)
        {
            return string.Format(CultureInfo.CurrentCulture, "{0:0.0} N", forceValue);
        }

        private bool TryGetForceConversionFactor(double contactPos, out double factor)
        {
            factor = 0.0;
            if (currentParameters == null) return false;
            var coeffs = currentParameters.CoeffsForceFactorOverContactPointPos;
            if (coeffs == null || coeffs.Count == 0) return false;

            double result = 0.0;
            for (int i = coeffs.Count - 1; i >= 0; i--)
            {
                result = result * contactPos + coeffs[i];
            }
            if (double.IsNaN(result) || double.IsInfinity(result) || Math.Abs(result) < 1e-9) return false;
            factor = result;
            return true;
        }

        private void UpdateMeteringSensorsFromConfig()
        {
            foreach (var sensor in barSensorVisuals)
            {
                if (!TryGetBarCenterFromConfig(sensor.PinIds, out double cx, out double cy))
                {
                    sensor.Icon.Visibility = Visibility.Hidden;
                    if (sensor.Label != null)
                    {
                        sensor.Label.Visibility = Visibility.Hidden;
                    }
                    continue;
                }
                UpdateMeteringSensorSizeFromConfig(sensor);
                var center = ToCanvas(cx, cy);
                Canvas.SetLeft(sensor.Icon, center.X - sensor.Icon.Width / 2.0);
                Canvas.SetTop(sensor.Icon, center.Y - sensor.Icon.Height / 2.0);
                UpdateMeteringSensorRotationConfig(sensor);
                string labelText = TryGetMeasuredForceLabel(out var measuredLabel) ? measuredLabel : null;
                UpdateMeteringSensorLabel(sensor, center, labelText);
                sensor.Icon.Visibility = Visibility.Visible;
            }
        }

        private void UpdateMeteringSensorsFromLive()
        {
            foreach (var sensor in barSensorVisuals)
            {
                if (!TryGetBarCenterFromLive(sensor.PinIds, out double cx, out double cy))
                {
                    sensor.Icon.Visibility = Visibility.Hidden;
                    if (sensor.Label != null)
                    {
                        sensor.Label.Visibility = Visibility.Hidden;
                    }
                    continue;
                }
                UpdateMeteringSensorSizeFromLive(sensor);
                var center = ToCanvas(cx, cy);
                Canvas.SetLeft(sensor.Icon, center.X - sensor.Icon.Width / 2.0);
                Canvas.SetTop(sensor.Icon, center.Y - sensor.Icon.Height / 2.0);
                UpdateMeteringSensorRotationLive(sensor);
                string labelText = TryGetMeasuredForceLabel(out var measuredLabel) ? measuredLabel : null;
                UpdateMeteringSensorLabel(sensor, center, labelText);
                sensor.Icon.Visibility = Visibility.Visible;
            }
        }

        private void UpdateMeteringSensorLabel(BarSensorVisual sensor, Point center, string labelText)
        {
            if (sensor.Label == null) return;
            if (string.IsNullOrWhiteSpace(labelText))
            {
                sensor.Label.Visibility = Visibility.Hidden;
                return;
            }

            sensor.Label.Text = labelText;
            sensor.Label.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
            var size = sensor.Label.DesiredSize;
            double iconWidth = sensor.Icon.Width > 0.0 ? sensor.Icon.Width : MeteringIconWidth;
            double iconHeight = sensor.Icon.Height > 0.0 ? sensor.Icon.Height : MeteringIconHeight;
            Canvas.SetLeft(sensor.Label, center.X - size.Width / 2.0);
            Canvas.SetTop(sensor.Label, center.Y - iconHeight / 2.0 - size.Height - 4.0);
            sensor.Label.Visibility = Visibility.Visible;
        }

        private void UpdateMeteringSensorSizeFromConfig(BarSensorVisual sensor)
        {
            if (!TryGetMeteringIconCanvasSize(out double width, out double height)) return;
            UpdateMeteringSensorSize(sensor, width, height);
        }

        private void UpdateMeteringSensorSizeFromLive(BarSensorVisual sensor)
        {
            if (!TryGetMeteringIconCanvasSize(out double width, out double height)) return;
            UpdateMeteringSensorSize(sensor, width, height);
        }

        private void UpdateMeteringSensorSize(BarSensorVisual sensor, double width, double height)
        {
            if (sensor?.Icon == null) return;
            sensor.Icon.Width = width;
            sensor.Icon.Height = height;
        }

        private bool TryGetMeteringIconCanvasSize(out double width, out double height)
        {
            double scale = plugin?.Settings?.kinematicDiagram_zeroPos_scale ?? 1.0;
            if (scale <= 0.0) scale = 1.0;
            width = MeteringIconWorldWidth / scale;
            height = MeteringIconWorldHeight / scale;
            return width > 1e-6 && height > 1e-6;
        }

        private void UpdateMeteringSensorRotationConfig(BarSensorVisual sensor)
        {
            if (!TryGetBarDirectionCanvasFromConfig(sensor.PinIds, out var dir)) return;
            sensor.Icon.RenderTransform = new RotateTransform(Math.Atan2(dir.Y, dir.X) * 180.0 / Math.PI);
        }

        private void UpdateMeteringSensorRotationLive(BarSensorVisual sensor)
        {
            if (!TryGetBarDirectionCanvasFromLive(sensor.PinIds, out var dir)) return;
            sensor.Icon.RenderTransform = new RotateTransform(Math.Atan2(dir.Y, dir.X) * 180.0 / Math.PI);
        }

        private bool TryGetBarCenterFromConfig(uint[] pinIds, out double cx, out double cy)
        {
            double sumX = 0.0;
            double sumY = 0.0;
            int count = 0;
            foreach (var pinId in pinIds)
            {
                if (!TryGetPointFromConfig(pinId, out var row)) continue;
                sumX += row.X;
                sumY += row.Y;
                count++;
            }
            if (count == 0)
            {
                cx = 0.0;
                cy = 0.0;
                return false;
            }
            cx = sumX / count;
            cy = sumY / count;
            return true;
        }

        private bool TryGetBarCenterFromLive(uint[] pinIds, out double cx, out double cy)
        {
            double sumX = 0.0;
            double sumY = 0.0;
            int count = 0;
            foreach (var pinId in pinIds)
            {
                if (!pinIndexById.TryGetValue(pinId, out int idx)) continue;
                sumX += currentX[idx];
                sumY += currentY[idx];
                count++;
            }
            if (count == 0)
            {
                cx = 0.0;
                cy = 0.0;
                return false;
            }
            cx = sumX / count;
            cy = sumY / count;
            return true;
        }

        private bool TryGetBarDirectionCanvasFromConfig(uint[] pinIds, out Vector direction)
        {
            direction = new Vector(1.0, 0.0);
            if (!TryGetTwoPinPointsFromConfig(pinIds, out var p1, out var p2)) return false;
            direction = p2 - p1;
            return direction.Length > 1e-6;
        }

        private bool TryGetBarDirectionCanvasFromLive(uint[] pinIds, out Vector direction)
        {
            direction = new Vector(1.0, 0.0);
            if (!TryGetTwoPinPointsFromLive(pinIds, out var p1, out var p2)) return false;
            direction = p2 - p1;
            return direction.Length > 1e-6;
        }

        private void UpdateMeteringBarSegmentsFromConfig(BarVisual bar)
        {
            if (bar == null || bar.SegmentA == null || bar.SegmentB == null) return;
            if (!TryGetTwoPinPointsFromConfig(bar.PinIds, out var p1, out var p2))
            {
                bar.SegmentA.Visibility = Visibility.Hidden;
                bar.SegmentB.Visibility = Visibility.Hidden;
                return;
            }
            UpdateMeteringBarSegments(bar, p1, p2);
        }

        private void UpdateMeteringBarSegmentsFromLive(BarVisual bar)
        {
            if (bar == null || bar.SegmentA == null || bar.SegmentB == null) return;
            if (!TryGetTwoPinPointsFromLive(bar.PinIds, out var p1, out var p2))
            {
                bar.SegmentA.Visibility = Visibility.Hidden;
                bar.SegmentB.Visibility = Visibility.Hidden;
                return;
            }
            UpdateMeteringBarSegments(bar, p1, p2);
        }

        private void UpdateMeteringBarSegments(BarVisual bar, Point p1, Point p2)
        {
            if (!TryGetMeteringIconCanvasSize(out double iconWidth, out _))
            {
                iconWidth = MeteringIconWidth;
            }

            Vector dir = p2 - p1;
            double length = dir.Length;
            if (length <= 1e-6)
            {
                bar.SegmentA.Visibility = Visibility.Hidden;
                bar.SegmentB.Visibility = Visibility.Hidden;
                return;
            }
            dir.Normalize();

            double gap = Math.Min(iconWidth + 4.0, length);
            double halfGap = gap / 2.0;
            var center = new Point((p1.X + p2.X) / 2.0, (p1.Y + p2.Y) / 2.0);
            var gapStart = center - dir * halfGap;
            var gapEnd = center + dir * halfGap;

            bar.SegmentA.X1 = p1.X;
            bar.SegmentA.Y1 = p1.Y;
            bar.SegmentA.X2 = gapStart.X;
            bar.SegmentA.Y2 = gapStart.Y;
            bar.SegmentB.X1 = gapEnd.X;
            bar.SegmentB.Y1 = gapEnd.Y;
            bar.SegmentB.X2 = p2.X;
            bar.SegmentB.Y2 = p2.Y;
            bar.SegmentA.Visibility = Visibility.Visible;
            bar.SegmentB.Visibility = Visibility.Visible;
        }

        private bool TryGetTwoPinPointsFromConfig(uint[] pinIds, out Point p1, out Point p2)
        {
            p1 = new Point();
            p2 = new Point();
            bool foundFirst = false;
            foreach (var pinId in pinIds)
            {
                if (!TryGetPointFromConfig(pinId, out var row)) continue;
                var point = ToCanvas(row.X, row.Y);
                if (!foundFirst)
                {
                    p1 = point;
                    foundFirst = true;
                }
                else
                {
                    p2 = point;
                    return true;
                }
            }
            return false;
        }

        private bool TryGetTwoPinPointsFromLive(uint[] pinIds, out Point p1, out Point p2)
        {
            p1 = new Point();
            p2 = new Point();
            bool foundFirst = false;
            foreach (var pinId in pinIds)
            {
                if (!pinIndexById.TryGetValue(pinId, out int idx)) continue;
                var point = ToCanvas(currentX[idx], currentY[idx]);
                if (!foundFirst)
                {
                    p1 = point;
                    foundFirst = true;
                }
                else
                {
                    p2 = point;
                    return true;
                }
            }
            return false;
        }

        private Brush CloneBrushWithOpacity(Brush brush, double opacity)
        {
            if (brush == null) return null;
            var cloned = brush.Clone();
            cloned.Opacity = opacity;
            return cloned;
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
            AutoFitCanvasToPoses(true);
        }

        private void PoseModeCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PoseModeCombo == null) return;
            PoseDisplayMode newMode = PoseDisplayMode.Live;
            switch (PoseModeCombo.SelectedIndex)
            {
                case 1:
                    newMode = PoseDisplayMode.Test;
                    break;
            }

            poseMode = newMode;
            if (TestPosePanel != null)
            {
                TestPosePanel.Visibility = poseMode == PoseDisplayMode.Test ? Visibility.Visible : Visibility.Collapsed;
            }
            if (poseMode == PoseDisplayMode.Test)
            {
                SetTestAxisPosition(testAxisPosition, true);
            }
            if (!isLoading)
            {
                RefreshPose();
            }
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

        private void RailTravel_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (isLoading) return;
            bool isNegative = ReferenceEquals(sender, TextRailNegative);
            bool isPositive = ReferenceEquals(sender, TextRailPositive);
            if (!isNegative && !isPositive) return;

            var textBox = isNegative ? TextRailNegative : TextRailPositive;
            double value = isNegative ? railTravelNegative : railTravelPositive;
            if (textBox != null && TryParseDouble(textBox.Text, out double parsed))
            {
                value = parsed;
            }

            double step = (Keyboard.Modifiers & ModifierKeys.Shift) != 0 ? RailTravelFineStep : RailTravelStep;
            value += e.Delta > 0 ? step : -step;
            value = NormalizeNonNegativeMillimeters(value);

            if (isNegative)
            {
                railTravelNegative = value;
            }
            else
            {
                railTravelPositive = value;
            }

            if (textBox != null)
            {
                isLoading = true;
                textBox.Text = value.ToString("0.###", CultureInfo.CurrentCulture);
                isLoading = false;
            }

            QueueRebuild();
            e.Handled = true;
        }

        private void PinGrid_PreviewMouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (isLoading) return;
            if (!(e.OriginalSource is DependencyObject source)) return;
            var cell = FindAncestor<DataGridCell>(source);
            if (cell == null) return;
            if (!(cell.DataContext is PinRow row)) return;
            if (!(cell.Column?.Header is string header)) return;

            bool isX = string.Equals(header, "X", StringComparison.OrdinalIgnoreCase);
            bool isY = string.Equals(header, "Y", StringComparison.OrdinalIgnoreCase);
            if (!isX && !isY) return;

            double step = (Keyboard.Modifiers & ModifierKeys.Shift) != 0 ? PinPositionFineStep : PinPositionStep;
            double delta = e.Delta > 0 ? step : -step;
            if (isX)
            {
                row.X += delta;
            }
            else
            {
                row.Y += delta;
            }

            e.Handled = true;
        }

        private void TestPose_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (isLoading) return;
            if (TryParseDouble(TextTestPosition.Text, out double position))
            {
                SetTestAxisPosition(position, false);
            }
            if (TryParseDouble(TextTestForce.Text, out double force))
            {
                SetTestAxisForce(force, false);
            }
            if (poseMode == PoseDisplayMode.Test)
            {
                UpdateTestPose();
            }
        }

        private void TestPose_LostFocus(object sender, RoutedEventArgs e)
        {
            SetTestAxisPosition(testAxisPosition, true);
            SetTestAxisForce(testAxisForce, true);
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
            value = NormalizeNonNegativeMillimeters(parsed);
            return true;
        }

        private bool TryParseDouble(string text, out double value)
        {
            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
        }

        private void TestPose_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (isLoading) return;
            if (sender == TextTestPosition)
            {
                double value = testAxisPosition;
                if (TryParseDouble(TextTestPosition.Text, out double parsed))
                {
                    value = parsed;
                }
                double step = (Keyboard.Modifiers & ModifierKeys.Shift) != 0 ? TestPosePositionFineStep : TestPosePositionStep;
                value += e.Delta > 0 ? step : -step;
                SetTestAxisPosition(value, true);
            }
            else if (sender == TextTestForce)
            {
                double value = testAxisForce;
                if (TryParseDouble(TextTestForce.Text, out double parsed))
                {
                    value = parsed;
                }
                value += e.Delta > 0 ? TestPoseForceStep : -TestPoseForceStep;
                SetTestAxisForce(value, true);
            }
            else
            {
                return;
            }

            if (poseMode == PoseDisplayMode.Test)
            {
                UpdateTestPose();
            }
            e.Handled = true;
        }

        private bool TryGetContactRange(out double min, out double max)
        {
            min = 0.0;
            max = 0.0;
            if (poseCache == null || poseCache.ContactPositions.Length == 0) return false;
            min = poseCache.ContactPositions.First();
            max = poseCache.ContactPositions.Last();
            if (min > max)
            {
                double tmp = min;
                min = max;
                max = tmp;
            }
            return true;
        }

        private void SetTestAxisPosition(double value, bool updateText)
        {
            value = RoundMillimeters(value);
            if (TryGetContactRange(out double min, out double max))
            {
                value = Clamp(value, min, max);
            }
            testAxisPosition = value;
            if (updateText && TextTestPosition != null)
            {
                isLoading = true;
                TextTestPosition.Text = testAxisPosition.ToString("0.###", CultureInfo.CurrentCulture);
                isLoading = false;
            }
        }

        private void SetTestAxisForce(double value, bool updateText)
        {
            testAxisForce = value;
            if (updateText && TextTestForce != null)
            {
                isLoading = true;
                TextTestForce.Text = testAxisForce.ToString("0.0", CultureInfo.CurrentCulture);
                isLoading = false;
            }
        }

        private static T FindAncestor<T>(DependencyObject source) where T : DependencyObject
        {
            DependencyObject current = source;
            while (current != null)
            {
                if (current is T match) return match;
                current = VisualTreeHelper.GetParent(current);
            }
            return null;
        }

        private FrameworkElement CreateGroundedMarker()
        {
            double width = GroundedMarkerWidth;
            double height = GroundedMarkerHeight;
            double center = width / 2.0;
            var stroke = Brushes.DarkGray;
            double thickness = 1.5;
            double apexY = 0.0;
            double baseY = height - 8.0;
            double baseHalf = width * 0.45;
            double triangleHalf = width * 0.32;

            var canvas = new Canvas
            {
                Width = width,
                Height = height,
                IsHitTestVisible = false
            };

            var triangle = new Polygon
            {
                Stroke = stroke,
                StrokeThickness = thickness,
                Fill = Brushes.Transparent,
                Points = new PointCollection
                {
                    new Point(center, apexY),
                    new Point(center - triangleHalf, baseY),
                    new Point(center + triangleHalf, baseY)
                }
            };
            canvas.Children.Add(triangle);

            var baseLine = new Line
            {
                X1 = center - baseHalf,
                Y1 = baseY,
                X2 = center + baseHalf,
                Y2 = baseY,
                Stroke = stroke,
                StrokeThickness = thickness,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round
            };
            canvas.Children.Add(baseLine);

            double hatchTop = baseY + 2.0;
            double hatchHeight = Math.Max(4.0, height - hatchTop - 1.0);
            double hatchWidth = Math.Min(hatchHeight * 0.9, baseHalf * 0.6);
            int hatchCount = 4;
            double left = center - baseHalf;
            double spacing = (baseHalf * 2.0) / hatchCount;
            for (int i = 0; i < hatchCount; i++)
            {
                double x = left + (i + 0.5) * spacing;
                AddGroundHatch(canvas, x, hatchTop, hatchWidth, hatchHeight, stroke, thickness);
            }

            return canvas;
        }

        private void AddGroundHatch(Canvas canvas, double x, double y, double width, double height, Brush stroke, double thickness)
        {
            var line = new Line
            {
                X1 = x - width / 2.0,
                Y1 = y,
                X2 = x + width / 2.0,
                Y2 = y + height,
                Stroke = stroke,
                StrokeThickness = thickness,
                StrokeStartLineCap = PenLineCap.Round,
                StrokeEndLineCap = PenLineCap.Round
            };
            canvas.Children.Add(line);
        }

        private void PositionGroundedMarker(FrameworkElement marker, Point pinPoint)
        {
            if (marker == null) return;
            Canvas.SetLeft(marker, pinPoint.X - marker.Width / 2.0);
            Canvas.SetTop(marker, pinPoint.Y + GroundedMarkerOffsetY);
        }

        private static double RoundMillimeters(double value)
        {
            return Math.Round(value, 1, MidpointRounding.AwayFromZero);
        }

        private static double NormalizeNonNegativeMillimeters(double value)
        {
            if (value < 0.0)
            {
                value = 0.0;
            }
            return RoundMillimeters(value);
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
