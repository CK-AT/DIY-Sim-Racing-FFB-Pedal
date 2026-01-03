using System;
using System;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using MahApps.Metro.Controls;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for ShifterConfigControl.xaml
    /// </summary>
    public partial class ShifterConfigControl : UserControl
    {
        private sealed class GateSegmentRow : INotifyPropertyChanged
        {
            public Func<double, double> RoundCoordinate { get; set; } = value => value;
            public Func<double, double> RoundLength { get; set; } = value => value;

            private double x0;
            private double y0;
            private double x1;
            private double y1;
            private double halfWidth;
            private double springCenter;
            private double springWall;

            public double X0
            {
                get => x0;
                set => SetTenthMillimeter(ref x0, value, nameof(X0));
            }

            public double Y0
            {
                get => y0;
                set => SetTenthMillimeter(ref y0, value, nameof(Y0));
            }

            public double X1
            {
                get => x1;
                set => SetTenthMillimeter(ref x1, value, nameof(X1));
            }

            public double Y1
            {
                get => y1;
                set => SetTenthMillimeter(ref y1, value, nameof(Y1));
            }

            public double HalfWidth
            {
                get => halfWidth;
                set
                {
                    double rounded = Math.Max(0.0, RoundLength(value));
                    if (Math.Abs(halfWidth - rounded) < 1e-9) return;
                    halfWidth = rounded;
                    OnPropertyChanged(nameof(HalfWidth));
                }
            }

            public double SpringCenter
            {
                get => springCenter;
                set => SetSpring(ref springCenter, value, nameof(SpringCenter));
            }

            public double SpringWall
            {
                get => springWall;
                set => SetSpring(ref springWall, value, nameof(SpringWall));
            }

            public event PropertyChangedEventHandler PropertyChanged;

            private void SetTenthMillimeter(ref double field, double value, string propertyName)
            {
                double rounded = RoundCoordinate(value);
                if (Math.Abs(field - rounded) < 1e-9) return;
                field = rounded;
                OnPropertyChanged(propertyName);
            }

            private void SetSpring(ref double field, double value, string propertyName)
            {
                double rounded = Math.Round(value, 2, MidpointRounding.AwayFromZero);
                if (Math.Abs(field - rounded) < 1e-6) return;
                field = rounded;
                OnPropertyChanged(propertyName);
            }

            private void OnPropertyChanged(string propertyName)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        private sealed class DetentRow : INotifyPropertyChanged
        {
            public Func<double, double> RoundCoordinate { get; set; } = value => value;
            public Func<double, double> RoundLength { get; set; } = value => value;

            private double x;
            private double y;
            private double radius;
            private double spring;

            public double X
            {
                get => x;
                set => SetTenthMillimeter(ref x, value, nameof(X));
            }

            public double Y
            {
                get => y;
                set => SetTenthMillimeter(ref y, value, nameof(Y));
            }

            public double Radius
            {
                get => radius;
                set
                {
                    double rounded = Math.Max(0.0, RoundLength(value));
                    if (Math.Abs(radius - rounded) < 1e-9) return;
                    radius = rounded;
                    OnPropertyChanged(nameof(Radius));
                }
            }

            public double Spring
            {
                get => spring;
                set
                {
                    double rounded = Math.Round(value, 2, MidpointRounding.AwayFromZero);
                    if (Math.Abs(spring - rounded) < 1e-6) return;
                    spring = rounded;
                    OnPropertyChanged(nameof(Spring));
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;

            private void SetTenthMillimeter(ref double field, double value, string propertyName)
            {
                double rounded = RoundCoordinate(value);
                if (Math.Abs(field - rounded) < 1e-9) return;
                field = rounded;
                OnPropertyChanged(propertyName);
            }

            private void OnPropertyChanged(string propertyName)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        private sealed class SlotRow : INotifyPropertyChanged
        {
            public Func<double, double> RoundCoordinate { get; set; } = value => value;
            public Func<double, double> RoundLength { get; set; } = value => value;

            private double centerX;
            private double centerY;
            private double halfWidth;
            private double halfHeight;
            private ShifterGear gearCode = ShifterGear.Neutral;

            public double CenterX
            {
                get => centerX;
                set => SetTenthMillimeter(ref centerX, value, nameof(CenterX));
            }

            public double CenterY
            {
                get => centerY;
                set => SetTenthMillimeter(ref centerY, value, nameof(CenterY));
            }

            public double HalfWidth
            {
                get => halfWidth;
                set
                {
                    double rounded = Math.Max(0.0, RoundLength(value));
                    if (Math.Abs(halfWidth - rounded) < 1e-9) return;
                    halfWidth = rounded;
                    OnPropertyChanged(nameof(HalfWidth));
                }
            }

            public double HalfHeight
            {
                get => halfHeight;
                set
                {
                    double rounded = Math.Max(0.0, RoundLength(value));
                    if (Math.Abs(halfHeight - rounded) < 1e-9) return;
                    halfHeight = rounded;
                    OnPropertyChanged(nameof(HalfHeight));
                }
            }

            public ShifterGear GearCode
            {
                get => gearCode;
                set
                {
                    ShifterGear clamped = ShifterConfigControl.ClampGear(value);
                    if (gearCode == clamped) return;
                    gearCode = clamped;
                    OnPropertyChanged(nameof(GearCode));
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;

            private void SetTenthMillimeter(ref double field, double value, string propertyName)
            {
                double rounded = RoundCoordinate(value);
                if (Math.Abs(field - rounded) < 1e-9) return;
                field = rounded;
                OnPropertyChanged(propertyName);
            }

            private void OnPropertyChanged(string propertyName)
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }

        public sealed class GearOption
        {
            public GearOption(ShifterGear code, string label)
            {
                Code = code;
                Label = label;
            }

            public ShifterGear Code { get; }
            public string Label { get; }
        }

        private const ShifterGear GearNeutral = ShifterGear.Neutral;
        private const ShifterGear GearFirst = ShifterGear._1;
        private const ShifterGear GearMax = ShifterGear._7;
        private const ShifterGear GearReverse = ShifterGear.Reverse;
        private const ShifterGear GearSeqUp = ShifterGear.SequentialUp;
        private const ShifterGear GearSeqDown = ShifterGear.SequentialDown;

        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private FunctionConfig function_config = new FunctionConfig();
        private Function function;
        private ShifterConfig shifter_config = new ShifterConfig();
        private ShifterDetectConfig detect_config = new ShifterDetectConfig();
        private AxisID axisX = AxisID.AxisUndefined;
        private AxisID axisY = AxisID.AxisUndefined;
        private bool hasAxisXState = false;
        private bool hasAxisYState = false;
        private double lastAxisXPosition = 0.0;
        private double lastAxisYPosition = 0.0;
        private bool useRelativeGeometry = true;
        private bool isUpdating = true;

        private readonly ObservableCollection<GateSegmentRow> gateRows = new ObservableCollection<GateSegmentRow>();
        private readonly ObservableCollection<DetentRow> detentRows = new ObservableCollection<DetentRow>();
        private readonly ObservableCollection<SlotRow> slotRows = new ObservableCollection<SlotRow>();
        private readonly ObservableCollection<GearOption> gearOptions = new ObservableCollection<GearOption>();
        private const double PreviewPadding = 18.0;
        private double previewScale = 1.0;
        private double previewOffsetX = 0.0;
        private double previewOffsetY = 0.0;

        public ObservableCollection<GearOption> GearOptions => gearOptions;

        public ShifterConfigControl()
        {
            shifter_config = GetDefaultConfig();
            InitializeComponent();
            Checkbox_relative_geometry.IsChecked = useRelativeGeometry;
            RefreshGearOptions();
            ApplyGearOptionsToColumn();
            GateGrid.ItemsSource = gateRows;
            DetentGrid.ItemsSource = detentRows;
            SlotGrid.ItemsSource = slotRows;
            gateRows.CollectionChanged += GateRows_CollectionChanged;
            detentRows.CollectionChanged += DetentRows_CollectionChanged;
            slotRows.CollectionChanged += SlotRows_CollectionChanged;
            canvas_shifter_preview.SizeChanged += CanvasShifterPreview_SizeChanged;
            isUpdating = false;
            BuildPreview();
        }

        private void ApplyGearOptionsToColumn()
        {
            if (SlotGrid == null)
            {
                return;
            }
            var gearColumn = SlotGrid.Columns.OfType<DataGridComboBoxColumn>().FirstOrDefault();
            if (gearColumn != null)
            {
                gearColumn.ItemsSource = gearOptions;
            }
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
        }

        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            ApplyAxisRangeLimits();
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            bool updated = false;
            if (axis_state.AxisId == axisX && axisX != AxisID.AxisUndefined)
            {
                lastAxisXPosition = axis_state.Position;
                hasAxisXState = true;
                updated = true;
            }
            if (axis_state.AxisId == axisY && axisY != AxisID.AxisUndefined)
            {
                lastAxisYPosition = axis_state.Position;
                hasAxisYState = true;
                updated = true;
            }
            if (updated)
            {
                BuildPreview();
            }
        }

        public static ShifterConfig GetDefaultConfig()
        {
            ShifterConfig config = new ShifterConfig();
            config.PosXMin = -60;
            config.PosXMax = 60;
            config.PosYMin = -60;
            config.PosYMax = 60;
            config.Damping = 0.5f;
            config.MaxForce = 50.0f;
            config.GridStep = 10;
            config.Sequential = false;
            return config;
        }

        public static AuxFunctionConfig GetDefaultDetectConfig()
        {
            AuxFunctionConfig config = new AuxFunctionConfig();
            config.LinkedAxes.AddRange(new AxisID[4] { AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined });
            config.ShifterDetect = new ShifterDetectConfig();
            config.ShifterDetect.Hysteresis = 0;
            return config;
        }

        public void SwitchFunction(Function function)
        {
            this.function = function;
            function_config = function.Config;
            if (function_config.Shifter == null)
            {
                function_config.Shifter = GetDefaultConfig();
            }
            EnsureDetectConfig();
            shifter_config = function_config.Shifter;

            isUpdating = true;
            function_config.Base.OutputMode = OutputMode.Travel;
            axisX = AxisID.AxisUndefined;
            axisY = AxisID.AxisUndefined;
            hasAxisXState = false;
            hasAxisYState = false;
            lastAxisXPosition = 0.0;
            lastAxisYPosition = 0.0;
            function_config.Base.ControllerOutputAxis = ControllerAxis.Undefined;
            AxisID baseAxis0 = GetLinkedAxis(0);
            AxisID baseAxis1 = GetLinkedAxis(1);
            AxisID baseAxisX = shifter_config.Sequential ? baseAxis1 : baseAxis0;
            AxisID baseAxisY = shifter_config.Sequential ? baseAxis0 : baseAxis1;
            AxisID auxAxisX = GetAuxLinkedAxis(0);
            AxisID auxAxisY = GetAuxLinkedAxis(1);
            axisX = auxAxisX != AxisID.AxisUndefined ? auxAxisX : baseAxisX;
            axisY = auxAxisY != AxisID.AxisUndefined ? auxAxisY : baseAxisY;
            if (shifter_config.Sequential)
            {
                axisY = axisY == AxisID.AxisUndefined ? baseAxis0 : axisY;
            }
            UpdateLinkedAxes();
            uc_axis_selector_x.Value = axisX;
            uc_axis_selector_y.Value = axisY;
            Checkbox_sequential.IsChecked = shifter_config.Sequential;
            Checkbox_relative_geometry.IsChecked = useRelativeGeometry;

            Rangeslider_x_range.LowerValue = shifter_config.PosXMin;
            Rangeslider_x_range.UpperValue = shifter_config.PosXMax;
            Rangeslider_y_range.LowerValue = shifter_config.PosYMin;
            Rangeslider_y_range.UpperValue = shifter_config.PosYMax;

            UpdateRangeLabels();
            ApplyAxisRangeLimits();
            UpdateOutputRange();
            UpdateSequentialUI();

            Slider_damping.Value = shifter_config.Damping;
            TextMaxForce.Text = shifter_config.MaxForce.ToString("0.##", CultureInfo.CurrentCulture);
            TextGridStep.Text = FromTenthMillimetersUnsigned(shifter_config.GridStep).ToString("0.##", CultureInfo.CurrentCulture);

            Slider_friction.Value = function_config.Friction;
            Slider_simulated_mass.Value = function_config.SimulatedMass;

            LoadRowsFromConfig();
            isUpdating = false;
            BuildPreview();
        }

        private AxisID GetLinkedAxis(int index)
        {
            return function_config.Base.LinkedAxes.Count > index ? function_config.Base.LinkedAxes[index] : AxisID.AxisUndefined;
        }

        private AxisID GetAuxLinkedAxis(int index)
        {
            if (function_config.AuxFunction == null || function_config.AuxFunction.LinkedAxes.Count <= index)
            {
                return AxisID.AxisUndefined;
            }
            return function_config.AuxFunction.LinkedAxes[index];
        }

        private void UpdateSequentialUI()
        {
            uc_axis_selector_x.IsEnabled = true;
            Rangeslider_x_range.IsEnabled = true;
            Label_x_min.IsEnabled = true;
            Label_x_max.IsEnabled = true;
        }

        private bool TryGetAxisTravelRange(AxisID axisId, out double minValue, out double maxValue)
        {
            minValue = 0.0;
            maxValue = 0.0;
            if (ui == null || axisId == AxisID.AxisUndefined)
            {
                return false;
            }
            if (!ui.axes.TryGetValue(axisId, out Axis axis) || axis?.Config?.KinematicParameters == null)
            {
                return false;
            }
            KinematicParameters parameters = axis.Config.KinematicParameters;
            double minAbs = parameters.ContactPointPosMinAbs / 10.0;
            double maxAbs = parameters.ContactPointPosMaxAbs / 10.0;
            if (Math.Abs(maxAbs - minAbs) < 1e-6)
            {
                return false;
            }
            minValue = Math.Min(minAbs, maxAbs);
            maxValue = Math.Max(minAbs, maxAbs);
            return true;
        }

        private void ApplyAxisRangeLimits()
        {
            if (Rangeslider_x_range == null || Rangeslider_y_range == null)
            {
                return;
            }

            bool hasX = TryGetAxisTravelRange(axisX, out double minX, out double maxX);
            bool hasY = TryGetAxisTravelRange(axisY, out double minY, out double maxY);
            if (!hasX && !hasY)
            {
                return;
            }

            bool wasUpdating = isUpdating;
            isUpdating = true;
            if (hasX)
            {
                ApplyRangeSliderLimits(Rangeslider_x_range, minX, maxX, shifter_config.PosXMin, shifter_config.PosXMax,
                    out int clampedMin, out int clampedMax);
                shifter_config.PosXMin = clampedMin;
                shifter_config.PosXMax = clampedMax;
            }
            if (hasY)
            {
                ApplyRangeSliderLimits(Rangeslider_y_range, minY, maxY, shifter_config.PosYMin, shifter_config.PosYMax,
                    out int clampedMin, out int clampedMax);
                shifter_config.PosYMin = clampedMin;
                shifter_config.PosYMax = clampedMax;
            }
            isUpdating = wasUpdating;
            UpdateRangeLabels();
            BuildPreview();
        }

        private static void ApplyRangeSliderLimits(RangeSlider slider, double minValue, double maxValue, int configMin, int configMax,
            out int clampedMinValue, out int clampedMaxValue)
        {
            if (maxValue < minValue)
            {
                double temp = minValue;
                minValue = maxValue;
                maxValue = temp;
            }

            slider.Minimum = minValue;
            slider.Maximum = maxValue;

            double clampedMin = Math.Max(minValue, Math.Min(maxValue, configMin));
            double clampedMax = Math.Max(minValue, Math.Min(maxValue, configMax));
            if (clampedMin > clampedMax)
            {
                double temp = clampedMin;
                clampedMin = clampedMax;
                clampedMax = temp;
            }

            clampedMinValue = Convert.ToInt32(Math.Round(clampedMin, MidpointRounding.AwayFromZero));
            clampedMaxValue = Convert.ToInt32(Math.Round(clampedMax, MidpointRounding.AwayFromZero));
            slider.LowerValue = clampedMinValue;
            slider.UpperValue = clampedMaxValue;
        }

        private void UpdateLinkedAxes()
        {
            function_config.Base.LinkedAxes.Clear();
            if (shifter_config.Sequential)
            {
                function_config.Base.LinkedAxes.Add(axisY);
                function_config.Base.LinkedAxes.Add(AxisID.AxisUndefined);
            }
            else
            {
                function_config.Base.LinkedAxes.Add(axisX);
                function_config.Base.LinkedAxes.Add(axisY);
            }
            function_config.Base.LinkedAxes.Add(AxisID.AxisUndefined);
            function_config.Base.LinkedAxes.Add(AxisID.AxisUndefined);

            EnsureDetectConfig();
            function_config.AuxFunction.LinkedAxes.Clear();
            function_config.AuxFunction.LinkedAxes.Add(axisX);
            function_config.AuxFunction.LinkedAxes.Add(axisY);
            function_config.AuxFunction.LinkedAxes.Add(AxisID.AxisUndefined);
            function_config.AuxFunction.LinkedAxes.Add(AxisID.AxisUndefined);
        }

        private void UpdateOutputRange()
        {
            if (shifter_config.Sequential)
            {
                function_config.Base.OutputMin = shifter_config.PosYMin;
                function_config.Base.OutputMax = shifter_config.PosYMax;
            }
            else
            {
                function_config.Base.OutputMin = shifter_config.PosXMin;
                function_config.Base.OutputMax = shifter_config.PosXMax;
            }
        }

        private void UpdateRangeLabels()
        {
            if (Label_x_min == null || Label_x_max == null || Label_y_min == null || Label_y_max == null)
            {
                return;
            }
            Label_x_min.Content = string.Format(CultureInfo.CurrentCulture, "Min\n{0}mm", shifter_config.PosXMin);
            Label_x_max.Content = string.Format(CultureInfo.CurrentCulture, "Max\n{0}mm", shifter_config.PosXMax);
            Label_y_min.Content = string.Format(CultureInfo.CurrentCulture, "Min\n{0}mm", shifter_config.PosYMin);
            Label_y_max.Content = string.Format(CultureInfo.CurrentCulture, "Max\n{0}mm", shifter_config.PosYMax);
        }

        private static ShifterGear ClampGear(ShifterGear gear)
        {
            int clamped = Math.Max((int)GearNeutral, Math.Min((int)GearSeqDown, (int)gear));
            return (ShifterGear)clamped;
        }

        private static bool IsSequentialGear(ShifterGear gear)
        {
            return gear == GearSeqUp || gear == GearSeqDown;
        }

        private static string GearLabel(ShifterGear gear)
        {
            if (gear == GearNeutral) return "N";
            int gearValue = (int)gear;
            if (gearValue >= (int)GearFirst && gearValue <= (int)GearMax)
            {
                return gearValue.ToString(CultureInfo.CurrentCulture);
            }
            if (gear == GearReverse) return "R";
            if (gear == GearSeqUp) return "Up";
            if (gear == GearSeqDown) return "Down";
            return "?";
        }

        private void RefreshGearOptions()
        {
            gearOptions.Clear();
            gearOptions.Add(new GearOption(GearNeutral, "Neutral"));
            for (int gear = (int)GearFirst; gear <= (int)GearMax; gear++)
            {
                gearOptions.Add(new GearOption((ShifterGear)gear, $"Gear {gear}"));
            }
            gearOptions.Add(new GearOption(GearReverse, "Reverse"));
            gearOptions.Add(new GearOption(GearSeqUp, "Seq Up"));
            gearOptions.Add(new GearOption(GearSeqDown, "Seq Down"));
        }

        private double RoundGeometryCoordinate(double value)
        {
            if (useRelativeGeometry)
            {
                return Math.Round(value, 3, MidpointRounding.AwayFromZero);
            }
            return RoundTenthMillimeters(value);
        }

        private double RoundGeometryLength(double value)
        {
            if (useRelativeGeometry)
            {
                return Math.Round(value, 3, MidpointRounding.AwayFromZero);
            }
            return RoundTenthMillimeters(value);
        }

        private double RangeXMin => Math.Min(shifter_config.PosXMin, shifter_config.PosXMax);
        private double RangeXMax => Math.Max(shifter_config.PosXMin, shifter_config.PosXMax);
        private double RangeYMin => Math.Min(shifter_config.PosYMin, shifter_config.PosYMax);
        private double RangeYMax => Math.Max(shifter_config.PosYMin, shifter_config.PosYMax);
        private double RangeXHalf => Math.Max(1e-6, (RangeXMax - RangeXMin) * 0.5);
        private double RangeYHalf => Math.Max(1e-6, (RangeYMax - RangeYMin) * 0.5);
        private double RangeMinHalf => Math.Max(1e-6, Math.Min(RangeXHalf, RangeYHalf));
        private double RangeXCenter => (RangeXMin + RangeXMax) * 0.5;
        private double RangeYCenter => (RangeYMin + RangeYMax) * 0.5;

        private double ToRelativeX(double mm) => (mm - RangeXCenter) / RangeXHalf;
        private double ToRelativeY(double mm) => (mm - RangeYCenter) / RangeYHalf;
        private double FromRelativeX(double rel) => RangeXCenter + (rel * RangeXHalf);
        private double FromRelativeY(double rel) => RangeYCenter + (rel * RangeYHalf);
        private double ToRelativeXLength(double mm) => mm / RangeXHalf;
        private double ToRelativeYLength(double mm) => mm / RangeYHalf;
        private double ToRelativeMinLength(double mm) => mm / RangeMinHalf;
        private double FromRelativeXLength(double rel) => rel * RangeXHalf;
        private double FromRelativeYLength(double rel) => rel * RangeYHalf;
        private double FromRelativeMinLength(double rel) => rel * RangeMinHalf;

        private double ToDisplayX(double mm) => useRelativeGeometry ? ToRelativeX(mm) : mm;
        private double ToDisplayY(double mm) => useRelativeGeometry ? ToRelativeY(mm) : mm;
        private double FromDisplayX(double value) => useRelativeGeometry ? FromRelativeX(value) : value;
        private double FromDisplayY(double value) => useRelativeGeometry ? FromRelativeY(value) : value;
        private double ToDisplayXLength(double mm) => useRelativeGeometry ? ToRelativeXLength(mm) : mm;
        private double ToDisplayYLength(double mm) => useRelativeGeometry ? ToRelativeYLength(mm) : mm;
        private double ToDisplayMinLength(double mm) => useRelativeGeometry ? ToRelativeMinLength(mm) : mm;
        private double FromDisplayXLength(double value) => useRelativeGeometry ? FromRelativeXLength(value) : value;
        private double FromDisplayYLength(double value) => useRelativeGeometry ? FromRelativeYLength(value) : value;
        private double FromDisplayMinLength(double value) => useRelativeGeometry ? FromRelativeMinLength(value) : value;

        private void ApplyRowRounding(GateSegmentRow row)
        {
            row.RoundCoordinate = RoundGeometryCoordinate;
            row.RoundLength = RoundGeometryLength;
        }

        private void ApplyRowRounding(DetentRow row)
        {
            row.RoundCoordinate = RoundGeometryCoordinate;
            row.RoundLength = RoundGeometryLength;
        }

        private void ApplyRowRounding(SlotRow row)
        {
            row.RoundCoordinate = RoundGeometryCoordinate;
            row.RoundLength = RoundGeometryLength;
        }

        private GateSegmentRow CreateGateRow(double x0, double y0, double x1, double y1, double halfWidth, double springCenter, double springWall)
        {
            var row = new GateSegmentRow
            {
                RoundCoordinate = RoundGeometryCoordinate,
                RoundLength = RoundGeometryLength,
                X0 = ToDisplayX(x0),
                Y0 = ToDisplayY(y0),
                X1 = ToDisplayX(x1),
                Y1 = ToDisplayY(y1),
                HalfWidth = ToDisplayMinLength(halfWidth),
                SpringCenter = springCenter,
                SpringWall = springWall
            };
            return row;
        }

        private DetentRow CreateDetentRow(double x, double y, double radius, double spring)
        {
            var row = new DetentRow
            {
                RoundCoordinate = RoundGeometryCoordinate,
                RoundLength = RoundGeometryLength,
                X = ToDisplayX(x),
                Y = ToDisplayY(y),
                Radius = ToDisplayMinLength(radius),
                Spring = spring
            };
            return row;
        }

        private SlotRow CreateSlotRow(double centerX, double centerY, double halfWidth, double halfHeight, ShifterGear gearCode)
        {
            var row = new SlotRow
            {
                RoundCoordinate = RoundGeometryCoordinate,
                RoundLength = RoundGeometryLength,
                CenterX = ToDisplayX(centerX),
                CenterY = ToDisplayY(centerY),
                HalfWidth = ToDisplayXLength(halfWidth),
                HalfHeight = ToDisplayYLength(halfHeight),
                GearCode = gearCode
            };
            return row;
        }

        private void LoadRowsFromConfig()
        {
            gateRows.Clear();
            foreach (var segment in shifter_config.GateSegments)
            {
                gateRows.Add(CreateGateRow(
                    FromTenthMillimeters(segment.X0),
                    FromTenthMillimeters(segment.Y0),
                    FromTenthMillimeters(segment.X1),
                    FromTenthMillimeters(segment.Y1),
                    FromTenthMillimetersUnsigned(segment.HalfWidth),
                    segment.SpringCenter,
                    segment.SpringWall));
            }
            detentRows.Clear();
            foreach (var detent in shifter_config.Detents)
            {
                detentRows.Add(CreateDetentRow(
                    FromTenthMillimeters(detent.X),
                    FromTenthMillimeters(detent.Y),
                    FromTenthMillimetersUnsigned(detent.Radius),
                    detent.Spring));
            }
            slotRows.Clear();
            if (detect_config != null)
            {
                foreach (var slot in detect_config.GearSlots)
                {
                    slotRows.Add(CreateSlotRow(
                        FromTenthMillimeters(slot.CenterX),
                        FromTenthMillimeters(slot.CenterY),
                        FromTenthMillimetersUnsigned(slot.HalfWidth),
                        FromTenthMillimetersUnsigned(slot.HalfHeight),
                        slot.Gear));
                }
                TextHysteresis.Text = FromTenthMillimetersUnsigned(detect_config.Hysteresis).ToString("0.##", CultureInfo.CurrentCulture);
            }
            BuildPreview();
        }

        private void GateRows_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.OldItems != null)
            {
                foreach (GateSegmentRow row in e.OldItems)
                {
                    row.PropertyChanged -= OnGateRowPropertyChanged;
                }
            }
            if (e.NewItems != null)
            {
                foreach (GateSegmentRow row in e.NewItems)
                {
                    ApplyRowRounding(row);
                    row.PropertyChanged += OnGateRowPropertyChanged;
                }
            }
            if (isUpdating) return;
            SyncGateSegments();
        }

        private void DetentRows_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.OldItems != null)
            {
                foreach (DetentRow row in e.OldItems)
                {
                    row.PropertyChanged -= OnDetentRowPropertyChanged;
                }
            }
            if (e.NewItems != null)
            {
                foreach (DetentRow row in e.NewItems)
                {
                    ApplyRowRounding(row);
                    row.PropertyChanged += OnDetentRowPropertyChanged;
                }
            }
            if (isUpdating) return;
            SyncDetents();
        }

        private void SlotRows_CollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            if (e.OldItems != null)
            {
                foreach (SlotRow row in e.OldItems)
                {
                    row.PropertyChanged -= OnSlotRowPropertyChanged;
                }
            }
            if (e.NewItems != null)
            {
                foreach (SlotRow row in e.NewItems)
                {
                    ApplyRowRounding(row);
                    row.PropertyChanged += OnSlotRowPropertyChanged;
                }
            }
            if (isUpdating) return;
            SyncSlots();
        }

        private void OnGateRowPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (isUpdating) return;
            SyncGateSegments();
        }

        private void OnDetentRowPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (isUpdating) return;
            SyncDetents();
        }

        private void OnSlotRowPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (isUpdating) return;
            SyncSlots();
        }

        private void SyncGateSegments(bool notify = true, bool rebuildPreview = true)
        {
            shifter_config.GateSegments.Clear();
            foreach (var row in gateRows)
            {
                shifter_config.GateSegments.Add(new ShifterGateSegment
                {
                    X0 = ToTenthMillimeters(FromDisplayX(row.X0)),
                    Y0 = ToTenthMillimeters(FromDisplayY(row.Y0)),
                    X1 = ToTenthMillimeters(FromDisplayX(row.X1)),
                    Y1 = ToTenthMillimeters(FromDisplayY(row.Y1)),
                    HalfWidth = ToTenthMillimetersUnsigned(FromDisplayMinLength(row.HalfWidth)),
                    SpringCenter = (float)row.SpringCenter,
                    SpringWall = (float)row.SpringWall
                });
            }
            if (notify)
            {
                function?.OnAxisUpdate();
            }
            if (rebuildPreview)
            {
                BuildPreview();
            }
        }

        private void SyncDetents(bool notify = true, bool rebuildPreview = true)
        {
            shifter_config.Detents.Clear();
            foreach (var row in detentRows)
            {
                shifter_config.Detents.Add(new ShifterDetentPoint
                {
                    X = ToTenthMillimeters(FromDisplayX(row.X)),
                    Y = ToTenthMillimeters(FromDisplayY(row.Y)),
                    Radius = ToTenthMillimetersUnsigned(FromDisplayMinLength(row.Radius)),
                    Spring = (float)row.Spring
                });
            }
            if (notify)
            {
                function?.OnAxisUpdate();
            }
            if (rebuildPreview)
            {
                BuildPreview();
            }
        }

        private void SyncSlots(bool notify = true, bool rebuildPreview = true)
        {
            EnsureDetectConfig();
            detect_config.GearSlots.Clear();
            foreach (var row in slotRows)
            {
                if (!shifter_config.Sequential && IsSequentialGear(row.GearCode))
                {
                    continue;
                }
                detect_config.GearSlots.Add(new ShifterGearSlot
                {
                    CenterX = ToTenthMillimeters(FromDisplayX(row.CenterX)),
                    CenterY = ToTenthMillimeters(FromDisplayY(row.CenterY)),
                    HalfWidth = ToTenthMillimetersUnsigned(FromDisplayXLength(row.HalfWidth)),
                    HalfHeight = ToTenthMillimetersUnsigned(FromDisplayYLength(row.HalfHeight)),
                    Gear = row.GearCode
                });
            }
            if (notify)
            {
                function?.OnAxisUpdate();
            }
            if (rebuildPreview)
            {
                BuildPreview();
            }
        }

        private void OnAxisXChanged(object sender, AxisSelector.AxisIDChangedEventArgs e)
        {
            axisX = e.Value;
            if (isUpdating) return;
            hasAxisXState = false;
            UpdateLinkedAxes();
            ApplyAxisRangeLimits();
            function?.OnAxisUpdate();
        }

        private void OnAxisYChanged(object sender, AxisSelector.AxisIDChangedEventArgs e)
        {
            axisY = e.Value;
            if (isUpdating) return;
            hasAxisYState = false;
            UpdateLinkedAxes();
            ApplyAxisRangeLimits();
            function?.OnAxisUpdate();
        }

        private void Checkbox_sequential_Checked(object sender, RoutedEventArgs e)
        {
            if (isUpdating) return;
            shifter_config.Sequential = Checkbox_sequential.IsChecked == true;
            UpdateLinkedAxes();
            UpdateSequentialUI();
            UpdateOutputRange();
            SyncSlots();
            function?.OnAxisUpdate();
        }

        private void Checkbox_relative_geometry_Checked(object sender, RoutedEventArgs e)
        {
            if (isUpdating) return;
            useRelativeGeometry = Checkbox_relative_geometry.IsChecked == true;
            isUpdating = true;
            LoadRowsFromConfig();
            isUpdating = false;
            BuildPreview();
        }

        private void Rangeslider_x_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!isUpdating)
            {
                shifter_config.PosXMin = Convert.ToInt32(e.NewValue);
                UpdateOutputRange();
            }
            UpdateRangeLabels();
            if (!isUpdating && useRelativeGeometry)
            {
                SyncGateSegments(false, false);
                SyncDetents(false, false);
                SyncSlots(false, false);
                function?.OnAxisUpdate();
                BuildPreview();
                return;
            }
            BuildPreview();
        }

        private void Rangeslider_x_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!isUpdating)
            {
                shifter_config.PosXMax = Convert.ToInt32(e.NewValue);
                UpdateOutputRange();
            }
            UpdateRangeLabels();
            if (!isUpdating && useRelativeGeometry)
            {
                SyncGateSegments(false, false);
                SyncDetents(false, false);
                SyncSlots(false, false);
                function?.OnAxisUpdate();
                BuildPreview();
                return;
            }
            BuildPreview();
        }

        private void Rangeslider_y_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!isUpdating)
            {
                shifter_config.PosYMin = Convert.ToInt32(e.NewValue);
                UpdateOutputRange();
            }
            UpdateRangeLabels();
            if (!isUpdating && useRelativeGeometry)
            {
                SyncGateSegments(false, false);
                SyncDetents(false, false);
                SyncSlots(false, false);
                function?.OnAxisUpdate();
                BuildPreview();
                return;
            }
            BuildPreview();
        }

        private void Rangeslider_y_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!isUpdating)
            {
                shifter_config.PosYMax = Convert.ToInt32(e.NewValue);
                UpdateOutputRange();
            }
            UpdateRangeLabels();
            if (!isUpdating && useRelativeGeometry)
            {
                SyncGateSegments(false, false);
                SyncDetents(false, false);
                SyncSlots(false, false);
                function?.OnAxisUpdate();
                BuildPreview();
                return;
            }
            BuildPreview();
        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            shifter_config.Damping = (float)e.NewValue;
            label_damping.Content = string.Format(CultureInfo.CurrentCulture, "Damping: {0:F3}N*mm/s", e.NewValue);
        }

        private void OnFrictionChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            function_config.Friction = (float)e.NewValue;
            label_friction.Content = string.Format(CultureInfo.CurrentCulture, "Friction: {0:F1}N", e.NewValue);
        }

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            function_config.SimulatedMass = (float)e.NewValue;
            label_simulated_mass.Content = string.Format(CultureInfo.CurrentCulture, "Simulated Mass: {0:F2}kg", e.NewValue);
        }

        private void TextMaxForce_LostFocus(object sender, RoutedEventArgs e)
        {
            if (TryParseDouble(TextMaxForce.Text, out double value))
            {
                value = Math.Max(0.0, value);
                shifter_config.MaxForce = (float)value;
            }
            TextMaxForce.Text = shifter_config.MaxForce.ToString("0.##", CultureInfo.CurrentCulture);
        }

        private void TextGridStep_LostFocus(object sender, RoutedEventArgs e)
        {
            if (TryParseDouble(TextGridStep.Text, out double value))
            {
                value = Math.Max(0.1, value);
                shifter_config.GridStep = ToTenthMillimetersUnsigned(value);
            }
            TextGridStep.Text = FromTenthMillimetersUnsigned(shifter_config.GridStep).ToString("0.##", CultureInfo.CurrentCulture);
            BuildPreview();
        }

        private void btn_add_gate_Click(object sender, RoutedEventArgs e)
        {
            gateRows.Add(CreateGateRow(
                0.0,
                -20.0,
                0.0,
                20.0,
                5.0,
                1.0,
                5.0));
            BuildPreview();
        }

        private void btn_remove_gate_Click(object sender, RoutedEventArgs e)
        {
            if (GateGrid.SelectedItem is GateSegmentRow row)
            {
                gateRows.Remove(row);
            }
            BuildPreview();
        }

        private void btn_add_detent_Click(object sender, RoutedEventArgs e)
        {
            detentRows.Add(CreateDetentRow(
                0.0,
                0.0,
                5.0,
                5.0));
            BuildPreview();
        }

        private void btn_remove_detent_Click(object sender, RoutedEventArgs e)
        {
            if (DetentGrid.SelectedItem is DetentRow row)
            {
                detentRows.Remove(row);
            }
            BuildPreview();
        }

        private void btn_add_slot_Click(object sender, RoutedEventArgs e)
        {
            slotRows.Add(CreateSlotRow(
                0.0,
                0.0,
                5.0,
                5.0,
                GearFirst));
            BuildPreview();
        }

        private void btn_remove_slot_Click(object sender, RoutedEventArgs e)
        {
            if (SlotGrid.SelectedItem is SlotRow row)
            {
                slotRows.Remove(row);
            }
            BuildPreview();
        }

        private static double RoundTenthMillimeters(double value)
        {
            return Math.Round(value, 1, MidpointRounding.AwayFromZero);
        }

        private void TextHysteresis_LostFocus(object sender, RoutedEventArgs e)
        {
            EnsureDetectConfig();
            if (TryParseDouble(TextHysteresis.Text, out double value))
            {
                value = Math.Max(0.0, value);
                detect_config.Hysteresis = ToTenthMillimetersUnsigned(value);
            }
            TextHysteresis.Text = FromTenthMillimetersUnsigned(detect_config.Hysteresis).ToString("0.##", CultureInfo.CurrentCulture);
        }

        private static int ToTenthMillimeters(double value)
        {
            return (int)Math.Round(value * 10.0, MidpointRounding.AwayFromZero);
        }

        private static uint ToTenthMillimetersUnsigned(double value)
        {
            int scaled = (int)Math.Round(value * 10.0, MidpointRounding.AwayFromZero);
            return (uint)Math.Max(0, scaled);
        }

        private static double FromTenthMillimeters(int value)
        {
            return value / 10.0;
        }

        private static double FromTenthMillimetersUnsigned(uint value)
        {
            return value / 10.0;
        }

        private static bool TryParseDouble(string text, out double value)
        {
            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
        }

        private void EnsureDetectConfig()
        {
            if (function_config.AuxFunction == null)
            {
                function_config.AuxFunction = GetDefaultDetectConfig();
            }
            else if (function_config.AuxFunction.ShifterDetect == null)
            {
                function_config.AuxFunction.ShifterDetect = new ShifterDetectConfig();
                if (function_config.AuxFunction.LinkedAxes.Count == 0)
                {
                    function_config.AuxFunction.LinkedAxes.AddRange(new AxisID[4]
                    {
                        AxisID.AxisUndefined,
                        AxisID.AxisUndefined,
                        AxisID.AxisUndefined,
                        AxisID.AxisUndefined
                    });
                }
            }
            detect_config = function_config.AuxFunction.ShifterDetect;
        }

        private void CanvasShifterPreview_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            BuildPreview();
        }

        private void BuildPreview()
        {
            if (canvas_shifter_preview == null) return;
            canvas_shifter_preview.Children.Clear();
            if (shifter_config == null) return;
            AutoFitPreview();
            DrawPreviewGrid();
            DrawPreviewRange();
            DrawPreviewSlots();
            DrawPreviewGateSegments();
            DrawPreviewDetents();
            DrawPreviewCurrentPosition();
        }

        private bool TryGetCurrentPosition(out double x, out double y)
        {
            x = 0.0;
            y = 0.0;
            if (shifter_config.Sequential)
            {
                if (!hasAxisYState)
                {
                    return false;
                }
                y = lastAxisYPosition;
                x = hasAxisXState ? lastAxisXPosition : RangeXCenter;
            }
            else
            {
                if (!hasAxisXState || !hasAxisYState)
                {
                    return false;
                }
                x = lastAxisXPosition;
                y = lastAxisYPosition;
            }
            x = Math.Max(RangeXMin, Math.Min(RangeXMax, x));
            y = Math.Max(RangeYMin, Math.Min(RangeYMax, y));
            return true;
        }

        private void AutoFitPreview()
        {
            if (!TryGetPreviewBounds(out double minX, out double maxX, out double minY, out double maxY)) return;
            double width = canvas_shifter_preview.ActualWidth;
            double height = canvas_shifter_preview.ActualHeight;
            if (width <= 1.0 || height <= 1.0)
            {
                width = canvas_shifter_preview.Width;
                height = canvas_shifter_preview.Height;
            }
            if (width <= 1.0 || height <= 1.0)
            {
                width = 400.0;
                height = 240.0;
            }
            double rangeX = Math.Max(1.0, maxX - minX);
            double rangeY = Math.Max(1.0, maxY - minY);
            double usableWidth = Math.Max(1.0, width - 2.0 * PreviewPadding);
            double usableHeight = Math.Max(1.0, height - 2.0 * PreviewPadding);
            previewScale = Math.Min(usableWidth / rangeX, usableHeight / rangeY);
            if (double.IsNaN(previewScale) || previewScale <= 0.0)
            {
                previewScale = 1.0;
            }
            double centerX = (minX + maxX) * 0.5;
            double centerY = (minY + maxY) * 0.5;
            previewOffsetX = width * 0.5 - centerX * previewScale;
            previewOffsetY = height * 0.5 + centerY * previewScale;
        }

        private bool TryGetPreviewBounds(out double minX, out double maxX, out double minY, out double maxY)
        {
            minX = Math.Min(shifter_config.PosXMin, shifter_config.PosXMax);
            maxX = Math.Max(shifter_config.PosXMin, shifter_config.PosXMax);
            minY = Math.Min(shifter_config.PosYMin, shifter_config.PosYMax);
            maxY = Math.Max(shifter_config.PosYMin, shifter_config.PosYMax);

            foreach (var row in gateRows)
            {
                double x0 = FromDisplayX(row.X0);
                double x1 = FromDisplayX(row.X1);
                double y0 = FromDisplayY(row.Y0);
                double y1 = FromDisplayY(row.Y1);
                double half = Math.Max(0.0, FromDisplayMinLength(row.HalfWidth));
                double segMinX = Math.Min(x0, x1) - half;
                double segMaxX = Math.Max(x0, x1) + half;
                double segMinY = Math.Min(y0, y1) - half;
                double segMaxY = Math.Max(y0, y1) + half;
                minX = Math.Min(minX, segMinX);
                maxX = Math.Max(maxX, segMaxX);
                minY = Math.Min(minY, segMinY);
                maxY = Math.Max(maxY, segMaxY);
            }

            foreach (var row in detentRows)
            {
                double x = FromDisplayX(row.X);
                double y = FromDisplayY(row.Y);
                double r = Math.Max(0.0, FromDisplayMinLength(row.Radius));
                minX = Math.Min(minX, x - r);
                maxX = Math.Max(maxX, x + r);
                minY = Math.Min(minY, y - r);
                maxY = Math.Max(maxY, y + r);
            }

            foreach (var row in slotRows)
            {
                if (!shifter_config.Sequential && IsSequentialGear(row.GearCode))
                {
                    continue;
                }
                double centerX = FromDisplayX(row.CenterX);
                double centerY = FromDisplayY(row.CenterY);
                double halfW = Math.Max(0.0, FromDisplayXLength(row.HalfWidth));
                double halfH = Math.Max(0.0, FromDisplayYLength(row.HalfHeight));
                minX = Math.Min(minX, centerX - halfW);
                maxX = Math.Max(maxX, centerX + halfW);
                minY = Math.Min(minY, centerY - halfH);
                maxY = Math.Max(maxY, centerY + halfH);
            }

            if (Math.Abs(maxX - minX) < 1e-6)
            {
                maxX = minX + 1.0;
                minX -= 1.0;
            }
            if (Math.Abs(maxY - minY) < 1e-6)
            {
                maxY = minY + 1.0;
                minY -= 1.0;
            }
            return true;
        }

        private void DrawPreviewGrid()
        {
            double step = FromTenthMillimetersUnsigned(shifter_config.GridStep);
            if (step <= 0.0) step = 10.0;
            double spacing = step * previewScale;
            const double minSpacing = 14.0;
            const double maxSpacing = 90.0;
            while (spacing < minSpacing) spacing *= 2.0;
            while (spacing > maxSpacing) spacing *= 0.5;
            if (spacing <= 0.0) return;

            double width = canvas_shifter_preview.ActualWidth;
            double height = canvas_shifter_preview.ActualHeight;
            if (width <= 1.0 || height <= 1.0) return;

            var origin = ToPreviewCanvas(0.0, 0.0);
            double ox = origin.X % spacing;
            double oy = origin.Y % spacing;
            if (ox < 0) ox += spacing;
            if (oy < 0) oy += spacing;

            var gridBrush = new SolidColorBrush(Color.FromArgb(35, 180, 200, 230));
            for (double x = ox; x <= width; x += spacing)
            {
                var line = new Line
                {
                    X1 = x,
                    Y1 = 0,
                    X2 = x,
                    Y2 = height,
                    Stroke = gridBrush,
                    StrokeThickness = 1
                };
                canvas_shifter_preview.Children.Add(line);
            }

            for (double y = oy; y <= height; y += spacing)
            {
                var line = new Line
                {
                    X1 = 0,
                    Y1 = y,
                    X2 = width,
                    Y2 = y,
                    Stroke = gridBrush,
                    StrokeThickness = 1
                };
                canvas_shifter_preview.Children.Add(line);
            }

            var axisBrush = new SolidColorBrush(Color.FromArgb(120, 180, 210, 240));
            var xAxis = new Line
            {
                X1 = 0,
                Y1 = origin.Y,
                X2 = width,
                Y2 = origin.Y,
                Stroke = axisBrush,
                StrokeThickness = 2
            };
            var yAxis = new Line
            {
                X1 = origin.X,
                Y1 = 0,
                X2 = origin.X,
                Y2 = height,
                Stroke = axisBrush,
                StrokeThickness = 2
            };
            canvas_shifter_preview.Children.Add(xAxis);
            canvas_shifter_preview.Children.Add(yAxis);
        }

        private void DrawPreviewRange()
        {
            double minX = Math.Min(shifter_config.PosXMin, shifter_config.PosXMax);
            double maxX = Math.Max(shifter_config.PosXMin, shifter_config.PosXMax);
            double minY = Math.Min(shifter_config.PosYMin, shifter_config.PosYMax);
            double maxY = Math.Max(shifter_config.PosYMin, shifter_config.PosYMax);
            var topLeft = ToPreviewCanvas(minX, maxY);
            var bottomRight = ToPreviewCanvas(maxX, minY);
            double left = Math.Min(topLeft.X, bottomRight.X);
            double top = Math.Min(topLeft.Y, bottomRight.Y);
            double width = Math.Abs(bottomRight.X - topLeft.X);
            double height = Math.Abs(bottomRight.Y - topLeft.Y);
            if (width < 1.0 || height < 1.0) return;

            var rect = new Rectangle
            {
                Width = width,
                Height = height,
                Stroke = new SolidColorBrush(Color.FromArgb(120, 120, 170, 210)),
                StrokeThickness = 1,
                StrokeDashArray = new DoubleCollection { 4, 4 },
                Fill = new SolidColorBrush(Color.FromArgb(18, 120, 170, 210))
            };
            Canvas.SetLeft(rect, left);
            Canvas.SetTop(rect, top);
            canvas_shifter_preview.Children.Add(rect);
        }

        private void DrawPreviewSlots()
        {
            var fill = new SolidColorBrush(Color.FromArgb(35, 130, 210, 140));
            var stroke = new SolidColorBrush(Color.FromArgb(160, 130, 210, 140));
            foreach (var row in slotRows)
            {
                if (!shifter_config.Sequential && IsSequentialGear(row.GearCode))
                {
                    continue;
                }
                double centerX = FromDisplayX(row.CenterX);
                double centerY = FromDisplayY(row.CenterY);
                double halfW = FromDisplayXLength(row.HalfWidth);
                double halfH = FromDisplayYLength(row.HalfHeight);
                double minX = centerX - halfW;
                double maxX = centerX + halfW;
                double minY = centerY - halfH;
                double maxY = centerY + halfH;
                var topLeft = ToPreviewCanvas(minX, maxY);
                var bottomRight = ToPreviewCanvas(maxX, minY);
                double left = Math.Min(topLeft.X, bottomRight.X);
                double top = Math.Min(topLeft.Y, bottomRight.Y);
                double width = Math.Abs(bottomRight.X - topLeft.X);
                double height = Math.Abs(bottomRight.Y - topLeft.Y);
                if (width < 1.0 || height < 1.0) continue;

                var rect = new Rectangle
                {
                    Width = width,
                    Height = height,
                    Stroke = stroke,
                    StrokeThickness = 1.5,
                    StrokeDashArray = new DoubleCollection { 3, 3 },
                    Fill = fill
                };
                Canvas.SetLeft(rect, left);
                Canvas.SetTop(rect, top);
                canvas_shifter_preview.Children.Add(rect);

                var center = ToPreviewCanvas(centerX, centerY);
                var label = new TextBlock
                {
                    Text = GearLabel(row.GearCode),
                    Foreground = Brushes.White,
                    FontFamily = new FontFamily("Arial"),
                    FontSize = 10,
                    Background = new SolidColorBrush(Color.FromArgb(140, 20, 25, 35)),
                    Padding = new Thickness(2, 0, 2, 0)
                };
                AddCenteredLabel(label, center);
                canvas_shifter_preview.Children.Add(label);
            }
        }

        private void DrawPreviewGateSegments()
        {
            var corridorBrush = new SolidColorBrush(Color.FromArgb(70, 90, 200, 255));
            var centerBrush = new SolidColorBrush(Color.FromArgb(200, 90, 200, 255));
            foreach (var row in gateRows)
            {
                double x0 = FromDisplayX(row.X0);
                double y0 = FromDisplayY(row.Y0);
                double x1 = FromDisplayX(row.X1);
                double y1 = FromDisplayY(row.Y1);
                double halfWidth = FromDisplayMinLength(row.HalfWidth);
                var start = ToPreviewCanvas(x0, y0);
                var end = ToPreviewCanvas(x1, y1);
                double width = Math.Max(2.0, halfWidth * 2.0 * previewScale);
                var corridor = new Line
                {
                    X1 = start.X,
                    Y1 = start.Y,
                    X2 = end.X,
                    Y2 = end.Y,
                    Stroke = corridorBrush,
                    StrokeThickness = width,
                    StrokeStartLineCap = PenLineCap.Round,
                    StrokeEndLineCap = PenLineCap.Round
                };
                var centerLine = new Line
                {
                    X1 = start.X,
                    Y1 = start.Y,
                    X2 = end.X,
                    Y2 = end.Y,
                    Stroke = centerBrush,
                    StrokeThickness = 2.0,
                    StrokeStartLineCap = PenLineCap.Round,
                    StrokeEndLineCap = PenLineCap.Round
                };
                canvas_shifter_preview.Children.Add(corridor);
                canvas_shifter_preview.Children.Add(centerLine);
            }
        }

        private void DrawPreviewDetents()
        {
            var stroke = new SolidColorBrush(Color.FromArgb(210, 255, 200, 120));
            var fill = new SolidColorBrush(Color.FromArgb(40, 255, 200, 120));
            foreach (var row in detentRows)
            {
                double x = FromDisplayX(row.X);
                double y = FromDisplayY(row.Y);
                double radius = Math.Max(1.5, FromDisplayMinLength(row.Radius) * previewScale);
                var center = ToPreviewCanvas(x, y);
                var circle = new Ellipse
                {
                    Width = radius * 2.0,
                    Height = radius * 2.0,
                    Stroke = stroke,
                    StrokeThickness = 1.5,
                    Fill = fill
                };
                Canvas.SetLeft(circle, center.X - radius);
                Canvas.SetTop(circle, center.Y - radius);
                canvas_shifter_preview.Children.Add(circle);

                var dot = new Ellipse
                {
                    Width = 4,
                    Height = 4,
                    Fill = stroke
                };
                Canvas.SetLeft(dot, center.X - 2);
                Canvas.SetTop(dot, center.Y - 2);
                canvas_shifter_preview.Children.Add(dot);
            }
        }

        private void DrawPreviewCurrentPosition()
        {
            if (!TryGetCurrentPosition(out double x, out double y))
            {
                return;
            }

            var center = ToPreviewCanvas(x, y);
            const double radius = 6.0;
            var marker = new Ellipse
            {
                Width = radius * 2.0,
                Height = radius * 2.0,
                Fill = new SolidColorBrush(Color.FromArgb(210, 255, 210, 85)),
                Stroke = new SolidColorBrush(Color.FromArgb(220, 25, 25, 25)),
                StrokeThickness = 1.5
            };
            Canvas.SetLeft(marker, center.X - radius);
            Canvas.SetTop(marker, center.Y - radius);
            canvas_shifter_preview.Children.Add(marker);

            var ring = new Ellipse
            {
                Width = radius * 2.8,
                Height = radius * 2.8,
                Stroke = new SolidColorBrush(Color.FromArgb(120, 255, 210, 85)),
                StrokeThickness = 1.2
            };
            Canvas.SetLeft(ring, center.X - radius * 1.4);
            Canvas.SetTop(ring, center.Y - radius * 1.4);
            canvas_shifter_preview.Children.Add(ring);
        }

        private Point ToPreviewCanvas(double x, double y)
        {
            return new Point((x * previewScale) + previewOffsetX, previewOffsetY - (y * previewScale));
        }

        private static void AddCenteredLabel(TextBlock label, Point center)
        {
            label.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
            var size = label.DesiredSize;
            Canvas.SetLeft(label, center.X - size.Width / 2.0);
            Canvas.SetTop(label, center.Y - size.Height / 2.0);
        }
    }
}
