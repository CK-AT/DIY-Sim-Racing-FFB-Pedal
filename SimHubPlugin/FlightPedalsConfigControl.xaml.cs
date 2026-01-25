using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using MahApps.Metro.Controls;
using User.PluginSdkDemo.GraphEditor;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for AutomotivePedalEffects.xaml
    /// </summary>
    public partial class FlightPedalsConfigControl : UserControl
    {
        public event FunctionConfigControl.DebugMessageEventHandler DebugMessage;
        public delegate void ABSTestStateChangeEventHandler(bool state);
        public event ABSTestStateChangeEventHandler ABSTestStateChange;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private FlightPedalsConfig config;
        private RudderBrakeConfig brake_config;
        private FunctionConfig function_config = new FunctionConfig();
        private Function function;
        private FunctionID current_function_id;
        bool is_updating = true;
        private double latestAxisPosition;
        private bool hasAxisPosition;
        private double latestAxisForce;
        private bool hasAxisForce;
        private double latestTrimCenter;
        private bool hasTrimCenter;
        private DispatcherTimer xplaneTimer;
        private float lastIasKts;
        private bool hasAxisRange;
        private bool autoTuneLoadGain;
        private DateTime autoTuneLastUpdateUtc = DateTime.MinValue;
        private const double AutoTuneUpdateMs = 250.0;
        private const double AutoTuneRatioLow = 0.25;
        private const double AutoTuneRatioHigh = 0.55;
        private const double AutoTuneGainStep = 0.0005;
        private const double AutoTuneMinForce = 0.05;
        private const double AutoTuneMinVrefRatio = 0.7;
        private Dictionary<string, FrameworkElement> graphParamControls = new Dictionary<string, FrameworkElement>();
        private Dictionary<string, Label> graphParamLabels = new Dictionary<string, Label>();
        private bool isUpdatingGraphParams = false;

        public FlightPedalsConfigControl()
        {
            config = GetDefaultConfig();
            InitializeComponent();
        }
        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;

            if (plugin != null)
            {
                plugin.ActiveGraphChanged += OnActiveGraphChanged;
                plugin.GraphParamChanged += OnGraphParamChanged;
            }

            is_updating = false;
            StartXPlaneTimer();
            RefreshGraphParams();
        }

        private void StartXPlaneTimer()
        {
            if (xplaneTimer != null)
            {
                return;
            }

            xplaneTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(200)
            };
            xplaneTimer.Tick += (sender, args) => UpdateXPlaneTelemetry();
            xplaneTimer.Start();
        }

        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            hasAxisRange = true;
            double min = parameters.ContactPointPosMinAbs / 10.0f;
            double max = parameters.ContactPointPosMaxAbs / 10.0f;
            Rangeslider_travel_range.Minimum = Math.Min(min, max);
            Rangeslider_travel_range.Maximum = Math.Max(min, max);
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (function_config?.Base == null || function_config.Base.LinkedAxes.Count == 0)
            {
                return;
            }

            AxisID primaryAxis = function_config.Base.LinkedAxes[0];
            if (primaryAxis == AxisID.AxisUndefined || (primaryAxis & AxisID.Mask) != axis_state.AxisId)
            {
                return;
            }

            latestAxisPosition = axis_state.Position;
            hasAxisPosition = true;
            latestAxisForce = axis_state.Force;
            hasAxisForce = true;
            UpdateTrimCenter();
            UpdateTravelMarkers();
        }

        public static FlightPedalsConfig GetDefaultConfig()
        {
            FlightPedalsConfig new_config = new FlightPedalsConfig();
            new_config.PosNearLim = 0;
            new_config.PosFarLim = 50;
            new_config.Damping = 0.5f;
            new_config.CenteringSpringConst = 1.5f;
            return new_config;
        }

        public static AuxFunctionConfig GetRudderBrakeDefaultConfig()
        {
            AuxFunctionConfig new_config = new AuxFunctionConfig();
            new_config.LinkedAxes.AddRange(new AxisID[4] { AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined });
            new_config.RudderBrake = new RudderBrakeConfig();
            new_config.RudderBrake.FMax = 150;
            new_config.RudderBrake.FMin = 50;
            return new_config;
        }

        public void SwitchFunction(Function function)
        {
            this.function = function;
            function_config = function.Config;
            config = function_config.FlightPedals;
            brake_config = function_config.AuxFunction.RudderBrake;
            current_function_id = function_config.Base.FunctionId;
            hasAxisRange = false;

            is_updating = true;
            uc_axis_selector_pilot_right.Value = function_config.Base.LinkedAxes[0];
            uc_axis_selector_pilot_left.Value = function_config.Base.LinkedAxes[1] & AxisID.Mask;
            uc_axis_selector_copilot_right.Value = function_config.Base.LinkedAxes[2];
            uc_axis_selector_copilot_left.Value = function_config.Base.LinkedAxes[3] & AxisID.Mask;

            if (function_config.Base.LinkedAxes[0] != AxisID.AxisUndefined)
            {
                var kinematic_parameters = ui.GetKinematicParameters(function_config.Base.LinkedAxes[0]);
                if (kinematic_parameters != null)
                {
                    OnKinematicParametersChanged(kinematic_parameters);
                }
            }
            if (!hasAxisRange)
            {
                ApplyFallbackTravelRange();
            }

            Slider_simulated_mass.Value = function_config.SimulatedMass;
            Slider_friction.Value = function_config.Friction;

            Slider_centering_spring_const.Value = config.CenteringSpringConst;
            Slider_damping.Value = config.Damping;
            uc_controller_axis_pedals.Value = function_config.Base.ControllerOutputAxis;
            uc_controller_axis_right_brake.Value = function_config.AuxFunction.RudderBrake.ControllerOutputAxisRightPedal;
            uc_controller_axis_left_brake.Value = function_config.AuxFunction.RudderBrake.ControllerOutputAxisLeftPedal;
            Rangeslider_travel_range.UpperValue = config.PosFarLim;
            function_config.Base.OutputMax = config.PosFarLim;
            Rangeslider_travel_range.LowerValue = config.PosNearLim;
            function_config.Base.OutputMin = config.PosNearLim;
            Rangeslider_brake_force_range.UpperValue = function_config.AuxFunction.RudderBrake.FMax / 9.81f;
            Rangeslider_brake_force_range.LowerValue = function_config.AuxFunction.RudderBrake.FMin / 9.81f;
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;
            RefreshGraphParams();

        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.Damping = (float)e.NewValue;
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", e.NewValue);
        }

        private void OnCentringSpringChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.CenteringSpringConst = (float)e.NewValue;
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", e.NewValue);
        }

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", e.NewValue);
            function_config.SimulatedMass = (float)e.NewValue;
        }

        private void AutomotivePedal_AxisSelector_AxisIDChanged(object sender, AxisSelector.AxisIDChangedEventArgs e)
        {
            function_config.Base.LinkedAxes.Clear();
            function_config.Base.LinkedAxes.AddRange(new AxisID[4] { e.Value, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined });
            var kinematic_parameters = ui.GetKinematicParameters(e.Value);
            if (kinematic_parameters != null) {
                OnKinematicParametersChanged(kinematic_parameters);
            }
        }

        private void OnAxisIDChanged(object sender, AxisSelector.AxisIDChangedEventArgs e)
        {
            if (!is_updating)
            {
                function_config.Base.OutputMode = OutputMode.Travel;
                function_config.Base.LinkedAxes.Clear();
                function_config.Base.LinkedAxes.Add(uc_axis_selector_pilot_right.Value);
                if (uc_axis_selector_pilot_left.Value != AxisID.AxisUndefined)
                {
                    function_config.Base.LinkedAxes.Add(uc_axis_selector_pilot_left.Value | AxisID.AxisSubtractive);
                }
                else
                {
                    function_config.Base.LinkedAxes.Add(AxisID.AxisUndefined);
                }
                function_config.Base.LinkedAxes.Add(uc_axis_selector_copilot_right.Value);
                if (uc_axis_selector_copilot_left.Value != AxisID.AxisUndefined)
                {
                    function_config.Base.LinkedAxes.Add(uc_axis_selector_copilot_left.Value | AxisID.AxisSubtractive);
                }
                else
                {
                    function_config.Base.LinkedAxes.Add(AxisID.AxisUndefined);
                }

                function_config.AuxFunction.LinkedAxes.Clear();
                function_config.AuxFunction.LinkedAxes.Add(uc_axis_selector_pilot_right.Value);
                function_config.AuxFunction.LinkedAxes.Add(uc_axis_selector_pilot_left.Value);
                function_config.AuxFunction.LinkedAxes.Add(uc_axis_selector_copilot_right.Value);
                function_config.AuxFunction.LinkedAxes.Add(uc_axis_selector_copilot_left.Value);

                var kinematic_parameters = ui.GetKinematicParameters(e.Value);
                if (kinematic_parameters != null)
                {
                    OnKinematicParametersChanged(kinematic_parameters);
                }
                else
                {
                    ApplyFallbackTravelRange();
                }
                function?.OnAxisUpdate();
            }
        }

        private void Rangeslider_brake_force_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                function_config.AuxFunction.RudderBrake.FMin = (float)(e.NewValue * 9.81);
            }
            if (Label_min_brake_force != null)
            {
                Label_min_brake_force.Content = String.Format("Preload\n{0:F1}kg", e.NewValue);
            }
        }

        private void Rangeslider_brake_force_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                function_config.AuxFunction.RudderBrake.FMax = (float)(e.NewValue * 9.81);
            }
            if (Label_max_brake_force != null)
            {
                Label_max_brake_force.Content = String.Format("Max\n{0:F1}kg", e.NewValue);
            }
        }

        private void uc_controller_axis_pedals_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.Base.ControllerOutputAxis = e.Value;
            function_config.AuxFunction.RudderBrake.ControllerOutputAxisFlightPedals = e.Value;
        }

        private void Rangeslider_travel_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.PosNearLim = Convert.ToInt16(e.NewValue);
                function_config.Base.OutputMin = Convert.ToInt16(e.NewValue);
            }
            if (Label_near_pos != null)
            {
                Label_near_pos.Content = String.Format("Near\n{0}mm", config.PosNearLim);
            }
            UpdateTravelMarkers();
        }

        private void Rangeslider_travel_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.PosFarLim = Convert.ToInt16(e.NewValue);
                function_config.Base.OutputMax = Convert.ToInt16(e.NewValue);
            }
            if (Label_far_pos != null)
            {
                Label_far_pos.Content = String.Format("Far\n{0}mm", config.PosFarLim);
            }
            UpdateTravelMarkers();
        }

        private void uc_controller_axis_right_brake_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.AuxFunction.RudderBrake.ControllerOutputAxisRightPedal = e.Value;
        }

        private void uc_controller_axis_left_brake_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.AuxFunction.RudderBrake.ControllerOutputAxisLeftPedal = e.Value;
        }

        private void OnFrictionChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_friction.Content = String.Format("Friction: {0:F1}N", e.NewValue);
            function_config.Friction = (float)e.NewValue;
        }

        private DiyFfbPluginSettings.FunctionSettings GetFunctionSettings()
        {
            if (plugin?.Settings?.function_settings == null)
            {
                return null;
            }

            int index = (int)current_function_id - 1;
            if (index < 0 || index >= plugin.Settings.function_settings.Length)
            {
                return null;
            }

            return plugin.Settings.function_settings[index];
        }

        private void UpdateXPlaneSettingsUi()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateXPlaneVisibility()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateXPlaneLabels()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void OnXPlaneKqChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFfbKq = (float)e.NewValue;
            UpdateXPlaneLabels();
            UpdateGainGraph();
        }

        private void Toggle_xplane_ffb_enabled_Checked(object sender, RoutedEventArgs e)
        {
            SetXPlaneFfbEnabled(true);
        }

        private void Toggle_xplane_ffb_enabled_Unchecked(object sender, RoutedEventArgs e)
        {
            SetXPlaneFfbEnabled(false);
        }

        private void SetXPlaneFfbEnabled(bool enabled)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFfbEnabled = enabled;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneKrateChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFfbKrate = (float)e.NewValue;
            UpdateXPlaneLabels();
            UpdateGainGraph();
        }

        private void OnXPlaneKcenterChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFfbKcenter = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneFrictionQChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFrictionQ = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneFrictionTorqueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFrictionTorque = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneFrictionLowRpmChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneFrictionLowRpm = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneRpmBlendChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneRpmBlend = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneLoadForceClampChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneLoadForceClamp = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneTrimScaleChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneTrimMmPerDeg = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneBuffetStartChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneBuffetStartDeg = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneBuffetFullChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneBuffetFullDeg = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneBuffetGainChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneBuffetGain = (float)e.NewValue;
            UpdateXPlaneLabels();
            UpdateGainGraph();
        }

        private void OnXPlaneWeathervaneGainChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneWeathervaneGain = (float)e.NewValue;
            UpdateXPlaneLabels();
            UpdateGainGraph();
        }

        private void OnXPlaneAeroMomentGainChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating)
            {
                UpdateXPlaneLabels();
                return;
            }

            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            settings.XPlaneAeroMomentGain = (float)e.NewValue;
            UpdateXPlaneLabels();
        }

        private void OnXPlaneTorqueRefChanged(object sender, TextChangedEventArgs e)
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void Toggle_xplane_auto_tune_Checked(object sender, RoutedEventArgs e)
        {
            autoTuneLoadGain = true;
            var settings = GetFunctionSettings();
            if (settings != null)
            {
                settings.XPlaneReferenceFlightMode = true;
            }
        }

        private void Toggle_xplane_auto_tune_Unchecked(object sender, RoutedEventArgs e)
        {
            autoTuneLoadGain = false;
            var settings = GetFunctionSettings();
            if (settings != null)
            {
                settings.XPlaneReferenceFlightMode = false;
            }
        }

        private void OnXPlaneVrefChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateTrimCenter()
        {
            float trimMm = 0.0f;
            hasTrimCenter = plugin != null && plugin.TryGetXPlaneTrimOffset(current_function_id, out trimMm);
            if (hasTrimCenter)
            {
                latestTrimCenter = trimMm;
            }
        }

        private void UpdateXPlaneTelemetry()
        {
            // X-Plane telemetry UI removed from this tab - keep method for compatibility
            if (plugin == null)
            {
                return;
            }

            float iasKts;
            float alphaDeg;
            float betaDeg;
            float elevTrim;
            float ailTrim;
            float rudTrim;
            if (!plugin.TryGetXPlaneTelemetry(out iasKts, out alphaDeg, out betaDeg, out elevTrim, out ailTrim, out rudTrim))
            {
                lastIasKts = 0.0f;
                return;
            }

            lastIasKts = iasKts;
            // X-Plane UI controls removed - telemetry still tracked for backend use

            UpdateFfbOutputs();
        }

        private void UpdateFfbOutputs()
        {
            if (plugin == null)
            {
                return;
            }

            const string prefix = "FlightPedals";
            Label_Output_Spring.Content = plugin.GetGraphOutputValue($"{prefix}.SpringGain").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Damper.Content = plugin.GetGraphOutputValue($"{prefix}.DamperGain").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Friction.Content = plugin.GetGraphOutputValue($"{prefix}.Friction").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Load.Content = plugin.GetGraphOutputValue($"{prefix}.LoadForce").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_TrimOffset.Content = plugin.GetGraphOutputValue($"{prefix}.TrimOffset").ToString("F2", CultureInfo.InvariantCulture);
        }

        private void UpdateGainGraph()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateGainCursor()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateAutoTuneLoadGain(DiyFfbPlugin.XPlaneFfbDiagnostics diagnostics)
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private string FormatTorqueRefNm()
        {
            var settings = GetFunctionSettings();
            if (settings == null || settings.XPlaneTorqueRefNm <= 0.0f)
            {
                return "--";
            }
            return settings.XPlaneTorqueRefNm.ToString("F0");
        }

        private string FormatMainRotorTorqueRefNm()
        {
            if (plugin?.Settings == null || plugin.Settings.XPlaneMainRotorTorqueRefNmSystem <= 0.0f)
            {
                return "--";
            }

            return plugin.Settings.XPlaneMainRotorTorqueRefNmSystem.ToString("F0");
        }

        public void RefreshXPlaneFfbSettings()
        {
            // X-Plane UI controls removed - method kept for compatibility
            return;
        }

        private void UpdateTravelMarkers()
        {
            if (Canvas_travel_markers == null || Rect_axis_position == null || Rect_trim_center == null)
            {
                return;
            }

            double width = Canvas_travel_markers.ActualWidth;
            if (width <= 0.0)
            {
                return;
            }

            double posMin = Rangeslider_travel_range?.LowerValue ?? config.PosNearLim;
            double posMax = Rangeslider_travel_range?.UpperValue ?? config.PosFarLim;
            double rangeMin = Rangeslider_travel_range?.Minimum ?? config.PosNearLim;
            double rangeMax = Rangeslider_travel_range?.Maximum ?? config.PosFarLim;

            if (hasAxisPosition)
            {
                if (Tools.TryComputeMarkerX(latestAxisPosition, posMin, posMax, rangeMin, rangeMax, width, out double posX))
                {
                    Canvas.SetLeft(Rect_axis_position, posX - Rect_axis_position.Width / 2.0);
                }
            }

            if (hasTrimCenter)
            {
                double center = (posMin + posMax) / 2.0;
                double trimPos = center + latestTrimCenter;
                if (Tools.TryComputeMarkerX(trimPos, posMin, posMax, rangeMin, rangeMax, width, out double trimX))
                {
                    Canvas.SetLeft(Rect_trim_center, trimX - Rect_trim_center.Width / 2.0);
                }
            }
        }

        private void Rangeslider_travel_range_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateTravelMarkers();
        }

        private void ApplyFallbackTravelRange()
        {
            if (Rangeslider_travel_range == null)
            {
                return;
            }

            double min = Math.Min(config.PosNearLim, config.PosFarLim);
            double max = Math.Max(config.PosNearLim, config.PosFarLim);
            Rangeslider_travel_range.Minimum = min;
            Rangeslider_travel_range.Maximum = max;
        }

        private void Toggle_ffb_enabled_Checked(object sender, RoutedEventArgs e)
        {
            // FFB toggle - currently no action needed
        }

        private void Toggle_ffb_enabled_Unchecked(object sender, RoutedEventArgs e)
        {
            // FFB toggle - currently no action needed
        }

        private void OnActiveGraphChanged(object sender, EventArgs e)
        {
            Dispatcher.Invoke(RefreshGraphParams);
        }

        private void OnGraphParamChanged(object sender, GraphParamChangedEventArgs e)
        {
            if (isUpdatingGraphParams)
            {
                return;
            }

            Dispatcher.Invoke(() =>
            {
                isUpdatingGraphParams = true;
                try
                {
                    if (graphParamLabels.TryGetValue(e.ParamName, out var label))
                    {
                        var allParams = plugin?.GetActiveGraphParams();
                        if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                        {
                            label.Content = FormatParamLabel(param, e.Value);
                        }
                    }

                    if (graphParamControls.TryGetValue(e.ParamName, out var control))
                    {
                        if (control is Slider slider)
                        {
                            slider.Value = e.Value;
                        }
                        else if (control is TextBox textBox)
                        {
                            int precision = 3;
                            var allParams = plugin?.GetActiveGraphParams();
                            if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                            {
                                precision = param.Ui?.Precision ?? 3;
                            }
                            textBox.Text = e.Value.ToString($"F{precision}");
                        }
                    }
                }
                finally
                {
                    isUpdatingGraphParams = false;
                }
            });
        }

        private string FormatParamLabel(GraphParam param, double currentValue)
        {
            string label = param.Ui?.Label ?? param.Name;
            int precision = param.Ui?.Precision ?? 3;
            string valueStr = currentValue.ToString($"F{precision}");

            if (!string.IsNullOrWhiteSpace(param.Ui?.Units))
            {
                return $"{label}: {valueStr}{param.Ui.Units}";
            }
            else
            {
                return $"{label}: {valueStr}";
            }
        }

        private void RefreshGraphParams()
        {
            try
            {
                GraphParamsPanel.Children.Clear();
                graphParamControls.Clear();
                graphParamLabels.Clear();

                if (plugin == null)
                {
                    return;
                }

                var allParams = plugin.GetActiveGraphParams();
                if (allParams == null || allParams.Count == 0)
                {
                    return;
                }

                string groupFilter = GetGraphParamGroupFilter();
                var filteredParams = allParams.Values
                    .Where(p => MatchesGroup(p.Ui?.Group, groupFilter))
                    .OrderBy(p => p.Ui?.Label ?? p.Name)
                    .ToList();

                foreach (var param in filteredParams)
                {
                    double currentValue = plugin.GetGraphParamValue(param.Name);

                    var panel = new StackPanel
                    {
                        Width = 400,
                        Height = 40,
                        Orientation = Orientation.Vertical,
                        Background = null
                    };

                    var label = new Label
                    {
                        Foreground = Brushes.White,
                        FontSize = 10,
                        FontFamily = new FontFamily("Arial"),
                        HorizontalAlignment = HorizontalAlignment.Left,
                        VerticalAlignment = VerticalAlignment.Top,
                        Content = FormatParamLabel(param, currentValue),
                        Padding = new Thickness(0, 0, 0, 8)
                    };

                    var control = GraphParamControlBuilder.BuildControl(
                        param,
                        value =>
                        {
                            if (!isUpdatingGraphParams)
                            {
                                try
                                {
                                    isUpdatingGraphParams = true;
                                    plugin.SetGraphParamValue(param.Name, value);
                                    if (graphParamLabels.TryGetValue(param.Name, out var lbl))
                                    {
                                        lbl.Content = FormatParamLabel(param, value);
                                    }
                                }
                                finally
                                {
                                    isUpdatingGraphParams = false;
                                }
                            }
                        },
                        width: 400,
                        initialValue: currentValue
                    );

                    panel.Children.Add(label);
                    panel.Children.Add(control);
                    GraphParamsPanel.Children.Add(panel);
                    graphParamControls[param.Name] = control;
                    graphParamLabels[param.Name] = label;
                }
            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Error($"[FlightPedals] RefreshGraphParams failed: {ex.Message}", ex);
            }
        }

        private string GetGraphParamGroupFilter()
        {
            if (current_function_id == FunctionID.FlightPedals)
            {
                return "FlightPedals";
            }
            return "";
        }

        private bool MatchesGroup(string paramGroup, string filter)
        {
            if (string.IsNullOrWhiteSpace(filter))
                return false;
            if (string.IsNullOrWhiteSpace(paramGroup))
                return false;
            return paramGroup.StartsWith(filter, StringComparison.OrdinalIgnoreCase);
        }

    }
}
