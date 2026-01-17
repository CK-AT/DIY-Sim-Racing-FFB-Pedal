using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using MahApps.Metro.Controls;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for FlightStickConfigControl.xaml
    /// </summary>
    public partial class FlightStickConfigControl : UserControl
    {
        public event FunctionConfigControl.DebugMessageEventHandler DebugMessage;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private FunctionConfig function_config = new FunctionConfig();
        private Function function;
        private FunctionID current_function_id;
        private FlightStickPitchConfig pitch_config;
        private FlightStickRollConfig roll_config;
        private FlightStickCollectiveConfig collective_config;
        private bool is_updating = true;
        private double latestAxisPosition;
        private bool hasAxisPosition;
        private double latestTrimCenter;
        private bool hasTrimCenter;
        private DispatcherTimer xplaneTimer;
        private float lastIasKts;
        private bool hasAxisRange;

        public FlightStickConfigControl()
        {
            pitch_config = GetDefaultPitchConfig();
            roll_config = GetDefaultRollConfig();
            collective_config = GetDefaultCollectiveConfig();
            InitializeComponent();
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            is_updating = false;
            StartXPlaneTimer();
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
            UpdateTrimCenter();
            UpdateTravelMarkers();
        }

        public static FlightStickPitchConfig GetDefaultPitchConfig()
        {
            FlightStickPitchConfig new_config = new FlightStickPitchConfig();
            new_config.PosMin = -50;
            new_config.PosMax = 50;
            new_config.Damping = 0.5f;
            new_config.CenteringSpringConst = 1.5f;
            return new_config;
        }

        public static FlightStickRollConfig GetDefaultRollConfig()
        {
            FlightStickRollConfig new_config = new FlightStickRollConfig();
            new_config.PosMin = -50;
            new_config.PosMax = 50;
            new_config.Damping = 0.5f;
            new_config.CenteringSpringConst = 1.5f;
            return new_config;
        }

        public static FlightStickCollectiveConfig GetDefaultCollectiveConfig()
        {
            FlightStickCollectiveConfig new_config = new FlightStickCollectiveConfig();
            new_config.PosMin = -50;
            new_config.PosMax = 50;
            new_config.Damping = 0.5f;
            new_config.CenteringSpringConst = 1.5f;
            return new_config;
        }

        private enum FlightStickMode
        {
            Pitch,
            Roll,
            Collective
        }

        private FlightStickMode GetMode()
        {
            switch (current_function_id)
            {
                case FunctionID.FlightStickRoll:
                    return FlightStickMode.Roll;
                case FunctionID.FlightStickCollective:
                    return FlightStickMode.Collective;
                default:
                    return FlightStickMode.Pitch;
            }
        }

        private void EnsureConfigInitialized()
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    if (function_config.FlightStickRoll == null)
                    {
                        function_config.FlightStickRoll = GetDefaultRollConfig();
                    }
                    roll_config = function_config.FlightStickRoll;
                    break;
                case FlightStickMode.Collective:
                    if (function_config.FlightStickCollective == null)
                    {
                        function_config.FlightStickCollective = GetDefaultCollectiveConfig();
                    }
                    collective_config = function_config.FlightStickCollective;
                    break;
                default:
                    if (function_config.FlightStickPitch == null)
                    {
                        function_config.FlightStickPitch = GetDefaultPitchConfig();
                    }
                    pitch_config = function_config.FlightStickPitch;
                    break;
            }
        }

        private int GetPosMin()
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    return roll_config.PosMin;
                case FlightStickMode.Collective:
                    return collective_config.PosMin;
                default:
                    return pitch_config.PosMin;
            }
        }

        private int GetPosMax()
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    return roll_config.PosMax;
                case FlightStickMode.Collective:
                    return collective_config.PosMax;
                default:
                    return pitch_config.PosMax;
            }
        }

        private void SetPosMin(int value)
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    roll_config.PosMin = value;
                    break;
                case FlightStickMode.Collective:
                    collective_config.PosMin = value;
                    break;
                default:
                    pitch_config.PosMin = value;
                    break;
            }
        }

        private void SetPosMax(int value)
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    roll_config.PosMax = value;
                    break;
                case FlightStickMode.Collective:
                    collective_config.PosMax = value;
                    break;
                default:
                    pitch_config.PosMax = value;
                    break;
            }
        }

        private float GetDamping()
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    return roll_config.Damping;
                case FlightStickMode.Collective:
                    return collective_config.Damping;
                default:
                    return pitch_config.Damping;
            }
        }

        private float GetCenteringSpringConst()
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    return roll_config.CenteringSpringConst;
                case FlightStickMode.Collective:
                    return collective_config.CenteringSpringConst;
                default:
                    return pitch_config.CenteringSpringConst;
            }
        }

        private void SetDamping(float value)
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    roll_config.Damping = value;
                    break;
                case FlightStickMode.Collective:
                    collective_config.Damping = value;
                    break;
                default:
                    pitch_config.Damping = value;
                    break;
            }
        }

        private void SetCenteringSpringConst(float value)
        {
            switch (GetMode())
            {
                case FlightStickMode.Roll:
                    roll_config.CenteringSpringConst = value;
                    break;
                case FlightStickMode.Collective:
                    collective_config.CenteringSpringConst = value;
                    break;
                default:
                    pitch_config.CenteringSpringConst = value;
                    break;
            }
        }

        public void SwitchFunction(Function function)
        {
            this.function = function;
            function_config = function.Config;
            current_function_id = function_config.Base.FunctionId;
            EnsureConfigInitialized();
            hasAxisRange = false;

            is_updating = true;
            function_config.Base.OutputMode = OutputMode.Travel;
            uc_axis_selector_stick.Value = function_config.Base.LinkedAxes[0];
            uc_controller_axis_stick.Value = function_config.Base.ControllerOutputAxis;

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
            Slider_centering_spring_const.Value = GetCenteringSpringConst();
            Slider_damping.Value = GetDamping();
            UpdateXPlaneSettingsUi();

            Rangeslider_travel_range.LowerValue = GetPosMin();
            function_config.Base.OutputMin = GetPosMin();
            Rangeslider_travel_range.UpperValue = GetPosMax();
            function_config.Base.OutputMax = GetPosMax();
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;
        }

        private void OnAxisIDChanged(object sender, AxisSelector.AxisIDChangedEventArgs e)
        {
            if (!is_updating)
            {
                function_config.Base.OutputMode = OutputMode.Travel;
                function_config.Base.LinkedAxes.Clear();
                function_config.Base.LinkedAxes.AddRange(new AxisID[4] { e.Value, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined });
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

        private void uc_controller_axis_stick_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.Base.ControllerOutputAxis = e.Value;
        }

        private void Rangeslider_travel_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                SetPosMin(Convert.ToInt16(e.NewValue));
                function_config.Base.OutputMin = Convert.ToInt16(e.NewValue);
            }
            if (Label_min_pos != null)
            {
                Label_min_pos.Content = String.Format("Min\n{0}mm", GetPosMin());
            }
            UpdateTravelMarkers();
        }

        private void Rangeslider_travel_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                SetPosMax(Convert.ToInt16(e.NewValue));
                function_config.Base.OutputMax = Convert.ToInt16(e.NewValue);
            }
            if (Label_max_pos != null)
            {
                Label_max_pos.Content = String.Format("Max\n{0}mm", GetPosMax());
            }
            UpdateTravelMarkers();
        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SetDamping((float)e.NewValue);
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", e.NewValue);
        }

        private void OnCentringSpringChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SetCenteringSpringConst((float)e.NewValue);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", e.NewValue);
        }

        private void OnFrictionChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_friction.Content = String.Format("Friction: {0:F1}N", e.NewValue);
            function_config.Friction = (float)e.NewValue;
        }

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", e.NewValue);
            function_config.SimulatedMass = (float)e.NewValue;
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
            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            Toggle_xplane_ffb_enabled.IsChecked = settings.XPlaneFfbEnabled;
            Slider_xplane_kq.Value = settings.XPlaneFfbKq;
            Slider_xplane_krate.Value = settings.XPlaneFfbKrate;
            if (current_function_id == FunctionID.FlightStickCollective)
            {
                Slider_xplane_vref.Minimum = 100.0;
                Slider_xplane_vref.Maximum = 600.0;
                Slider_xplane_vref.SmallChange = 5.0;
                Slider_xplane_vref.TickFrequency = 5.0;
                Slider_xplane_trim_mm_per_deg.Maximum = 200.0;
                Slider_xplane_trim_mm_per_deg.TickFrequency = 1.0;
                Slider_xplane_trim_mm_per_deg.SmallChange = 0.5;
            }
            else
            {
                Slider_xplane_vref.Minimum = 10.0;
                Slider_xplane_vref.Maximum = 200.0;
                Slider_xplane_vref.SmallChange = 1.0;
                Slider_xplane_vref.TickFrequency = 1.0;
                Slider_xplane_trim_mm_per_deg.Maximum = 50.0;
                Slider_xplane_trim_mm_per_deg.TickFrequency = 0.01;
                Slider_xplane_trim_mm_per_deg.SmallChange = 0.01;
            }
            Slider_xplane_trim_mm_per_deg.Value = settings.XPlaneTrimMmPerDeg;
            Slider_xplane_buffet_start_deg.Value = settings.XPlaneBuffetStartDeg;
            Slider_xplane_buffet_full_deg.Value = settings.XPlaneBuffetFullDeg;
            Slider_xplane_buffet_gain.Value = settings.XPlaneBuffetGain;
            Slider_xplane_weathervane_gain.Value = settings.XPlaneWeathervaneGain;
            Slider_xplane_vref.Value = settings.XPlaneVrefKts;

            UpdateCollectiveLoadRange();
            UpdateXPlaneLabels();
            UpdateGainGraph();
        }

        private void UpdateCollectiveLoadRange()
        {
            if (Slider_xplane_kq == null)
            {
                return;
            }

            if (current_function_id == FunctionID.FlightStickCollective)
            {
                Slider_xplane_kq.Minimum = 0.0;
                Slider_xplane_kq.Maximum = 0.1;
                Slider_xplane_kq.SmallChange = 0.0001;
                Slider_xplane_kq.TickFrequency = 0.0001;
            }
            else
            {
                Slider_xplane_kq.Minimum = 0.0;
                Slider_xplane_kq.Maximum = 2.0;
                Slider_xplane_kq.SmallChange = 0.001;
                Slider_xplane_kq.TickFrequency = 0.001;
            }
        }

        private void UpdateXPlaneLabels()
        {
            if (label_xplane_ffb_enabled != null)
            {
                label_xplane_ffb_enabled.Content = Toggle_xplane_ffb_enabled.IsChecked == true ? "Enable FFB" : "FFB Disabled";
            }
            if (label_xplane_kq != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_kq.Content = String.Format("Load Gain (N/Nm): {0:F4}", Slider_xplane_kq.Value);
                }
                else
                {
                    label_xplane_kq.Content = String.Format("Spring Gain @ Vref: {0:F3}", Slider_xplane_kq.Value);
                }
            }
            if (label_xplane_krate != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_krate.Content = String.Format("Damper Gain @ Nominal RPM: {0:F3}", Slider_xplane_krate.Value);
                }
                else
                {
                    label_xplane_krate.Content = String.Format("Damper Gain @ Vref: {0:F3}", Slider_xplane_krate.Value);
                }
            }
            if (label_xplane_trim_mm_per_deg != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_trim_mm_per_deg.Content = String.Format("Trim Scale (mm/ratio): {0:F1}", Slider_xplane_trim_mm_per_deg.Value);
                }
                else
                {
                    label_xplane_trim_mm_per_deg.Content = String.Format("Trim Scale: {0:F3} mm/unit", Slider_xplane_trim_mm_per_deg.Value);
                }
            }
            if (label_xplane_buffet_start_deg != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_buffet_start_deg.Content = "Buffet Start (unused)";
                }
                else
                {
                    label_xplane_buffet_start_deg.Content = String.Format("Buffet Start (deg): {0:F1}", Slider_xplane_buffet_start_deg.Value);
                }
            }
            if (label_xplane_buffet_full_deg != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_buffet_full_deg.Content = "Buffet Full (unused)";
                }
                else
                {
                    label_xplane_buffet_full_deg.Content = String.Format("Buffet Full (deg): {0:F1}", Slider_xplane_buffet_full_deg.Value);
                }
            }
            if (label_xplane_buffet_gain != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_buffet_gain.Content = "Buffet Gain (unused)";
                }
                else
                {
                    label_xplane_buffet_gain.Content = String.Format("Buffet Gain @ Vref: {0:F3}", Slider_xplane_buffet_gain.Value);
                }
            }
            if (label_xplane_weathervane_gain != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_weathervane_gain.Content = "Weathervane Gain (unused)";
                }
                else
                {
                    label_xplane_weathervane_gain.Content = String.Format("Weather-Vaning Gain @ Vref: {0:F3}", Slider_xplane_weathervane_gain.Value);
                }
            }
            if (label_xplane_vref != null)
            {
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    label_xplane_vref.Content = String.Format("Nominal RPM: {0:F0}", Slider_xplane_vref.Value);
                }
                else
                {
                    label_xplane_vref.Content = String.Format("Vref (kts): {0:F0}", Slider_xplane_vref.Value);
                }
            }
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

        private void OnXPlaneVrefChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
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

            settings.XPlaneVrefKts = (float)e.NewValue;
            UpdateXPlaneLabels();
            UpdateGainGraph();
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
            if (plugin == null || TextBlock_xplane_ias == null)
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
                if (current_function_id == FunctionID.FlightStickCollective)
                {
                    TextBlock_xplane_ias.Text = "RPM: --";
                }
                else
                {
                    TextBlock_xplane_ias.Text = "IAS: -- kt";
                }
                if (TextBlock_xplane_alpha != null)
                {
                    TextBlock_xplane_alpha.Text = "Alpha: -- deg";
                }
                if (TextBlock_xplane_beta != null)
                {
                    TextBlock_xplane_beta.Text = "Beta: -- deg";
                }
                if (TextBlock_xplane_trim != null)
                {
                    TextBlock_xplane_trim.Text = "Trim: -- mm";
                }
                if (TextBlock_xplane_vane != null)
                {
                    TextBlock_xplane_vane.Text = "Vane: -- mm";
                }
                if (TextBlock_xplane_spring_value != null)
                {
                    TextBlock_xplane_spring_value.Text = "--";
                }
                if (TextBlock_xplane_damper_value != null)
                {
                    TextBlock_xplane_damper_value.Text = "--";
                }
                UpdateXPlaneDiagnosticsUnavailable();
                UpdateGainCursor();
                return;
            }

            lastIasKts = iasKts;
            float trimDeg = 0.0f;
            float vaneDeg = 0.0f;
            switch (current_function_id)
            {
                case FunctionID.FlightStickPitch:
                    trimDeg = elevTrim;
                    vaneDeg = alphaDeg;
                    break;
                case FunctionID.FlightStickRoll:
                    trimDeg = ailTrim;
                    vaneDeg = 0.0f;
                    break;
                case FunctionID.FlightStickCollective:
                    trimDeg = 0.0f;
                    vaneDeg = 0.0f;
                    break;
            }

            float trimMm = trimDeg * (float)Slider_xplane_trim_mm_per_deg.Value;
            float qScale = XPlaneFfbMath.ComputeQScaleFromIasKts(iasKts, (float)Slider_xplane_vref.Value);
            float vaneMm = (float)Slider_xplane_weathervane_gain.Value * qScale * vaneDeg;
            float springGain = (float)Slider_xplane_kq.Value * qScale;
            float damperGain = (float)Slider_xplane_krate.Value * qScale;

            if (current_function_id == FunctionID.FlightStickCollective &&
                plugin.TryGetXPlaneFfbDiagnostics(current_function_id, out DiyFfbPlugin.XPlaneFfbDiagnostics diagnostics) &&
                diagnostics.RotorIndex >= 0)
            {
                lastIasKts = diagnostics.OmegaRpm;
                TextBlock_xplane_ias.Text = String.Format("RPM: {0:F0}", diagnostics.OmegaRpm);
            }
            else
            {
                lastIasKts = iasKts;
                TextBlock_xplane_ias.Text = String.Format("IAS: {0:F0} kt", iasKts);
            }
            if (TextBlock_xplane_alpha != null)
            {
                TextBlock_xplane_alpha.Text = String.Format("Alpha: {0:F1} deg", alphaDeg);
            }
            if (TextBlock_xplane_beta != null)
            {
                TextBlock_xplane_beta.Text = String.Format("Beta: {0:F1} deg", betaDeg);
            }
            if (TextBlock_xplane_trim != null)
            {
                TextBlock_xplane_trim.Text = String.Format("Trim: {0:F2} mm", trimMm);
            }
            if (TextBlock_xplane_vane != null)
            {
                TextBlock_xplane_vane.Text = String.Format("Vane: {0:F2} mm", vaneMm);
            }
            if (TextBlock_xplane_spring_value != null)
            {
                TextBlock_xplane_spring_value.Text = String.Format("{0:F3}", springGain);
            }
            if (TextBlock_xplane_damper_value != null)
            {
                TextBlock_xplane_damper_value.Text = String.Format("{0:F3}", damperGain);
            }
            UpdateXPlaneDiagnostics();
            UpdateGainCursor();
        }

        private void UpdateXPlaneDiagnostics()
        {
            if (plugin == null)
            {
                return;
            }

            if (!plugin.TryGetXPlaneFfbDiagnostics(current_function_id, out DiyFfbPlugin.XPlaneFfbDiagnostics diagnostics))
            {
                UpdateXPlaneDiagnosticsUnavailable();
                return;
            }

            if (TextBlock_xplane_tas != null)
            {
                TextBlock_xplane_tas.Text = String.Format("TAS: {0:F1} m/s", diagnostics.TasMps);
            }
            if (TextBlock_xplane_qhat != null)
            {
                TextBlock_xplane_qhat.Text = String.Format("qHat: {0:F1}", diagnostics.QHat);
            }
            if (TextBlock_xplane_g != null)
            {
                TextBlock_xplane_g.Text = String.Format("G: {0:F2}", diagnostics.GNrml);
            }
            if (TextBlock_xplane_on_ground != null)
            {
                TextBlock_xplane_on_ground.Text = diagnostics.OnGround ? "Ground: yes" : "Ground: no";
            }
            if (TextBlock_xplane_qscale != null)
            {
                TextBlock_xplane_qscale.Text = String.Format("Scale: {0:F3}", diagnostics.QScale);
            }
            if (TextBlock_xplane_buffet != null)
            {
                TextBlock_xplane_buffet.Text = String.Format("Buffet: {0:F3}", diagnostics.Buffet);
            }

            bool hasRotor = diagnostics.RotorIndex >= 0;
            if (TextBlock_xplane_rotor != null)
            {
                TextBlock_xplane_rotor.Text = hasRotor ? String.Format("Rotor: {0}", diagnostics.RotorIndex + 1) : "Rotor: --";
            }
            if (TextBlock_xplane_torque != null)
            {
                TextBlock_xplane_torque.Text = hasRotor ? String.Format("Torque: {0:F1} Nm", diagnostics.TorqueNm) : "Torque: --";
            }
            if (TextBlock_xplane_omega != null)
            {
                TextBlock_xplane_omega.Text = hasRotor ? String.Format("Omega: {0:F0} RPM", diagnostics.OmegaRpm) : "Omega: --";
            }
            if (TextBlock_xplane_prop != null)
            {
                TextBlock_xplane_prop.Text = hasRotor ? String.Format("Prop: {0:F2}", diagnostics.PropRatio) : "Prop: --";
            }
            if (TextBlock_xplane_nominal_rpm != null)
            {
                TextBlock_xplane_nominal_rpm.Text = hasRotor ? String.Format("Nominal: {0:F0} RPM", diagnostics.NominalRpm) : "Nominal: --";
            }
            if (TextBlock_xplane_omega_scale != null)
            {
                TextBlock_xplane_omega_scale.Text = hasRotor ? String.Format("Omega scale: {0:F2}", diagnostics.OmegaScale) : "Omega scale: --";
            }
            if (TextBlock_xplane_gscale != null)
            {
                TextBlock_xplane_gscale.Text = hasRotor ? String.Format("G scale: {0:F2}", diagnostics.GScale) : "G scale: --";
            }
            if (TextBlock_xplane_load != null)
            {
                TextBlock_xplane_load.Text = hasRotor ? String.Format("Load: {0:F2}", diagnostics.LoadForce) : "Load: --";
            }
            if (TextBlock_xplane_trim_deg != null)
            {
                TextBlock_xplane_trim_deg.Text = String.Format("Trim: {0:F2} deg", diagnostics.TrimDeg);
            }
        }

        private void UpdateXPlaneDiagnosticsUnavailable()
        {
            if (TextBlock_xplane_tas != null)
            {
                TextBlock_xplane_tas.Text = "TAS: -- m/s";
            }
            if (TextBlock_xplane_qhat != null)
            {
                TextBlock_xplane_qhat.Text = "qHat: --";
            }
            if (TextBlock_xplane_g != null)
            {
                TextBlock_xplane_g.Text = "G: --";
            }
            if (TextBlock_xplane_on_ground != null)
            {
                TextBlock_xplane_on_ground.Text = "Ground: --";
            }
            if (TextBlock_xplane_qscale != null)
            {
                TextBlock_xplane_qscale.Text = "Scale: --";
            }
            if (TextBlock_xplane_buffet != null)
            {
                TextBlock_xplane_buffet.Text = "Buffet: --";
            }
            if (TextBlock_xplane_rotor != null)
            {
                TextBlock_xplane_rotor.Text = "Rotor: --";
            }
            if (TextBlock_xplane_torque != null)
            {
                TextBlock_xplane_torque.Text = "Torque: --";
            }
            if (TextBlock_xplane_omega != null)
            {
                TextBlock_xplane_omega.Text = "Omega: --";
            }
            if (TextBlock_xplane_prop != null)
            {
                TextBlock_xplane_prop.Text = "Prop: --";
            }
            if (TextBlock_xplane_nominal_rpm != null)
            {
                TextBlock_xplane_nominal_rpm.Text = "Nominal: --";
            }
            if (TextBlock_xplane_omega_scale != null)
            {
                TextBlock_xplane_omega_scale.Text = "Omega scale: --";
            }
            if (TextBlock_xplane_gscale != null)
            {
                TextBlock_xplane_gscale.Text = "G scale: --";
            }
            if (TextBlock_xplane_load != null)
            {
                TextBlock_xplane_load.Text = "Load: --";
            }
            if (TextBlock_xplane_trim_deg != null)
            {
                TextBlock_xplane_trim_deg.Text = "Trim: -- deg";
            }
        }

        private void UpdateGainGraph()
        {
            if (Canvas_xplane_gain == null || Polyline_xplane_spring == null || Polyline_xplane_damper == null)
            {
                return;
            }

            double width = Canvas_xplane_gain.Width;
            double height = Canvas_xplane_gain.Height;
            if (width <= 0.0 || height <= 0.0)
            {
                return;
            }

            float vrefKts = (float)Slider_xplane_vref.Value;
            float springRef = (float)Slider_xplane_kq.Value;
            float damperRef = (float)Slider_xplane_krate.Value;
            PointCollection springPoints;
            PointCollection damperPoints;
            float maxIasKts;
            float maxGain;
            XPlaneFfbGraph.BuildGainCurves(vrefKts, springRef, damperRef, width, height,
                                           out springPoints, out damperPoints,
                                           out maxIasKts, out maxGain);
            XPlaneFfbGraph.UpdateGainGrid(Canvas_xplane_gain, vrefKts, maxIasKts, maxGain);

            Polyline_xplane_spring.Points = springPoints;
            Polyline_xplane_damper.Points = damperPoints;
            UpdateGainCursor();
        }

        private void UpdateGainCursor()
        {
            if (Line_xplane_cursor == null || Canvas_xplane_gain == null)
            {
                return;
            }

            double width = Canvas_xplane_gain.Width;
            if (width <= 0.0)
            {
                return;
            }

            float vrefKts = (float)Slider_xplane_vref.Value;
            float maxIasKts = XPlaneFfbGraph.GetMaxIasKts(vrefKts);
            float clampedIas = Math.Max(0.0f, Math.Min(lastIasKts, maxIasKts));
            double x = (clampedIas / maxIasKts) * width;
            Line_xplane_cursor.X1 = x;
            Line_xplane_cursor.X2 = x;
        }

        public void RefreshXPlaneFfbSettings()
        {
            if (is_updating)
            {
                return;
            }

            is_updating = true;
            UpdateXPlaneSettingsUi();
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;
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

            double posMin = Rangeslider_travel_range?.LowerValue ?? GetPosMin();
            double posMax = Rangeslider_travel_range?.UpperValue ?? GetPosMax();
            double rangeMin = Rangeslider_travel_range?.Minimum ?? GetPosMin();
            double rangeMax = Rangeslider_travel_range?.Maximum ?? GetPosMax();

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

            double min = Math.Min(GetPosMin(), GetPosMax());
            double max = Math.Max(GetPosMin(), GetPosMax());
            Rangeslider_travel_range.Minimum = min;
            Rangeslider_travel_range.Maximum = max;
        }
    }
}
