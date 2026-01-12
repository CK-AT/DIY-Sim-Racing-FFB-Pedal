using System;
using System.Windows;
using System.Windows.Controls;
using MahApps.Metro.Controls;

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
        private double latestTrimCenter;
        private bool hasTrimCenter;

        public FlightPedalsConfigControl()
        {
            config = GetDefaultConfig();
            InitializeComponent();
        }
        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            is_updating = false;
        }

        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            Rangeslider_travel_range.Minimum = parameters.ContactPointPosMinAbs / 10.0f;
            Rangeslider_travel_range.Maximum = parameters.ContactPointPosMaxAbs / 10.0f;
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

            Slider_simulated_mass.Value = function_config.SimulatedMass;
            Slider_friction.Value = function_config.Friction;

            Slider_centering_spring_const.Value = config.CenteringSpringConst;
            Slider_damping.Value = config.Damping;
            uc_controller_axis_pedals.Value = function_config.Base.ControllerOutputAxis;
            uc_controller_axis_right_brake.Value = function_config.AuxFunction.RudderBrake.ControllerOutputAxisRightPedal;
            uc_controller_axis_left_brake.Value = function_config.AuxFunction.RudderBrake.ControllerOutputAxisLeftPedal;
            UpdateXPlaneSettingsUi();
            Rangeslider_travel_range.UpperValue = config.PosFarLim;
            function_config.Base.OutputMax = config.PosFarLim;
            Rangeslider_travel_range.LowerValue = config.PosNearLim;
            function_config.Base.OutputMin = config.PosNearLim;
            Rangeslider_brake_force_range.UpperValue = function_config.AuxFunction.RudderBrake.FMax / 9.81f;
            Rangeslider_brake_force_range.LowerValue = function_config.AuxFunction.RudderBrake.FMin / 9.81f;
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;

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
            var settings = GetFunctionSettings();
            if (settings == null)
            {
                return;
            }

            Toggle_xplane_ffb_enabled.IsChecked = settings.XPlaneFfbEnabled;
            Slider_xplane_kq.Value = settings.XPlaneFfbKq;
            Slider_xplane_krate.Value = settings.XPlaneFfbKrate;
            Slider_xplane_trim_mm_per_deg.Value = settings.XPlaneTrimMmPerDeg;
            Slider_xplane_buffet_start_deg.Value = settings.XPlaneBuffetStartDeg;
            Slider_xplane_buffet_full_deg.Value = settings.XPlaneBuffetFullDeg;
            Slider_xplane_buffet_gain.Value = settings.XPlaneBuffetGain;

            UpdateXPlaneLabels();
        }

        private void UpdateXPlaneLabels()
        {
            if (label_xplane_ffb_enabled != null)
            {
                label_xplane_ffb_enabled.Content = Toggle_xplane_ffb_enabled.IsChecked == true ? "Enable FFB" : "FFB Disabled";
            }
            if (label_xplane_kq != null)
            {
                label_xplane_kq.Content = String.Format("Spring Gain (kq): {0:F4}", Slider_xplane_kq.Value);
            }
            if (label_xplane_krate != null)
            {
                label_xplane_krate.Content = String.Format("Damper Gain (krate): {0:F4}", Slider_xplane_krate.Value);
            }
            if (label_xplane_trim_mm_per_deg != null)
            {
                label_xplane_trim_mm_per_deg.Content = String.Format("Trim Scale: {0:F2} mm/deg", Slider_xplane_trim_mm_per_deg.Value);
            }
            if (label_xplane_buffet_start_deg != null)
            {
                label_xplane_buffet_start_deg.Content = String.Format("Buffet Start (deg): {0:F1}", Slider_xplane_buffet_start_deg.Value);
            }
            if (label_xplane_buffet_full_deg != null)
            {
                label_xplane_buffet_full_deg.Content = String.Format("Buffet Full (deg): {0:F1}", Slider_xplane_buffet_full_deg.Value);
            }
            if (label_xplane_buffet_gain != null)
            {
                label_xplane_buffet_gain.Content = String.Format("Buffet Gain: {0:F3}", Slider_xplane_buffet_gain.Value);
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

            double posMin = config.PosNearLim;
            double posMax = config.PosFarLim;
            double range = posMax - posMin;
            if (range <= 0.0)
            {
                return;
            }

            if (hasAxisPosition)
            {
                double posNorm = (latestAxisPosition - posMin) / range;
                posNorm = Math.Max(0.0, Math.Min(1.0, posNorm));
                double posX = posNorm * width;
                Canvas.SetLeft(Rect_axis_position, posX - Rect_axis_position.Width / 2.0);
            }

            if (hasTrimCenter)
            {
                double trimNorm = (latestTrimCenter - posMin) / range;
                trimNorm = Math.Max(0.0, Math.Min(1.0, trimNorm));
                double trimX = trimNorm * width;
                Canvas.SetLeft(Rect_trim_center, trimX - Rect_trim_center.Width / 2.0);
            }
        }

        private void Rangeslider_travel_range_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateTravelMarkers();
        }

    }
}
