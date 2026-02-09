using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using MahApps.Metro.Controls;
using DiyFfb.Controls;

namespace DiyFfb
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
        private bool allowOverrideCreation = false;  // Only true after initial load stabilizes
        private DispatcherTimer xplaneTimer;
        private bool hasAxisRange;
        private bool isUpdatingOutputToggle = false;
        private BadgeHelper _badgeHelper;
        private GraphParamHelper _graphParamHelper;
        private TravelDisplayHelper _travelHelper;

        public FlightPedalsConfigControl()
        {
            config = GetDefaultConfig();
            InitializeComponent();
            _travelHelper = new TravelDisplayHelper(
                Canvas_travel_markers, Rect_axis_position, Rect_trim_center,
                Rangeslider_travel_range,
                () => config.PosNearLim, () => config.PosFarLim);
            Loaded += OnLoaded;
            Unloaded += OnUnloaded;
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            _badgeHelper = new BadgeHelper(this, () => this.plugin, () => function, OnBadgeOverrideCleared);
            _graphParamHelper = new GraphParamHelper(
                GraphParamsPanel,
                () => this.plugin,
                () => current_function_id == FunctionID.FlightPedals ? "FlightPedals" : "",
                "FlightPedals",
                Dispatcher);

            if (plugin != null)
            {
                _graphParamHelper.Subscribe();

                if (IsLoaded)
                    _badgeHelper.Subscribe();
            }

            is_updating = false;
            StartXPlaneTimer();
            _graphParamHelper.Refresh();
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            _badgeHelper?.Subscribe();
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            _badgeHelper?.Unsubscribe();
        }

        private void OnBadgeOverrideCleared(object sender, LayerBadgeWrapper.OverrideClearedEventArgs e)
        {
            if (plugin == null || function == null) return;

            var mergedConfig = plugin.FunctionConfigManager.GetCurrentConfig((int)function.ID);
            if (mergedConfig == null) return;

            is_updating = true;
            switch (e.FieldPath)
            {
                case "flight_pedals.motion_range":
                    config.PosNearLim = mergedConfig.FlightPedals.PosNearLim;
                    config.PosFarLim = mergedConfig.FlightPedals.PosFarLim;
                    TieredConfig.FlightPedalsProcessor.ReconcileDerivedFields(function_config);
                    Rangeslider_travel_range.LowerValue = config.PosNearLim;
                    Rangeslider_travel_range.UpperValue = config.PosFarLim;
                    if (Label_near_pos != null)
                        Label_near_pos.Content = String.Format("Near\n{0}mm", config.PosNearLim);
                    if (Label_far_pos != null)
                        Label_far_pos.Content = String.Format("Far\n{0}mm", config.PosFarLim);
                    _travelHelper.UpdateTravelMarkers();
                    break;

                case "flight_pedals.damping":
                    config.Damping = mergedConfig.FlightPedals.Damping;
                    Slider_damping.Value = config.Damping;
                    label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", config.Damping);
                    break;

                case "flight_pedals.centering_spring_const":
                    config.CenteringSpringConst = mergedConfig.FlightPedals.CenteringSpringConst;
                    Slider_centering_spring_const.Value = config.CenteringSpringConst;
                    label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", config.CenteringSpringConst);
                    break;

                case "simulated_mass":
                    function_config.SimulatedMass = mergedConfig.SimulatedMass;
                    Slider_simulated_mass.Value = mergedConfig.SimulatedMass;
                    label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", mergedConfig.SimulatedMass);
                    break;

                case "friction":
                    function_config.Friction = mergedConfig.Friction;
                    Slider_friction.Value = mergedConfig.Friction;
                    label_friction.Content = String.Format("Friction: {0:F1}N", mergedConfig.Friction);
                    break;

                case "aux_function.rudder_brake.force_range":
                    function_config.AuxFunction.RudderBrake.FMin = mergedConfig.AuxFunction.RudderBrake.FMin;
                    function_config.AuxFunction.RudderBrake.FMax = mergedConfig.AuxFunction.RudderBrake.FMax;
                    Rangeslider_brake_force_range.LowerValue = mergedConfig.AuxFunction.RudderBrake.FMin / 9.81f;
                    Rangeslider_brake_force_range.UpperValue = mergedConfig.AuxFunction.RudderBrake.FMax / 9.81f;
                    if (Label_min_brake_force != null)
                        Label_min_brake_force.Content = String.Format("Preload\n{0:F1}kg", mergedConfig.AuxFunction.RudderBrake.FMin / 9.81f);
                    if (Label_max_brake_force != null)
                        Label_max_brake_force.Content = String.Format("Max\n{0:F1}kg", mergedConfig.AuxFunction.RudderBrake.FMax / 9.81f);
                    break;
            }
            is_updating = false;
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
            if (!KinematicBoundsHelper.TryGetTravelBounds(parameters, out double boundsMin, out double boundsMax))
                return;

            hasAxisRange = true;

            bool wasUpdating = is_updating;
            if (!wasUpdating) is_updating = true;

            KinematicBoundsHelper.ApplyBoundsToSlider(
                Rangeslider_travel_range, boundsMin, boundsMax,
                config.PosNearLim, config.PosFarLim);

            if (!wasUpdating)
            {
                Dispatcher.BeginInvoke(new Action(() => is_updating = false),
                    System.Windows.Threading.DispatcherPriority.ContextIdle);
            }
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (_travelHelper.TryUpdateAxisState(function_config, axis_state))
            {
                _travelHelper.UpdateTrimCenter(plugin, current_function_id);
                _travelHelper.UpdateTravelMarkers();
            }
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
            allowOverrideCreation = false;  // Reset until function switch stabilizes

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
            Rangeslider_travel_range.LowerValue = config.PosNearLim;
            TieredConfig.FlightPedalsProcessor.ReconcileDerivedFields(function_config);
            Rangeslider_brake_force_range.UpperValue = function_config.AuxFunction.RudderBrake.FMax / 9.81f;
            Rangeslider_brake_force_range.LowerValue = function_config.AuxFunction.RudderBrake.FMin / 9.81f;
            _travelHelper.UpdateTrimCenter(plugin, current_function_id);
            _travelHelper.UpdateTravelMarkers();
            is_updating = false;

            // Update labels with config values (event handlers were blocked by is_updating flag)
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", function_config.SimulatedMass);
            label_friction.Content = String.Format("Friction: {0:F1}N", function_config.Friction);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", config.CenteringSpringConst);
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", config.Damping);
            _graphParamHelper?.Refresh();
            UpdateDisableOutputsToggle();
            _badgeHelper?.InitializeBadges();

            // Allow override creation only after all deferred events have been processed
            Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
                System.Windows.Threading.DispatcherPriority.ContextIdle);
        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            config.Damping = newValue;
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.damping",
                    overrides => overrides.FlightPedalsDamping = newValue);
            }
        }

        private void OnCentringSpringChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            config.CenteringSpringConst = newValue;
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.centering_spring_const",
                    overrides => overrides.FlightPedalsCenteringSpringConst = newValue);
            }
        }

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", newValue);
            function_config.SimulatedMass = newValue;

            if (plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
                    overrides => overrides.SimulatedMass = newValue);
            }
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
                var newValueN = (float)(e.NewValue * 9.81);
                var oldValue = function_config.AuxFunction.RudderBrake.FMin;
                function_config.AuxFunction.RudderBrake.FMin = newValueN;

                if (allowOverrideCreation && Math.Abs(newValueN - oldValue) > 0.01f &&
                    plugin != null && function != null &&
                    plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "aux_function.rudder_brake.force_range",
                        overrides =>
                        {
                            if (overrides.RudderBrakeForceRange == null)
                                overrides.RudderBrakeForceRange = new TieredConfig.ForceRangeOverrides();
                            overrides.RudderBrakeForceRange.Min = newValueN;
                        });
                }
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
                var newValueN = (float)(e.NewValue * 9.81);
                var oldValue = function_config.AuxFunction.RudderBrake.FMax;
                function_config.AuxFunction.RudderBrake.FMax = newValueN;

                if (allowOverrideCreation && Math.Abs(newValueN - oldValue) > 0.01f &&
                    plugin != null && function != null &&
                    plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "aux_function.rudder_brake.force_range",
                        overrides =>
                        {
                            if (overrides.RudderBrakeForceRange == null)
                                overrides.RudderBrakeForceRange = new TieredConfig.ForceRangeOverrides();
                            overrides.RudderBrakeForceRange.Max = newValueN;
                        });
                }
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
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events
                if (Rangeslider_travel_range != null &&
                    Convert.ToInt16(Rangeslider_travel_range.LowerValue) != newValue)
                    return;

                var oldValue = config.PosNearLim;
                config.PosNearLim = newValue;
                TieredConfig.FlightPedalsProcessor.ReconcileDerivedFields(function_config);

                if (allowOverrideCreation && newValue != oldValue &&
                    plugin != null && function != null &&
                    plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightPedalsMotionRange == null)
                                overrides.FlightPedalsMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightPedalsMotionRange.NearLim = newValue;
                        });
                }
            }
            if (Label_near_pos != null)
            {
                Label_near_pos.Content = String.Format("Near\n{0}mm", config.PosNearLim);
            }
            _travelHelper.UpdateTravelMarkers();
        }

        private void Rangeslider_travel_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events
                if (Rangeslider_travel_range != null &&
                    Convert.ToInt16(Rangeslider_travel_range.UpperValue) != newValue)
                    return;

                var oldValue = config.PosFarLim;
                config.PosFarLim = newValue;
                TieredConfig.FlightPedalsProcessor.ReconcileDerivedFields(function_config);

                if (allowOverrideCreation && newValue != oldValue &&
                    plugin != null && function != null &&
                    plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightPedalsMotionRange == null)
                                overrides.FlightPedalsMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightPedalsMotionRange.FarLim = newValue;
                        });
                }
            }
            if (Label_far_pos != null)
            {
                Label_far_pos.Content = String.Format("Far\n{0}mm", config.PosFarLim);
            }
            _travelHelper.UpdateTravelMarkers();
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
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            label_friction.Content = String.Format("Friction: {0:F1}N", newValue);
            function_config.Friction = newValue;

            if (plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "friction",
                    overrides => overrides.Friction = newValue);
            }
        }

        private void UpdateDisableOutputsToggle()
        {
            if (Toggle_disable_outputs == null)
            {
                return;
            }

            bool disabled = plugin != null && plugin.IsFunctionOutputDisabled(current_function_id);
            isUpdatingOutputToggle = true;
            Toggle_disable_outputs.IsChecked = disabled;
            isUpdatingOutputToggle = false;
        }

        private void Toggle_disable_outputs_Checked(object sender, RoutedEventArgs e)
        {
            SetOutputsDisabled(true);
        }

        private void Toggle_disable_outputs_Unchecked(object sender, RoutedEventArgs e)
        {
            SetOutputsDisabled(false);
        }

        private void SetOutputsDisabled(bool disabled)
        {
            if (isUpdatingOutputToggle || is_updating)
            {
                return;
            }

            plugin?.SetFunctionOutputDisabled(current_function_id, disabled);
        }

        private void UpdateXPlaneTelemetry()
        {
            // Graph output refresh (no telemetry UI in this tab)
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
            Label_Output_Buffet.Content = plugin.GetGraphOutputValue($"{prefix}.BuffetAmplitude").ToString("F2", CultureInfo.InvariantCulture);
        }

        private void Rangeslider_travel_range_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            _travelHelper.UpdateTravelMarkers();
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

    }
}
