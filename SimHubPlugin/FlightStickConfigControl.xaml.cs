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
        private FlightControlConfig stick_config;
        private bool is_updating = true;
        private bool allowOverrideCreation = false;  // Only true after initial load stabilizes
        private DispatcherTimer xplaneTimer;
        private bool hasAxisRange;
        private bool isUpdatingOutputToggle = false;
        private BadgeHelper _badgeHelper;
        private GraphParamHelper _graphParamHelper;
        private TravelDisplayHelper _travelHelper;

        public FlightStickConfigControl()
        {
            stick_config = GetDefaultConfig();
            InitializeComponent();
            _travelHelper = new TravelDisplayHelper(
                Canvas_travel_markers, Rect_axis_position, Rect_trim_center,
                Rangeslider_travel_range,
                () => (double)stick_config.PosMin, () => (double)stick_config.PosMax);
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
                () =>
                {
                    switch (current_function_id)
                    {
                        case FunctionID.FlightStickPitch: return "FlightStickPitch";
                        case FunctionID.FlightStickRoll: return "FlightStickRoll";
                        case FunctionID.FlightStickCollective: return "FlightStickCollective";
                        default: return "";
                    }
                },
                "FlightStick",
                Dispatcher);

            if (plugin != null)
            {
                _graphParamHelper.Subscribe();

                if (IsLoaded)
                {
                    _badgeHelper.Subscribe();
                    plugin.FlightSafetyDamperChanged += OnSafetyDamperChanged;
                }
            }

            is_updating = false;
            StartXPlaneTimer();
            _graphParamHelper.Refresh();
        }

        private void OnSafetyDamperChanged(bool engaged)
        {
            if (Toggle_safety_damper == null) return;
            Dispatcher.BeginInvoke(new Action(() => {
                isUpdatingOutputToggle = true;
                Toggle_safety_damper.IsChecked = engaged;
                isUpdatingOutputToggle = false;
            }));
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            _badgeHelper?.Subscribe();
            if (plugin != null)
            {
                // Re-subscribe on every Load — WPF's TabControl unloads tab content on
                // switch, so without this the handler dies after the first tab change
                // and the safety toggle stops reflecting external triggers.
                plugin.FlightSafetyDamperChanged += OnSafetyDamperChanged;
            }
            // Catch up on any state changes that happened while we were unloaded.
            UpdateDisableOutputsToggle();
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            _badgeHelper?.Unsubscribe();
            if (plugin != null)
                plugin.FlightSafetyDamperChanged -= OnSafetyDamperChanged;
        }

        private void OnBadgeOverrideCleared(object sender, LayerBadgeWrapper.OverrideClearedEventArgs e)
        {
            // When an override is cleared, update the slider to show the baseline value
            if (plugin == null || function == null)
                return;

            // Get the merged config (now without the cleared override)
            var mergedConfig = plugin.FunctionConfigManager.GetCurrentConfig((int)function.ID);
            if (mergedConfig == null)
                return;

            // Update function_config and UI based on which field was cleared
            bool deferUnlock = false;
            is_updating = true;
            try
            {
                switch (e.FieldPath)
                {
                    case "simulated_mass":
                        function_config.SimulatedMass = mergedConfig.SimulatedMass;
                        Slider_simulated_mass.Value = mergedConfig.SimulatedMass;
                        label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", mergedConfig.SimulatedMass);
                        break;

                    case "flight_control.motion_range":
                        var mergedStick = mergedConfig.FlightControl;
                        if (mergedStick != null)
                        {
                            stick_config.PosMin = mergedStick.PosMin;
                            stick_config.PosMax = mergedStick.PosMax;
                            if (Rangeslider_travel_range != null)
                            {
                                Rangeslider_travel_range.LowerValue = stick_config.PosMin;
                                Rangeslider_travel_range.UpperValue = stick_config.PosMax;
                            }
                            deferUnlock = true;
                        }
                        TieredConfig.FlightControlProcessor.ReconcileDerivedFields(function_config);
                        if (Label_min_pos != null)
                            Label_min_pos.Content = String.Format("Min\n{0}mm", stick_config.PosMin);
                        if (Label_max_pos != null)
                            Label_max_pos.Content = String.Format("Max\n{0}mm", stick_config.PosMax);
                        _travelHelper?.UpdateTravelMarkers();
                        break;

                    case "flight_control.damping":
                        if (mergedConfig.FlightControl != null)
                        {
                            stick_config.Damping = mergedConfig.FlightControl.Damping;
                            Slider_damping.Value = mergedConfig.FlightControl.Damping;
                            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", mergedConfig.FlightControl.Damping);
                        }
                        break;

                    case "flight_control.centering_spring_const":
                        if (mergedConfig.FlightControl != null)
                        {
                            stick_config.CenteringSpringConst = mergedConfig.FlightControl.CenteringSpringConst;
                            Slider_centering_spring_const.Value = mergedConfig.FlightControl.CenteringSpringConst;
                            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", mergedConfig.FlightControl.CenteringSpringConst);
                        }
                        break;

                    case "friction":
                        function_config.Friction = mergedConfig.Friction;
                        Slider_friction.Value = mergedConfig.Friction;
                        label_friction.Content = String.Format("Friction: {0:F1}N", mergedConfig.Friction);
                        break;
                }
            }
            finally
            {
                if (deferUnlock)
                {
                    Dispatcher.BeginInvoke(new Action(() => is_updating = false),
                        System.Windows.Threading.DispatcherPriority.ContextIdle);
                }
                else
                {
                    is_updating = false;
                }
            }
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
                stick_config.PosMin, stick_config.PosMax);

            if (!wasUpdating)
            {
                Dispatcher.BeginInvoke(new Action(() => is_updating = false),
                    System.Windows.Threading.DispatcherPriority.ContextIdle);
            }
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (_travelHelper != null && _travelHelper.TryUpdateAxisState(function_config, axis_state))
            {
                _travelHelper.UpdateTrimCenter(plugin, current_function_id);
                _travelHelper.UpdateTravelMarkers();
            }
        }

        public static FlightControlConfig GetDefaultConfig()
        {
            return new FlightControlConfig
            {
                PosMin = -50,
                PosMax = 50,
                Damping = 0.5f,
                CenteringSpringConst = 1.5f
            };
        }

        private void EnsureConfigInitialized()
        {
            if (function_config.FlightControl == null)
                function_config.FlightControl = GetDefaultConfig();
            stick_config = function_config.FlightControl;
        }

        public void SwitchFunction(Function function)
        {
            this.function = function;
            function_config = function.Config;
            current_function_id = function_config.Base.FunctionId;
            EnsureConfigInitialized();
            hasAxisRange = false;
            allowOverrideCreation = false;  // Reset until function switch stabilizes

            var linkedAxis = function_config.Base.LinkedAxes[0];

            is_updating = true;
            function_config.Base.OutputMode = OutputMode.Travel;
            uc_axis_selector_stick.Value = linkedAxis;
            uc_controller_axis_stick.Value = function_config.Base.ControllerOutputAxis;

            if (linkedAxis != AxisID.AxisUndefined)
            {
                var kinematic_parameters = ui.GetKinematicParameters(linkedAxis);
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
            var sub = stick_config;
            Slider_centering_spring_const.Value = sub.CenteringSpringConst;
            Slider_damping.Value = sub.Damping;

            Rangeslider_travel_range.LowerValue = sub.PosMin;
            Rangeslider_travel_range.UpperValue = sub.PosMax;
            TieredConfig.FlightControlProcessor.ReconcileDerivedFields(function_config);
            _travelHelper?.UpdateTrimCenter(plugin, current_function_id);
            _travelHelper?.UpdateTravelMarkers();
            is_updating = false;

            // Update labels with merged config values (event handlers were blocked by is_updating flag)
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", function_config.SimulatedMass);
            label_friction.Content = String.Format("Friction: {0:F1}N", function_config.Friction);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", sub.CenteringSpringConst);
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", sub.Damping);
            _graphParamHelper?.Refresh();
            UpdateDisableOutputsToggle();
            _badgeHelper?.InitializeBadges();

            // Allow override creation only after all deferred events have been processed
            Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
                System.Windows.Threading.DispatcherPriority.ContextIdle);
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
                PersistAxisConfigToBaseline();
            }
        }

        // LinkedAxes (physical axis binding) and OutputMode live in the Baseline
        // layer with no override-registry entry, so changing them must update the
        // baseline and re-upload — otherwise the merge re-applies the baseline's
        // old axes and the selection is silently discarded.
        private void PersistAxisConfigToBaseline()
        {
            if (!allowOverrideCreation || plugin == null || function == null ||
                !plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                return;

            plugin.ConfigOrchestrator.UpdateFunctionBaseline((int)function.ID, baseline =>
            {
                baseline.Base.OutputMode = function_config.Base.OutputMode;
                baseline.Base.LinkedAxes.Clear();
                baseline.Base.LinkedAxes.AddRange(function_config.Base.LinkedAxes);
            });
        }

        private void uc_controller_axis_stick_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.Base.ControllerOutputAxis = e.Value;

            // The HID controller-output-axis lives in the Baseline layer, not the
            // override system, so persist it to the baseline and re-upload — otherwise
            // the merged config keeps the baseline's axis and the selection is lost.
            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                var axis = e.Value;
                plugin.ConfigOrchestrator.UpdateFunctionBaseline((int)function.ID,
                    baseline => baseline.Base.ControllerOutputAxis = axis);
            }
        }

        private void Rangeslider_travel_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events - if newValue doesn't match slider's current value, ignore
                if (Rangeslider_travel_range != null && Convert.ToInt16(Rangeslider_travel_range.LowerValue) != newValue)
                    return;

                var activeSub = stick_config;
                var oldValue = activeSub.PosMin;

                activeSub.PosMin = newValue;
                TieredConfig.FlightControlProcessor.ReconcileDerivedFields(function_config);

                // Create override for badge system (only after init stabilizes, baseline exists, AND value changed)
                if (allowOverrideCreation && newValue != oldValue && plugin != null && function != null && plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_control.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightControlMotionRange == null)
                                overrides.FlightControlMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightControlMotionRange.Min = newValue;
                        });
                }
            }
            if (Label_min_pos != null)
            {
                Label_min_pos.Content = String.Format("Min\n{0}mm", stick_config.PosMin);
            }
            _travelHelper?.UpdateTravelMarkers();
        }

        private void Rangeslider_travel_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events - if newValue doesn't match slider's current value, ignore
                if (Rangeslider_travel_range != null && Convert.ToInt16(Rangeslider_travel_range.UpperValue) != newValue)
                    return;

                var activeSub = stick_config;
                var oldValue = activeSub.PosMax;

                activeSub.PosMax = newValue;
                TieredConfig.FlightControlProcessor.ReconcileDerivedFields(function_config);

                // Create override for badge system (only after init stabilizes, baseline exists, AND value changed)
                if (allowOverrideCreation && newValue != oldValue && plugin != null && function != null && plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
                {
                    plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_control.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightControlMotionRange == null)
                                overrides.FlightControlMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightControlMotionRange.Max = newValue;
                        });
                }
            }
            if (Label_max_pos != null)
            {
                Label_max_pos.Content = String.Format("Max\n{0}mm", stick_config.PosMax);
            }
            _travelHelper?.UpdateTravelMarkers();
        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            stick_config.Damping = newValue;
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_control.damping",
                    overrides => overrides.FlightControlDamping = newValue);
            }
        }

        private void OnCentringSpringChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            stick_config.CenteringSpringConst = newValue;
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "flight_control.centering_spring_const",
                    overrides => overrides.FlightControlCenteringSpringConst = newValue);
            }
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

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;

            var newValue = (float)e.NewValue;

            // Update label immediately for user feedback
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", newValue);

            // Update function config
            function_config.SimulatedMass = newValue;

            // Create override for badge system (only if baseline exists to avoid config corruption)
            if (plugin != null && function != null && plugin.ConfigOrchestrator.HasFunctionBaseline((int)function.ID))
            {
                plugin.ConfigOrchestrator.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
                    overrides => overrides.SimulatedMass = newValue);
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
            if (Toggle_safety_damper != null && plugin != null)
                Toggle_safety_damper.IsChecked = plugin.IsFlightSafetyDamperEngaged();
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

        private void Toggle_safety_damper_Checked(object sender, RoutedEventArgs e)
        {
            if (isUpdatingOutputToggle || is_updating) return;
            plugin?.SetFlightSafetyDamperEngaged(true);
        }

        private void Toggle_safety_damper_Unchecked(object sender, RoutedEventArgs e)
        {
            if (isUpdatingOutputToggle || is_updating) return;
            plugin?.SetFlightSafetyDamperEngaged(false);
        }

        private void UpdateXPlaneTelemetry()
        {
            // Legacy X-Plane diagnostic UI removed - diagnostics now handled by FFB Parameters
            UpdateFfbOutputs();
        }

        private void UpdateFfbOutputs()
        {
            if (plugin == null)
            {
                return;
            }

            string prefix;
            switch (current_function_id)
            {
                case FunctionID.FlightStickPitch:
                    prefix = "FlightStickPitch";
                    break;
                case FunctionID.FlightStickRoll:
                    prefix = "FlightStickRoll";
                    break;
                case FunctionID.FlightStickCollective:
                    prefix = "FlightStickCollective";
                    break;
                default:
                    prefix = "FlightStickPitch";
                    break;
            }

            Label_Output_Spring.Content = plugin.GetGraphOutputValue($"{prefix}.SpringGain").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Damper.Content = plugin.GetGraphOutputValue($"{prefix}.DamperGain").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Friction.Content = plugin.GetGraphOutputValue($"{prefix}.Friction").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Load.Content = plugin.GetGraphOutputValue($"{prefix}.LoadForce").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_TrimOffset.Content = plugin.GetGraphOutputValue($"{prefix}.TrimOffset").ToString("F2", CultureInfo.InvariantCulture);
            Label_Output_Buffet.Content = plugin.GetGraphOutputValue($"{prefix}.BuffetAmplitude").ToString("F2", CultureInfo.InvariantCulture);
        }

        private void Rangeslider_travel_range_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            _travelHelper?.UpdateTravelMarkers();
        }

        private void ApplyFallbackTravelRange()
        {
            if (Rangeslider_travel_range == null)
            {
                return;
            }

            // Use reasonable fixed limits when axis kinematics not available
            // Don't use current pos min/max - that would lock the slider to current range!
            Rangeslider_travel_range.Minimum = -100;
            Rangeslider_travel_range.Maximum = 100;
        }
    }
}
