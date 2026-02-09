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
        private FlightStickPitchConfig pitch_config;
        private FlightStickRollConfig roll_config;
        private FlightStickCollectiveConfig collective_config;
        private bool is_updating = true;
        private bool allowOverrideCreation = false;  // Only true after initial load stabilizes
        private double latestAxisPosition;
        private bool hasAxisPosition;
        private double latestAxisForce;
        private double latestTrimCenter;
        private bool hasTrimCenter;
        private DispatcherTimer xplaneTimer;
        private bool hasAxisRange;
        private bool isUpdatingOutputToggle = false;
        private BadgeHelper _badgeHelper;
        private GraphParamHelper _graphParamHelper;

        public FlightStickConfigControl()
        {
            pitch_config = GetDefaultPitchConfig();
            roll_config = GetDefaultRollConfig();
            collective_config = GetDefaultCollectiveConfig();
            InitializeComponent();
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
            // When an override is cleared, update the slider to show the baseline value
            if (plugin == null || function == null)
                return;

            // Get the merged config (now without the cleared override)
            var mergedConfig = plugin.FunctionConfigManager.GetCurrentConfig((int)function.ID);
            if (mergedConfig == null)
                return;

            // Update function_config and UI based on which field was cleared
            is_updating = true;
            switch (e.FieldPath)
            {
                case "simulated_mass":
                    function_config.SimulatedMass = mergedConfig.SimulatedMass;
                    Slider_simulated_mass.Value = mergedConfig.SimulatedMass;
                    label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", mergedConfig.SimulatedMass);
                    break;

                case "flight_stick.motion_range":
                    // Get pos min/max from merged config's mode-specific sub-config
                    int posMin, posMax;
                    switch (GetMode())
                    {
                        case FlightStickMode.Roll:
                            posMin = mergedConfig.FlightStickRoll.PosMin;
                            posMax = mergedConfig.FlightStickRoll.PosMax;
                            break;
                        case FlightStickMode.Collective:
                            posMin = mergedConfig.FlightStickCollective.PosMin;
                            posMax = mergedConfig.FlightStickCollective.PosMax;
                            break;
                        default:
                            posMin = mergedConfig.FlightStickPitch.PosMin;
                            posMax = mergedConfig.FlightStickPitch.PosMax;
                            break;
                    }

                    SetPosMin(posMin);
                    SetPosMax(posMax);
                    TieredConfig.FlightStickProcessor.ReconcileDerivedFields(function_config);
                    if (Label_min_pos != null)
                        Label_min_pos.Content = String.Format("Min\n{0}mm", posMin);
                    if (Label_max_pos != null)
                        Label_max_pos.Content = String.Format("Max\n{0}mm", posMax);
                    UpdateTravelMarkers();
                    break;

                case "flight_stick.damping":
                    float mergedDamping;
                    switch (GetMode())
                    {
                        case FlightStickMode.Roll: mergedDamping = mergedConfig.FlightStickRoll.Damping; break;
                        case FlightStickMode.Collective: mergedDamping = mergedConfig.FlightStickCollective.Damping; break;
                        default: mergedDamping = mergedConfig.FlightStickPitch.Damping; break;
                    }
                    SetDamping(mergedDamping);
                    Slider_damping.Value = mergedDamping;
                    label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", mergedDamping);
                    break;

                case "flight_stick.centering_spring_const":
                    float mergedSpring;
                    switch (GetMode())
                    {
                        case FlightStickMode.Roll: mergedSpring = mergedConfig.FlightStickRoll.CenteringSpringConst; break;
                        case FlightStickMode.Collective: mergedSpring = mergedConfig.FlightStickCollective.CenteringSpringConst; break;
                        default: mergedSpring = mergedConfig.FlightStickPitch.CenteringSpringConst; break;
                    }
                    SetCenteringSpringConst(mergedSpring);
                    Slider_centering_spring_const.Value = mergedSpring;
                    label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", mergedSpring);
                    break;

                case "friction":
                    function_config.Friction = mergedConfig.Friction;
                    Slider_friction.Value = mergedConfig.Friction;
                    label_friction.Content = String.Format("Friction: {0:F1}N", mergedConfig.Friction);
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
            double min = parameters.ContactPointPosMinAbs / 10.0f;
            double max = parameters.ContactPointPosMaxAbs / 10.0f;
            double newMin = Math.Min(min, max);
            double newMax = Math.Max(min, max);

            // Skip degenerate bounds (e.g., from uncomputed ESP32 KinematicParameters
            // where both fields default to 0)
            if (newMin >= newMax)
                return;

            hasAxisRange = true;

            // Changing slider bounds can clamp current values, firing ValueChanged events.
            // When called from SwitchFunction, is_updating is already true (safe).
            // When called externally (e.g., axis tab timer), we must block those events
            // to prevent clamped values from overwriting the config.
            bool wasUpdating = is_updating;
            if (!wasUpdating) is_updating = true;

            Rangeslider_travel_range.Minimum = newMin;
            Rangeslider_travel_range.Maximum = newMax;

            // Restore slider values from config to counteract WPF clamping.
            // Without this, async bounds changes leave the slider thumbs at
            // clamped positions even though the config values are correct.
            Rangeslider_travel_range.LowerValue = GetPosMin();
            Rangeslider_travel_range.UpperValue = GetPosMax();

            if (!wasUpdating)
            {
                // Defer clearing is_updating until ContextIdle so that any WPF
                // deferred clamping events are also suppressed.
                Dispatcher.BeginInvoke(new Action(() => is_updating = false),
                    System.Windows.Threading.DispatcherPriority.ContextIdle);
            }
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
            Slider_centering_spring_const.Value = GetCenteringSpringConst();
            Slider_damping.Value = GetDamping();

            Rangeslider_travel_range.LowerValue = GetPosMin();
            Rangeslider_travel_range.UpperValue = GetPosMax();
            TieredConfig.FlightStickProcessor.ReconcileDerivedFields(function_config);
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;

            // Update labels with merged config values (event handlers were blocked by is_updating flag)
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", function_config.SimulatedMass);
            label_friction.Content = String.Format("Friction: {0:F1}N", function_config.Friction);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", GetCenteringSpringConst());
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", GetDamping());
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
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events - if newValue doesn't match slider's current value, ignore
                if (Rangeslider_travel_range != null && Convert.ToInt16(Rangeslider_travel_range.LowerValue) != newValue)
                    return;

                var oldValue = GetPosMin();

                SetPosMin(newValue);
                TieredConfig.FlightStickProcessor.ReconcileDerivedFields(function_config);

                // Create override for badge system (only after init stabilizes, baseline exists, AND value changed)
                if (allowOverrideCreation && newValue != oldValue && plugin != null && function != null && plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_stick.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightStickMotionRange == null)
                                overrides.FlightStickMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightStickMotionRange.Min = newValue;
                        });
                }
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
                var newValue = Convert.ToInt16(e.NewValue);

                // Skip stale deferred events - if newValue doesn't match slider's current value, ignore
                if (Rangeslider_travel_range != null && Convert.ToInt16(Rangeslider_travel_range.UpperValue) != newValue)
                    return;

                var oldValue = GetPosMax();

                SetPosMax(newValue);
                TieredConfig.FlightStickProcessor.ReconcileDerivedFields(function_config);

                // Create override for badge system (only after init stabilizes, baseline exists, AND value changed)
                if (allowOverrideCreation && newValue != oldValue && plugin != null && function != null && plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_stick.motion_range",
                        overrides =>
                        {
                            if (overrides.FlightStickMotionRange == null)
                                overrides.FlightStickMotionRange = new TieredConfig.MotionRangeOverrides();
                            overrides.FlightStickMotionRange.Max = newValue;
                        });
                }
            }
            if (Label_max_pos != null)
            {
                Label_max_pos.Content = String.Format("Max\n{0}mm", GetPosMax());
            }
            UpdateTravelMarkers();
        }

        private void OnDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            SetDamping(newValue);
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "flight_stick.damping",
                    overrides => overrides.FlightStickDamping = newValue);
            }
        }

        private void OnCentringSpringChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            SetCenteringSpringConst(newValue);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "flight_stick.centering_spring_const",
                    overrides => overrides.FlightStickCenteringSpringConst = newValue);
            }
        }

        private void OnFrictionChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (is_updating) return;
            var newValue = (float)e.NewValue;
            label_friction.Content = String.Format("Friction: {0:F1}N", newValue);
            function_config.Friction = newValue;

            if (plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "friction",
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
            if (plugin != null && function != null && plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
                    overrides => overrides.SimulatedMass = newValue);
            }
        }

        private void UpdateTrimCenter()
        {
            float trimMm = 0.0f;
            hasTrimCenter = plugin != null && plugin.TryGetGraphTrimOffset(current_function_id, out trimMm);
            if (hasTrimCenter)
            {
                latestTrimCenter = trimMm;
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

            // Use reasonable fixed limits when axis kinematics not available
            // Don't use current pos min/max - that would lock the slider to current range!
            Rangeslider_travel_range.Minimum = -100;
            Rangeslider_travel_range.Maximum = 100;
        }
    }
}
