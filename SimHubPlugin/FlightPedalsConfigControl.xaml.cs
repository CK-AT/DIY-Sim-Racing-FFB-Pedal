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
using DiyFfb.GraphEditor;
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
        private double latestAxisPosition;
        private bool hasAxisPosition;
        private double latestAxisForce;
        private double latestTrimCenter;
        private bool hasTrimCenter;
        private DispatcherTimer xplaneTimer;
        private bool hasAxisRange;
        private Dictionary<string, FrameworkElement> graphParamControls = new Dictionary<string, FrameworkElement>();
        private Dictionary<string, Label> graphParamLabels = new Dictionary<string, Label>();
        private bool isUpdatingGraphParams = false;
        private bool isUpdatingOutputToggle = false;
        private BadgeHelper _badgeHelper;

        public FlightPedalsConfigControl()
        {
            config = GetDefaultConfig();
            InitializeComponent();
            Loaded += OnLoaded;
            Unloaded += OnUnloaded;
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            _badgeHelper = new BadgeHelper(this, () => this.plugin, () => function, OnBadgeOverrideCleared);

            if (plugin != null)
            {
                plugin.ActiveGraphChanged += OnActiveGraphChanged;
                plugin.GraphParamChanged += OnGraphParamChanged;

                if (IsLoaded)
                    _badgeHelper.Subscribe();
            }

            is_updating = false;
            StartXPlaneTimer();
            RefreshGraphParams();
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
                    UpdateTravelMarkers();
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
            double min = parameters.ContactPointPosMinAbs / 10.0f;
            double max = parameters.ContactPointPosMaxAbs / 10.0f;
            double newMin = Math.Min(min, max);
            double newMax = Math.Max(min, max);

            // Skip degenerate bounds (e.g., from uncomputed ESP32 KinematicParameters)
            if (newMin >= newMax)
                return;

            hasAxisRange = true;

            bool wasUpdating = is_updating;
            if (!wasUpdating) is_updating = true;

            Rangeslider_travel_range.Minimum = newMin;
            Rangeslider_travel_range.Maximum = newMax;

            // Restore slider values from config to counteract WPF clamping
            Rangeslider_travel_range.LowerValue = config.PosNearLim;
            Rangeslider_travel_range.UpperValue = config.PosFarLim;

            if (!wasUpdating)
            {
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
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;

            // Update labels with config values (event handlers were blocked by is_updating flag)
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", function_config.SimulatedMass);
            label_friction.Content = String.Format("Friction: {0:F1}N", function_config.Friction);
            label_centering_spring_const.Content = String.Format("Centering Spring Constant: {0:F2}N/mm", config.CenteringSpringConst);
            label_damping.Content = String.Format("Damping: {0:F3}N*mm/s", config.Damping);
            RefreshGraphParams();
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
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.damping",
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
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.centering_spring_const",
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
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
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
                    plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "aux_function.rudder_brake.force_range",
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
                    plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "aux_function.rudder_brake.force_range",
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
                    plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
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
            UpdateTravelMarkers();
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
                    plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "flight_pedals.motion_range",
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
                var orderedNames = plugin.GetActiveGraphParamOrder();
                var filteredParams = allParams.Values
                    .Where(p => MatchesGroup(p.Ui?.Group, groupFilter))
                    .ToList();
                var orderedParams = OrderParamsByGraph(orderedNames, filteredParams);

                foreach (var param in orderedParams)
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

        private static List<GraphParam> OrderParamsByGraph(IReadOnlyList<string> orderedNames, IEnumerable<GraphParam> parameters)
        {
            var map = new Dictionary<string, GraphParam>(StringComparer.OrdinalIgnoreCase);
            foreach (var param in parameters)
            {
                if (!string.IsNullOrWhiteSpace(param?.Name))
                {
                    map[param.Name] = param;
                }
            }

            var ordered = new List<GraphParam>();
            if (orderedNames != null)
            {
                foreach (var name in orderedNames)
                {
                    if (map.TryGetValue(name, out var param))
                    {
                        ordered.Add(param);
                        map.Remove(name);
                    }
                }
            }

            ordered.AddRange(map.Values.OrderBy(p => p.Ui?.Label ?? p.Name));
            return ordered;
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
