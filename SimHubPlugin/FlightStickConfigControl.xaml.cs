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
        private double latestAxisForce;
        private bool hasAxisForce;
        private double latestTrimCenter;
        private bool hasTrimCenter;
        private DispatcherTimer xplaneTimer;
        private bool hasAxisRange;
        private Dictionary<string, FrameworkElement> graphParamControls = new Dictionary<string, FrameworkElement>();
        private Dictionary<string, Label> graphParamLabels = new Dictionary<string, Label>();
        private bool isUpdatingGraphParams = false;
        private bool isUpdatingOutputToggle = false;

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

            if (plugin != null)
            {
                plugin.ActiveGraphChanged += OnActiveGraphChanged;
                plugin.GraphParamChanged += OnGraphParamChanged;

                // Subscribe to plugin events for badge refresh if already loaded
                if (IsLoaded)
                {
                    plugin.ContextChanged += OnContextChanged;
                    plugin.OverrideFieldChanged += OnOverrideFieldChanged;
                }
            }

            is_updating = false;
            StartXPlaneTimer();
            RefreshGraphParams();
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            if (plugin != null)
            {
                plugin.ContextChanged += OnContextChanged;
                plugin.OverrideFieldChanged += OnOverrideFieldChanged;
            }
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            if (plugin != null)
            {
                plugin.ContextChanged -= OnContextChanged;
                plugin.OverrideFieldChanged -= OnOverrideFieldChanged;
            }
        }

        private void OnContextChanged(object sender, EventArgs e)
        {
            RefreshAllBadges();
        }

        private void OnOverrideFieldChanged(object sender, OverrideFieldChangedEventArgs e)
        {
            if (function != null && e.FunctionId == (int)function.ID)
            {
                RefreshBadgeForField(e.FieldPath);
            }
        }

        private void RefreshAllBadges()
        {
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.UpdateBadge();
            }
        }

        private void RefreshBadgeForField(string fieldPath)
        {
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                if (wrapper.FieldPath == fieldPath)
                {
                    wrapper.UpdateBadge();
                }
            }
        }

        private static IEnumerable<T> FindVisualChildren<T>(DependencyObject parent) where T : DependencyObject
        {
            if (parent == null) yield break;
            for (int i = 0; i < VisualTreeHelper.GetChildrenCount(parent); i++)
            {
                var child = VisualTreeHelper.GetChild(parent, i);
                if (child is T t) yield return t;
                foreach (var descendant in FindVisualChildren<T>(child)) yield return descendant;
            }
        }

        private void InitializeBadges()
        {
            if (plugin == null || function == null) return;
            int functionId = (int)function.ID;
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.Plugin = plugin;
                wrapper.FunctionId = functionId;
            }
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

            // Update label for changed parameter
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

                    // Also update the control value if it exists
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

            Rangeslider_travel_range.LowerValue = GetPosMin();
            function_config.Base.OutputMin = GetPosMin();
            Rangeslider_travel_range.UpperValue = GetPosMax();
            function_config.Base.OutputMax = GetPosMax();
            UpdateTrimCenter();
            UpdateTravelMarkers();
            is_updating = false;
            RefreshGraphParams();
            UpdateDisableOutputsToggle();
            InitializeBadges();
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
            if (is_updating) return;

            var newValue = (float)e.NewValue;
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", newValue);

            // Keep old direct edit for immediate UI feedback
            function_config.SimulatedMass = newValue;

            // Also create override for badge system
            if (plugin != null && function != null)
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "simulated_mass",
                    overrides => overrides.SimulatedMass = newValue);
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

                // Filter by function group
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
                                    // Update label to show new value
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
                SimHub.Logging.Current.Error($"[FlightStick] RefreshGraphParams failed: {ex.Message}", ex);
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

        private string FormatParamLabel(GraphParam param, double currentValue)
        {
            string label = param.Ui?.Label ?? param.Name;

            // Format value with appropriate precision
            int precision = param.Ui?.Precision ?? 3;
            string valueStr = currentValue.ToString($"F{precision}");

            // Build label as: <name>: <value><unit>
            if (!string.IsNullOrWhiteSpace(param.Ui?.Units))
            {
                return $"{label}: {valueStr}{param.Ui.Units}";
            }
            else
            {
                return $"{label}: {valueStr}";
            }
        }

        private string GetGraphParamGroupFilter()
        {
            switch (current_function_id)
            {
                case FunctionID.FlightStickPitch:
                    return "FlightStickPitch";
                case FunctionID.FlightStickRoll:
                    return "FlightStickRoll";
                case FunctionID.FlightStickCollective:
                    return "FlightStickCollective";
                default:
                    return "";
            }
        }

        private bool MatchesGroup(string paramGroup, string filter)
        {
            if (string.IsNullOrWhiteSpace(filter))
                return false; // No filter = show nothing (avoid clutter)
            if (string.IsNullOrWhiteSpace(paramGroup))
                return false; // Param has no group = don't show
            return paramGroup.StartsWith(filter, StringComparison.OrdinalIgnoreCase);
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

            double min = Math.Min(GetPosMin(), GetPosMax());
            double max = Math.Max(GetPosMin(), GetPosMax());
            Rangeslider_travel_range.Minimum = min;
            Rangeslider_travel_range.Maximum = max;
        }
    }
}
