using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Globalization;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using SimHub.Plugins.OutputPlugins.ControlRemapper.Models;
using DiyFfb.Controls;

namespace DiyFfb
{
    /// <summary>
    /// Interaction logic for FunctionConfigControl.xaml
    /// </summary>
    public partial class FunctionConfigControl : UserControl
    {
        public FunctionConfigControl()
        {
            config = GetDefaultConfig(FunctionID.Undefined);
            InitializeComponent();
            Loaded += OnLoaded;
            Unloaded += OnUnloaded;
        }

        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public event AutomotivePedalConfigControl.ABSTestStateChangeEventHandler ABSTestStateChange;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private Function function;
        private FunctionConfig config;
        private FunctionID current_function_id;
        private bool updatingStaticBalanceUi;
        private bool allowOverrideCreation = false;

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            AutomotivePedalConfig.SetGui(ui, plugin);
            AutomotivePedalConfig.ABSTestStateChange += OnABSTestStateChange;
            AutomotivePedalConfig.DebugMessage += OnDebugMessage;
            FlightPedalsConfig.SetGui(ui, plugin);
            FlightStickConfig.SetGui(ui, plugin);
            ShifterConfig.SetGui(ui, plugin);

            // Subscribe to plugin events for badge refresh if already loaded
            if (IsLoaded && plugin != null)
            {
                plugin.ContextChanged += OnContextChanged;
                plugin.OverrideFieldChanged += OnOverrideFieldChanged;
            }
        }

        private void OnDebugMessage(string message)
        {
            DebugMessage?.Invoke(message);
        }

        private void OnABSTestStateChange(bool state)
        {
            ABSTestStateChange?.Invoke(state);
        }

        public static FunctionConfig GetDefaultConfig(FunctionID function_id)
        {
            FunctionConfig new_config = new FunctionConfig();
            new_config.Base = new FunctionBase();
            new_config.Base.FunctionId = function_id;
            new_config.Base.LinkedAxes.AddRange(new AxisID[4] { AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined });
            new_config.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning
            {
                Enabled = false,
                Gain = 1.0f
            };
            switch (function_id)
            {
                case FunctionID.BrakePedal:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.ClutchPedal:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.AcceleratorPedal:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.FlightPedals:
                    new_config.FlightPedals = FlightPedalsConfigControl.GetDefaultConfig();
                    new_config.AuxFunction = FlightPedalsConfigControl.GetRudderBrakeDefaultConfig(); 
                    break;
                case FunctionID.FlightStickPitch:
                    new_config.FlightStickPitch = FlightStickConfigControl.GetDefaultPitchConfig();
                    break;
                case FunctionID.FlightStickRoll:
                    new_config.FlightStickRoll = FlightStickConfigControl.GetDefaultRollConfig();
                    break;
                case FunctionID.FlightStickCollective:
                    new_config.FlightStickCollective = FlightStickConfigControl.GetDefaultCollectiveConfig();
                    break;
                case FunctionID.Shifter:
                    new_config.Shifter = ShifterConfigControl.GetDefaultConfig();
                    new_config.AuxFunction = ShifterConfigControl.GetDefaultDetectConfig();
                    break;
            }
            return new_config;
        }

        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            switch (tc_specific_function.SelectedIndex)
            {
                case 0:
                    AutomotivePedalConfig.OnKinematicParametersChanged(parameters);
                    break;
                case 1:
                    FlightPedalsConfig.OnKinematicParametersChanged(parameters);
                    break;
                case 2:
                    FlightStickConfig.OnKinematicParametersChanged(parameters);
                    break;
                case 3:
                    ShifterConfig.OnKinematicParametersChanged(parameters);
                    break;
            }
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            switch (tc_specific_function.SelectedIndex)
            {
                case 0:
                    AutomotivePedalConfig.OnAxisStateUpdate(axis_state);
                    break;
                case 1:
                    FlightPedalsConfig.OnAxisStateUpdate(axis_state);
                    break;
                case 2:
                    FlightStickConfig.OnAxisStateUpdate(axis_state);
                    break;
                case 3:
                    ShifterConfig.OnAxisStateUpdate(axis_state);
                    break;
                
            }
        }

        public void SwitchFunction(Function function)
        {
            this.function = function;
            config = function.Config;
            allowOverrideCreation = false;
            EnsureStaticBalanceTuningConfig();
            UpdateStaticBalanceTuningUi();

            // Defer badge initialization until after the UI has been fully rendered
            Dispatcher.BeginInvoke(new Action(() => InitializeBadges()), System.Windows.Threading.DispatcherPriority.ContextIdle);

            // Allow override creation only after all deferred events have been processed
            Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
                System.Windows.Threading.DispatcherPriority.ContextIdle);

            switch (function.ID)
            {
                case FunctionID.BrakePedal:
                    AutomotivePedalConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.ClutchPedal:
                    AutomotivePedalConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.AcceleratorPedal:
                    AutomotivePedalConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.FlightPedals:
                    FlightPedalsConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 1;
                    break;
                case FunctionID.FlightStickPitch:
                    FlightStickConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 2;
                    break;
                case FunctionID.FlightStickRoll:
                    FlightStickConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 2;
                    break;
                case FunctionID.FlightStickCollective:
                    FlightStickConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 2;
                    break;
                case FunctionID.Shifter:
                    ShifterConfig.SwitchFunction(function);
                    tc_specific_function.SelectedIndex = 3;
                    break;
            }
        }

        private FunctionConfig.Types.StaticBalanceTuning EnsureStaticBalanceTuningConfig()
        {
            if (config.StaticBalanceTuning == null)
            {
                config.StaticBalanceTuning = new FunctionConfig.Types.StaticBalanceTuning
                {
                    Enabled = false,
                    Gain = 1.0f
                };
            }
            return config.StaticBalanceTuning;
        }

        private void UpdateStaticBalanceTuningUi()
        {
            if (updatingStaticBalanceUi)
            {
                return;
            }
            var tuning = EnsureStaticBalanceTuningConfig();
            updatingStaticBalanceUi = true;
            ToggleStaticBalanceEnabled.IsChecked = tuning.Enabled;
            SliderStaticBalanceGain.Value = tuning.Gain;
            TextStaticBalanceGain.Text = tuning.Gain.ToString("0.###", CultureInfo.CurrentCulture);
            updatingStaticBalanceUi = false;
        }

        private void StaticBalanceEnabled_Checked(object sender, RoutedEventArgs e)
        {
            if (config == null || updatingStaticBalanceUi) return;
            EnsureStaticBalanceTuningConfig().Enabled = true;

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "static_balance_tuning.enabled",
                    overrides =>
                    {
                        if (overrides.StaticBalanceTuning == null)
                            overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                        overrides.StaticBalanceTuning.Enabled = true;
                    });
            }
        }

        private void StaticBalanceEnabled_Unchecked(object sender, RoutedEventArgs e)
        {
            if (config == null || updatingStaticBalanceUi) return;
            EnsureStaticBalanceTuningConfig().Enabled = false;

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "static_balance_tuning.enabled",
                    overrides =>
                    {
                        if (overrides.StaticBalanceTuning == null)
                            overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                        overrides.StaticBalanceTuning.Enabled = false;
                    });
            }
        }

        private void StaticBalanceGain_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (config == null || updatingStaticBalanceUi)
            {
                return;
            }
            var tuning = EnsureStaticBalanceTuningConfig();
            var newValue = (float)e.NewValue;
            tuning.Gain = newValue;
            updatingStaticBalanceUi = true;
            TextStaticBalanceGain.Text = tuning.Gain.ToString("0.###", CultureInfo.CurrentCulture);
            updatingStaticBalanceUi = false;

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "static_balance_tuning.gain",
                    overrides =>
                    {
                        if (overrides.StaticBalanceTuning == null)
                            overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                        overrides.StaticBalanceTuning.Gain = newValue;
                    });
            }
        }

        private void StaticBalanceGain_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (config == null || updatingStaticBalanceUi)
            {
                return;
            }
            if (float.TryParse(TextStaticBalanceGain.Text, NumberStyles.Float, CultureInfo.CurrentCulture, out float value))
            {
                var tuning = EnsureStaticBalanceTuningConfig();
                tuning.Gain = value;
                updatingStaticBalanceUi = true;
                SliderStaticBalanceGain.Value = value;
                updatingStaticBalanceUi = false;

                if (allowOverrideCreation && plugin != null && function != null &&
                    plugin.HasFunctionBaseline((int)function.ID))
                {
                    plugin.UpdateFunctionOverrideField((int)function.ID, "static_balance_tuning.gain",
                        overrides =>
                        {
                            if (overrides.StaticBalanceTuning == null)
                                overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                            overrides.StaticBalanceTuning.Gain = value;
                        });
                }
            }
        }

        private void StaticBalanceGain_LostFocus(object sender, RoutedEventArgs e)
        {
            UpdateStaticBalanceTuningUi();
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            if (plugin != null)
            {
                plugin.ContextChanged += OnContextChanged;
                plugin.OverrideFieldChanged += OnOverrideFieldChanged;
            }

            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.OverrideCleared += OnBadgeOverrideCleared;
            }

            // Initialize badges for the currently displayed function after UI is fully loaded
            Dispatcher.BeginInvoke(new Action(() => InitializeBadges()), System.Windows.Threading.DispatcherPriority.ContextIdle);
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            if (plugin != null)
            {
                plugin.ContextChanged -= OnContextChanged;
                plugin.OverrideFieldChanged -= OnOverrideFieldChanged;
            }

            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.OverrideCleared -= OnBadgeOverrideCleared;
            }
        }

        private void OnBadgeOverrideCleared(object sender, LayerBadgeWrapper.OverrideClearedEventArgs e)
        {
            if (plugin == null || function == null) return;

            var mergedConfig = plugin.FunctionConfigManager.GetCurrentConfig((int)function.ID);
            if (mergedConfig == null) return;

            updatingStaticBalanceUi = true;
            switch (e.FieldPath)
            {
                case "static_balance_tuning.enabled":
                    var tuningEnabled = mergedConfig.StaticBalanceTuning ?? new FunctionConfig.Types.StaticBalanceTuning { Enabled = false, Gain = 1.0f };
                    config.StaticBalanceTuning.Enabled = tuningEnabled.Enabled;
                    ToggleStaticBalanceEnabled.IsChecked = tuningEnabled.Enabled;
                    break;

                case "static_balance_tuning.gain":
                    var tuningGain = mergedConfig.StaticBalanceTuning ?? new FunctionConfig.Types.StaticBalanceTuning { Enabled = false, Gain = 1.0f };
                    config.StaticBalanceTuning.Gain = tuningGain.Gain;
                    SliderStaticBalanceGain.Value = tuningGain.Gain;
                    TextStaticBalanceGain.Text = tuningGain.Gain.ToString("0.###", CultureInfo.CurrentCulture);
                    break;

                case "static_balance_tuning":
                    var tuningFull = mergedConfig.StaticBalanceTuning ?? new FunctionConfig.Types.StaticBalanceTuning { Enabled = false, Gain = 1.0f };
                    config.StaticBalanceTuning.Enabled = tuningFull.Enabled;
                    config.StaticBalanceTuning.Gain = tuningFull.Gain;
                    ToggleStaticBalanceEnabled.IsChecked = tuningFull.Enabled;
                    SliderStaticBalanceGain.Value = tuningFull.Gain;
                    TextStaticBalanceGain.Text = tuningFull.Gain.ToString("0.###", CultureInfo.CurrentCulture);
                    break;
            }
            updatingStaticBalanceUi = false;
        }

        private void OnContextChanged(object sender, EventArgs e)
        {
            RefreshAllBadges();
        }

        private void OnOverrideFieldChanged(object sender, OverrideFieldChangedEventArgs e)
        {
            // If the changed field is for our current function, refresh that badge
            if (function != null && e.FunctionId == (int)function.ID)
            {
                RefreshBadgeForField(e.FieldPath);
            }
        }

        public void RefreshAllBadges()
        {
            // Refresh all LayerBadgeWrapper controls in the visual tree
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.UpdateBadge();
            }
        }

        private void RefreshBadgeForField(string fieldPath)
        {
            // Find and refresh the badge matching the field path
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
                if (child is T t)
                {
                    yield return t;
                }
                foreach (var descendant in FindVisualChildren<T>(child))
                {
                    yield return descendant;
                }
            }
        }

        private void InitializeBadges()
        {
            if (plugin == null || function == null) return;

            int functionId = (int)function.ID;

            // Set Plugin and FunctionId on all badge wrappers found in the visual tree
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(this))
            {
                wrapper.Plugin = plugin;
                wrapper.FunctionId = functionId;
            }
        }
    }
}
