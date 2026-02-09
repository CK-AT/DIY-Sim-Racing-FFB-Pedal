using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using MahApps.Metro.Controls;
using DiyFfb.Controls;
using DiyFfb.TieredConfig;

namespace DiyFfb
{
    /// <summary>
    /// Interaction logic for AutomotivePedalEffects.xaml
    /// </summary>
    public partial class AutomotivePedalConfigControl : UserControl
    {
        public event FunctionConfigControl.DebugMessageEventHandler DebugMessage;
        public delegate void ABSTestStateChangeEventHandler(bool state);
        public event ABSTestStateChangeEventHandler ABSTestStateChange;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private AutomotivePedalConfig config;
        private FunctionConfig function_config = new FunctionConfig();
        private Function function;
        private FunctionID current_function_id;
        private bool update_lockout = false;
        private bool allowOverrideCreation = false;

        public AutomotivePedalConfigControl()
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
            AutomotivePedal_SplineForceCurve.SetGui(ui, plugin);
            AutomotivePedal_SplineForceCurve.RangeSettingsChanged += OnRangeSettingsChanged;

            // Subscribe to plugin events for badge refresh if already loaded
            if (IsLoaded && plugin != null)
            {
                plugin.ContextChanged += OnContextChanged;
                plugin.OverrideFieldChanged += OnOverrideFieldChanged;
            }
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

            update_lockout = true;
            switch (e.FieldPath)
            {
                case "damper_config.positive_factor":
                    config.DamperConfig.PositiveFactor = mergedConfig.AutomotivePedal.DamperConfig.PositiveFactor;
                    Slider_damping_push.Value = config.DamperConfig.PositiveFactor;
                    label_damping_push.Content = String.Format("Damping (Push): {0:F3}N*mm/s", config.DamperConfig.PositiveFactor);
                    break;

                case "damper_config.negative_factor":
                    config.DamperConfig.NegativeFactor = mergedConfig.AutomotivePedal.DamperConfig.NegativeFactor;
                    Slider_damping_pull.Value = config.DamperConfig.NegativeFactor;
                    label_damping_pull.Content = String.Format("Damping (Pull): {0:F3}N*mm/s", config.DamperConfig.NegativeFactor);
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

                case "force_curve":
                    config.ForceCurveConfig = mergedConfig.AutomotivePedal.ForceCurveConfig.Clone();
                    allowOverrideCreation = false;
                    AutomotivePedal_SplineForceCurve.UpdateConfig(config.ForceCurveConfig);
                    Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
                        System.Windows.Threading.DispatcherPriority.ContextIdle);
                    break;
            }
            update_lockout = false;
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
            for (int i = 0; i < System.Windows.Media.VisualTreeHelper.GetChildrenCount(parent); i++)
            {
                var child = System.Windows.Media.VisualTreeHelper.GetChild(parent, i);
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

        private void OnRangeSettingsChanged(SplineForceCurve spline_force_curve)
        {
            AutomotivePedalProcessor.ReconcileDerivedFields(function_config);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "force_curve",
                    overrides => overrides.ForceCurve = config.ForceCurveConfig.Clone());
            }
        }

        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            AutomotivePedal_SplineForceCurve.OnKinematicParametersChanged(parameters);
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (function_config.Base.LinkedAxes[0] == axis_state.AxisId)
            {
                AutomotivePedal_SplineForceCurve.OnAxisStateUpdate(axis_state);
            }
        }

        public static AutomotivePedalConfig GetDefaultConfig()
        {
            AutomotivePedalConfig new_config = new AutomotivePedalConfig();
            new_config.ForceCurveConfig = SplineForceCurve.GetDefaultConfig();
            new_config.DamperConfig = new DamperConfig();
            new_config.DamperConfig.PositiveFactor = 0.25f;
            new_config.DamperConfig.NegativeFactor = 0.25f;
            new_config.AbsEffectConfig = new ABSEffectConfig();
            new_config.AbsEffectConfig.Enabled = false;
            new_config.RpmEffectConfig = new RPMEffectConfig();
            new_config.RpmEffectConfig.Enabled = false;
            new_config.BitePointEffectConfig = new BitePointEffectConfig();
            new_config.BitePointEffectConfig.Enabled = false;
            new_config.GForceEffectConfig = new GForceEffectConfig();
            new_config.GForceEffectConfig.Enabled = false;
            new_config.WheelSlipEffectConfig = new WheelSlipEffectConfig();
            new_config.WheelSlipEffectConfig.Enabled = false;
            new_config.RoadImpactEffectConfig = new RoadImpactEffectConfig();
            new_config.RoadImpactEffectConfig.Enabled = false;
            new_config.Cv1EffectConfig = new CustomVibrationEffectConfig();
            new_config.Cv1EffectConfig.Enabled = false;
            new_config.Cv2EffectConfig = new CustomVibrationEffectConfig();
            new_config.Cv2EffectConfig.Enabled = false;
            return new_config;
        }
        public void SwitchFunction(Function function)
        {
            this.function = function;
            function_config = function.Config;
            config = function_config.AutomotivePedal;
            current_function_id = function_config.Base.FunctionId;
            allowOverrideCreation = false;

            update_lockout = true;

            AutomotivePedal_AxisSelector.Value = function_config.Base.LinkedAxes[0];
            AutomotivePedal_SplineForceCurve.ResetAxisRange();

            if (function_config.Base.LinkedAxes[0] != AxisID.AxisUndefined)
            {
                var kinematic_parameters = ui.GetKinematicParameters(function_config.Base.LinkedAxes[0]);
                if (kinematic_parameters != null)
                {
                    OnKinematicParametersChanged(kinematic_parameters);
                }
            }

            Slider_simulated_mass.Value = function_config.SimulatedMass;
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", function_config.SimulatedMass);
            Slider_friction.Value = function_config.Friction;
            label_friction.Content = String.Format("Friction: {0:F1}N", function_config.Friction);

            switch (function_config.Base.OutputMode)
            {
                case OutputMode.Force:
                    cb_controller_output_mode.SelectedIndex = 0;
                    break;
                case OutputMode.Travel:
                    cb_controller_output_mode.SelectedIndex = 1;
                    break;
                default:
                    break;
            }

            AutomotivePedal_ControllerAxisSelector.Value = function_config.Base.ControllerOutputAxis;

            AutomotivePedal_SplineForceCurve.UpdateConfig(config.ForceCurveConfig);

            if (config.DamperConfig == null)
            {
                config.DamperConfig = new DamperConfig();
                config.DamperConfig.PositiveFactor = 0.25f;
                config.DamperConfig.NegativeFactor = 0.25f;
            }
            Slider_damping_push.Value = config.DamperConfig.PositiveFactor;
            label_damping_push.Content = String.Format("Damping (Push): {0:F3}N*mm/s", config.DamperConfig.PositiveFactor);
            Slider_damping_pull.Value = config.DamperConfig.NegativeFactor;
            label_damping_pull.Content = String.Format("Damping (Pull): {0:F3}N*mm/s", config.DamperConfig.NegativeFactor);

            if (config.RoadImpactEffectConfig == null) config.RoadImpactEffectConfig = new RoadImpactEffectConfig();
            Slider_impact_smoothness.Value = config.RoadImpactEffectConfig.Window;
            label_impact_window.Content = "Impact Smoothness: " + config.RoadImpactEffectConfig.Window;
            Slider_impact_multi.Value = config.RoadImpactEffectConfig.Multi;
            label_impact_multi.Content = "Impact Multiplier: " + config.RoadImpactEffectConfig.Multi + "%";

            if (config.WheelSlipEffectConfig == null) config.WheelSlipEffectConfig = new WheelSlipEffectConfig();
            Slider_WS_freq.Value = config.WheelSlipEffectConfig.Freq;
            label_WS_freq.Content = "Notification Frequency: " + config.WheelSlipEffectConfig.Freq + "Hz";
            Slider_WS_AMP.Value = (float)(config.WheelSlipEffectConfig.Amp);
            label_WS_AMP.Content = "Notification Amplitude: " + (float)config.WheelSlipEffectConfig.Amp + "kg";
            Slider_WS_trigger.Value = plugin.Settings.WS_trigger;
            label_WS_trigger.Content = "Notification Trigger: " + (plugin.Settings.WS_trigger + 50) + "%";

            if (config.GForceEffectConfig == null) config.GForceEffectConfig = new GForceEffectConfig();
            Slider_G_force_smoothness.Value = config.GForceEffectConfig.Window;
            label_G_force_window.Content = "G Force Smoothness: " + config.GForceEffectConfig.Window;
            Slider_G_force_multi.Value = config.GForceEffectConfig.Multi;
            label_G_force_multi.Content = "G Force Multiplier: " + config.GForceEffectConfig.Multi + "%";

            if (config.BitePointEffectConfig == null) config.BitePointEffectConfig = new BitePointEffectConfig();
            Slider_BP_freq.Value = config.BitePointEffectConfig.Freq;
            label_BP_freq.Content = "Bite Point Frequency: " + config.BitePointEffectConfig.Freq + "Hz";
            Slider_BP_AMP.Value = (float)(config.BitePointEffectConfig.Amp) / 9.81f;
            label_BP_AMP.Content = "Bite Point Amplitude: " + (float)config.BitePointEffectConfig.Amp / 9.81f + "kg";

            if (config.RpmEffectConfig == null) config.RpmEffectConfig = new RPMEffectConfig();
            Rangeslider_RPM_freq.LowerValue = config.RpmEffectConfig.MinFreq;
            Rangeslider_RPM_freq.UpperValue = config.RpmEffectConfig.MaxFreq;
            label_RPM_freq_max.Content = "MAX:" + config.RpmEffectConfig.MaxFreq + "Hz";
            label_RPM_freq_min.Content = "MIN:" + config.RpmEffectConfig.MinFreq + "Hz";
            Slider_RPM_AMP.Value = (float)(config.RpmEffectConfig.Amp) / 9.81f;
            label_RPM_AMP.Content = "Effect Amplitude: " + (float)(config.RpmEffectConfig.Amp) / 9.81f + "kg";

            if (config.AbsEffectConfig == null) config.AbsEffectConfig = new ABSEffectConfig();
            Slider_ABS_freq.Value = config.AbsEffectConfig.Freq;
            label_ABS_freq.Content = "ABS/TC Frequency: " + config.AbsEffectConfig.Freq + "Hz";

            if (config.Cv1EffectConfig == null) config.Cv1EffectConfig = new CustomVibrationEffectConfig();
            if (config.Cv2EffectConfig == null) config.Cv2EffectConfig = new CustomVibrationEffectConfig();
            checkbox_enable_CV1.IsChecked = config.Cv1EffectConfig.Enabled;
            checkbox_enable_CV2.IsChecked = config.Cv2EffectConfig.Enabled;

            Slider_CV1_trigger.Value = plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_trigger_level;
            Slider_CV1_AMP.Value = (float)config.Cv1EffectConfig.Amp / 9.81f;
            Slider_CV1_freq.Value = config.Cv1EffectConfig.Freq;
            Slider_CV2_trigger.Value = plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_trigger_level;
            Slider_CV2_AMP.Value = (float)config.Cv2EffectConfig.Amp / 9.81f;
            Slider_CV2_freq.Value = config.Cv2EffectConfig.Freq;
            label_CV1_trigger.Content = "Effect Trigger:" + plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_trigger_level;
            label_CV1_AMP.Content = "Effect Amplitude:" + (float)config.Cv1EffectConfig.Amp / 9.81f + "kg";
            label_CV1_freq.Content = "Effect Frequency:" + config.Cv1EffectConfig.Freq + "Hz";
            label_CV2_trigger.Content = "Effect Trigger:" + plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_trigger_level;
            label_CV2_AMP.Content = "Effect Amplitude:" + (float)config.Cv2EffectConfig.Amp / 9.81f + "kg";
            label_CV2_freq.Content = "Effect Frequency:" + config.Cv2EffectConfig.Freq + "Hz";
            textBox_CV1_string.Text = plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_binding;
            textBox_CV2_string.Text = plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_binding;

            switch (config.AbsEffectConfig.Mode)
            {
                case ABSMode.Force:
                    label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}kg", config.AbsEffectConfig.Ampl / 9.81f);
                    Slider_ABS_AMP.Value = Math.Round((float)config.AbsEffectConfig.Ampl / 9.81);
                    EffectAppliedOnForceOrTravel_combobox.SelectedIndex = 0;
                    break;
                case ABSMode.Travel:
                    label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}mm", config.AbsEffectConfig.Ampl);
                    Slider_ABS_AMP.Value = (float)config.AbsEffectConfig.Ampl;
                    EffectAppliedOnForceOrTravel_combobox.SelectedIndex = 1;
                    AbsPattern.SelectedIndex = 1;
                    break;
                default:
                    config.AbsEffectConfig.Mode = ABSMode.Force;
                    label_ABS_AMP.Content = "ABS/TC Amplitude: " + (float)config.AbsEffectConfig.Ampl / 9.81f + "kg";
                    Slider_ABS_AMP.Value = Math.Round((float)config.AbsEffectConfig.Ampl / 9.81);
                    EffectAppliedOnForceOrTravel_combobox.SelectedIndex = 0;
                    break;
            }
            //Simulated ABS trigger
            Simulate_ABS_check.IsChecked = (config.AbsEffectConfig.SimLevel > 0);

            switch (config.AbsEffectConfig.Pattern)
            {
                case ABSPattern.Sine:
                    AbsPattern.SelectedIndex = 0;
                    break;
                case ABSPattern.Sawtooth:
                    AbsPattern.SelectedIndex = 1;
                    break;
                default:
                    config.AbsEffectConfig.Pattern = ABSPattern.Sawtooth;
                    AbsPattern.SelectedIndex = 1;
                    break;
            }

            if (config.RpmEffectConfig.Enabled)
            {
                checkbox_enable_RPM.IsChecked = true;
                checkbox_enable_RPM.Content = "Effect Enabled";
            }
            else
            {
                checkbox_enable_RPM.IsChecked = false;
                checkbox_enable_RPM.Content = "Effect Disabled";
            }

            if (config.AbsEffectConfig.Enabled)
            {
                checkbox_enable_ABS.IsChecked = true;
                checkbox_enable_ABS.Content = "ABS/TC Effect Enabled";
            }
            else
            {
                checkbox_enable_ABS.IsChecked = false;
                checkbox_enable_ABS.Content = "ABS/TC Effect Disabled";
            }
            if (config.BitePointEffectConfig.Enabled)
            {
                checkbox_enable_bite_point.IsChecked = true;
                checkbox_enable_bite_point.Content = "Bite Point Vibration Enabled";

            }
            else
            {
                checkbox_enable_bite_point.IsChecked = false;
                checkbox_enable_bite_point.Content = "Bite Point Vibration Disabled";
            }

            if (current_function_id == FunctionID.BrakePedal)
            {
                checkbox_enable_G_force.IsEnabled = true;
                if (config.GForceEffectConfig.Enabled)
                {
                    checkbox_enable_G_force.IsChecked = true;
                    checkbox_enable_G_force.Content = "G Force Effect Enabled";
                }
                else
                {
                    checkbox_enable_G_force.IsChecked = false;
                    checkbox_enable_G_force.Content = "G Force Effect Disabled";
                }
            }
            else
            {
                checkbox_enable_G_force.IsEnabled = false;
                checkbox_enable_G_force.IsChecked = false;
                checkbox_enable_G_force.Content = "G Force Effect Disabled";
            }

            switch (config.RpmEffectConfig.Type)
            {
                case RPMEffectType.RpmEffectRpm:
                    RPMeffecttype_Sel_1.IsChecked = true;
                    break;
                case RPMEffectType.RpmEffectSpeedRumble:
                    RPMeffecttype_Sel_2.IsChecked = true;
                    break;
                default:
                    config.RpmEffectConfig.Type = RPMEffectType.RpmEffectRpm;
                    RPMeffecttype_Sel_1.IsChecked = true;
                    break;
            }

            checkbox_enable_wheelslip.IsChecked = config.WheelSlipEffectConfig.Enabled;
            checkbox_enable_impact.IsChecked = config.RoadImpactEffectConfig.Enabled;
            textBox_wheelslip_effect_string.Text = plugin.Settings.WSeffect_bind;
            textBox_impact_effect_string.Text = plugin.Settings.Road_impact_bind;

            update_plot_ABS();
            update_plot_BP();
            update_plot_WS();
            update_plot_RPM();

            InitializeBadges();

            update_lockout = false;

            Dispatcher.BeginInvoke(new Action(() => allowOverrideCreation = true),
                System.Windows.Threading.DispatcherPriority.ContextIdle);
        }

        public void TestAbs_click(object sender, RoutedEventArgs e)
        {
            //if (indexOfSelectedPedal_u == 1)
            if (TestAbs_check.IsChecked == false)
            {
                TestAbs_check.IsChecked = true;
                ABSTestStateChange?.Invoke(true);
                DebugMessage?.Invoke("ABS-Test begin");
            }
            else
            {
                TestAbs_check.IsChecked = false;
                ABSTestStateChange?.Invoke(false);
                DebugMessage?.Invoke("ABS-Test stopped");
            }

        }

        private void checkbox_enable_bite_point_Checked(object sender, RoutedEventArgs e)
        {

            config.BitePointEffectConfig.Enabled = true;
            checkbox_enable_bite_point.Content = "Bite Point Vibration Enabled";


        }

        private void checkbox_enable_bite_point_Unchecked(object sender, RoutedEventArgs e)
        {

            config.BitePointEffectConfig.Enabled = false;
            checkbox_enable_bite_point.Content = "Bite Point Vibration Disabled";


        }

        public void AbsPatternChanged(object sender, SelectionChangedEventArgs e)
        {
            if (update_lockout) return;
            switch (AbsPattern.SelectedIndex)
            {
                case 0:
                    config.AbsEffectConfig.Pattern = ABSPattern.Sine;
                    break;
                case 1:
                    config.AbsEffectConfig.Pattern = ABSPattern.Sawtooth;
                    break;
                default:
                    config.AbsEffectConfig.Pattern = ABSPattern.Sawtooth;
                    break;
            }
            update_plot_ABS();
        }

        private void checkbox_enable_G_force_Unchecked(object sender, RoutedEventArgs e)
        {
            config.GForceEffectConfig.Enabled = false;
            checkbox_enable_G_force.Content = "G Force Effect Disabled";
        }
        private void checkbox_enable_G_force_Checked(object sender, RoutedEventArgs e)
        {
            config.GForceEffectConfig.Enabled = true;
            checkbox_enable_G_force.Content = "G Force Effect Enabled";
        }
        private void RPMeffecttype_Sel_1_Checked(object sender, RoutedEventArgs e)
        {
            if (RPMeffecttype_Sel_1.IsChecked == true)
            {
                config.RpmEffectConfig.Type = RPMEffectType.RpmEffectRpm;
            }
            else if (RPMeffecttype_Sel_2.IsChecked == true)
            {
                config.RpmEffectConfig.Type = RPMEffectType.RpmEffectSpeedRumble;
            }
        }

        private void effect_bind_click(object sender, RoutedEventArgs e)
        {
            plugin.Settings.WSeffect_bind = (string)textBox_wheelslip_effect_string.Text;
            config.WheelSlipEffectConfig.Enabled = true;
        }
        private void effect_clear_click(object sender, RoutedEventArgs e)
        {
            plugin.Settings.WSeffect_bind = "";
            textBox_wheelslip_effect_string.Text = "";
            config.WheelSlipEffectConfig.Enabled = false;
        }

        private void Bind_Impacteffect_Click(object sender, RoutedEventArgs e)
        {
            plugin.Settings.Road_impact_bind = (string)textBox_impact_effect_string.Text;
            config.RoadImpactEffectConfig.Enabled = true;
        }

        private void checkbox_enable_impact_Checked(object sender, RoutedEventArgs e)
        {
            config.RoadImpactEffectConfig.Enabled = true;
        }

        private void checkbox_enable_impact_Unchecked(object sender, RoutedEventArgs e)
        {
            config.RoadImpactEffectConfig.Enabled = false;
        }

        private void Clear_Impacteffect_Click(object sender, RoutedEventArgs e)
        {
            plugin.Settings.Road_impact_bind = "";
            config.RoadImpactEffectConfig.Enabled = false;
        }

        private void checkbox_enable_WS_Checked(object sender, RoutedEventArgs e)
        {
            config.WheelSlipEffectConfig.Enabled = true;
            //checkbox_enable_RPM.Content = "Effect Enabled";
        }

        private void checkbox_enable_WS_Unchecked(object sender, RoutedEventArgs e)
        {
            config.WheelSlipEffectConfig.Enabled = false;
            //checkbox_enable_RPM.Content = "Effect Disabled";
        }

        private void Simulate_ABS_check_Checked(object sender, RoutedEventArgs e)
        {
            config.AbsEffectConfig.SimLevel = 1;
            DebugMessage?.Invoke("simulateABS: on");
            //rect_SABS.Visibility = Visibility.Visible;
            //rect_SABS_Control.Visibility = Visibility.Visible;
            //text_SABS.Visibility = Visibility.Visible;

        }
        private void Simulate_ABS_check_Unchecked(object sender, RoutedEventArgs e)
        {
            config.AbsEffectConfig.SimLevel = 0;
            DebugMessage?.Invoke("simulateABS: off");
            //rect_SABS.Visibility = Visibility.Hidden;
            //rect_SABS_Control.Visibility = Visibility.Hidden;
            //text_SABS.Visibility = Visibility.Hidden;

        }


        private void Slider_impact_smoothness_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.RoadImpactEffectConfig.Window = (Byte)Slider_impact_smoothness.Value;
            label_impact_window.Content = "Impact Smoothness: " + config.RoadImpactEffectConfig.Window;


        }

        private void Slider_impact_multi_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.RoadImpactEffectConfig.Multi = (Byte)e.NewValue;
            label_impact_multi.Content = "Impact Multiplier: " + config.RoadImpactEffectConfig.Multi + "%";
        }

        private void Slider_WS_freq_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.WheelSlipEffectConfig.Freq = (Byte)e.NewValue;
            label_WS_freq.Content = "Notification Frequency: " + config.WheelSlipEffectConfig.Freq + "Hz";
            update_plot_WS();
        }

        private void Slider_WS_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.WheelSlipEffectConfig.Amp = (Byte)(e.NewValue * 9.81);
            label_WS_AMP.Content = "Notification Amplitude: " + e.NewValue + "kg";
            update_plot_WS();
        }

        private void Slider_WS_trigger_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (plugin != null)
            {
                plugin.Settings.WS_trigger = (int)e.NewValue;
                label_WS_trigger.Content = "Notification Trigger: " + (plugin.Settings.WS_trigger + 50) + "%";
            }

        }

        private void Slider_G_force_smoothness_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.GForceEffectConfig.Window = (Byte)e.NewValue;
            label_G_force_window.Content = "G Force Smoothness: " + config.GForceEffectConfig.Window;

        }

        private void Slider_G_force_multi_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.GForceEffectConfig.Multi = (Byte)e.NewValue;
            label_G_force_multi.Content = "G Force Multiplier: " + config.GForceEffectConfig.Multi + "%";
        }

        private void Slider_BP_freq_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.BitePointEffectConfig.Freq = (Byte)e.NewValue;
            label_BP_freq.Content = "Bite Point Frequency: " + config.BitePointEffectConfig.Freq + "Hz";
            update_plot_BP();
        }

        private void Slider_BP_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.BitePointEffectConfig.Amp = (Byte)(e.NewValue * 9.81);
            label_BP_AMP.Content = "Bite Point Amplitude: " + e.NewValue + "kg";
            update_plot_BP();
        }

        private void Slider_RPM_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.RpmEffectConfig.Amp = (Byte)(e.NewValue * 9.81);
            label_RPM_AMP.Content = "Effect Amplitude: " + e.NewValue + "kg";
            update_plot_RPM();
        }

        private void Rangeslider_RPM_freq_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            config.RpmEffectConfig.MinFreq = (byte)e.NewValue;
            label_RPM_freq_min.Content = "MIN:" + config.RpmEffectConfig.MinFreq + "Hz";

            update_plot_RPM();
        }

        private void Rangeslider_RPM_freq_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            config.RpmEffectConfig.MaxFreq = (byte)e.NewValue;
            label_RPM_freq_max.Content = "MAX:" + config.RpmEffectConfig.MaxFreq + "Hz";
            update_plot_RPM();
        }

        private void Slider_ABS_freq_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.AbsEffectConfig.Freq = (Byte)(e.NewValue);
            label_ABS_freq.Content = "ABS/TC Frequency: " + config.AbsEffectConfig.Freq + "Hz";
            update_plot_ABS();
        }

        private void Slider_ABS_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (update_lockout) return;
            switch (config.AbsEffectConfig.Mode)
            {
                case ABSMode.Force:
                    config.AbsEffectConfig.Ampl = (Byte)Math.Round(e.NewValue * 9.81);
                    label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}kg", config.AbsEffectConfig.Ampl / 9.81f);
                    break;
                case ABSMode.Travel:
                    config.AbsEffectConfig.Ampl = (Byte)Math.Round(e.NewValue);
                    label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}mm", config.AbsEffectConfig.Ampl);
                    break;
                default:
                    break;
            }
            update_plot_ABS();
        }

        private void Bind_CV1_Click(object sender, RoutedEventArgs e)
        {
            if (plugin.Ncalc_reading(textBox_CV1_string.Text) != "Error")
            {
                plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_binding = (string)textBox_CV1_string.Text;
                plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_enabled = true;
            }
            else
            {
                plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_enabled = false;
                string MSG_tmp = "ERROR! String can not be evaluated";
                ThemedMessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);

            }
            //updateTheGuiFromConfig();
        }

        private void checkbox_enable_CV_1_Checked(object sender, RoutedEventArgs e)
        {
            config.Cv1EffectConfig.Enabled = true;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_enabled = true;
        }

        private void checkbox_enable_CV_1_Unchecked(object sender, RoutedEventArgs e)
        {
            config.Cv1EffectConfig.Enabled = false;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_enabled = false;
        }

        private void Clear_CV1_Click(object sender, RoutedEventArgs e)
        {
            textBox_CV1_string.Text = "";
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_binding = (string)textBox_CV1_string.Text;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_enabled = false;
            //updateTheGuiFromConfig();
        }

        private void Slider_CV2_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.Cv2EffectConfig.Amp = (Byte)(e.NewValue * 9.81);
            label_CV2_AMP.Content = "Effect Amplitude: " + e.NewValue + "kg";
        }

        private void Slider_CV1_AMP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.Cv1EffectConfig.Amp = (Byte)(e.NewValue * 9.81);
            label_CV1_AMP.Content = "Effect Amplitude: " + e.NewValue + "kg";
        }

        private void Slider_CV1_trigger_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_trigger_level = (Byte)e.NewValue;
            label_CV1_trigger.Content = "Effect Trigger:" + plugin.Settings.function_settings[((int)current_function_id - 1)].CV1_trigger_level;
        }

        private void Slider_CV1_freq_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.Cv1EffectConfig.Freq = (Byte)e.NewValue;
            label_CV1_freq.Content = "Effect Frequency:" + config.Cv1EffectConfig.Freq + "Hz";
        }

        private void checkbox_enable_CV2_Checked(object sender, RoutedEventArgs e)
        {
            config.Cv2EffectConfig.Enabled = true;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_enabled = true;
        }

        private void checkbox_enable_CV2_Unchecked(object sender, RoutedEventArgs e)
        {
            config.Cv2EffectConfig.Enabled = false;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_enabled = false;
        }

        private void Bind_CV2_Click(object sender, RoutedEventArgs e)
        {
            if (plugin.Ncalc_reading(textBox_CV2_string.Text) != "Error")
            {
                plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_binding = (string)textBox_CV2_string.Text;
               plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_enabled = true;
            }
            else
            {
                plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_enabled = false;
                string MSG_tmp = "ERROR! String can not be evaluated";
                ThemedMessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);

            }
            /*
            Plugin.Settings.CV2_bindings[indexOfSelectedPedal_u] = (string)textBox_CV2_string.Text;
            Plugin.Settings.CV2_enable_flag[indexOfSelectedPedal_u] = true;
            */
        }

        private void Clear_CV2_Click(object sender, RoutedEventArgs e)
        {
            textBox_CV2_string.Text = "";
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_binding = (string)textBox_CV2_string.Text;
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_enabled = false;
        }

        private void Slider_CV2_trigger_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_trigger_level = (Byte)e.NewValue;
            label_CV2_trigger.Content = "Effect Trigger:" + plugin.Settings.function_settings[((int)current_function_id - 1)].CV2_trigger_level;
        }

        private void Slider_CV2_freq_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.Cv2EffectConfig.Freq = (Byte)e.NewValue;
            label_CV2_freq.Content = "Effect Frequency:" + config.Cv2EffectConfig.Freq + "Hz";
        }


        private void textBox_CV1_string_TextChanged(object sender, TextChangedEventArgs e)
        {
            string var1 = "";
            var1 = plugin.Ncalc_reading(textBox_CV1_string.Text.ToString());
            Label_NCALC_CUS1.Content = var1;
        }

        private void textBox_CV2_string_TextChanged(object sender, TextChangedEventArgs e)
        {
            string var1 = "";
            var1 = plugin.Ncalc_reading(textBox_CV2_string.Text.ToString());
            Label_NCALC_CUS2.Content = var1;
        }

        private void TabControl_SelectionChanged_1(object sender, SelectionChangedEventArgs e)
        {
            //if (plugin != null)
            //{
            //    Update_CV_textbox = true;
            //    updateTheGuiFromConfig();
            //}

        }


        private void update_plot_ABS()
        {
            int x_quantity = 200;
            double[] x = new double[x_quantity];
            double[] y = new double[x_quantity];

            double y_max = 50;
            double dx = canvas_plot_ABS.Width / x_quantity;
            double dy = canvas_plot_ABS.Height / y_max;
            double freq = config.AbsEffectConfig.Freq;
            double max_force = 255 / 20;
            double amp = (double)config.AbsEffectConfig.Ampl / 9.81;
            double peroid = x_quantity / freq;
            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();
            switch(config.AbsEffectConfig.Pattern)
            {
                case ABSPattern.Sine:
                    for (int idx = 0; idx < x_quantity; idx++)
                    {
                        x[idx] = idx;
                        y[idx] = -1 * amp / max_force * Math.Sin(2 * x[idx] / peroid * Math.PI) * y_max / 2;
                        System.Windows.Point Pointlcl = new System.Windows.Point(dx * x[idx], dy * y[idx] + 25);
                        myPointCollection2.Add(Pointlcl);
                    }
                    break;
                case ABSPattern.Sawtooth:
                    for (int idx = 0; idx < x_quantity; idx++)
                    {
                        x[idx] = idx;
                        y[idx] = -1 * amp / max_force * y_max * (x[idx] % peroid) / peroid + 0.5 * amp / max_force * y_max;
                        System.Windows.Point Pointlcl = new System.Windows.Point(dx * x[idx], dy * y[idx] + 25);
                        myPointCollection2.Add(Pointlcl);
                    }
                    break;
                default:
                    break;
            }
            this.Polyline_plot_ABS.Points = myPointCollection2;
        }
        private void update_plot_BP()
        {
            int x_quantity = 200;
            double[] x = new double[x_quantity];
            double[] y = new double[x_quantity];

            double y_max = 50;
            double dx = canvas_plot_BP.Width / x_quantity;
            double dy = canvas_plot_BP.Height / y_max;
            double freq = config.BitePointEffectConfig.Freq;
            double max_force = 200 / 20;
            double amp = (double)config.BitePointEffectConfig.Amp / 9.81;
            double peroid = x_quantity / freq;
            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();
            for (int idx = 0; idx < x_quantity; idx++)
            {
                x[idx] = idx;
                y[idx] = -1 * amp / max_force * Math.Sin(2 * x[idx] / peroid * Math.PI) * y_max / 2;
                System.Windows.Point Pointlcl = new System.Windows.Point(dx * x[idx], dy * y[idx] + 25);
                myPointCollection2.Add(Pointlcl);
            }
            this.Polyline_plot_BP.Points = myPointCollection2;
        }
        private void update_plot_WS()
        {
            int x_quantity = 200;
            double[] x = new double[x_quantity];
            double[] y = new double[x_quantity];

            double y_max = 50;
            double dx = canvas_plot_WS.Width / x_quantity;
            double dy = canvas_plot_WS.Height / y_max;
            double freq = config.WheelSlipEffectConfig.Freq;
            double max_force = 250 / 20;
            double amp = (double)config.WheelSlipEffectConfig.Amp / 9.81;
            double peroid = x_quantity / freq;
            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();
            for (int idx = 0; idx < x_quantity; idx++)
            {
                x[idx] = idx;
                y[idx] = -1 * amp / max_force * Math.Sin(2 * x[idx] / peroid * Math.PI) * y_max / 2;
                System.Windows.Point Pointlcl = new System.Windows.Point(dx * x[idx], dy * y[idx] + 25);
                myPointCollection2.Add(Pointlcl);
            }
            this.Polyline_plot_WS.Points = myPointCollection2;
        }
        private void update_plot_RPM()
        {
            int x_quantity = 1601;
            double[] x = new double[x_quantity];
            double[] y = new double[x_quantity];
            double[] peroid_x = new double[x_quantity];
            double[] freq = new double[x_quantity];
            double[] amp = new double[x_quantity];
            double y_max = 50;
            double dx = canvas_plot_RPM.Width / (x_quantity - 1);
            double dy = canvas_plot_RPM.Height / y_max;
            double freq_max = config.RpmEffectConfig.MaxFreq;
            double freq_min = config.RpmEffectConfig.MinFreq;
            double max_force = 200 / 20 * 1.3;
            double amp_base = (double)config.RpmEffectConfig.Amp / 9.81;
            //double peroid = x_quantity / freq;
            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();
            for (int idx = 0; idx < x_quantity; idx++)
            {
                x[idx] = idx;
                freq[idx] = freq_min + (((double)idx) / (double)x_quantity) * (freq_max - freq_min);
                peroid_x[idx] = x_quantity / freq[idx];
                amp[idx] = amp_base + amp_base * idx / x_quantity * 0.3;
                y[idx] = -1 * amp[idx] / max_force * Math.Sin(2 * x[idx] / peroid_x[idx] * Math.PI) * y_max / 2;
                System.Windows.Point Pointlcl = new System.Windows.Point(dx * x[idx], dy * y[idx] + 25);
                myPointCollection2.Add(Pointlcl);
            }
            this.Polyline_plot_RPM.Points = myPointCollection2;
        }


         private void checkbox_enable_ABS_Checked(object sender, RoutedEventArgs e)
        {
            plugin.Settings.function_settings[ui.indexOfSelectedPedal_u].ABS_enabled = true;
            config.AbsEffectConfig.Enabled = true;
            checkbox_enable_ABS.Content = "ABS/TC Effect Enabled";
        }
        private void checkbox_enable_ABS_Unchecked(object sender, RoutedEventArgs e)
        {
            plugin.Settings.function_settings[ui.indexOfSelectedPedal_u].ABS_enabled = false;
            config.AbsEffectConfig.Enabled = false;
            checkbox_enable_ABS.Content = "ABS/TC Effect Disabled";
        }

        private void checkbox_enable_RPM_Checked(object sender, RoutedEventArgs e)
        {
            plugin.Settings.function_settings[ui.indexOfSelectedPedal_u].RPM_enabled = true;
            config.RpmEffectConfig.Enabled = true;
            checkbox_enable_RPM.Content = "Effect Enabled";
        }

        private void checkbox_enable_RPM_Unchecked(object sender, RoutedEventArgs e)
        {
            plugin.Settings.function_settings[ui.indexOfSelectedPedal_u].RPM_enabled = false;
            config.RpmEffectConfig.Enabled = false;
            checkbox_enable_RPM.Content = "Effect Disabled";
        }
        public void EffectAppliedOnForceOrTravel_combobox_changed(object sender, SelectionChangedEventArgs e)
        {
            if (update_lockout) return;
            try
            {
                if (EffectAppliedOnForceOrTravel_combobox.SelectedIndex == 0)
                {
                    config.AbsEffectConfig.Mode = ABSMode.Force;
                }
                else
                {
                    config.AbsEffectConfig.Mode = ABSMode.Travel;
                }

                if (label_ABS_AMP != null)
                {
                    switch (config.AbsEffectConfig.Mode)
                    {
                        case ABSMode.Force:
                            config.AbsEffectConfig.Ampl = (uint)Math.Round((float)config.AbsEffectConfig.Ampl * 9.81f);
                            label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}kg", config.AbsEffectConfig.Ampl / 9.81f);
                            break;
                        case ABSMode.Travel:
                            config.AbsEffectConfig.Ampl = (uint)Math.Round((float)config.AbsEffectConfig.Ampl / 9.81f);
                            label_ABS_AMP.Content = String.Format("ABS/TC Amplitude: {0:F1}mm", config.AbsEffectConfig.Ampl);
                            break;
                        default:
                            break;
                    }
                }


            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                DebugMessage?.Invoke(errorMessage);
            }

        }

        private void OnPushDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (update_lockout) return;
            var newValue = (float)e.NewValue;
            config.DamperConfig.PositiveFactor = newValue;
            label_damping_push.Content = String.Format("Damping (Push): {0:F3}N*mm/s", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "damper_config.positive_factor",
                    overrides =>
                    {
                        if (overrides.DamperConfig == null)
                            overrides.DamperConfig = new TieredConfig.DamperConfigOverrides();
                        overrides.DamperConfig.PositiveFactor = newValue;
                    });
            }
        }

        private void OnPullDampingChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (update_lockout) return;
            var newValue = (float)e.NewValue;
            config.DamperConfig.NegativeFactor = newValue;
            label_damping_pull.Content = String.Format("Damping (Pull): {0:F3}N*mm/s", newValue);

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "damper_config.negative_factor",
                    overrides =>
                    {
                        if (overrides.DamperConfig == null)
                            overrides.DamperConfig = new TieredConfig.DamperConfigOverrides();
                        overrides.DamperConfig.NegativeFactor = newValue;
                    });
            }
        }

        private void OnSimulatedMassChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (update_lockout) return;
            var newValue = (float)e.NewValue;
            label_simulated_mass.Content = String.Format("Simulated Mass: {0:F2}kg", newValue);
            function_config.SimulatedMass = newValue;

            if (allowOverrideCreation && plugin != null && function != null &&
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
            AutomotivePedal_SplineForceCurve.ResetAxisRange();
            var kinematic_parameters = ui.GetKinematicParameters(e.Value);
            if (kinematic_parameters != null) {
                OnKinematicParametersChanged(kinematic_parameters);
            }
            function?.OnAxisUpdate();
        }

        private void cb_controller_output_mode_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (function_config.Base == null) return;
            switch (cb_controller_output_mode.SelectedIndex)
            {
                case 0:
                    function_config.Base.OutputMode = OutputMode.Force;
                    break;
                case 1:
                    function_config.Base.OutputMode = OutputMode.Travel;
                    break;
                default:
                    break;
            }
            AutomotivePedalProcessor.ReconcileDerivedFields(function_config);
        }

        private void AutomotivePedal_ControllerAxisSelector_ControllerAxisChanged(object sender, ControllerAxisSelector.ControllerAxisChangedEventArgs e)
        {
            function_config.Base.ControllerOutputAxis = e.Value;
        }
        
        private void OnFrictionChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (update_lockout) return;
            var newValue = (float)e.NewValue;
            label_friction.Content = String.Format("Friction: {0:F1}N", newValue);
            function_config.Friction = newValue;

            if (allowOverrideCreation && plugin != null && function != null &&
                plugin.HasFunctionBaseline((int)function.ID))
            {
                plugin.UpdateFunctionOverrideField((int)function.ID, "friction",
                    overrides => overrides.Friction = newValue);
            }
        }
    }
}
