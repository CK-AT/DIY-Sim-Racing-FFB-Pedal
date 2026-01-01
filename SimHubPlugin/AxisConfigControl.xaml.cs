using System;
using System.Windows;
using System.Windows.Controls;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for AxisConfigControl.xaml
    /// </summary>
    public partial class AxisConfigControl : UserControl
    {
        private AxisConfig config;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private bool suppressKinematicSelection;
        private DIYPedalKinematicConfig cachedDiyConfig;
        private GeneralKinematicConfig cachedGeneralConfig;
        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;

        public AxisConfigControl()
        {
            config = GetDefaultConfig(AxisID.AxisUndefined);
            InitializeComponent();
            DiyPedalKinematicsControl.KinematicParametersChanged += DiyPedalKinematicsControl_KinematicParametersChanged;
            GeneralKinematicsControl.KinematicParametersChanged += GeneralKinematicsControl_KinematicParametersChanged;
        }

        private void DiyPedalKinematicsControl_KinematicParametersChanged(KinematicParameters parameters)
        {
            config.KinematicParameters = parameters;
            KinematicParametersChanged?.Invoke(parameters);
        }

        private void GeneralKinematicsControl_KinematicParametersChanged(KinematicParameters parameters)
        {
            config.KinematicParameters = parameters;
            KinematicParametersChanged?.Invoke(parameters);
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (config.AxisId == axis_state.AxisId)
            {
                if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.DiyPedal)
                {
                    DiyPedalKinematicsControl.OnAxisStateUpdate(axis_state);
                }
                else if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.GeneralKinematic)
                {
                    GeneralKinematicsControl.OnAxisStateUpdate(axis_state);
                }
            }
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            DiyPedalKinematicsControl.SetGui(ui, plugin);
            GeneralKinematicsControl.SetGui(ui, plugin);
        }

        public static AxisConfig GetDefaultConfig(AxisID axis_id)
        {
            AxisConfig new_config = new AxisConfig();
            new_config.AxisId = axis_id;
            new_config.KfConstVel = new KFConstVelConfig();
            new_config.KfConstVel.NoiseScaling = 128;
            new_config.BLoadcellInverted = false;
            new_config.BMotorInverted = false;
            new_config.DiyPedal = DiyPedalKinematics.GetDefaultConfig();
            new_config.KinematicParameters = DiyPedalKinematics.CalcKinematicParameters(new_config.DiyPedal);
            new_config.FMaxLoadcell = (uint)Math.Round(200 * 9.81f);
            new_config.StepsPerMm = 1000;
            new_config.MmPerRev = 5;
            new_config.PhysicsIterationsPerSample = 16;
            new_config.Store = false;
            new_config.HomingDirection = HomingDirection.HomingDirNegative;

            return new_config;
        }
        public void UpdateConfig(AxisConfig new_config)
        {
            config = new_config;
            CacheCurrentKinematicConfig();
            SyncKinematicSelection();
            DiyPedalKinematicsControl.Visibility = config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.DiyPedal
                ? Visibility.Visible
                : Visibility.Collapsed;
            GeneralKinematicsControl.Visibility = config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.GeneralKinematic
                ? Visibility.Visible
                : Visibility.Collapsed;

            switch (config.KinematicConfigCase)
            {
                case AxisConfig.KinematicConfigOneofCase.DiyPedal:
                    if (config.DiyPedal == null)
                    {
                        config.DiyPedal = DiyPedalKinematics.GetDefaultConfig();
                    }
                    cachedDiyConfig = config.DiyPedal;
                    DiyPedalKinematicsControl.UpdateConfig(config.DiyPedal);
                    break;
                case AxisConfig.KinematicConfigOneofCase.GeneralKinematic:
                    try
                    {
                        if (config.GeneralKinematic == null)
                        {
                            config.GeneralKinematic = new GeneralKinematicConfig();
                        }
                        cachedGeneralConfig = config.GeneralKinematic;
                        GeneralKinematicsControl.UpdateConfig(config.GeneralKinematic);
                    }
                    catch (Exception caughtEx)
                    {
                        DebugMessage?.Invoke(caughtEx.Message);
                    }
                    break;
                default:
                    break;
            }

            Slider_LC_rate.Value = Math.Round(config.FMaxLoadcell / 9.81);

            switch (config.LoadCellFilterConfigCase)
            {
                case AxisConfig.LoadCellFilterConfigOneofCase.KfConstVel:
                    Slider_KF.Value = config.KfConstVel.NoiseScaling;
                    KF_filter_order.SelectedIndex = 0;
                    sp_filter_slider.Visibility = Visibility.Visible;
                    break;
                case AxisConfig.LoadCellFilterConfigOneofCase.KfConstAccel:
                    Slider_KF.Value = config.KfConstAccel.NoiseScaling;
                    KF_filter_order.SelectedIndex = 1;
                    sp_filter_slider.Visibility = Visibility.Visible;
                    break;
                case AxisConfig.LoadCellFilterConfigOneofCase.FilterNone:
                    KF_filter_order.SelectedIndex = 2;
                    sp_filter_slider.Visibility = Visibility.Hidden;
                    break;
                default:
                    break;
            }

            // spindle pitch
            try
            {
                SpindlePitch.SelectedIndex = (byte)config.MmPerRev;
            }
            catch (Exception caughtEx)
            {
            }

            // these are disabled for now
            EnableStepLossRecov_check.IsEnabled = false;
            EnableCrashDetection_check.IsEnabled = false;

            InvertLoadcellReading_check.IsChecked = config.BLoadcellInverted;
            InvertMotorDir_check.IsChecked = config.BMotorInverted;
            HomingDirectionCombo.SelectedIndex = config.HomingDirection == HomingDirection.HomingDirPositive ? 1 : 0;

            Slider_steps_per_mm.Value = config.StepsPerMm;
            Slider_physics_oversampling.Value = config.PhysicsIterationsPerSample;
        }

        private void CacheCurrentKinematicConfig()
        {
            if (config == null) return;
            switch (config.KinematicConfigCase)
            {
                case AxisConfig.KinematicConfigOneofCase.DiyPedal:
                    cachedDiyConfig = config.DiyPedal;
                    break;
                case AxisConfig.KinematicConfigOneofCase.GeneralKinematic:
                    cachedGeneralConfig = config.GeneralKinematic;
                    break;
            }
        }

        private void SyncKinematicSelection()
        {
            suppressKinematicSelection = true;
            switch (config.KinematicConfigCase)
            {
                case AxisConfig.KinematicConfigOneofCase.GeneralKinematic:
                    KinematicModelCombo.SelectedIndex = 1;
                    break;
                case AxisConfig.KinematicConfigOneofCase.DiyPedal:
                default:
                    KinematicModelCombo.SelectedIndex = 0;
                    break;
            }
            suppressKinematicSelection = false;
        }

        private void KinematicModelCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (suppressKinematicSelection) return;
            switch (KinematicModelCombo.SelectedIndex)
            {
                case 1:
                    SwitchToGeneralKinematics();
                    break;
                case 0:
                default:
                    SwitchToDiyPedal();
                    break;
            }
        }

        private void SwitchToDiyPedal()
        {
            if (config == null) return;
            if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.DiyPedal) return;
            if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.GeneralKinematic)
            {
                cachedGeneralConfig = config.GeneralKinematic;
            }
            if (cachedDiyConfig == null)
            {
                cachedDiyConfig = DiyPedalKinematics.GetDefaultConfig();
            }
            config.DiyPedal = cachedDiyConfig;
            DiyPedalKinematicsControl.Visibility = Visibility.Visible;
            GeneralKinematicsControl.Visibility = Visibility.Collapsed;
            DiyPedalKinematicsControl.UpdateConfig(config.DiyPedal);
        }

        private void SwitchToGeneralKinematics()
        {
            if (config == null) return;
            if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.GeneralKinematic) return;
            if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.DiyPedal)
            {
                cachedDiyConfig = config.DiyPedal;
            }
            if (cachedGeneralConfig == null)
            {
                cachedGeneralConfig = new GeneralKinematicConfig();
            }
            config.GeneralKinematic = cachedGeneralConfig;
            DiyPedalKinematicsControl.Visibility = Visibility.Collapsed;
            GeneralKinematicsControl.Visibility = Visibility.Visible;
            GeneralKinematicsControl.UpdateConfig(config.GeneralKinematic);
        }
        private void KF_filter_order_changed(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                switch (KF_filter_order.SelectedIndex)
                {
                    case 0:
                        config.KfConstVel = new KFConstVelConfig();
                        config.KfConstVel.NoiseScaling = (uint)Slider_KF.Value;
                        sp_filter_slider.Visibility = Visibility.Visible;
                        break;
                    case 1:
                        config.KfConstAccel = new KFConstAccelConfig();
                        config.KfConstAccel.NoiseScaling = (uint)Slider_KF.Value;
                        sp_filter_slider.Visibility = Visibility.Visible;
                        break;
                    case 2:
                        config.FilterNone = true;
                        sp_filter_slider.Visibility = Visibility.Hidden;
                        break;
                    default:
                        break;
                }
            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                DebugMessage?.Invoke(errorMessage);
            }
        }

        private void SpindlePitchChanged(object sender, SelectionChangedEventArgs e)
        {
            try
            {
                config.MmPerRev = (byte)SpindlePitch.SelectedIndex;
            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                DebugMessage?.Invoke(errorMessage);
            }
        }


        private void Slider_LC_rate_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.FMaxLoadcell = (uint)Math.Round(e.NewValue * 9.81);
            label_LC_rate.Content = String.Format("Loadcell rating: {0:F0}kg", e.NewValue);
        }

        private void Slider_KF_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            switch (config.LoadCellFilterConfigCase)
            {
                case AxisConfig.LoadCellFilterConfigOneofCase.KfConstVel:
                    config.KfConstVel.NoiseScaling = (uint)e.NewValue;
                    break;
                case AxisConfig.LoadCellFilterConfigOneofCase.KfConstAccel:
                    config.KfConstAccel.NoiseScaling = (uint)e.NewValue;
                    break;
                default:
                    break;
            }
            label_KF.Content = "KF: " + e.NewValue;
        }

        private void OnStepsPerMMChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.StepsPerMm = (uint)e.NewValue;
            label_steps_per_mm.Content = String.Format("Resolution: {0} steps per mm of linear axis travel", config.StepsPerMm);
        }
        private void InvertLoadcellReading_checked(object sender, RoutedEventArgs e)
        {
            config.BLoadcellInverted = true;
        }
        private void InvertLoadcellReading_unchecked(object sender, RoutedEventArgs e)
        {
            config.BLoadcellInverted = false;
        }


        private void InvertMotorDir_checked(object sender, RoutedEventArgs e)
        {
            config.BMotorInverted = true;
        }
        private void InvertMotorDir_unchecked(object sender, RoutedEventArgs e)
        {
            config.BMotorInverted = false;
        }

        private void HomingDirectionChanged(object sender, SelectionChangedEventArgs e)
        {
            switch (HomingDirectionCombo.SelectedIndex)
            {
                case 0:
                    config.HomingDirection = HomingDirection.HomingDirNegative;
                    break;
                case 1:
                    config.HomingDirection = HomingDirection.HomingDirPositive;
                    break;
                default:
                    config.HomingDirection = HomingDirection.HomingDirNegative;
                    break;
            }
        }

        private void Slider_physics_oversampling_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            config.PhysicsIterationsPerSample = (uint)e.NewValue;
            label_physics_oversampling.Content = String.Format("Physics Oversampling: {0}x", config.PhysicsIterationsPerSample);

        }
    }
}
