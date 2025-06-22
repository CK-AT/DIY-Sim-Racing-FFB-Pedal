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
        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;

        public AxisConfigControl()
        {
            config = GetDefaultConfig(AxisID.AxisUndefined);
            InitializeComponent();
            DiyPedalKinematicsControl.KinematicParametersChanged += DiyPedalKinematicsControl_KinematicParametersChanged;
        }

        private void DiyPedalKinematicsControl_KinematicParametersChanged(KinematicParameters parameters)
        {
            config.KinematicParameters = parameters;
            KinematicParametersChanged?.Invoke(parameters);
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (config.AxisId == axis_state.AxisId)
            {
                DiyPedalKinematicsControl.OnAxisStateUpdate(axis_state);
            }
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            DiyPedalKinematicsControl.SetGui(ui, plugin);
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
            new_config.Store = false;

            return new_config;
        }
        public void UpdateConfig(AxisConfig new_config)
        {
            config = new_config;

            switch (config.KinematicConfigCase)
            {
                case AxisConfig.KinematicConfigOneofCase.DiyPedal:
                    DiyPedalKinematicsControl.UpdateConfig(config.DiyPedal);
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
                    break;
                case AxisConfig.LoadCellFilterConfigOneofCase.KfConstAccel:
                    Slider_KF.Value = config.KfConstAccel.NoiseScaling;
                    KF_filter_order.SelectedIndex = 1;
                    break;
                case AxisConfig.LoadCellFilterConfigOneofCase.FilterNone:
                    KF_filter_order.SelectedIndex = 2;
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

            Slider_steps_per_mm.Value = config.StepsPerMm;
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
                        break;
                    case 1:
                        config.KfConstAccel = new KFConstAccelConfig();
                        config.KfConstAccel.NoiseScaling = (uint)Slider_KF.Value;
                        break;
                    case 2:
                        config.FilterNone = true;
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



    }
}
