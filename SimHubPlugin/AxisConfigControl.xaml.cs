using System;
using System.Windows;
using System.Windows.Controls;
using System.Globalization;

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
            GeneralKinematicsControl.KinematicParametersChanged += GeneralKinematicsControl_KinematicParametersChanged;
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
                GeneralKinematicsControl.OnAxisStateUpdate(axis_state);
            }
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
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
            new_config.GeneralKinematic = BuildDefaultGeneralKinematicConfig();
            new_config.KinematicParameters = GeneralKinematics.CalcKinematicParameters(new_config.GeneralKinematic);
            new_config.FMaxLoadcell = (uint)Math.Round(200 * 9.81f);
            new_config.StepsPerMm = 1000;
            new_config.MmPerRev = 5;
            new_config.PhysicsIterationsPerSample = 16;
            new_config.Store = false;
            new_config.HomingDirection = HomingDirection.HomingDirNegative;
            new_config.OscillationGuard = BuildDefaultOscillationGuard();

            return new_config;
        }

        private static GeneralKinematicConfig BuildDefaultGeneralKinematicConfig()
        {
            GeneralKinematicConfig config = new GeneralKinematicConfig
            {
                RailTravelNegative = 10.0f,
                RailTravelPositive = 10.0f
            };
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 1,
                X = 0.0f,
                Y = 0.0f,
                Grounded = true
            });
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 2,
                X = 100.0f,
                Y = 0.0f,
                IsRailInterface = true
            });
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 3,
                X = 50.0f,
                Y = 50.0f,
                IsContactPoint = true
            });

            GeneralKinematicBar metering = new GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(3);
            config.Bars.Add(metering);

            GeneralKinematicBar link = new GeneralKinematicBar();
            link.PinIds.Add(1);
            link.PinIds.Add(3);
            config.Bars.Add(link);

            return config;
        }

        private static AxisConfig.Types.OscillationGuard BuildDefaultOscillationGuard()
        {
            return new AxisConfig.Types.OscillationGuard
            {
                KMax = 0.5f,
                MinAmplitude = 0.2f,
                MinVelocity = 0.5f,
                MinFrequencyHz = 2.0f,
                MaxFrequencyHz = 100.0f,
                HoldTimeMs = 150,
                RampTimeMs = 80,
                RequiredHits = 2
            };
        }

        private AxisConfig.Types.OscillationGuard EnsureOscillationGuardConfig()
        {
            if (config.OscillationGuard == null)
            {
                config.OscillationGuard = BuildDefaultOscillationGuard();
            }
            return config.OscillationGuard;
        }
        public void UpdateConfig(AxisConfig new_config)
        {
            config = new_config;
            GeneralKinematicsControl.Visibility = Visibility.Visible;

            try
            {
                if (config.KinematicConfigCase == AxisConfig.KinematicConfigOneofCase.DiyPedal && config.DiyPedal != null)
                {
                    config.GeneralKinematic = ConvertDiyPedalToGeneral(config.DiyPedal);
                }
                if (config.GeneralKinematic == null)
                {
                    config.GeneralKinematic = BuildDefaultGeneralKinematicConfig();
                }
                GeneralKinematicsControl.UpdateConfig(config.GeneralKinematic);
            }
            catch (Exception caughtEx)
            {
                DebugMessage?.Invoke(caughtEx.Message);
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

            UpdateOscillationGuardUi(EnsureOscillationGuardConfig());
        }

        private static GeneralKinematicConfig ConvertDiyPedalToGeneral(DIYPedalKinematicConfig diy)
        {
            if (diy == null)
            {
                return BuildDefaultGeneralKinematicConfig();
            }

            double pivotX = 0.0;
            double pivotY = 0.0;
            double stroke = diy.LSledStroke;
            if (stroke <= 0.0)
            {
                return BuildDefaultGeneralKinematicConfig();
            }

            double railX = diy.LPivotSledXMin;
            double railY = diy.LPivotSledY;
            double rPivotLink = diy.LPivotLink;
            double rLink = diy.LLink;

            if (rPivotLink <= 0.0 || rLink <= 0.0)
            {
                return BuildDefaultGeneralKinematicConfig();
            }

            double dx = railX - pivotX;
            double dy = railY - pivotY;
            double d = Math.Sqrt(dx * dx + dy * dy);
            if (d <= 1e-6 || d > rPivotLink + rLink || d < Math.Abs(rPivotLink - rLink))
            {
                return BuildDefaultGeneralKinematicConfig();
            }

            double a = (rPivotLink * rPivotLink - rLink * rLink + d * d) / (2.0 * d);
            double h2 = rPivotLink * rPivotLink - a * a;
            if (h2 < 0.0)
            {
                h2 = 0.0;
            }
            double h = Math.Sqrt(h2);

            double x2 = pivotX + a * dx / d;
            double y2 = pivotY + a * dy / d;
            double rx = -dy / d;
            double ry = dx / d;

            double cx1 = x2 + h * rx;
            double cy1 = y2 + h * ry;
            double cx2 = x2 - h * rx;
            double cy2 = y2 - h * ry;

            double linkX = cx1;
            double linkY = cy1;
            if (cy2 > cy1)
            {
                linkX = cx2;
                linkY = cy2;
            }

            double pedalScale = diy.LPivotFoot / rPivotLink;
            double contactX = linkX * pedalScale;
            double contactY = linkY * pedalScale;

            GeneralKinematicConfig config = new GeneralKinematicConfig
            {
                RailTravelNegative = 0.0f,
                RailTravelPositive = (float)stroke
            };

            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 1,
                X = (float)pivotX,
                Y = (float)pivotY,
                Grounded = true
            });
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 2,
                X = (float)railX,
                Y = (float)railY,
                IsRailInterface = true
            });
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 3,
                X = (float)linkX,
                Y = (float)linkY
            });
            config.Pins.Add(new GeneralKinematicPin
            {
                PinId = 4,
                X = (float)contactX,
                Y = (float)contactY,
                IsContactPoint = true
            });

            GeneralKinematicBar metering = new GeneralKinematicBar
            {
                IsMetering = true
            };
            metering.PinIds.Add(2);
            metering.PinIds.Add(3);
            config.Bars.Add(metering);

            GeneralKinematicBar pedal = new GeneralKinematicBar();
            pedal.PinIds.Add(1);
            pedal.PinIds.Add(3);
            pedal.PinIds.Add(4);
            config.Bars.Add(pedal);

            return config;
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

        private void OscillationGuard_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (config == null)
            {
                return;
            }

            var guard = EnsureOscillationGuardConfig();

            if (ReferenceEquals(sender, TextOscKMax))
            {
                if (TryParseFloat(TextOscKMax.Text, out float value))
                {
                    guard.KMax = Math.Max(0.0f, value);
                }
            }
            else if (ReferenceEquals(sender, TextOscMinAmplitude))
            {
                if (TryParseFloat(TextOscMinAmplitude.Text, out float value))
                {
                    guard.MinAmplitude = Math.Max(0.0f, value);
                }
            }
            else if (ReferenceEquals(sender, TextOscMinVelocity))
            {
                if (TryParseFloat(TextOscMinVelocity.Text, out float value))
                {
                    guard.MinVelocity = Math.Max(0.0f, value);
                }
            }
            else if (ReferenceEquals(sender, TextOscMinFrequencyHz))
            {
                if (TryParseFloat(TextOscMinFrequencyHz.Text, out float value))
                {
                    guard.MinFrequencyHz = Math.Max(0.0f, value);
                }
            }
            else if (ReferenceEquals(sender, TextOscMaxFrequencyHz))
            {
                if (TryParseFloat(TextOscMaxFrequencyHz.Text, out float value))
                {
                    guard.MaxFrequencyHz = Math.Max(0.0f, value);
                }
            }
            else if (ReferenceEquals(sender, TextOscHoldMs))
            {
                if (TryParseUInt(TextOscHoldMs.Text, out uint value))
                {
                    guard.HoldTimeMs = value;
                }
            }
            else if (ReferenceEquals(sender, TextOscRampMs))
            {
                if (TryParseUInt(TextOscRampMs.Text, out uint value))
                {
                    guard.RampTimeMs = value;
                }
            }
            else if (ReferenceEquals(sender, TextOscRequiredHits))
            {
                if (TryParseUInt(TextOscRequiredHits.Text, out uint value))
                {
                    guard.RequiredHits = Math.Max(1U, value);
                }
            }
        }

        private void OscillationGuard_LostFocus(object sender, RoutedEventArgs e)
        {
            if (config == null)
            {
                return;
            }

            UpdateOscillationGuardUi(EnsureOscillationGuardConfig());
        }

        private void UpdateOscillationGuardUi(AxisConfig.Types.OscillationGuard guard)
        {
            if (guard == null)
            {
                return;
            }

            TextOscKMax.Text = guard.KMax.ToString("0.###", CultureInfo.CurrentCulture);
            TextOscMinAmplitude.Text = guard.MinAmplitude.ToString("0.###", CultureInfo.CurrentCulture);
            TextOscMinVelocity.Text = guard.MinVelocity.ToString("0.###", CultureInfo.CurrentCulture);
            TextOscMinFrequencyHz.Text = guard.MinFrequencyHz.ToString("0.###", CultureInfo.CurrentCulture);
            TextOscMaxFrequencyHz.Text = guard.MaxFrequencyHz.ToString("0.###", CultureInfo.CurrentCulture);
            TextOscHoldMs.Text = guard.HoldTimeMs.ToString(CultureInfo.CurrentCulture);
            TextOscRampMs.Text = guard.RampTimeMs.ToString(CultureInfo.CurrentCulture);
            TextOscRequiredHits.Text = guard.RequiredHits.ToString(CultureInfo.CurrentCulture);
        }

        private static bool TryParseFloat(string text, out float value)
        {
            return float.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value);
        }

        private static bool TryParseUInt(string text, out uint value)
        {
            return uint.TryParse(text, NumberStyles.Integer, CultureInfo.CurrentCulture, out value);
        }
    }
}
