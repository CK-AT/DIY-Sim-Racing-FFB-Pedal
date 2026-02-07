using System;
using System.Windows;
using System.Windows.Controls;
using System.Globalization;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Linq;
using DiyFfb.TieredConfig;

namespace DiyFfb
{
    /// <summary>
    /// Item for the function selector ComboBox.
    /// </summary>
    public class FunctionSelectorItem
    {
        public int FunctionId { get; set; }
        public string DisplayName { get; set; }
        public bool HasOverride { get; set; }
        public bool IsAxisBase { get; set; }
    }

    /// <summary>
    /// Editing mode for axis config - base or function override.
    /// </summary>
    public enum AxisEditingMode
    {
        AxisBase,
        FunctionOverride
    }

    /// <summary>
    /// Interaction logic for AxisConfigControl.xaml
    /// </summary>
    public partial class AxisConfigControl : UserControl
    {
        private AxisConfig config;
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private bool updatingStaticBalanceUi;
        private List<float> staticBalanceSamples = new List<float>();
        private float staticBalanceXMin;
        private float staticBalanceXMax;
        private float staticBalanceStep;
        private const string StaticBalanceGridTag = "StaticBalanceGrid";
        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;

        // Function override editing state
        private AxisEditingMode _editingMode = AxisEditingMode.AxisBase;
        private int _selectedFunctionId = -1;
        private bool _updatingFunctionSelector = false;
        private bool _suppressOverrideSave = false;

        // Baseline snapshots: config is a reference to axis.Config, so applying
        // overrides mutates it.  We snapshot these before overwriting so that
        // switching back to Axis-Base restores the originals.
        private GeneralKinematicConfig _baselineGeometry;
        private KinematicParameters _baselineKinematics;
        private AxisConfig.Types.StaticBalanceConfig _baselineStaticBalance;

        private static readonly Google.Protobuf.JsonFormatter _protoJsonFormatter =
            new Google.Protobuf.JsonFormatter(Google.Protobuf.JsonFormatter.Settings.Default);
        private static readonly Google.Protobuf.JsonParser _protoJsonParser =
            new Google.Protobuf.JsonParser(Google.Protobuf.JsonParser.Settings.Default);

        public AxisConfigControl()
        {
            config = GetDefaultConfig(AxisID.AxisUndefined);
            InitializeComponent();
            GeneralKinematicsControl.KinematicParametersChanged += GeneralKinematicsControl_KinematicParametersChanged;
            if (Canvas_static_balance != null)
            {
                Canvas_static_balance.SizeChanged += StaticBalanceCanvas_SizeChanged;
            }
        }

        private void GeneralKinematicsControl_KinematicParametersChanged(KinematicParameters parameters)
        {
            config.KinematicParameters = parameters;

            if (_suppressOverrideSave)
                return;

            // In override mode, save to function override instead of raising the event
            if (_editingMode == AxisEditingMode.FunctionOverride && _selectedFunctionId >= 0)
            {
                SaveKinematicsChange(parameters);
            }
            else
            {
                KinematicParametersChanged?.Invoke(parameters);
            }
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

        #region Function Selector for Override Mode

        /// <summary>
        /// Refresh the function selector dropdown with functions linking to this axis.
        /// </summary>
        public void RefreshFunctionSelector()
        {
            if (FunctionSelector == null || plugin == null || config == null)
                return;

            _updatingFunctionSelector = true;
            try
            {
                var items = new List<FunctionSelectorItem>();
                int axisId = (int)config.AxisId;

                // Add the "Axis Base" option first
                string axisName = config.AxisId.ToString().Replace("Axis", "Axis ");
                items.Add(new FunctionSelectorItem
                {
                    FunctionId = -1,
                    DisplayName = $"{axisName} (base)",
                    HasOverride = false,
                    IsAxisBase = true
                });

                // Get functions linking to this axis
                var linkedFunctions = plugin.GetFunctionsLinkingToAxis(axisId);
                foreach (var func in linkedFunctions)
                {
                    items.Add(new FunctionSelectorItem
                    {
                        FunctionId = func.FunctionId,
                        DisplayName = func.FunctionName,
                        HasOverride = func.HasOverride,
                        IsAxisBase = false
                    });
                }

                FunctionSelector.ItemsSource = items;

                // Select the appropriate item
                if (_editingMode == AxisEditingMode.AxisBase || _selectedFunctionId < 0)
                {
                    FunctionSelector.SelectedIndex = 0;
                }
                else
                {
                    var matchingItem = items.FirstOrDefault(i => i.FunctionId == _selectedFunctionId);
                    if (matchingItem != null)
                    {
                        FunctionSelector.SelectedItem = matchingItem;
                    }
                    else
                    {
                        FunctionSelector.SelectedIndex = 0;
                    }
                }

                // Show/hide selector panel based on whether there are functions to select
                FunctionSelectorPanel.Visibility = items.Count > 1 ? Visibility.Visible : Visibility.Collapsed;
            }
            finally
            {
                _updatingFunctionSelector = false;
            }
        }

        /// <summary>
        /// Handle function selector selection change.
        /// </summary>
        private void FunctionSelector_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_updatingFunctionSelector || FunctionSelector.SelectedItem == null)
                return;

            var selectedItem = FunctionSelector.SelectedItem as FunctionSelectorItem;
            if (selectedItem == null)
                return;

            if (selectedItem.IsAxisBase)
            {
                // Switch to axis base editing mode
                _editingMode = AxisEditingMode.AxisBase;
                _selectedFunctionId = -1;
                BtnClearOverride.Visibility = Visibility.Collapsed;

                // Reload the axis base config
                ReloadBaseConfig();
            }
            else
            {
                // Switch to function override editing mode
                _editingMode = AxisEditingMode.FunctionOverride;
                _selectedFunctionId = selectedItem.FunctionId;
                BtnClearOverride.Visibility = selectedItem.HasOverride ? Visibility.Visible : Visibility.Collapsed;

                // Load the override config (or base if no override exists)
                LoadFunctionOverrideConfig();
            }

            DebugMessage?.Invoke($"Editing mode: {_editingMode}, Function: {_selectedFunctionId}");
        }

        /// <summary>
        /// Reload the base axis config from the UI cache.
        /// Restores baseline geometry/kinematics/static-balance that may have been
        /// overwritten while editing a function override (config == axis.Config).
        /// </summary>
        private void ReloadBaseConfig()
        {
            if (ui == null || config == null)
                return;

            if (ui.axes.TryGetValue(config.AxisId, out var axis) && axis.Config != null)
            {
                // Restore baseline values that were overwritten by function overrides
                if (_baselineGeometry != null)
                {
                    axis.Config.GeneralKinematic = _baselineGeometry;
                    _baselineGeometry = null;
                }
                if (_baselineKinematics != null)
                {
                    axis.Config.KinematicParameters = _baselineKinematics;
                    _baselineKinematics = null;
                }
                if (_baselineStaticBalance != null)
                {
                    axis.Config.StaticBalanceConfig = _baselineStaticBalance;
                    _baselineStaticBalance = null;
                }

                UpdateConfig(axis.Config);
            }
        }

        /// <summary>
        /// Load the axis config for function override editing.
        /// If an override exists, use it; otherwise use the base axis config.
        /// Note: The UI always shows base geometry; overrides store computed parameters.
        /// </summary>
        private void LoadFunctionOverrideConfig()
        {
            if (plugin == null || config == null || _selectedFunctionId < 0)
                return;

            // Suppress override saves while loading - QueueRebuild fires a 200ms timer
            // that would otherwise overwrite stored overrides with base values.
            _suppressOverrideSave = true;

            // Undo any mutations from a previous override before loading base config.
            // config is a direct reference to axis.Config, so previous override
            // applies or user edits have corrupted it.
            if (ui != null && ui.axes.TryGetValue(config.AxisId, out var axis) && axis.Config != null)
            {
                if (_baselineGeometry != null)
                    axis.Config.GeneralKinematic = _baselineGeometry;
                if (_baselineKinematics != null)
                    axis.Config.KinematicParameters = _baselineKinematics;
                if (_baselineStaticBalance != null)
                    axis.Config.StaticBalanceConfig = _baselineStaticBalance;

                LoadConfigIntoUi(axis.Config);
            }

            // Snapshot baseline values now — config is a direct reference to
            // axis.Config, so any later mutations (override apply or user edits)
            // would corrupt the base.  Must happen before overrides AND before
            // user edits can create a first override.
            _baselineGeometry = config.GeneralKinematic?.Clone();
            _baselineKinematics = config.KinematicParameters?.Clone();
            _baselineStaticBalance = config.StaticBalanceConfig?.Clone();

            int axisId = (int)config.AxisId;
            var overrides = plugin.GetAxisParameterOverride(_selectedFunctionId, axisId);

            if (overrides != null)
            {
                // If the override has stored geometry, load it into the kinematics editor
                if (overrides.GeometryJson != null)
                {
                    try
                    {
                        var geometry = _protoJsonParser.Parse<GeneralKinematicConfig>(overrides.GeometryJson);
                        config.GeneralKinematic = geometry;
                        GeneralKinematicsControl.UpdateConfig(geometry);
                    }
                    catch
                    {
                        // Fall back to base geometry if parse fails
                    }
                }

                if (overrides.Kinematics != null)
                {
                    config.KinematicParameters = overrides.Kinematics.Clone();
                }

                if (overrides.StaticBalance != null)
                {
                    config.StaticBalanceConfig = overrides.StaticBalance.Clone();
                    UpdateStaticBalanceUi(config.StaticBalanceConfig);
                    UpdateStaticBalancePlot();
                }

                BtnClearOverride.Visibility = Visibility.Visible;
            }
            else
            {
                BtnClearOverride.Visibility = Visibility.Collapsed;
            }

            // Re-enable override saves after the QueueRebuild timer has had a chance to fire
            Dispatcher.BeginInvoke(new Action(() => _suppressOverrideSave = false),
                System.Windows.Threading.DispatcherPriority.ContextIdle);
        }

        /// <summary>
        /// Handle clear override button click.
        /// </summary>
        private void ClearOverride_Click(object sender, RoutedEventArgs e)
        {
            if (plugin == null || config == null || _selectedFunctionId < 0)
                return;

            int axisId = (int)config.AxisId;
            plugin.ClearAxisParameterOverride(_selectedFunctionId, axisId);

            // Reload base config
            LoadFunctionOverrideConfig();

            // Refresh the selector to update the [F] badge
            RefreshFunctionSelector();

            DebugMessage?.Invoke($"Cleared axis override for function {_selectedFunctionId}, axis {axisId}");
        }

        /// <summary>
        /// Save kinematics changes to the appropriate location (base or override).
        /// </summary>
        private void SaveKinematicsChange(KinematicParameters parameters)
        {
            if (_editingMode == AxisEditingMode.FunctionOverride && _selectedFunctionId >= 0 && plugin != null)
            {
                int axisId = (int)config.AxisId;
                // Store both computed parameters and the geometry that produced them
                string geometryJson = null;
                if (config.GeneralKinematic != null)
                {
                    try { geometryJson = _protoJsonFormatter.Format(config.GeneralKinematic); }
                    catch { /* best-effort */ }
                }
                plugin.UpdateAxisParameterOverride(_selectedFunctionId, axisId, overrides =>
                {
                    overrides.Kinematics = parameters.Clone();
                    overrides.GeometryJson = geometryJson;
                });

                // Show clear button now that we have an override
                BtnClearOverride.Visibility = Visibility.Visible;

                // Refresh to update [F] badge
                RefreshFunctionSelector();
            }
            // In AxisBase mode, the existing KinematicParametersChanged event handles it
        }

        /// <summary>
        /// Save static balance changes to the appropriate location (base or override).
        /// </summary>
        private void SaveStaticBalanceChange(AxisConfig.Types.StaticBalanceConfig staticBalance)
        {
            if (_editingMode == AxisEditingMode.FunctionOverride && _selectedFunctionId >= 0 && plugin != null)
            {
                int axisId = (int)config.AxisId;
                plugin.UpdateAxisParameterOverride(_selectedFunctionId, axisId, overrides =>
                {
                    overrides.StaticBalance = staticBalance.Clone();
                });

                // Show clear button now that we have an override
                BtnClearOverride.Visibility = Visibility.Visible;

                // Refresh to update [F] badge
                RefreshFunctionSelector();
            }
            // In AxisBase mode, the base config is already modified
        }

        /// <summary>
        /// Get the current editing mode.
        /// </summary>
        public AxisEditingMode EditingMode => _editingMode;

        /// <summary>
        /// Get the currently selected function ID for override editing.
        /// Returns -1 if in AxisBase mode.
        /// </summary>
        public int SelectedFunctionId => _selectedFunctionId;

        #endregion

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
            new_config.StaticBalanceConfig = BuildDefaultStaticBalanceConfig();

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

        private static AxisConfig.Types.StaticBalanceConfig BuildDefaultStaticBalanceConfig()
        {
            AxisConfig.Types.StaticBalanceConfig config = new AxisConfig.Types.StaticBalanceConfig
            {
                XCenter = 0.0f,
                XHalfRange = 1.0f
            };
            while (config.Coeffs.Count < 5)
            {
                config.Coeffs.Add(0.0f);
            }
            return config;
        }

        private AxisConfig.Types.OscillationGuard EnsureOscillationGuardConfig()
        {
            if (config.OscillationGuard == null)
            {
                config.OscillationGuard = BuildDefaultOscillationGuard();
            }
            return config.OscillationGuard;
        }

        private static AxisConfig.Types.StaticBalanceConfig EnsureStaticBalanceConfig(AxisConfig axisConfig)
        {
            if (axisConfig.StaticBalanceConfig == null)
            {
                axisConfig.StaticBalanceConfig = BuildDefaultStaticBalanceConfig();
            }
            while (axisConfig.StaticBalanceConfig.Coeffs.Count < 5)
            {
                axisConfig.StaticBalanceConfig.Coeffs.Add(0.0f);
            }
            return axisConfig.StaticBalanceConfig;
        }

        private AxisConfig.Types.StaticBalanceConfig EnsureStaticBalanceConfig()
        {
            return EnsureStaticBalanceConfig(config);
        }
        public void UpdateConfig(AxisConfig new_config)
        {
            LoadConfigIntoUi(new_config);

            // Reset to base mode when loading a new axis config
            _editingMode = AxisEditingMode.AxisBase;
            _selectedFunctionId = -1;
            RefreshFunctionSelector();
        }

        /// <summary>
        /// Load axis config into UI controls without resetting the function selector.
        /// Used by LoadFunctionOverrideConfig to reload base geometry while keeping
        /// the editing mode and selected function intact.
        /// </summary>
        private void LoadConfigIntoUi(AxisConfig new_config)
        {
            config = new_config;
            GeneralKinematicsControl.Visibility = Visibility.Visible;
            ClearStaticBalanceSamples();

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
            UpdateStaticBalanceUi(EnsureStaticBalanceConfig());
            UpdateStaticBalancePlot();
            if (LabelStaticBalanceStatus != null && staticBalanceSamples.Count == 0)
            {
                LabelStaticBalanceStatus.Content = "Idle";
            }
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

        public void OnStaticBalanceResult(StaticBalanceResult result)
        {
            ApplyStaticBalanceResult(config, result);

            staticBalanceSamples = new List<float>(result.FOffset);
            staticBalanceXMin = result.XMin;
            staticBalanceXMax = result.XMax;
            staticBalanceStep = result.SampleStep;

            LabelStaticBalanceStatus.Content = $"Samples: {staticBalanceSamples.Count}";
            UpdateStaticBalanceUi(EnsureStaticBalanceConfig());
            UpdateStaticBalancePlot();
        }

        private void ClearStaticBalance_Click(object sender, RoutedEventArgs e)
        {
            ClearStaticBalanceSamples();
            var staticCfg = EnsureStaticBalanceConfig();
            staticCfg.XCenter = 0.0f;
            staticCfg.XHalfRange = 1.0f;
            staticCfg.Coeffs.Clear();
            while (staticCfg.Coeffs.Count < 5)
            {
                staticCfg.Coeffs.Add(0.0f);
            }
            UpdateStaticBalanceUi(staticCfg);
            UpdateStaticBalancePlot();
            LabelStaticBalanceStatus.Content = "Cleared";
            DebugMessage?.Invoke("Static balance cleared. Upload axis config to apply.");
        }

        private void ClearStaticBalanceSamples()
        {
            staticBalanceSamples.Clear();
            staticBalanceXMin = 0.0f;
            staticBalanceXMax = 0.0f;
            staticBalanceStep = 0.0f;
        }

        public static void ApplyStaticBalanceResult(AxisConfig axisConfig, StaticBalanceResult result)
        {
            if (axisConfig == null || result == null)
            {
                return;
            }
            if (axisConfig.AxisId != result.AxisId)
            {
                return;
            }

            float xMin = result.XMin;
            float xMax = result.XMax;
            float step = result.SampleStep;
            float center = (xMin + xMax) * 0.5f;
            float halfRange = (xMax - xMin) * 0.5f;
            if (halfRange <= 0.0f)
            {
                halfRange = 1.0f;
            }

            float[] coeffs = FitPolynomial(result.FOffset, xMin, step, center, halfRange, 4);
            var staticCfg = EnsureStaticBalanceConfig(axisConfig);
            staticCfg.XCenter = center;
            staticCfg.XHalfRange = halfRange;
            staticCfg.Coeffs.Clear();
            foreach (float coeff in coeffs)
            {
                staticCfg.Coeffs.Add(coeff);
            }
        }

        private void StartStaticBalanceCalibration_Click(object sender, RoutedEventArgs e)
        {
            if (config == null || ui == null)
            {
                return;
            }
            LabelStaticBalanceStatus.Content = "Calibrating...";
            ui.RequestStaticBalanceCalibration(config.AxisId);
        }

        private void StaticBalance_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (config == null || updatingStaticBalanceUi)
            {
                return;
            }

            var staticCfg = EnsureStaticBalanceConfig();

            if (ReferenceEquals(sender, TextStaticCenter))
            {
                if (TryParseFloat(TextStaticCenter.Text, out float value))
                {
                    staticCfg.XCenter = value;
                }
            }
            else if (ReferenceEquals(sender, TextStaticHalfRange))
            {
                if (TryParseFloat(TextStaticHalfRange.Text, out float value))
                {
                    staticCfg.XHalfRange = Math.Max(0.0f, value);
                }
            }
            else
            {
                float[] coeffs = ReadStaticBalanceCoeffs();
                staticCfg.Coeffs.Clear();
                foreach (float coeff in coeffs)
                {
                    staticCfg.Coeffs.Add(coeff);
                }
            }

            UpdateStaticBalancePlot();

            // In override mode, save to function override
            if (_editingMode == AxisEditingMode.FunctionOverride && _selectedFunctionId >= 0)
            {
                SaveStaticBalanceChange(staticCfg);
            }
        }

        private void StaticBalance_LostFocus(object sender, RoutedEventArgs e)
        {
            UpdateStaticBalanceUi(EnsureStaticBalanceConfig());
        }

        private void StaticBalanceCanvas_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateStaticBalancePlot();
        }

        private void UpdateStaticBalanceUi(AxisConfig.Types.StaticBalanceConfig staticCfg)
        {
            if (staticCfg == null)
            {
                return;
            }

            updatingStaticBalanceUi = true;
            TextStaticCenter.Text = staticCfg.XCenter.ToString("0.###", CultureInfo.CurrentCulture);
            TextStaticHalfRange.Text = staticCfg.XHalfRange.ToString("0.###", CultureInfo.CurrentCulture);
            TextStaticCoeff0.Text = staticCfg.Coeffs.Count > 0 ? staticCfg.Coeffs[0].ToString("0.#####", CultureInfo.CurrentCulture) : "0";
            TextStaticCoeff1.Text = staticCfg.Coeffs.Count > 1 ? staticCfg.Coeffs[1].ToString("0.#####", CultureInfo.CurrentCulture) : "0";
            TextStaticCoeff2.Text = staticCfg.Coeffs.Count > 2 ? staticCfg.Coeffs[2].ToString("0.#####", CultureInfo.CurrentCulture) : "0";
            TextStaticCoeff3.Text = staticCfg.Coeffs.Count > 3 ? staticCfg.Coeffs[3].ToString("0.#####", CultureInfo.CurrentCulture) : "0";
            TextStaticCoeff4.Text = staticCfg.Coeffs.Count > 4 ? staticCfg.Coeffs[4].ToString("0.#####", CultureInfo.CurrentCulture) : "0";
            updatingStaticBalanceUi = false;
        }

        private float[] ReadStaticBalanceCoeffs()
        {
            float[] coeffs = new float[5];
            if (TryParseFloat(TextStaticCoeff0.Text, out float c0)) coeffs[0] = c0;
            if (TryParseFloat(TextStaticCoeff1.Text, out float c1)) coeffs[1] = c1;
            if (TryParseFloat(TextStaticCoeff2.Text, out float c2)) coeffs[2] = c2;
            if (TryParseFloat(TextStaticCoeff3.Text, out float c3)) coeffs[3] = c3;
            if (TryParseFloat(TextStaticCoeff4.Text, out float c4)) coeffs[4] = c4;
            return coeffs;
        }

        private void UpdateStaticBalancePlot()
        {
            if (Canvas_static_balance == null || Polyline_static_raw == null || Polyline_static_fit == null)
            {
                return;
            }
            double width = Canvas_static_balance.ActualWidth > 0 ? Canvas_static_balance.ActualWidth : Canvas_static_balance.Width;
            double height = Canvas_static_balance.ActualHeight > 0 ? Canvas_static_balance.ActualHeight : Canvas_static_balance.Height;
            if (width <= 1 || height <= 1)
            {
                return;
            }

            bool hasSamples = staticBalanceSamples.Count > 0;
            double xMin = staticBalanceXMin;
            double xMax = staticBalanceXMax;
            var staticCfg = EnsureStaticBalanceConfig();
            double center = staticCfg.XCenter;
            double halfRange = staticCfg.XHalfRange;
            if (!hasSamples)
            {
                bool rangeFromKinematics = false;
                if (config?.KinematicParameters != null)
                {
                    double kMin = config.KinematicParameters.ContactPointPosMinAbs / 10.0;
                    double kMax = config.KinematicParameters.ContactPointPosMaxAbs / 10.0;
                    if (kMax > kMin)
                    {
                        xMin = kMin;
                        xMax = kMax;
                        rangeFromKinematics = true;
                    }
                }
                if (!rangeFromKinematics)
                {
                    if (halfRange <= 0.0)
                    {
                        halfRange = 1.0;
                    }
                    xMin = center - halfRange;
                    xMax = center + halfRange;
                }
            }
            if (xMax <= xMin)
            {
                xMax = xMin + 1.0;
            }

            if (halfRange <= 0.0)
            {
                halfRange = (xMax - xMin) * 0.5;
                if (halfRange <= 0.0)
                {
                    halfRange = 1.0;
                }
            }

            int pointCount = hasSamples ? staticBalanceSamples.Count : 64;
            double step = hasSamples ? staticBalanceStep : (xMax - xMin) / Math.Max(1, pointCount - 1);
            if (step <= 0.0 || double.IsNaN(step) || double.IsInfinity(step))
            {
                step = (xMax - xMin) / Math.Max(1, pointCount - 1);
            }

            List<double> fitValues = new List<double>(pointCount);
            double yMin = double.MaxValue;
            double yMax = double.MinValue;
            for (int i = 0; i < pointCount; i++)
            {
                double x = xMin + (i * step);
                double xNorm = (x - center) / halfRange;
                xNorm = Math.Max(-1.0, Math.Min(1.0, xNorm));
                double yFit = EvaluatePoly(staticCfg, xNorm);
                fitValues.Add(yFit);
                if (hasSamples)
                {
                    yMin = Math.Min(yMin, staticBalanceSamples[i]);
                    yMax = Math.Max(yMax, staticBalanceSamples[i]);
                }
                yMin = Math.Min(yMin, yFit);
                yMax = Math.Max(yMax, yFit);
            }

            if (yMin == double.MaxValue || yMax == double.MinValue)
            {
                yMin = -1.0;
                yMax = 1.0;
            }

            double pad = (yMax - yMin) * 0.1;
            if (pad <= 0.001)
            {
                pad = 0.1;
            }
            yMin -= pad;
            yMax += pad;

            UpdateStaticBalanceGrid(xMin, xMax, yMin, yMax, width, height);

            PointCollection rawPoints = new PointCollection();
            PointCollection fitPoints = new PointCollection();
            for (int i = 0; i < pointCount; i++)
            {
                double x = xMin + (i * step);
                double xN = (x - xMin) / (xMax - xMin);
                double rawX = xN * width;
                if (hasSamples)
                {
                    double yRaw = staticBalanceSamples[i];
                    double rawY = height - ((yRaw - yMin) / (yMax - yMin) * height);
                    rawPoints.Add(new Point(rawX, rawY));
                }
                double yFit = fitValues[i];
                double fitY = height - ((yFit - yMin) / (yMax - yMin) * height);
                fitPoints.Add(new Point(rawX, fitY));
            }

            Polyline_static_raw.Points = rawPoints;
            Polyline_static_fit.Points = fitPoints;
            Panel.SetZIndex(Polyline_static_raw, 1);
            Panel.SetZIndex(Polyline_static_fit, 1);
        }

        private void UpdateStaticBalanceGrid(double xMin, double xMax, double yMin, double yMax, double width, double height)
        {
            if (Canvas_static_balance == null)
            {
                return;
            }

            for (int i = Canvas_static_balance.Children.Count - 1; i >= 0; i--)
            {
                if (Canvas_static_balance.Children[i] is FrameworkElement element &&
                    element.Tag as string == StaticBalanceGridTag)
                {
                    Canvas_static_balance.Children.RemoveAt(i);
                }
            }

            if (width <= 1 || height <= 1 || xMax <= xMin || yMax <= yMin)
            {
                return;
            }

            int xTicks = 5;
            int yTicks = 5;

            AddGridLine(0, 0, 0, height, true);
            AddGridLine(0, height, width, height, true);

            for (int i = 0; i < xTicks; i++)
            {
                double t = xTicks == 1 ? 0.5 : (double)i / (xTicks - 1);
                double x = t * width;
                AddGridLine(x, 0, x, height, false);
                string label = (xMin + (xMax - xMin) * t).ToString("0.##", CultureInfo.CurrentCulture) + " mm";
                AddXTickLabel(x, height, label);
            }

            for (int i = 0; i < yTicks; i++)
            {
                double t = yTicks == 1 ? 0.5 : (double)i / (yTicks - 1);
                double y = height - (t * height);
                AddGridLine(0, y, width, y, false);
                string label = (yMin + (yMax - yMin) * t).ToString("0.##", CultureInfo.CurrentCulture) + " N";
                AddYTickLabel(y, label);
            }

            void AddGridLine(double x1, double y1, double x2, double y2, bool axis)
            {
                Line line = new Line
                {
                    X1 = x1,
                    Y1 = y1,
                    X2 = x2,
                    Y2 = y2,
                    Stroke = new SolidColorBrush(Color.FromArgb(axis ? (byte)140 : (byte)80, 255, 255, 255)),
                    StrokeThickness = axis ? 1.0 : 0.5,
                    Tag = StaticBalanceGridTag
                };
                Panel.SetZIndex(line, 0);
                Canvas_static_balance.Children.Add(line);
            }

            void AddXTickLabel(double x, double plotHeight, string text)
            {
                TextBlock label = new TextBlock
                {
                    Text = text,
                    Foreground = Brushes.White,
                    FontSize = 9,
                    Opacity = 0.7,
                    Tag = StaticBalanceGridTag
                };
                label.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
                double left = x - (label.DesiredSize.Width * 0.5);
                left = Math.Max(0, Math.Min(width - label.DesiredSize.Width, left));
                Canvas.SetLeft(label, left);
                Canvas.SetTop(label, plotHeight + 2);
                Panel.SetZIndex(label, 0);
                Canvas_static_balance.Children.Add(label);
            }

            void AddYTickLabel(double y, string text)
            {
                TextBlock label = new TextBlock
                {
                    Text = text,
                    Foreground = Brushes.White,
                    FontSize = 9,
                    Opacity = 0.7,
                    Tag = StaticBalanceGridTag
                };
                label.Measure(new Size(double.PositiveInfinity, double.PositiveInfinity));
                double top = y - (label.DesiredSize.Height * 0.5);
                top = Math.Max(0, Math.Min(height - label.DesiredSize.Height, top));
                Canvas.SetLeft(label, -label.DesiredSize.Width - 4);
                Canvas.SetTop(label, top);
                Panel.SetZIndex(label, 0);
                Canvas_static_balance.Children.Add(label);
            }
        }

        private static double EvaluatePoly(AxisConfig.Types.StaticBalanceConfig staticCfg, double x)
        {
            if (staticCfg == null || staticCfg.Coeffs.Count == 0)
            {
                return 0.0;
            }
            double value = staticCfg.Coeffs[staticCfg.Coeffs.Count - 1];
            for (int idx = staticCfg.Coeffs.Count - 2; idx >= 0; --idx)
            {
                value = (value * x) + staticCfg.Coeffs[idx];
            }
            return value;
        }

        private static float[] FitPolynomial(IList<float> samples, float xMin, float step, float center, float halfRange, int degree)
        {
            int n = degree + 1;
            double[,] mat = new double[n, n];
            double[] rhs = new double[n];
            for (int i = 0; i < samples.Count; i++)
            {
                double x = xMin + (i * step);
                double xNorm = (x - center) / halfRange;
                xNorm = Math.Max(-1.0, Math.Min(1.0, xNorm));
                double[] powers = new double[2 * n];
                powers[0] = 1.0;
                for (int p = 1; p < powers.Length; p++)
                {
                    powers[p] = powers[p - 1] * xNorm;
                }

                double y = samples[i];
                for (int r = 0; r < n; r++)
                {
                    rhs[r] += y * powers[r];
                    for (int c = 0; c < n; c++)
                    {
                        mat[r, c] += powers[r + c];
                    }
                }
            }

            double[] coeffs = SolveLinearSystem(mat, rhs);
            float[] result = new float[n];
            for (int i = 0; i < n; i++)
            {
                result[i] = (float)coeffs[i];
            }
            return result;
        }

        private static double[] SolveLinearSystem(double[,] mat, double[] rhs)
        {
            int n = rhs.Length;
            double[,] a = new double[n, n + 1];
            for (int r = 0; r < n; r++)
            {
                for (int c = 0; c < n; c++)
                {
                    a[r, c] = mat[r, c];
                }
                a[r, n] = rhs[r];
            }

            for (int i = 0; i < n; i++)
            {
                int pivot = i;
                double max = Math.Abs(a[i, i]);
                for (int r = i + 1; r < n; r++)
                {
                    double val = Math.Abs(a[r, i]);
                    if (val > max)
                    {
                        max = val;
                        pivot = r;
                    }
                }
                if (pivot != i)
                {
                    for (int c = i; c <= n; c++)
                    {
                        double tmp = a[i, c];
                        a[i, c] = a[pivot, c];
                        a[pivot, c] = tmp;
                    }
                }

                double diag = a[i, i];
                if (Math.Abs(diag) < 1e-9)
                {
                    continue;
                }
                for (int c = i; c <= n; c++)
                {
                    a[i, c] /= diag;
                }
                for (int r = 0; r < n; r++)
                {
                    if (r == i)
                    {
                        continue;
                    }
                    double factor = a[r, i];
                    for (int c = i; c <= n; c++)
                    {
                        a[r, c] -= factor * a[i, c];
                    }
                }
            }

            double[] result = new double[n];
            for (int i = 0; i < n; i++)
            {
                result[i] = a[i, n];
            }
            return result;
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
