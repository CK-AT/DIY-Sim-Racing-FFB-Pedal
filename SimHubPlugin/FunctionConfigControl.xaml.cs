using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using SimHub.Plugins.OutputPlugins.ControlRemapper.Models;

namespace User.PluginSdkDemo
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
        }

        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public event AutomotivePedalConfigControl.ABSTestStateChangeEventHandler ABSTestStateChange;
        private DiyFfbUI ui;
        private DiyFfbPlugin plugin;
        private FunctionConfig config;
        private FunctionID current_function_id;

        public void SetGui(DiyFfbUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            AutomotivePedalConfig.SetGui(ui, plugin);
            AutomotivePedalConfig.ABSTestStateChange += OnABSTestStateChange;
            AutomotivePedalConfig.DebugMessage += OnDebugMessage;
            FlightPedalsConfig.SetGui(ui, plugin);
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
            switch (function_id)
            {
                case FunctionID.Brake:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.Clutch:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.Accelerator:
                    new_config.AutomotivePedal = AutomotivePedalConfigControl.GetDefaultConfig();
                    break;
                case FunctionID.FlightPedals:
                    new_config.FlightPedals = FlightPedalsConfigControl.GetDefaultConfig();
                    new_config.AuxFunction = FlightPedalsConfigControl.GetRudderBrakeDefaultConfig(); 
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
            }
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            if (config.Base.LinkedAxes[0] == axis_state.AxisId)
            {
                switch (tc_specific_function.SelectedIndex)
                {
                    case 0:
                        AutomotivePedalConfig.OnAxisStateUpdate(axis_state);
                        break;
                    case 1:
                        FlightPedalsConfig.OnAxisStateUpdate(axis_state);
                        break;
                }
            }
        }

        public void UpdateConfig(FunctionConfig function_config)
        {
            config = function_config;
            switch (config.Base.FunctionId)
            {
                case FunctionID.Brake:
                    AutomotivePedalConfig.UpdateConfig(function_config);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.Clutch:
                    AutomotivePedalConfig.UpdateConfig(function_config);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.Accelerator:
                    AutomotivePedalConfig.UpdateConfig(function_config);
                    tc_specific_function.SelectedIndex = 0;
                    break;
                case FunctionID.FlightPedals:
                    FlightPedalsConfig.UpdateConfig(function_config);
                    tc_specific_function.SelectedIndex = 1;
                    break;
            }
        }
    }
}
