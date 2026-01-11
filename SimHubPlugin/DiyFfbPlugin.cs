using GameReaderCommon;
using NCalc;
using ProtbufTest;


//using log4net.Plugin;
using SimHub.Plugins;
using System;
using System.IO.Ports;
using System.Threading;
using System.Windows.Media;
using Windows.UI.Notifications;
using IPlugin = SimHub.Plugins.IPlugin;
namespace User.PluginSdkDemo
{
    [PluginDescription("This Plugin handles DIY FFB axes and gateways, communicating via USB.")]
    [PluginAuthor("OpenSource")]
    [PluginName("DIY FFB plugin")]
    public class DiyFfbPlugin : IPlugin, IDataPlugin, IWPFSettingsV2
    {
        DiyFfbPluginUI ui;

        public bool sendAbsSignal = false;
        public byte rpm_last_value = 0 ;
        public double g_force_last_value = 128;
        public byte Road_impact_last = 0;
        public byte game_running_index = 0 ;
        public uint testValue = 0;
        public uint[] profile_flag = new uint[4] { 0,0,0,0};
        public uint[] select_button_flag = new uint[2] { 0, 0, };// define the up and down selection
        public uint slotA_flag = 0;
        public uint slotB_flag = 0;
        public uint slotC_flag = 0;
        public uint slotD_flag = 0;
        public uint sendconfig_flag = 0;
        public uint in_game_flag = 0; // check current game is off or pause
        public string current_profile = "NA" ;
        public uint profile_index = 0;
        //public uint Page_update_flag = 0;
        public bool binding_check=false;
        public bool pedal_select_update_flag = false;
        public string current_pedal = "NA";
        public string current_action = "NA";
        public bool Page_update_flag =false;
        public uint overlay_display = 0;
        public string simhub_theme_color = "#7E87CEFA";
        public uint debug_value = 0;
        public bool clear_action = false;
        public byte pedal_state_in_ratio = 0;
        public bool Sync_esp_connection_flag=false;
        public byte PedalErrorCode = 0;
        public byte PedalErrorIndex = 0;
        public byte[] random_pedal_action_interval=new byte[3] { 50,51,53};
        public string Simhub_version = "";
        public bool Version_Check_Simhub_MSFS = false;

        //effect trigger timer
        DateTime[] Action_currentTime = new DateTime[3];
        DateTime[] Action_lastTime = new DateTime[3];
        

        // ABS trigger timer
        DateTime absTrigger_currentTime = DateTime.Now;
        DateTime absTrigger_lastTime = DateTime.Now;

        //G force timer
        DateTime GTrigger_currentTime = DateTime.Now;
        DateTime GTrigger_lastTime = DateTime.Now;

        //Road effect
        DateTime RoadTrigger_currentTime = DateTime.Now;
        DateTime RoadTrigger_lastTime = DateTime.Now;
        //https://www.c-sharpcorner.com/uploadfile/eclipsed4utoo/communicating-with-serial-port-in-C-Sharp/
        public SerialPort[] _serialPort = new SerialPort[8] {new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),
            new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),
            new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),
        new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),
            new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),
            new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One),new SerialPort("COM7", 921600, Parity.None, 8, StopBits.One)};

        public ProtobufSerial<Message> ESPsync_serialPort = new ProtobufSerial<Message>("COM7", 3000000);
        private const int GatewayReconnectIntervalMs = 2000;
        private Timer gatewayReconnectTimer;
        private int gatewayReconnectBusy = 0;

        //for (byte pedalIdx_lcl = 0; pedalIdx_lcl< 3; pedalIdx_lcl++)
        //{
        //    _serialPortt[pedalIdx_lcl].RtsEnable = false;
        //    _serialPort[pedalIdx_lcl].DtrEnable = true;
        //}   







        public bool[] connectSerialPort = { false, false, false };


        public DiyFfbPluginSettings Settings;



        /// <summary>
        /// Instance of the current plugin manager
        /// </summary>
        public PluginManager PluginManager { get; set; }

        /// <summary>
        /// Gets the left menu icon. Icon must be 24x24 and compatible with black and white display.
        /// </summary>
        public ImageSource PictureIcon => this.ToIcon(Properties.Resources.menuicon);

        /// <summary>
        /// Gets a short plugin title to show in left menu. Return null if you want to use the title as defined in PluginName attribute.
        /// </summary>
        public string LeftMenuTitle => "DIY FFB Dashboard";

        public string Ncalc_reading(String expression)
        {
            string value = "";
            try
            {
                NCalc.Expression exp = new NCalc.Expression(expression);
                exp.ResolveParameter += delegate (string name, ParameterResolveArgs rarg)
                {
                    rarg.Result = () => PluginManager.GetPropertyValue(name);
                };

                if (exp.HasErrors() == false)
                {
                    value = exp.Evaluate().ToString();
                }
                else
                {
                    value = "Error";
                }

            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Error(ex.Message);
            }

            return value;
        }

        private void UpdateFFBData(GameData data)
        {
            for (uint function_idx = 0; function_idx < Settings.function_settings.Length; function_idx++)
            {
                FunctionID function_id = (FunctionID)function_idx + 1;
                Message tmp = new Message();
                tmp.FfbAction = new FFBAction();
                tmp.FfbAction.FunctionId = function_id;
                switch (function_id)
                {
                    case FunctionID.BrakePedal:
                        tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                        if (data.NewData?.ABSActive > 0)
                        {
                            tmp.FfbAction.AutomotivePedal.TriggerAbs = true;
                            if (ESPsync_serialPort.IsOpen)
                            {
                                ESPsync_serialPort.WriteMessage(tmp);
                            }
                        }
                        break;
                    case FunctionID.AcceleratorPedal:
                        tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                        break;
                    case FunctionID.ClutchPedal:
                        tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                        break;
                }
            }
        }

        /// <summary>
        /// Called one time per game data update, contains all normalized game data,
        /// raw data are intentionnally "hidden" under a generic object type (A plugin SHOULD NOT USE IT)
        ///
        /// This method is on the critical path, it must execute as fast as possible and avoid throwing any error
        ///
        /// </summary>
        /// <param name="pluginManager"></param>
        /// <param name="data">Current game data, including current and previous data frame.</param>
        /// 
        public void DataUpdate(PluginManager pluginManager, ref GameData data)
        {
			
			bool sendAbsSignal_local_b = false;
            bool sendTcSignal_local_b = false;
            double RPM_value =0;
            double RPM_MAX = 0;
            double _G_force = 128;
            byte WS_value = 0;
            byte Road_impact_value = 0;
            byte CV1_value = 0;
            byte CV2_value = 0;
            double MSFS_RPM_Value_Simhub = 0;
            double RUDDER_DEFLECTION_Simhub = 0;
            double RELATIVE_WIND_VELOCITY_BODY_Z_Simhub = 0;
            double ACCELERATION_BODY_Z_Simhub = 0;
            double ACCELERATION_BODY_Y_Simhub = 0;
            bool MSFS_running_simhub = false;
            
            //bool WS_flag = false;

            if (data.GamePaused | (!data.GameRunning))
            {
                in_game_flag = 0;
            }
            else 
            {
                in_game_flag = 1;
            }
            //for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            //{
            //    if (_serialPort[pedalIdx].IsOpen)
            //    {
            //        int receivedLength = _serialPort[pedalIdx].BytesToRead;

            //        settings.TextBox_debugOutput.Text = "Test";
            //    }
            //}


            // Send ABS signal when triggered by the game
            if (data.GameRunning)
            {
                if (data.OldData != null && data.NewData != null)
                {
                    if (data.NewData.ABSActive > 0)
                    {
                        sendAbsSignal_local_b = true;
                    }

                    if (data.NewData.TCActive > 0)
                    {
                        sendTcSignal_local_b = true;
                    }


                    // when test signal is activated, overwrite trigger signal
                    if (sendAbsSignal)
                    {
                        sendAbsSignal_local_b = true;
                        sendTcSignal_local_b = true;
                    }



                    //fill the RPM value
                    if (Settings.RPM_effect_type == 0)
                    {
                        if (data.NewData.CarSettings_MaxRPM == 0)
                        {
                            RPM_MAX = 10000;
                        }
                        else
                        {
                            RPM_MAX = data.NewData.CarSettings_MaxRPM;
                        }

                        RPM_value = (data.NewData.Rpms / RPM_MAX * 100);
                    }
                    else
                    {
                        if (data.NewData.MaxSpeedKmh == 0)
                        {
                            RPM_MAX = 300;
                        }
                        else
                        { 
                            RPM_MAX= data.NewData.MaxSpeedKmh;
                        }
                        RPM_value = (data.NewData.SpeedKmh / RPM_MAX * 100);
                    }

                    
                    if (data.NewData.GlobalAccelerationG != 0)
                    {
                        _G_force = -1 * data.NewData.GlobalAccelerationG + 128;
                    }
                    else
                    {
                        _G_force = 128;
                    }

                    game_running_index = 1;
                    

                }
                else
                {
                    RPM_value = 0;
                    _G_force = 128;
                    


                }
            }
            else
            {
                RPM_value = 0;
                _G_force = 128;
                
            }
			




            absTrigger_currentTime = DateTime.Now;
            TimeSpan diff = absTrigger_currentTime - absTrigger_lastTime;
            int millisceonds = (int)diff.TotalMilliseconds;
            if (millisceonds <= 10)
            {
                sendAbsSignal_local_b = false;
                sendTcSignal_local_b = false;
                

            }
            else
            {
                absTrigger_lastTime = DateTime.Now;
            }




            bool update_flag = false;

            if (data.GameRunning)
            {
                UpdateFFBData(data);
                // Send ABS trigger signal via serial
                //for (uint function_idx = 0; function_idx < Settings.function_settings.Length; function_idx++)
                //{
                //    FunctionID function_id = (FunctionID)function_idx + 1;
                //    Message tmp = new Message();
                //    tmp.FfbAction = new FFBAction();
                //    switch ((FunctionID)function_idx + 1)
                //    {
                //        case FunctionID.Brake:
                //            tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                //            break;
                //        case FunctionID.Accelerator:
                //            tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                //            break;
                //        case FunctionID.Clutch:
                //            tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                //            break;
                //    }
                //    tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();

                //        if (Settings.function_settings[function_idx].G_force_enabled)
                //        {
                //            tmp.FfbAction.AutomotivePedal.G = (Byte)g_force_last_value;
                //        }
                //        else
                //        {
                //            tmp.FfbAction.AutomotivePedal.G = 128;
                //        }


                //        if (Settings.function_settings[function_idx].RPM_enabled)
                //        {

                //            if (Math.Abs(RPM_value - rpm_last_value) > 3)
                //            {
                //            tmp.FfbAction.AutomotivePedal.Rpm = (Byte)RPM_value;
                //                update_flag = true;
                //                rpm_last_value = (Byte)RPM_value;
                //            }

                //        }
                //        else
                //        {
                //        tmp.FfbAction.AutomotivePedal.Rpm = 0;
                //        }

                //        //G force effect only effect on brake
                //        if (function_idx == 1)
                //        {

                //            GTrigger_currentTime = DateTime.Now;
                //            TimeSpan diff_G = GTrigger_currentTime - GTrigger_lastTime;
                //            int millisceonds_G = (int)diff_G.TotalMilliseconds;
                //            if (millisceonds <= 10)
                //            {
                //                _G_force = g_force_last_value;
                //            }
                //            else
                //            {
                //                GTrigger_lastTime = DateTime.Now;
                //            }
                //            if (Settings.function_settings[function_idx].G_force_enabled)
                //            {
                //                //double value_check_g = 1 - _G_force / ((double)g_force_last_value);
                //                double value_check_g = (_G_force - (double)g_force_last_value);
                //                if (Math.Abs(value_check_g) > 2)
                //                {
                //                    tmp.FfbAction.AutomotivePedal.G = (Byte)_G_force;
                //                    update_flag = true;
                //                    g_force_last_value = (Byte)_G_force;
                //                }

                //            }
                //        }

                //        //Wheel slip

                //        if (Settings.function_settings[function_idx].WS_enabled)
                //        {
                //            if (pluginManager.GetPropertyValue(Settings.WSeffect_bind) != null)
                //            {
                //                /*object tmp_ws = (pluginManager.GetPropertyValue(Settings.WSeffect_bind));
                //                int tmp_ws_number = Int32.Parse(tmp_ws.ToString());
                //                WS_value = (byte)tmp_ws_number;
                //                */
                //                WS_value = Convert.ToByte(pluginManager.GetPropertyValue(Settings.WSeffect_bind));
                //                //pluginManager.SetPropertyValue("Wheelslip-test", this.GetType(), WS_value);
                //                if (WS_value >= (Settings.WS_trigger + 50))
                //                {
                //                    tmp.FfbAction.AutomotivePedal.TriggerWs = true;
                //                    update_flag = true;
                //                }
                //            }
                //        }
                //        //Road impact
                //        if (Settings.function_settings[function_idx].Road_impact_enabled)
                //        {
                //            if (pluginManager.GetPropertyValue(Settings.Road_impact_bind) != null)
                //            {
                //                Road_impact_value = Convert.ToByte(pluginManager.GetPropertyValue(Settings.Road_impact_bind));

                //                RoadTrigger_currentTime = DateTime.Now;
                //                TimeSpan diff_Road = RoadTrigger_currentTime - RoadTrigger_lastTime;
                //                int millisceonds_G = (int)diff_Road.TotalMilliseconds;
                //                if (millisceonds <= 10)
                //                {
                //                    Road_impact_value = Road_impact_last;
                //                }
                //                else
                //                {
                //                    RoadTrigger_lastTime = DateTime.Now;
                //                }
                //                if (true)
                //                {
                //                    //double value_check_g = 1 - _G_force / ((double)g_force_last_value);
                //                    double value_check_road = Road_impact_value - Road_impact_last;
                //                    if (Math.Abs(value_check_road) > 2)
                //                    {
                //                        tmp.FfbAction.AutomotivePedal.ImpactValue = Road_impact_value;
                //                        update_flag = true;
                //                        Road_impact_last = Road_impact_value;
                //                        debug_value = Road_impact_value;
                //                    }

                //                }
                //            }
                //        }
                //     //custom effcts
                //     if (Settings.function_settings[function_idx].CV1_enabled == true)
                //     {
                //        //CV1_value = Convert.ToByte(pluginManager.GetPropertyValue(Settings.CV1_bindings[pedalIdx]));
                //        string temp_string = Ncalc_reading(Settings.function_settings[function_idx].CV1_binding);
                //        if (temp_string != "Error")
                //        {
                //            CV1_value = Convert.ToByte(temp_string);
                //        }
                //        else
                //        {
                //            CV1_value = 0;
                //            SimHub.Logging.Current.Error("CV1 Reading error");
                //        }


                //        if (CV1_value > (Settings.function_settings[function_idx].CV1_trigger_level))
                //        {
                //            tmp.FfbAction.AutomotivePedal.TriggerCv1 = true;
                //            update_flag = true;
                //        }
                //    }
                //     if (Settings.function_settings[function_idx].CV2_enabled == true)
                //     {

                //        //CV2_value = Convert.ToByte(pluginManager.GetPropertyValue(Settings.CV2_bindings[pedalIdx]));
                //        string temp_string = Ncalc_reading(Settings.function_settings[function_idx].CV2_binding);
                //        if (temp_string != "Error")
                //        {
                //            CV2_value = Convert.ToByte(temp_string);
                //        }
                //        else
                //        {
                //            CV2_value = 0;
                //            SimHub.Logging.Current.Error("CV2 Reading error");
                //        }
                //        if (CV2_value > (Settings.function_settings[function_idx].CV2_trigger_level))
                //        {
                //            tmp.FfbAction.AutomotivePedal.TriggerCv2 = true;
                //            update_flag = true;
                //        }

                //    }




                //        if (function_idx == 1)
                //        {
                //            if (sendAbsSignal_local_b && Settings.function_settings[function_idx].ABS_enabled)
                //            {
                //            //_serialPort[1].Write("2");

                //            // compute checksum
                //                tmp.FfbAction.AutomotivePedal.TriggerAbs = true;
                //                update_flag = true;

                //            }
                //        }
                //        if (function_idx == 2)
                //        {
                //            if (sendTcSignal_local_b && Settings.function_settings[function_idx].ABS_enabled)
                //            {
                //            // compute checksum

                //                tmp.FfbAction.AutomotivePedal.TriggerAbs = true;
                //                update_flag = true;

                //            }
                //        }
                //    // check the update interval
                //    if (update_flag)
                //    {
                //        Action_currentTime[function_idx] = DateTime.Now;
                //        TimeSpan diff_action = Action_currentTime[function_idx] - Action_lastTime[function_idx];
                //        int millisceonds_action = (int)diff_action.TotalMilliseconds;
                //        if (millisceonds_action <= Settings.function_settings[function_idx].action_interval)
                //        {
                //            update_flag = false;
                //        }
                //        else
                //        {
                //            Action_lastTime[function_idx] = DateTime.Now;

                //        }


                //    }


                //    if (update_flag)
                //    {

                //            if (Settings.axis_settings[function_idx].via_gateway)
                //            {
                //                if (ESPsync_serialPort.IsOpen)
                //                {
                //                    tmp.FfbAction.FunctionId = FunctionID.Brake; // TODO: set correctly
                //                    ESPsync_serialPort.WriteMessage(tmp);
                //                    //ESPsync_serialPort.DiscardInBuffer();
                //                    //ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                //                    System.Threading.Thread.Sleep(7);
                //                }

                //            }
                //            else
                //            {
                //                if (_serialPort[function_idx].IsOpen)
                //                {
                //                    // clear inbuffer 
                //                    _serialPort[function_idx].DiscardInBuffer();

                //                    // send query command
                //                    //_serialPort[pedalIdx].Write(newBuffer, 0, newBuffer.Length);
                //                }

                //            }



                //    }

                //}

            }
            else
            {
                if (game_running_index == 1)
                {
                    game_running_index = 0;
                    clear_action = true;   
                }
            }



            // Send ABS test signal if requested
            if (sendAbsSignal)
            {
                Message tmp = new Message();
                tmp.FfbAction = new FFBAction();
                tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                tmp.FfbAction.AutomotivePedal.TriggerAbs = true;

                if (ESPsync_serialPort.IsOpen)
                {
                    ESPsync_serialPort.WriteMessage(tmp);
                }
            }
            if (clear_action)
            {
                Message tmp = new Message();
                tmp.FfbAction = new FFBAction();
                tmp.FfbAction.AutomotivePedal = new AutomotivePedalFFBAction();
                tmp.FfbAction.AutomotivePedal.G = 128;
                tmp.FfbAction.AutomotivePedal.TriggerAbs = true;

                for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
                {
                    tmp.FfbAction.FunctionId = FunctionID.BrakePedal; // TODO: set correctly
                    if (Settings.axis_settings[pedalIdx].via_gateway)
                    {
                        if (ESPsync_serialPort.IsOpen)
                        {
                            ESPsync_serialPort.WriteMessage(tmp);
                            //ESPsync_serialPort.DiscardInBuffer();
                            //ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                            System.Threading.Thread.Sleep(10);
                        }
                    }
                    else
                    {
                        if (_serialPort[pedalIdx].IsOpen)
                        {
                            // clear inbuffer 
                            _serialPort[pedalIdx].DiscardInBuffer();

                            // send query command
                            //_serialPort[pedalIdx].Write(newBuffer, 0, newBuffer.Length);
                        }

                    }
                }
                clear_action = false;
            }


            this.AttachDelegate("CurrentProfile", () => current_profile);
            pluginManager.SetPropertyValue("SelectedPedal", this.GetType(), current_pedal);
            pluginManager.SetPropertyValue("Action", this.GetType(), current_action);
            pluginManager.SetPropertyValue("ABS_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].ABS_enabled);
            pluginManager.SetPropertyValue("RPM_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].RPM_enabled);
            pluginManager.SetPropertyValue("Gforce_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].G_force_enabled);
            pluginManager.SetPropertyValue("WheelSlip_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].WS_enabled);
            pluginManager.SetPropertyValue("RoadImpact_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].Road_impact_enabled);
            pluginManager.SetPropertyValue("Overlay_display", this.GetType(), overlay_display);
            pluginManager.SetPropertyValue("Theme_color", this.GetType(), simhub_theme_color);
            pluginManager.SetPropertyValue("ProfileIndex", this.GetType(), profile_index);
            pluginManager.SetPropertyValue("debugvalue", this.GetType(), debug_value);
            pluginManager.SetPropertyValue("pedal_position", this.GetType(), pedal_state_in_ratio);
            pluginManager.SetPropertyValue("PedalErrorIndex", this.GetType(), PedalErrorIndex);
            pluginManager.SetPropertyValue("PedalErrorCode", this.GetType(), PedalErrorCode);
        }




        /// <summary>
        /// Returns the settings control, return null if no settings control is required
        /// </summary>
        /// <param name="pluginManager"></param>
        /// <returns></returns>
        public System.Windows.Controls.Control GetWPFSettingsControl(PluginManager pluginManager)
        {
            ui = new DiyFfbPluginUI(this);
            return ui;
        }


        /// <summary>
        /// Called at plugin manager stop, close/dispose anything needed here !
        /// Plugins are rebuilt at game change
        /// </summary>
        /// <param name="pluginManager"></param>
        public void End(PluginManager pluginManager)
        {           
            // Save settings
            this.SaveCommonSettings("GeneralSettings", Settings);

            StopGatewayAutoReconnect();

            // close serial communication
            if (ui != null)
            {

                try
                {
                    //wpfHandle.joystick.Release();
                    //wpfHandle.joystick.Dispose();
                    if (ui.joystick != null)
                    {
                        ui.joystick.RelinquishVJD(Settings.vjoy_order);
                    }
                    
                    
                }
                catch (Exception caughtEx)
                { 
                }
                
                ui.CloseSerialPorts();
            }
            else if (ESPsync_serialPort != null && ESPsync_serialPort.IsOpen)
            {
                ESPsync_serialPort.Close();
            }
            
            if (ToastNotificationManager.History.GetHistory("Pedal_notification").Count != 0)
            {
                ToastNotificationManager.History.Remove("Pedal_notification");
            }
            

        }



        public bool PortExists(string portName)
        {
            string[] portNames = SerialPort.GetPortNames();
            return Array.Exists(portNames, name => name.Equals(portName, StringComparison.OrdinalIgnoreCase));
        }

        private void StartGatewayAutoReconnect()
        {
            if (gatewayReconnectTimer != null)
            {
                return;
            }

            gatewayReconnectTimer = new Timer(_ => AutoReconnectGateway(), null, 0, GatewayReconnectIntervalMs);
        }

        private void StopGatewayAutoReconnect()
        {
            if (gatewayReconnectTimer != null)
            {
                gatewayReconnectTimer.Dispose();
                gatewayReconnectTimer = null;
            }
        }

        private void AutoReconnectGateway()
        {
            if (Settings == null || !Settings.Pedal_ESPNow_auto_connect_flag)
            {
                return;
            }

            string portName = Settings.ESPNow_port;
            if (string.IsNullOrWhiteSpace(portName) || portName == "NA")
            {
                return;
            }

            if (!PortExists(portName))
            {
                return;
            }

            if (ESPsync_serialPort != null && ESPsync_serialPort.IsOpen)
            {
                if (string.Equals(ESPsync_serialPort.PortName, portName, StringComparison.OrdinalIgnoreCase))
                {
                    return;
                }

                ESPsync_serialPort.Close();
            }

            if (Interlocked.Exchange(ref gatewayReconnectBusy, 1) == 1)
            {
                return;
            }

            try
            {
                if (ESPsync_serialPort == null)
                {
                    ESPsync_serialPort = new ProtobufSerial<Message>(portName, 3000000);
                }
                else if (!string.Equals(ESPsync_serialPort.PortName, portName, StringComparison.OrdinalIgnoreCase))
                {
                    ESPsync_serialPort.PortName = portName;
                }

                ESPsync_serialPort.Open();
            }
            catch (Exception)
            {
            }
            finally
            {
                Interlocked.Exchange(ref gatewayReconnectBusy, 0);
            }
        }




        /// <summary>
        /// Called once after plugins startup
        /// Plugins are rebuilt at game change
        /// </summary>
        /// <param name="pluginManager"></param>
        public void Init(PluginManager pluginManager)
        {
            SimHub.Logging.Current.Info("Starting DIY active pedal plugin");

            // Load settings
            Settings = this.ReadCommonSettings<DiyFfbPluginSettings>("GeneralSettings", () => new DiyFfbPluginSettings());
            Simhub_version = (String)pluginManager.GetPropertyValue("DataCorePlugin.SimHubVersion");
            // Declare a property available in the property list, this gets evaluated "on demand" (when shown or used in formulas)
            //this.AttachDelegate("CurrentDateTime", () => DateTime.Now);
            pluginManager.AddProperty("ProfileIndex", this.GetType(), profile_index);
            pluginManager.AddProperty("SelectedPedal", this.GetType(), current_pedal);
            pluginManager.AddProperty("Action", this.GetType(), current_action);
            pluginManager.AddProperty("ABS_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].ABS_enabled);
            pluginManager.AddProperty("RPM_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].RPM_enabled);
            pluginManager.AddProperty("Gforce_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].G_force_enabled);
            pluginManager.AddProperty("WheelSlip_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].WS_enabled);
            pluginManager.AddProperty("RoadImpact_effect_status", this.GetType(), Settings.function_settings[Settings.function_tab_selected].Road_impact_enabled);
            pluginManager.AddProperty("Overlay_display", this.GetType(), overlay_display);
            pluginManager.AddProperty("Theme_color", this.GetType(), simhub_theme_color);
            pluginManager.AddProperty("debugvalue", this.GetType(), debug_value);
            pluginManager.AddProperty("pedal_position", this.GetType(), pedal_state_in_ratio);
            pluginManager.AddProperty("PedalErrorIndex", this.GetType(), PedalErrorIndex);
            pluginManager.AddProperty("PedalErrorCode", this.GetType(), PedalErrorCode);
            for (uint pedali=0; pedali < 3; pedali++)
            {
                Action_currentTime[pedali] = new DateTime();
                Action_currentTime[pedali]=DateTime.Now;
                Action_lastTime[pedali] = new DateTime();
                Action_lastTime[pedali] = DateTime.Now;
            }

            Version inputVersion = new Version(Simhub_version);
            string MSFS_Version_Above = "9.5.99";
            Version versionThreshold = new Version(MSFS_Version_Above);
            if (inputVersion > versionThreshold)
            {
                Version_Check_Simhub_MSFS = true;
            }
            else
            {
                Version_Check_Simhub_MSFS = false;
            }

            // Declare an event
            //this.AddEvent("SpeedWarning");


            // Declare an action which can be called
            /*
            this.AddAction("IncrementSpeedWarning",(a, b) =>
            {
                Settings.SpeedWarningLevel++;
                SimHub.Logging.Current.Info("Speed warning changed");
            });

            // Declare an action which can be called
            this.AddAction("DecrementSpeedWarning", (a, b) =>
            {
                Settings.SpeedWarningLevel--;
            });

            */


            this.AddAction("ChangeSlotA", (a, b) =>
            {
                profile_index = 0;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotA");
                current_profile = "Slot A";
                current_action= "Slot A";
            });

            this.AddAction("ChangeSlotB", (a, b) =>
            {
                
                profile_index = 1;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotB");
                current_profile = "Slot B";
                current_action = "Slot B";
            });

            this.AddAction("ChangeSlotC", (a, b) =>
            {
                profile_index = 2;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotC");
                current_profile = "Slot C";
                current_action = "Slot C";
            });

            this.AddAction("ChangeSlotD", (a, b) =>
            {
                profile_index = 3;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotD");
                current_profile = "Slot D";
                current_action = "Slot D";
            });
            this.AddAction("ChangeSlotE", (a, b) =>
            {
                profile_index = 4;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotE");
                current_profile = "Slot E";
                current_action = "Slot E";
            });
            this.AddAction("ChangeSlotF", (a, b) =>
            {
                profile_index = 5;
                Page_update_flag = true;
                SimHub.Logging.Current.Info("SlotF");
                current_profile = "Slot F";
                current_action = "Slot F";
            });
            this.AddAction("SendConfigToPedal", (a, b) =>
            {
                sendconfig_flag =1;
                SimHub.Logging.Current.Info("SendConfig");
                current_action = "Send Config to Pedal";
            });

            this.AddAction("PreviousProfile", (a, b) =>
            {
                if (profile_index == 0)
                {
                    profile_index=5;
                }
                else
                {
                    profile_index--;
                }
                

                Page_update_flag = true;
                SimHub.Logging.Current.Info("PreviousProfile");
                current_action = "Previous Profile";
            });

            this.AddAction("NextProfile", (a, b) =>
            {
                profile_index++;
                if (profile_index > 5)
                {
                    profile_index = 0;
                }
                Page_update_flag = true;
                SimHub.Logging.Current.Info("NextProfile");
                current_action = "Next Profile";
            });
            this.AddAction("NextPedal", (a, b) =>
            {
                Settings.function_tab_selected++;
                if (Settings.function_tab_selected >= 4)
                {
                    Settings.function_tab_selected = 0;
                }
                Page_update_flag = true;
                SimHub.Logging.Current.Info("NextPedal");
                current_action = "Next Pedal";
            });
            this.AddAction("PreviousPedal", (a, b) =>
            {
                
                if (Settings.function_tab_selected == 0)
                {
                    Settings.function_tab_selected = 3;
                }
                else
                {
                    Settings.function_tab_selected--;
                }
                Page_update_flag = true;
                SimHub.Logging.Current.Info("PreviousPedal");
                current_action = "Previous Pedal";
            });
            this.AddAction("ABStoggle", (a, b) =>
            {
                if (!Settings.function_settings[Settings.function_tab_selected].ABS_enabled)
                {
                    Settings.function_settings[Settings.function_tab_selected].ABS_enabled = true;
                    SimHub.Logging.Current.Info("ABS on");
                    current_action = "ABS On";
                }
                else
                {
                    Settings.function_settings[Settings.function_tab_selected].ABS_enabled = false;
                    SimHub.Logging.Current.Info("ABS off");
                    current_action = "ABS Off";
                }
                Page_update_flag = true;
            });
            this.AddAction("RPMtoggle", (a, b) =>
            {
                if (!Settings.function_settings[Settings.function_tab_selected].RPM_enabled)
                {
                    Settings.function_settings[Settings.function_tab_selected].RPM_enabled = true;
                    SimHub.Logging.Current.Info("RPM on");
                    current_action = "RPM On";
                }
                else
                {
                    Settings.function_settings[Settings.function_tab_selected].RPM_enabled = false;
                    SimHub.Logging.Current.Info("RPM off");
                    current_action = "RPM Off";
                }
                Page_update_flag = true;
            });
            this.AddAction("Gforce_toggle", (a, b) =>
            {
                if (Settings.function_tab_selected == 1)
                {
                    if (!Settings.function_settings[Settings.function_tab_selected].G_force_enabled)
                    {
                        Settings.function_settings[Settings.function_tab_selected].G_force_enabled = true;
                        SimHub.Logging.Current.Info("Gforce on");
                        current_action = "Gforce On";
                    }
                    else
                    {
                        Settings.function_settings[Settings.function_tab_selected].G_force_enabled = false;
                        SimHub.Logging.Current.Info("Gforce off");
                        current_action = "Gforce Off";
                    }
                    Page_update_flag = true;
                }

            });
            this.AddAction("WheelSliptoggle", (a, b) =>
            {
                if (!Settings.function_settings[Settings.function_tab_selected].WS_enabled)
                {
                    Settings.function_settings[Settings.function_tab_selected].WS_enabled = true;
                    SimHub.Logging.Current.Info("WheelSlip on");
                    current_action = "Wheel Slip On";
                }
                else
                {
                    Settings.function_settings[Settings.function_tab_selected].WS_enabled = false;
                    SimHub.Logging.Current.Info("WheelSlip off");
                    current_action = "Wheel Slip Off";
                }
                Page_update_flag = true;
            });

            this.AddAction("RoadImpacttoggle", (a, b) =>
            {
                if (!Settings.function_settings[Settings.function_tab_selected].Road_impact_enabled)
                {
                    Settings.function_settings[Settings.function_tab_selected].Road_impact_enabled = true;
                    SimHub.Logging.Current.Info("RoadImpact on");
                    current_action = "Wheel Slip On";
                }
                else
                {
                    Settings.function_settings[Settings.function_tab_selected].Road_impact_enabled = false;
                    SimHub.Logging.Current.Info("RoadImpact off");
                    current_action = "RoadImpact Off";
                }
                Page_update_flag = true;
            });

            this.AddAction("OverlayToggle", (a, b) =>
            {
                Page_update_flag = true;
                SimHub.Logging.Current.Info("OverlayToggle");               
                current_action = "OverlayToggle";
                if (overlay_display == 1)
                {
                    overlay_display = 0;
                }
                else
                {
                    overlay_display = 1;
                }
            });
            //Settings.selectedJsonIndexLast[0]
            //SimHub.Logging.Current.Info("Diy active pedas plugin - Test 1");
            //SimHub.Logging.Current.Info("Diy active pedas plugin - COM port: " + Settings.selectedComPortNames[0]);




            // get WPF handler
            //wpfHandler = (DiyFfbPluginUI)GetWPFSettingsControl(pluginManager);

            //if (wpfHandler.)
            {
                // prepare serial port interfaces
                for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
                {

                    _serialPort[pedalIdx].Handshake = Handshake.None;
                    _serialPort[pedalIdx].RtsEnable = false;
                    _serialPort[pedalIdx].DtrEnable = false;


                    if (_serialPort[pedalIdx].IsOpen)
                    {
                        System.Threading.Thread.Sleep(300);
                    }


                    try
                    {
                        _serialPort[pedalIdx].PortName = Settings.axis_settings[pedalIdx].com_port_name; ;
                    }
                    catch (Exception caughtEx)
                    {
                    }
                    
                    ////try connect back to com port
                    //if (Settings.axis_settings[pedalIdx].auto_connect_flag == 1)
                    //{

                    //    if (Settings.connect_status[pedalIdx] == 1)
                    //    {
                    //        //_serialPort[pedalIdx].PortName = Settings.selectedComPortNames[pedalIdx];
                    //        //SerialPort.GetPortNames
                    //        if (PortExists(_serialPort[pedalIdx].PortName))
                    //        {
                    //            if (_serialPort[pedalIdx].IsOpen == false)
                    //            {
                    //                //if (wpfHandle != null)
                    //                //{
                    //                //    wpfHandle.openSerialAndAddReadCallback(pedalIdx);
                    //                //}

                    //                connectSerialPort[pedalIdx] = true;
                    //            }
                    //            else
                    //            {
                    //                //if (wpfHandle != null)
                    //                //{
                    //                //    wpfHandle.closeSerialAndStopReadCallback(pedalIdx);
                    //                //}
                    //                //ConnectToPedal.IsChecked = false;
                    //                //TextBox_debugOutput.Text = "Serialport already open, close it";
                    //                //Settings.connect_status[pedalIdx] = 0;
                    //                connectSerialPort[pedalIdx] = false;
                    //            }


                    //        }
                    //        else
                    //        {
                    //            //Settings.connect_status[pedalIdx] = 0;
                    //            connectSerialPort[pedalIdx] = false;
                    //        }
                    //    }
                    //    else
                    //    {
                    //        //Settings.connect_status[pedalIdx] = 0;
                    //        connectSerialPort[pedalIdx] = false;
                    //    }

                    //}
                    

                }

            }

            StartGatewayAutoReconnect();

        }
    }
}
