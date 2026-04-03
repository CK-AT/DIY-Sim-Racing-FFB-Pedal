using GameReaderCommon;
using NCalc;
using ProtbufTest;


//using log4net.Plugin;
using SimHub.Plugins;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Windows.Media;
using DiyFfb.GraphEditor;
using DiyFfb.TieredConfig;
using Windows.UI.Notifications;
using IPlugin = SimHub.Plugins.IPlugin;
namespace DiyFfb
{
    /// <summary>
    /// Category of the active graph, used to customize the Vehicle/Aircraft tab.
    /// </summary>
    public enum GraphCategory
    {
        Vehicle,      // Default: car icon, "VEHICLE" label
        Aircraft,     // Airplane icon, "AIRCRAFT" label (FlightStick*, FlightPedals*)
        Helicopter    // Helicopter icon, "AIRCRAFT" label (FlightStickCollective)
    }

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
        private const uint XPlanePacketMagic = 0x46464244;
        private const ushort XPlanePacketVersion = 3;
        private const int XPlaneMaxRotors = 4;
        private const int XPlanePacketSizeBytes = 132;
        private const double XPlaneTelemetryFreshnessMs = 200.0;
        private const double XPlaneRotorWindowSeconds = 3.0;
        private readonly object xplaneLock = new object();
        private UdpClient xplaneUdpClient;
        private Thread xplaneUdpThread;
        private CancellationTokenSource xplaneUdpCts;
        private XPlaneUdpPacket latestXPlanePacket;
        private uint xplaneLastSequence;
        private int xplaneDropouts;
        private DateTime xplaneLastReceivedUtc = DateTime.MinValue;
        private DateTime xplaneLastSendUtc = DateTime.MinValue;
        private string activeCarId;
        private string activeCarName;
        private string activeGameId;
        private string activeGraphKey;
        private string activeGraphPath;
        private GraphDefinition activeVehicleGraph;
        private GraphValidationResult activeGraphValidation;
        private DiyFfb.GraphTest.GraphDefinition activeGraphRuntime;
        private DiyFfb.GraphTest.GraphCompiledEvaluator activeGraphEvaluator;
        private DiyFfb.GraphTest.GraphIncludeResolver activeGraphResolver;
        private DiyFfb.GraphTest.IncludeContextCache activeIncludeContextCache;
        private readonly Dictionary<string, double> graphInputs = new Dictionary<string, double>();
        private readonly Dictionary<string, double> graphParams = new Dictionary<string, double>();
        private ButtonInputReader _buttonInputReader;
        private long _lastGraphEvalTicks;
        internal ButtonInputReader ButtonInputReader => _buttonInputReader;
        private DiyFfb.GraphTest.GraphEvaluationResult lastGraphEvaluation;
        private static Func<GameData, string> gameIdGetter;
        private DiyFfbPluginSettings.AircraftFfbProfile pendingFfbProfile;
        private bool hasPendingFfbProfile;
        private readonly HashSet<FunctionID> disabledOutputFunctions = new HashSet<FunctionID>();
        private readonly object outputDisableLock = new object();
        private readonly Queue<RotorRpmSample>[] rotorRpmHistory = new Queue<RotorRpmSample>[XPlaneMaxRotors];
        private int lastAutoRotorIndex = 0;
        private bool hasAutoRotorIndex = false;

        // Tiered config managers for profile/user overrides
        private readonly FunctionConfigManager _functionConfigManager = new FunctionConfigManager();
        private readonly AxisConfigManager _axisConfigManager = new AxisConfigManager();
        private TieredConfigOrchestrator _configOrchestrator;

        /// <summary>
        /// Orchestrates tiered config lifecycle (Baseline → Profile → User).
        /// </summary>
        public TieredConfigOrchestrator ConfigOrchestrator => _configOrchestrator;

        /// <summary>
        /// Manages function config lifecycle for profile/user overrides.
        /// </summary>
        public FunctionConfigManager FunctionConfigManager => _functionConfigManager;

        /// <summary>
        /// Manages axis config lifecycle for function overrides.
        /// </summary>
        public AxisConfigManager AxisConfigManager => _axisConfigManager;

        internal sealed class XPlaneUdpPacket
        {
            public uint Sequence;
            public float IasKts;
            public float TasMps;
            public float AlphaDeg;
            public float BetaDeg;
            public float PRate;
            public float QRate;
            public float RRate;
            public float ElevDefDeg;
            public float AilDefDeg;
            public float RudDefDeg;
            public float ElevTrimNorm;
            public float AilTrimNorm;
            public float RudTrimNorm;
            public float GNrml;
            public float[] TorqueNm = new float[XPlaneMaxRotors];
            public float[] OmegaRad = new float[XPlaneMaxRotors];
            public float[] PropRatio = new float[XPlaneMaxRotors];
            public float LAero;
            public float MAero;
            public float NAero;
            public bool OnGround;
            public DateTime ReceivedUtc;
        }

        private struct RotorRpmSample
        {
            public DateTime Utc;
            public float Rpm;
        }

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

            if (data.NewData != null)
            {
                string gameId = GetGameIdSafe(data);
                HandleGameChange(gameId);
                HandleAircraftChange(data, gameId);
            }

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
                EvaluateActiveGraph(data);
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


            ProcessGraphFfb();

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
            if (!string.IsNullOrWhiteSpace(activeCarId) && HasUnsavedProfileChanges(activeGameId, activeCarId))
            {
                bool saveCurrent = false;
                if (ui != null)
                {
                    saveCurrent = (bool)ui.Dispatcher.Invoke(new Func<bool>(() =>
                        ui.ConfirmSaveCurrentProfile(activeCarName, activeCarId)));
                }

                if (saveCurrent)
                {
                    SaveCurrentAircraftProfile(activeGameId, activeCarId);
                }
                else
                {
                    // User chose to discard - restore profile to its original state
                    DiscardProfileChanges();
                }
            }

            // Save settings
            this.SaveCommonSettings("GeneralSettings", Settings);

            StopGatewayAutoReconnect();
            StopXPlaneUdpReceiver();

            _buttonInputReader?.Dispose();
            _buttonInputReader = null;

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

        private void StartXPlaneUdpReceiver()
        {
            if (xplaneUdpThread != null || Settings == null || !Settings.XPlaneUdpEnabled)
            {
                return;
            }

            xplaneUdpCts = new CancellationTokenSource();
            try
            {
                xplaneUdpClient = new UdpClient(Settings.XPlaneUdpPort);
                xplaneUdpClient.Client.ReceiveTimeout = 500;
            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Error($"XPlane UDP receiver failed to start: {ex.Message}");
                StopXPlaneUdpReceiver();
                return;
            }

            xplaneUdpThread = new Thread(XPlaneUdpLoop)
            {
                IsBackground = true,
                Name = "XPlaneUdpReceiver"
            };
            xplaneUdpThread.Start();
        }

        private void StopXPlaneUdpReceiver()
        {
            if (xplaneUdpCts != null)
            {
                xplaneUdpCts.Cancel();
            }

            if (xplaneUdpClient != null)
            {
                try
                {
                    xplaneUdpClient.Close();
                }
                catch
                {
                }
            }

            xplaneUdpThread = null;
            xplaneUdpClient = null;
            xplaneUdpCts = null;
        }

        private void XPlaneUdpLoop()
        {
            if (xplaneUdpClient == null || xplaneUdpCts == null)
            {
                return;
            }

            var endpoint = new IPEndPoint(IPAddress.Any, 0);
            while (!xplaneUdpCts.IsCancellationRequested)
            {
                try
                {
                    byte[] data = xplaneUdpClient.Receive(ref endpoint);
                    if (data != null && data.Length >= XPlanePacketSizeBytes)
                    {
                        ParseXPlanePacket(data);
                    }
                }
                catch (SocketException ex)
                {
                    if (ex.SocketErrorCode != SocketError.TimedOut)
                    {
                        Thread.Sleep(50);
                    }
                }
                catch (ObjectDisposedException)
                {
                    break;
                }
                catch (Exception)
                {
                }
            }
        }

        private void ParseXPlanePacket(byte[] data)
        {
            int offset = 0;
            uint magic = ReadUInt32(data, ref offset);
            if (magic != XPlanePacketMagic)
            {
                return;
            }

            ushort version = ReadUInt16(data, ref offset);
            if (version != XPlanePacketVersion)
            {
                return;
            }

            ushort size = ReadUInt16(data, ref offset);
            if (size > data.Length || size < XPlanePacketSizeBytes)
            {
                return;
            }

            uint sequence = ReadUInt32(data, ref offset);

            var packet = new XPlaneUdpPacket
            {
                Sequence = sequence,
                IasKts = ReadSingle(data, ref offset),
                TasMps = ReadSingle(data, ref offset),
                AlphaDeg = ReadSingle(data, ref offset),
                BetaDeg = ReadSingle(data, ref offset),
                PRate = ReadSingle(data, ref offset),
                QRate = ReadSingle(data, ref offset),
                RRate = ReadSingle(data, ref offset),
                ElevDefDeg = ReadSingle(data, ref offset),
                AilDefDeg = ReadSingle(data, ref offset),
                RudDefDeg = ReadSingle(data, ref offset),
                ElevTrimNorm = ReadSingle(data, ref offset),
                AilTrimNorm = ReadSingle(data, ref offset),
                RudTrimNorm = ReadSingle(data, ref offset),
                GNrml = ReadSingle(data, ref offset),
                ReceivedUtc = DateTime.UtcNow
            };
            for (int idx = 0; idx < XPlaneMaxRotors; idx++)
            {
                packet.TorqueNm[idx] = ReadSingle(data, ref offset);
            }
            for (int idx = 0; idx < XPlaneMaxRotors; idx++)
            {
                packet.OmegaRad[idx] = ReadSingle(data, ref offset);
            }
            for (int idx = 0; idx < XPlaneMaxRotors; idx++)
            {
                packet.PropRatio[idx] = ReadSingle(data, ref offset);
            }
            packet.LAero = ReadSingle(data, ref offset);
            packet.MAero = ReadSingle(data, ref offset);
            packet.NAero = ReadSingle(data, ref offset);
            packet.OnGround = ReadByte(data, ref offset) != 0;
            offset += 3;

            lock (xplaneLock)
            {
                if (xplaneLastSequence != 0 && sequence > xplaneLastSequence + 1)
                {
                    xplaneDropouts += (int)(sequence - xplaneLastSequence - 1);
                }
                xplaneLastSequence = sequence;
                latestXPlanePacket = packet;
                xplaneLastReceivedUtc = packet.ReceivedUtc;
                UpdateRotorRpmHistory(packet);
            }
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Min(max, Math.Max(min, value));
        }

        private static float Clamp01(float value)
        {
            return Clamp(value, 0.0f, 1.0f);
        }

        private static float ClampLoad(float value, float limit)
        {
            if (limit <= 0.0f)
            {
                return value;
            }

            return Clamp(value, -limit, limit);
        }

        private static float Lerp(float start, float end, float t)
        {
            return start + (end - start) * t;
        }

        internal static float ToRpm(float omegaRad)
        {
            return omegaRad * 60.0f / (float)(2.0 * Math.PI);
        }

        private static bool IsTelemetryFresh(DateTime utc)
        {
            return (DateTime.UtcNow - utc).TotalMilliseconds <= XPlaneTelemetryFreshnessMs;
        }

        private void UpdateRotorRpmHistory(XPlaneUdpPacket packet)
        {
            if (packet.OnGround || !IsTelemetryFresh(packet.ReceivedUtc))
            {
                return;
            }

            DateTime cutoff = packet.ReceivedUtc - TimeSpan.FromSeconds(XPlaneRotorWindowSeconds);
            for (int idx = 0; idx < XPlaneMaxRotors; idx++)
            {
                Queue<RotorRpmSample> history = rotorRpmHistory[idx];
                if (history == null)
                {
                    history = new Queue<RotorRpmSample>();
                    rotorRpmHistory[idx] = history;
                }

                history.Enqueue(new RotorRpmSample
                {
                    Utc = packet.ReceivedUtc,
                    Rpm = ToRpm(packet.OmegaRad[idx])
                });

                while (history.Count > 0 && history.Peek().Utc < cutoff)
                {
                    history.Dequeue();
                }
            }
        }

        private static ushort ReadUInt16(byte[] data, ref int offset)
        {
            ushort value = BitConverter.ToUInt16(data, offset);
            offset += 2;
            return value;
        }

        private static uint ReadUInt32(byte[] data, ref int offset)
        {
            uint value = BitConverter.ToUInt32(data, offset);
            offset += 4;
            return value;
        }

        private static float ReadSingle(byte[] data, ref int offset)
        {
            float value = BitConverter.ToSingle(data, offset);
            offset += 4;
            return value;
        }

        private static byte ReadByte(byte[] data, ref int offset)
        {
            byte value = data[offset];
            offset += 1;
            return value;
        }

        private void ProcessGraphFfb()
        {
            if (ESPsync_serialPort == null || !ESPsync_serialPort.IsOpen)
            {
                return;
            }

            XPlaneUdpPacket packet;
            lock (xplaneLock)
            {
                if (latestXPlanePacket == null)
                {
                    return;
                }
                packet = latestXPlanePacket;
            }

            if (!IsTelemetryFresh(packet.ReceivedUtc))
            {
                return;
            }

            if ((DateTime.UtcNow - xplaneLastSendUtc).TotalMilliseconds < 20)
            {
                return;
            }
            xplaneLastSendUtc = DateTime.UtcNow;

            if (lastGraphEvaluation?.Outputs == null)
            {
                return;
            }

            SendGraphFfbForFunction(FunctionID.FlightStickPitch);
            SendGraphFfbForFunction(FunctionID.FlightStickRoll);
            SendGraphFfbForFunction(FunctionID.FlightPedals);
            SendGraphFfbForFunction(FunctionID.FlightStickCollective);
        }

        private int ResolveXPlaneRotorIndex(XPlaneUdpPacket packet)
        {
            if (Settings == null)
            {
                return 0;
            }
            if (Settings.XPlaneRotorIndex >= 0 && Settings.XPlaneRotorIndex < XPlaneMaxRotors)
            {
                return Settings.XPlaneRotorIndex;
            }

            if (packet.OnGround || !IsTelemetryFresh(packet.ReceivedUtc))
            {
                return hasAutoRotorIndex ? lastAutoRotorIndex : 0;
            }

            DateTime cutoff = packet.ReceivedUtc - TimeSpan.FromSeconds(XPlaneRotorWindowSeconds);
            int bestIndex = -1;
            float bestRpm = float.MaxValue;
            for (int idx = 0; idx < XPlaneMaxRotors; idx++)
            {
                Queue<RotorRpmSample> history = rotorRpmHistory[idx];
                if (history == null || history.Count == 0)
                {
                    continue;
                }

                float sum = 0.0f;
                int count = 0;
                foreach (var sample in history)
                {
                    if (sample.Utc < cutoff)
                    {
                        continue;
                    }
                    sum += sample.Rpm;
                    count++;
                }

                if (count == 0)
                {
                    continue;
                }

                float avgRpm = sum / count;
                if (avgRpm > 0.0f && avgRpm < bestRpm)
                {
                    bestRpm = avgRpm;
                    bestIndex = idx;
                }
            }

            if (bestIndex >= 0)
            {
                lastAutoRotorIndex = bestIndex;
                hasAutoRotorIndex = true;
                return bestIndex;
            }

            return hasAutoRotorIndex ? lastAutoRotorIndex : 0;
        }

        internal XPlaneUdpPacket GetLatestXPlanePacket()
        {
            lock (xplaneLock)
            {
                return latestXPlanePacket;
            }
        }

        internal int ResolveXPlaneRotorIndexForGraph(XPlaneUdpPacket packet)
        {
            return ResolveXPlaneRotorIndex(packet);
        }

        private void SendFlightFfb(FunctionID functionId, float kSpring, float kDamper, float kFriction, float trimOffset, float buffetAmp, float loadForce)
        {
            Message msg = new Message
            {
                FfbAction = new FFBAction
                {
                    FunctionId = functionId,
                    FlightFfb = new FlightFfbAction
                    {
                        KSpring = kSpring,
                        KDamper = kDamper,
                        KFriction = kFriction,
                        TrimOffset = trimOffset,
                        BuffetAmp = buffetAmp,
                        LoadForce = loadForce
                    }
                }
            };
            ESPsync_serialPort.WriteMessage(msg);
        }

        private void SendGraphFfbForFunction(FunctionID functionId)
        {
            if (IsFunctionOutputDisabled(functionId))
            {
                return;
            }

            if (!TryGetGraphFlightOutputs(functionId, out float spring, out float damper, out float friction, out float trim, out float buffet, out float load))
            {
                return;
            }

            SendFlightFfb(functionId, spring, damper, friction, trim, buffet, load);
        }

        private bool TryGetGraphFlightOutputs(FunctionID functionId, out float spring, out float damper, out float friction,
            out float trimOffset, out float buffetAmp, out float loadForce)
        {
            spring = 0.0f;
            damper = 0.0f;
            friction = 0.0f;
            trimOffset = 0.0f;
            buffetAmp = 0.0f;
            loadForce = 0.0f;

            string prefix = GetGraphFunctionPrefix(functionId);
            if (string.IsNullOrWhiteSpace(prefix))
            {
                return false;
            }

            bool hasOutput = false;
            if (TryGetGraphOutput($"{prefix}.SpringGain", out float value))
            {
                spring = value;
                hasOutput = true;
            }
            if (TryGetGraphOutput($"{prefix}.DamperGain", out value))
            {
                damper = value;
                hasOutput = true;
            }
            if (TryGetGraphOutput($"{prefix}.Friction", out value))
            {
                friction = value;
                hasOutput = true;
            }
            if (TryGetGraphOutput($"{prefix}.TrimOffset", out value))
            {
                trimOffset = value;
                hasOutput = true;
            }
            if (TryGetGraphOutput($"{prefix}.LoadForce", out value))
            {
                loadForce = value;
                hasOutput = true;
            }
            if (TryGetGraphOutput($"{prefix}.BuffetAmplitude", out value))
            {
                buffetAmp = value;
                hasOutput = true;
            }

            return hasOutput;
        }

        public void SetFunctionOutputDisabled(FunctionID functionId, bool disabled)
        {
            lock (outputDisableLock)
            {
                if (disabled)
                {
                    disabledOutputFunctions.Add(functionId);
                }
                else
                {
                    disabledOutputFunctions.Remove(functionId);
                }
            }
        }

        public bool IsFunctionOutputDisabled(FunctionID functionId)
        {
            lock (outputDisableLock)
            {
                return disabledOutputFunctions.Contains(functionId);
            }
        }

        private static string GetGraphFunctionPrefix(FunctionID functionId)
        {
            switch (functionId)
            {
                case FunctionID.FlightStickPitch:
                    return "FlightStickPitch";
                case FunctionID.FlightStickRoll:
                    return "FlightStickRoll";
                case FunctionID.FlightPedals:
                    return "FlightPedals";
                case FunctionID.FlightStickCollective:
                    return "FlightStickCollective";
                default:
                    return null;
            }
        }

        private bool TryGetGraphOutput(string key, out float value)
        {
            value = 0.0f;
            if (lastGraphEvaluation?.Outputs == null)
            {
                return false;
            }

            if (lastGraphEvaluation.Outputs.TryGetValue(key, out var raw))
            {
                value = (float)raw;
                return true;
            }

            return false;
        }

        public bool TryGetGraphTrimOffset(FunctionID functionId, out float trimMm)
        {
            trimMm = 0.0f;
            string prefix = GetGraphFunctionPrefix(functionId);
            if (string.IsNullOrWhiteSpace(prefix))
            {
                return false;
            }

            return TryGetGraphOutput($"{prefix}.TrimOffset", out trimMm);
        }

        public void ApplyXPlaneUdpSettings(bool enabled, int port)
        {
            if (Settings == null)
            {
                return;
            }

            Settings.XPlaneUdpEnabled = enabled;
            Settings.XPlaneUdpPort = port;
            StopXPlaneUdpReceiver();
            StartXPlaneUdpReceiver();
        }

        private DiyFfbPluginSettings.FunctionSettings GetFunctionSettings(FunctionID functionId)
        {
            if (Settings?.function_settings == null)
            {
                return null;
            }

            int index = (int)functionId - 1;
            if (index < 0 || index >= Settings.function_settings.Length)
            {
                return null;
            }

            return Settings.function_settings[index];
        }

        private static bool NearlyEqual(float a, float b)
        {
            return Math.Abs(a - b) < 0.000001f;
        }

        private void HandleGameChange(string gameId)
        {
            if (string.IsNullOrWhiteSpace(gameId))
            {
                return;
            }

            if (string.Equals(gameId, activeGameId, StringComparison.Ordinal))
            {
                return;
            }

            activeGameId = gameId;
            if (!string.IsNullOrWhiteSpace(activeCarId))
            {
                ResolveActiveGraph(activeGameId, activeCarId);
                BuildGraphParams();
            }
        }

        private static string GetGameIdSafe(GameData data)
        {
            if (data == null)
            {
                return "";
            }

            if (gameIdGetter == null)
            {
                var type = data.GetType();
                var property = type.GetProperty("GameName") ?? type.GetProperty("GameId") ?? type.GetProperty("Game");
                if (property != null && property.PropertyType == typeof(string))
                {
                    gameIdGetter = d => (string)property.GetValue(d, null);
                }
                else
                {
                    gameIdGetter = d => "";
                }
            }

            return (gameIdGetter(data) ?? "").Trim();
        }

        private static string BuildVehicleGraphKey(string gameId, string carId)
        {
            if (string.IsNullOrWhiteSpace(carId))
            {
                return "";
            }

            string trimmedGame = (gameId ?? "").Trim();
            return string.IsNullOrWhiteSpace(trimmedGame) ? carId : $"{trimmedGame}::{carId}";
        }

        /// <summary>
        /// Builds the key used for AircraftFfbProfiles dictionary.
        /// Uses same format as vehicle graph keys (gameId::carId) for consistency.
        /// </summary>
        private static string BuildProfileKey(string gameId, string carId)
        {
            return BuildVehicleGraphKey(gameId, carId);
        }

        /// <summary>
        /// Migrates an old-style profile key (carId only) to new format (gameId::carId).
        /// Call this when accessing a profile to ensure backward compatibility.
        /// </summary>
        private void MigrateProfileKeyIfNeeded(string gameId, string carId)
        {
            if (Settings?.AircraftFfbProfiles == null || string.IsNullOrWhiteSpace(carId))
            {
                return;
            }

            string newKey = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(newKey) || !newKey.Contains("::"))
            {
                // No game context, can't migrate
                return;
            }

            // If new key already exists, no migration needed
            if (Settings.AircraftFfbProfiles.ContainsKey(newKey))
            {
                return;
            }

            // Check if old-style key (carId only) exists
            if (Settings.AircraftFfbProfiles.TryGetValue(carId, out var oldProfile))
            {
                // Migrate: copy to new key and remove old key
                Settings.AircraftFfbProfiles[newKey] = oldProfile;
                Settings.AircraftFfbProfiles.Remove(carId);
                SimHub.Logging.Current.Info($"[DIY-FFB] Migrated profile key '{carId}' -> '{newKey}'");
            }
        }

        /// <summary>
        /// Migrates legacy VehicleGraphPaths entries into AircraftFfbProfiles.GraphPath.
        /// Called once at startup.
        /// </summary>
        private void MigrateVehicleGraphPaths()
        {
            if (Settings?.VehicleGraphPaths == null || Settings.VehicleGraphPaths.Count == 0)
            {
                return;
            }

            if (Settings.AircraftFfbProfiles == null)
            {
                Settings.AircraftFfbProfiles = new Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            }

            int migratedCount = 0;
            foreach (var kvp in Settings.VehicleGraphPaths)
            {
                string profileKey = kvp.Key;
                string graphPath = kvp.Value;

                if (string.IsNullOrWhiteSpace(profileKey) || string.IsNullOrWhiteSpace(graphPath))
                {
                    continue;
                }

                // Create or update profile with graph path
                if (!Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile))
                {
                    profile = new DiyFfbPluginSettings.AircraftFfbProfile();
                    Settings.AircraftFfbProfiles[profileKey] = profile;
                }

                // Only set if not already set (don't overwrite existing)
                if (string.IsNullOrWhiteSpace(profile.GraphPath))
                {
                    profile.GraphPath = graphPath;
                    migratedCount++;
                }
            }

            if (migratedCount > 0)
            {
                SimHub.Logging.Current.Info($"[DIY-FFB] Migrated {migratedCount} vehicle graph paths to profiles");
                // Clear old data after migration
                Settings.VehicleGraphPaths.Clear();
            }
        }

        private string ResolveGraphPath(string gameId, string carId)
        {
            if (Settings == null)
            {
                return "";
            }

            // Check vehicle profile
            string profileKey = BuildProfileKey(gameId, carId);
            if (!string.IsNullOrWhiteSpace(profileKey) &&
                Settings.AircraftFfbProfiles != null &&
                Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile) &&
                !string.IsNullOrWhiteSpace(profile?.GraphPath))
            {
                return profile.GraphPath;
            }

            return "";
        }

        public static string ResolveGraphFilePath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return "";
            }

            return Path.IsPathRooted(path)
                ? path
                : Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, path));
        }

        /// <summary>
        /// Gets the currently loaded vehicle graph definition.
        /// </summary>
        public GraphEditor.GraphDefinition GetActiveVehicleGraph()
        {
            return activeVehicleGraph;
        }

        private string PromptForGraphTemplate(string gameId, string carId)
        {
            if (ui == null)
            {
                return null;
            }

            // Show ProfileBrowserDialog in NewVehicle mode on UI thread
            var result = ui.Dispatcher.Invoke(new Func<(string graphPath, DiyFfbPluginSettings.AircraftFfbProfile profile, bool useTuning)?>(() =>
            {
                var dialog = new ProfileBrowser.ProfileBrowserDialog(this, ProfileBrowser.ProfileBrowserMode.NewVehicle, gameId, carId);

                var parentWindow = System.Windows.Window.GetWindow(ui);
                if (parentWindow != null)
                {
                    dialog.Owner = parentWindow;
                }

                if (dialog.ShowDialog() == true && dialog.SelectedEntry != null)
                {
                    var entry = dialog.SelectedEntry;
                    string selectedPath = entry.GraphPath;

                    // Resolve template path if needed
                    if (entry.Source == ProfileBrowser.ProfileEntrySource.Template && entry.TemplateEntry != null)
                    {
                        selectedPath = GraphEditor.GraphTemplateRegistry.ResolveTemplatePath(
                            entry.TemplateEntry.TemplatePath, AppDomain.CurrentDomain.BaseDirectory);
                    }

                    return (selectedPath, entry.Profile, dialog.UseTuning);
                }

                return null;
            }));

            if (result == null)
            {
                return null;
            }

            var (graphPath, profile, useTuning) = result.Value;

            if (string.IsNullOrWhiteSpace(graphPath))
            {
                return null;
            }

            // Store graph path in profile (no copying - reference directly)
            SetVehicleGraphPath(gameId, carId, graphPath);

            // Copy tuning parameters if requested
            if (useTuning && profile?.GraphParamValues != null)
            {
                string profileKey = BuildProfileKey(gameId, carId);
                if (Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var currentProfile))
                {
                    foreach (var kvp in profile.GraphParamValues)
                    {
                        currentProfile.GraphParamValues[kvp.Key] = kvp.Value;
                    }

                    // Rebuild graph params to reflect copied tuning values
                    BuildGraphParams();
                }
            }

            return graphPath;
        }

        private void ResolveActiveGraph(string gameId, string carId)
        {
            activeGraphKey = BuildVehicleGraphKey(gameId, carId);
            activeGraphPath = ResolveGraphPath(gameId, carId);
            activeVehicleGraph = null;
            activeGraphValidation = null;
            activeGraphRuntime = null;
            activeGraphEvaluator = null;
            activeGraphResolver = null;
            activeIncludeContextCache = null;
            lastGraphEvaluation = null;

            bool autoAssigned = false;
            if (string.IsNullOrWhiteSpace(activeGraphPath))
            {
                // Check if exactly one template matches this game — auto-assign without dialog
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                var templates = GraphEditor.GraphTemplateRegistry.GetTemplates(gameId, baseDir).ToList();
                if (templates.Count == 1)
                {
                    string templatePath = GraphEditor.GraphTemplateRegistry.ResolveTemplatePath(
                        templates[0].TemplatePath, baseDir);
                    if (!string.IsNullOrWhiteSpace(templatePath))
                    {
                        string key = BuildProfileKey(gameId, carId);
                        if (!string.IsNullOrWhiteSpace(key))
                        {
                            if (Settings.AircraftFfbProfiles == null)
                                Settings.AircraftFfbProfiles = new Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
                            if (!Settings.AircraftFfbProfiles.TryGetValue(key, out var profile))
                            {
                                profile = new DiyFfbPluginSettings.AircraftFfbProfile();
                                Settings.AircraftFfbProfiles[key] = profile;
                            }
                            profile.GraphPath = templatePath;
                        }
                        activeGraphPath = templatePath;
                        autoAssigned = true;
                        SimHub.Logging.Current.Info($"[Graph] Auto-assigned template '{templates[0].Name}' for {gameId}/{carId}");
                    }
                }

                if (string.IsNullOrWhiteSpace(activeGraphPath))
                {
                    // Multiple templates or auto-assign failed — prompt user
                    string templatePath = PromptForGraphTemplate(gameId, carId);
                    if (!string.IsNullOrWhiteSpace(templatePath))
                        activeGraphPath = templatePath;
                    else
                        return;
                }
            }

            string resolvedPath = ResolveGraphFilePath(activeGraphPath);
            if (!File.Exists(resolvedPath))
            {
                activeGraphValidation = new GraphValidationResult();
                activeGraphValidation.Errors.Add($"Graph file not found: {resolvedPath}");
                return;
            }

            try
            {
                string json = File.ReadAllText(resolvedPath);
                activeVehicleGraph = GraphSerializer.Deserialize(json, out activeGraphValidation);

                if (activeVehicleGraph != null && activeGraphValidation != null && activeGraphValidation.IsValid)
                {
                    string baseDir = Path.GetDirectoryName(resolvedPath) ?? AppDomain.CurrentDomain.BaseDirectory;

                    // Populate Include node ports from their included graphs (v3 schema doesn't serialize them)
                    GraphSerializer.PopulateIncludePorts(activeVehicleGraph, baseDir);

                    activeGraphRuntime = GraphRuntimeConverter.Convert(activeVehicleGraph);
                    activeGraphResolver = GraphRuntimeConverter.CreateResolver(baseDir);
                    activeIncludeContextCache = new DiyFfb.GraphTest.IncludeContextCache();
                    activeGraphEvaluator = new DiyFfb.GraphTest.GraphCompiledEvaluator(
                        activeGraphRuntime, activeGraphResolver, activeIncludeContextCache, baseDir);

                    // Notify UI that graph has changed
                    ActiveGraphChanged?.Invoke(this, EventArgs.Empty);

                    // Check for param migration needs
                    CheckParamMigration(resolvedPath, gameId, carId);

                    // Seed default active functions for auto-assigned templates
                    if (autoAssigned)
                        _configOrchestrator.SeedDefaultActiveFunctionIds(gameId, carId);
                }
            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Error($"[Graph] Graph load exception: {ex.Message}", ex);
                activeGraphValidation = new GraphValidationResult();
                activeGraphValidation.Errors.Add($"Graph load failed: {ex.Message}");
            }
        }

        private void CheckParamMigration(string resolvedPath, string gameId, string carId)
        {
            if (activeVehicleGraph == null)
            {
                return;
            }

            var profile = GetCurrentAircraftProfile();
            if (profile == null)
            {
                return;
            }

            // Compute current hash
            string currentHash = GraphHashComputer.ComputeGraphTreeHash(resolvedPath, activeVehicleGraph);
            if (string.IsNullOrWhiteSpace(currentHash))
            {
                return;
            }

            // First-time setup: initialize hash without triggering notification
            if (string.IsNullOrWhiteSpace(profile.LastReviewedGraphHash))
            {
                InitializeParamSnapshots(profile, currentHash);
                return;
            }

            // Compare to stored hash
            if (currentHash == profile.LastReviewedGraphHash)
            {
                return;
            }

            // Hash changed - run migration
            var result = MigrateParamOverrides(profile, currentHash);
            if (result.HasChangesToReview)
            {
                ParamMigrationDetected?.Invoke(this, result);
            }
            else
            {
                // No notable changes, just update hash silently
                profile.LastReviewedGraphHash = currentHash;
                profile.LastReviewedParamSnapshots = result.CurrentSnapshots;
            }
        }

        private void InitializeParamSnapshots(DiyFfbPluginSettings.AircraftFfbProfile profile, string hash)
        {
            var allParams = CollectAllGraphParams(activeVehicleGraph, activeGraphResolver);
            var paramInfos = allParams.Select(p => new ParamMigrationHelper.ParamInfo
            {
                Name = p.Name,
                DefaultValue = p.DefaultValue,
                Min = p.Min,
                Max = p.Max
            });

            profile.LastReviewedGraphHash = hash;
            profile.LastReviewedParamSnapshots = ParamMigrationHelper.CreateSnapshots(paramInfos);
        }

        private ParamMigrationResult MigrateParamOverrides(
            DiyFfbPluginSettings.AircraftFfbProfile profile,
            string newHash)
        {
            var allParams = CollectAllGraphParams(activeVehicleGraph, activeGraphResolver);
            var paramInfos = allParams.Select(p => new ParamMigrationHelper.ParamInfo
            {
                Name = p.Name,
                DefaultValue = p.DefaultValue,
                Min = p.Min,
                Max = p.Max
            });

            return ParamMigrationHelper.Migrate(
                paramInfos,
                profile.LastReviewedParamSnapshots,
                profile.GraphParamValues,
                newHash);
        }

        private void EvaluateActiveGraph(GameData data)
        {
            if (activeGraphEvaluator == null)
            {
                return;
            }

            try
            {
                BuildGraphInputs(data);
                BuildGraphParams();
                // Clear context cache before top-level evaluation so include contexts are fresh
                activeIncludeContextCache?.Clear();
                long now = System.Diagnostics.Stopwatch.GetTimestamp();
                double dt = _lastGraphEvalTicks > 0
                    ? (double)(now - _lastGraphEvalTicks) / System.Diagnostics.Stopwatch.Frequency
                    : 0.0;
                _lastGraphEvalTicks = now;
                lastGraphEvaluation = activeGraphEvaluator.EvaluateWithTrace(graphInputs, graphParams, dt);
            }
            catch
            {
                // Ignore evaluation errors to keep runtime stable; outputs remain from the last successful evaluation.
            }
        }

        /// <summary>
        /// Resets persistent state in the active graph evaluator (accumulators, sample-holds, etc.).
        /// Call on profile or vehicle switch so trim offsets don't carry over.
        /// </summary>
        internal void ResetGraphState()
        {
            activeGraphEvaluator?.ResetState();
        }

        // --- Axis position tracking for graph inputs ---
        private readonly Dictionary<AxisID, float> _lastAxisPositions = new Dictionary<AxisID, float>();

        /// <summary>
        /// Called from UI layer when AxisState message is received.
        /// Caches position for use as graph input.
        /// </summary>
        internal void UpdateAxisPosition(AxisID axisId, float position)
        {
            _lastAxisPositions[axisId] = position;
        }

        /// <summary>
        /// Returns the last known position (mm) for the given axis, or 0 if unknown.
        /// </summary>
        internal double GetLastAxisPosition(AxisID axisId)
        {
            return _lastAxisPositions.TryGetValue(axisId, out var pos) ? pos : 0.0;
        }

        /// <summary>
        /// Returns the last known position (mm) for the axis linked to the given function.
        /// Resolves function → primary linked axis → cached AxisState position.
        /// </summary>
        internal double GetFunctionPosition(FunctionID functionId)
        {
            var config = _functionConfigManager.GetCurrentConfig((int)functionId);
            if (config?.Base == null || config.Base.LinkedAxes.Count == 0)
                return 0.0;
            var axisId = config.Base.LinkedAxes[0];
            if (axisId == AxisID.AxisUndefined)
                return 0.0;
            return GetLastAxisPosition(axisId);
        }

        private void BuildGraphInputs(GameData data)
        {
            graphInputs.Clear();
            GraphSignalCatalog.BuildXPlaneInputs(this, data, graphInputs);
            _buttonInputReader?.SetActiveBindings(Settings?.GripButtonBindings);
            try { _buttonInputReader?.Poll(); } catch { }
            GraphSignalCatalog.BuildGripInputs(_buttonInputReader, Settings?.GripButtonBindings, graphInputs);
            GraphSignalCatalog.BuildAxisInputs(this, graphInputs);
        }

        internal Dictionary<string, double> GetLiveGraphInputs()
        {
            var inputs = new Dictionary<string, double>();
            GraphSignalCatalog.BuildXPlaneInputs(this, null, inputs);
            // Grip/Axis inputs: only include cached values, no COM calls.
            // These may be zero if no bindings are configured — that's fine.
            try
            {
                GraphSignalCatalog.BuildGripInputs(_buttonInputReader, Settings?.GripButtonBindings, inputs);
                GraphSignalCatalog.BuildAxisInputs(this, inputs);
            }
            catch { }
            return inputs;
        }

        private void BuildGraphParams()
        {
            graphParams.Clear();
            if (activeVehicleGraph == null)
            {
                return;
            }

            // Collect all params from graph + includes (includes provide defaults)
            var allParams = CollectAllGraphParams(activeVehicleGraph, activeGraphResolver);

            foreach (var param in allParams)
            {
                graphParams[param.Name] = ResolveParamValue(param.Name, param.DefaultValue);
            }
        }

        /// <summary>
        /// Resolves a param value using three-tier resolution:
        /// Tier 1: defaultValue (from param definition)
        /// Tier 2: Graph-level template defaults (from graph JSON ParamValues, read-only at runtime)
        /// Tier 3: Vehicle profile override (GraphParamValues, user edits go here)
        /// Higher tiers take precedence. User edits only write to Tier 3.
        /// </summary>
        private double ResolveParamValue(string paramName, double defaultValue)
        {
            double value = defaultValue;

            // Tier 2: Graph-level override
            if (activeVehicleGraph?.ParamValues != null
                && activeVehicleGraph.ParamValues.TryGetValue(paramName, out var graphOverride))
            {
                value = graphOverride;
            }

            // Tier 3: Vehicle profile override
            var profile = GetCurrentAircraftProfile();
            if (profile?.GraphParamValues != null
                && profile.GraphParamValues.TryGetValue(paramName, out var profileOverride))
            {
                value = profileOverride;
            }

            return value;
        }

        private List<GraphParam> CollectAllGraphParams(
            GraphEditor.GraphDefinition graph,
            DiyFfb.GraphTest.GraphIncludeResolver resolver)
        {
            string baseDir = GetActiveGraphBaseDirectory();
            return CollectAllGraphParams(graph, resolver, baseDir);
        }

        private List<GraphParam> CollectAllGraphParams(
            GraphEditor.GraphDefinition graph,
            DiyFfb.GraphTest.GraphIncludeResolver resolver,
            string baseDir)
        {
            var result = new Dictionary<string, GraphEditor.GraphParam>();

            if (graph == null)
            {
                return new List<GraphEditor.GraphParam>();
            }

            // First, recursively collect params from includes (these are defaults)
            if (graph.Nodes != null)
            {
                foreach (var node in graph.Nodes)
                {
                    if (node.Kind == GraphEditor.GraphNodeKind.Include
                        && !string.IsNullOrWhiteSpace(node.IncludePath))
                    {
                        string resolvedPath = ResolveIncludePath(baseDir, node.IncludePath);
                        var includedGraph = LoadIncludeGraphFromPath(resolvedPath);
                        if (includedGraph != null)
                        {
                            string includeDir = Path.GetDirectoryName(resolvedPath) ?? baseDir;
                            var includeParams = CollectAllGraphParams(includedGraph, resolver, includeDir);
                            foreach (var param in includeParams)
                            {
                                // Add if not already present (parent can override)
                                if (!result.ContainsKey(param.Name))
                                {
                                    result[param.Name] = param;
                                }
                            }
                        }
                    }
                }
            }

            // Then collect params from this graph (these override include defaults)
            foreach (var param in graph.Params.Values)
            {
                result[param.Name] = param;
            }

            return result.Values.ToList();
        }

        private GraphEditor.GraphDefinition LoadIncludeGraphFromPath(string resolvedPath)
        {
            if (string.IsNullOrWhiteSpace(resolvedPath) || !File.Exists(resolvedPath))
            {
                return null;
            }

            try
            {
                string json = File.ReadAllText(resolvedPath);
                return GraphEditor.GraphSerializer.Deserialize(json, out _);
            }
            catch
            {
                return null;
            }
        }

        public DiyFfbPluginSettings.AircraftFfbProfile GetCurrentAircraftProfile()
        {
            if (Settings?.AircraftFfbProfiles == null || string.IsNullOrWhiteSpace(activeCarId))
            {
                return null;
            }

            string profileKey = BuildProfileKey(activeGameId, activeCarId);
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                return null;
            }

            if (Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile))
            {
                return profile;
            }

            return null;
        }

        private void HandleAircraftChange(GameData data, string gameId)
        {
            string carId = data.NewData?.CarId;
            if (string.IsNullOrWhiteSpace(carId))
            {
                return;
            }

            if (string.Equals(carId, activeCarId, StringComparison.Ordinal))
            {
                return;
            }

            if (!string.IsNullOrWhiteSpace(activeCarId) && HasUnsavedProfileChanges(activeGameId, activeCarId))
            {
                bool saveCurrent = false;
                if (ui != null)
                {
                    saveCurrent = (bool)ui.Dispatcher.Invoke(new Func<bool>(() =>
                        ui.ConfirmSaveCurrentProfile(activeCarName, activeCarId)));
                }

                if (saveCurrent)
                {
                    SaveCurrentAircraftProfile(activeGameId, activeCarId);
                }
                else
                {
                    // User chose to discard - restore profile to its original state
                    DiscardProfileChanges();
                }
            }

            bool applyPending = false;
            if (hasPendingFfbProfile && pendingFfbProfile != null)
            {
                if (ui != null)
                {
                    applyPending = (bool)ui.Dispatcher.Invoke(new Func<bool>(() =>
                        ui.ConfirmApplyPendingProfile(activeCarName, carId)));
                }

                if (applyPending)
                {
                    string profileKey = BuildProfileKey(gameId, carId);
                    if (!string.IsNullOrWhiteSpace(profileKey))
                    {
                        if (Settings.AircraftFfbProfiles == null)
                        {
                            Settings.AircraftFfbProfiles = new System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
                        }
                        Settings.AircraftFfbProfiles[profileKey] = pendingFfbProfile;
                    }
                }

                pendingFfbProfile = null;
                hasPendingFfbProfile = false;
            }

            // Set activeCarId before applying profile so IsFunctionActive()
            // checks the NEW profile during config-changed event handling.
            activeCarId = carId;
            activeCarName = !string.IsNullOrWhiteSpace(data.NewData?.CarModel) ? data.NewData.CarModel : carId;

            // Migrate old-style key before resolving graph so ResolveGraphPath
            // finds the profile under the new key format.
            MigrateProfileKeyIfNeeded(gameId, carId);

            // Resolve graph first — auto-assign creates the profile and
            // SeedDefaultActiveFunctionIds populates ActiveFunctionIds.
            // ApplyAircraftProfile must run after so it sees populated IDs.
            ResolveActiveGraph(gameId, carId);
            ApplyAircraftProfile(gameId, carId);
            BuildGraphParams();

            // Fire ContextChanged event for badge/UI refresh
            _configOrchestrator.OnContextChanged();

            if (ui != null)
            {
                string carName = activeCarName;
                string carIdLabel = activeCarId;
                string gameIdCapture = gameId;
                ui.Dispatcher.BeginInvoke(new Action(() =>
                {
                    ui.RefreshGraphSelection();
                    ui.UpdateActiveAircraftLabel(carName, carIdLabel, gameIdCapture);
                    ui.RefreshVehicleParams();
                    ui.RefreshFunctionSelection();
                }));
            }
        }

        private void SaveCurrentAircraftProfile(string gameId, string carId)
        {
            if (Settings == null || string.IsNullOrWhiteSpace(carId))
            {
                return;
            }

            string profileKey = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                return;
            }

            if (Settings.AircraftFfbProfiles == null)
            {
                Settings.AircraftFfbProfiles = new System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            }

            Settings.AircraftFfbProfiles[profileKey] = BuildCurrentAircraftProfile();
            ClearDirtyState();
        }

        /// <summary>
        /// Explicitly stores the current vehicle's profile settings.
        /// </summary>
        /// <returns>True if profile was stored, false if no active vehicle.</returns>
        public bool StoreCurrentProfile()
        {
            if (string.IsNullOrWhiteSpace(activeCarId))
            {
                return false;
            }

            SaveCurrentAircraftProfile(activeGameId, activeCarId);
            return true;
        }

        private void ApplyAircraftProfile(string gameId, string carId)
        {
            if (Settings == null || string.IsNullOrWhiteSpace(carId))
            {
                return;
            }

            // Migrate old-style key if needed
            MigrateProfileKeyIfNeeded(gameId, carId);

            string profileKey = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                ApplyFfbProfileToCurrentSettings(new DiyFfbPluginSettings.AircraftFfbProfile());
                return;
            }

            if (Settings.AircraftFfbProfiles == null)
            {
                Settings.AircraftFfbProfiles = new System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            }

            if (Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile))
            {
                Settings.XPlaneRotorIndex = profile.XPlaneRotorIndex;
                _configOrchestrator.ApplyProfileFunctionOverrides(profile);
            }
            else
            {
                // No profile - clear any active overrides
                _configOrchestrator.ApplyProfileFunctionOverrides(null);
            }
        }

        private DiyFfbPluginSettings.AircraftFfbProfile BuildCurrentAircraftProfile()
        {
            var profile = new DiyFfbPluginSettings.AircraftFfbProfile();
            profile.XPlaneRotorIndex = Settings.XPlaneRotorIndex;

            // Copy all fields from current stored profile
            var currentProfile = GetCurrentAircraftProfile();
            if (currentProfile != null)
            {
                profile.GraphPath = currentProfile.GraphPath;
                if (currentProfile.GraphParamValues != null)
                    profile.GraphParamValues = new Dictionary<string, double>(currentProfile.GraphParamValues);
                profile.LastReviewedGraphHash = currentProfile.LastReviewedGraphHash;
                if (currentProfile.LastReviewedParamSnapshots != null)
                    profile.LastReviewedParamSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>(currentProfile.LastReviewedParamSnapshots);
                if (currentProfile.FunctionOverrides != null)
                    profile.FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>(currentProfile.FunctionOverrides);
                if (currentProfile.ActiveFunctionIds != null)
                    profile.ActiveFunctionIds = new HashSet<int>(currentProfile.ActiveFunctionIds);
            }

            return profile;
        }

        private bool HasUnsavedProfileChanges(string gameId, string carId)
        {
            if (Settings == null || string.IsNullOrWhiteSpace(carId))
            {
                return false;
            }

            // Graph param changes are tracked separately since they modify the profile in-place
            if (hasDirtyGraphParams)
            {
                return true;
            }

            string profileKey = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                return false;
            }

            var current = BuildCurrentAircraftProfile();
            if (Settings.AircraftFfbProfiles != null &&
                Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var stored))
            {
                return !AreProfilesEqual(current, stored);
            }

            return !AreProfilesEqual(current, new DiyFfbPluginSettings.AircraftFfbProfile());
        }

        private bool AreProfilesEqual(DiyFfbPluginSettings.AircraftFfbProfile left, DiyFfbPluginSettings.AircraftFfbProfile right)
        {
            if (left == null || right == null)
            {
                return left == right;
            }

            return string.Equals(left.GraphPath, right.GraphPath, System.StringComparison.OrdinalIgnoreCase) &&
                   left.XPlaneRotorIndex == right.XPlaneRotorIndex &&
                   AreGraphParamValuesEqual(left.GraphParamValues, right.GraphParamValues) &&
                   AreFunctionOverridesEqual(left.FunctionOverrides, right.FunctionOverrides) &&
                   AreActiveFunctionIdsEqual(left.ActiveFunctionIds, right.ActiveFunctionIds);
        }

        private bool AreGraphParamValuesEqual(Dictionary<string, double> left, Dictionary<string, double> right)
        {
            if (left == null && right == null) return true;
            if (left == null || right == null) return false;
            if (left.Count != right.Count) return false;

            foreach (var kvp in left)
            {
                if (!right.TryGetValue(kvp.Key, out var rightValue))
                {
                    return false;
                }
                if (!NearlyEqual((float)kvp.Value, (float)rightValue))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool AreFunctionOverridesEqual(
            Dictionary<int, FunctionConfigOverrides> left,
            Dictionary<int, FunctionConfigOverrides> right)
        {
            // Collect non-empty entries from each side (null/empty dict treated as equivalent)
            var leftEffective = new Dictionary<int, FunctionConfigOverrides>();
            var rightEffective = new Dictionary<int, FunctionConfigOverrides>();

            if (left != null)
            {
                foreach (var kvp in left)
                {
                    if (kvp.Value != null && !kvp.Value.IsEmpty)
                        leftEffective[kvp.Key] = kvp.Value;
                }
            }

            if (right != null)
            {
                foreach (var kvp in right)
                {
                    if (kvp.Value != null && !kvp.Value.IsEmpty)
                        rightEffective[kvp.Key] = kvp.Value;
                }
            }

            if (leftEffective.Count != rightEffective.Count) return false;

            foreach (var kvp in leftEffective)
            {
                if (!rightEffective.TryGetValue(kvp.Key, out var rightVal))
                    return false;
                if (!ConfigComparer.AreEqual(kvp.Value, rightVal))
                    return false;
            }

            return true;
        }

        private static bool AreActiveFunctionIdsEqual(HashSet<int> left, HashSet<int> right)
        {
            bool leftEmpty = left == null || left.Count == 0;
            bool rightEmpty = right == null || right.Count == 0;

            if (leftEmpty && rightEmpty) return true;
            if (leftEmpty || rightEmpty) return false;

            return left.SetEquals(right);
        }

        public string GetActiveCarId()
        {
            return activeCarId;
        }

        public string GetActiveCarName()
        {
            return activeCarName;
        }

        public string GetActiveGameId()
        {
            return activeGameId;
        }

        /// <summary>
        /// Resets the current vehicle's GraphParamValues to defaults (clears all overrides).
        /// Returns true if reset was performed, false if no profile exists.
        /// </summary>
        public bool ResetCurrentProfileToDefaults()
        {
            var profile = GetCurrentAircraftProfile();
            if (profile == null)
            {
                return false;
            }

            // Clear all param overrides
            profile.GraphParamValues?.Clear();

            // Clear dirty flag and snapshot since we're intentionally resetting
            ClearDirtyState();

            // Rebuild params from graph defaults (Tier 1) and graph template (Tier 2)
            BuildGraphParams();

            // Notify listeners so UI can refresh
            ActiveGraphChanged?.Invoke(this, EventArgs.Empty);

            return true;
        }

        /// <summary>
        /// Gets the profile key for the current active vehicle (gameId::carId format).
        /// </summary>
        public string GetActiveProfileKey()
        {
            return BuildProfileKey(activeGameId, activeCarId);
        }

        public string GetVehicleGraphPath(string gameId, string carId)
        {
            if (Settings?.AircraftFfbProfiles == null)
            {
                return "";
            }

            string key = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(key))
            {
                return "";
            }

            return Settings.AircraftFfbProfiles.TryGetValue(key, out var profile) ? profile?.GraphPath ?? "" : "";
        }

        public string GetActiveGraphStatus()
        {
            if (string.IsNullOrWhiteSpace(activeGraphPath))
            {
                return "(none)";
            }

            if (activeGraphValidation == null)
            {
                return $"{activeGraphPath}";
            }

            if (activeGraphValidation.IsValid)
            {
                return $"{activeGraphPath}";
            }

            string error = activeGraphValidation.Errors.Count > 0
                ? activeGraphValidation.Errors[0]
                : "Invalid graph.";
            return $"{activeGraphPath} ({error})";
        }

        public string GetActiveGraphPath()
        {
            if (string.IsNullOrWhiteSpace(activeGraphPath))
                return "";

            // Return resolved absolute path to avoid path resolution inconsistencies
            return ResolveGraphFilePath(activeGraphPath);
        }

        /// <summary>
        /// Gets the graph path for a profile key (gameId::carId format).
        /// </summary>
        public string GetGraphPathForProfileKey(string profileKey)
        {
            if (string.IsNullOrWhiteSpace(profileKey))
                return "";

            // Profile key format: "gameId::carId"
            int sep = profileKey.IndexOf("::");
            if (sep > 0 && sep < profileKey.Length - 2)
            {
                string gameId = profileKey.Substring(0, sep);
                string carId = profileKey.Substring(sep + 2);
                return GetVehicleGraphPath(gameId, carId);
            }

            // Legacy format: carId only
            return GetVehicleGraphPath("", profileKey);
        }

        /// <summary>
        /// Applies a profile selected from the Profile Browser to the current vehicle.
        /// </summary>
        /// <param name="graphPath">The graph path to use.</param>
        /// <param name="profile">The profile to copy (can be null for templates).</param>
        /// <param name="useTuning">If true, copies GraphParamValues from the source profile.</param>
        public void ApplyProfileFromBrowser(string graphPath, DiyFfbPluginSettings.AircraftFfbProfile profile, bool useTuning)
        {
            if (string.IsNullOrWhiteSpace(activeCarId))
                return;

            // 1. Set graph path for current vehicle if provided
            if (!string.IsNullOrWhiteSpace(graphPath))
            {
                SetVehicleGraphPath(activeGameId, activeCarId, graphPath);
            }

            // 2. Copy tuning params if requested and source has a profile
            if (useTuning && profile?.GraphParamValues != null && profile.GraphParamValues.Count > 0)
            {
                var currentProfile = GetCurrentAircraftProfile();
                if (currentProfile == null)
                {
                    currentProfile = new DiyFfbPluginSettings.AircraftFfbProfile();
                    string currentKey = BuildProfileKey(activeGameId, activeCarId);
                    if (Settings.AircraftFfbProfiles == null)
                        Settings.AircraftFfbProfiles = new Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
                    Settings.AircraftFfbProfiles[currentKey] = currentProfile;
                }

                // Copy param values
                if (currentProfile.GraphParamValues == null)
                    currentProfile.GraphParamValues = new Dictionary<string, double>();

                foreach (var kvp in profile.GraphParamValues)
                {
                    currentProfile.GraphParamValues[kvp.Key] = kvp.Value;
                }

                // Copy other settings
                currentProfile.XPlaneRotorIndex = profile.XPlaneRotorIndex;
            }

            // 3. Reload graph for current vehicle
            ResolveActiveGraph(activeGameId, activeCarId);
            BuildGraphParams();

            // 4. Notify UI
            ActiveGraphChanged?.Invoke(this, EventArgs.Empty);
            ui?.Dispatcher?.BeginInvoke(new Action(() =>
            {
                ui.RefreshGraphSelection();
            }));
        }

        /// <summary>
        /// Gets the include context cache for the active graph.
        /// Used by the graph editor to show sub-graph previews with parent context.
        /// </summary>
        public DiyFfb.GraphTest.IncludeContextCache ActiveIncludeContextCache => activeIncludeContextCache;

        public IReadOnlyDictionary<string, GraphParam> GetActiveGraphParams()
        {
            if (activeVehicleGraph == null)
            {
                return new Dictionary<string, GraphParam>();
            }

            // Collect all params from main graph and includes
            var allParams = CollectAllGraphParams(activeVehicleGraph, activeGraphResolver);
            var result = new Dictionary<string, GraphParam>();
            foreach (var param in allParams)
            {
                result[param.Name] = param;
            }
            return result;
        }

        public IReadOnlyList<string> GetActiveGraphParamOrder()
        {
            if (activeVehicleGraph == null)
            {
                return Array.Empty<string>();
            }

            string baseDir = GetActiveGraphBaseDirectory();
            var ordered = new List<string>();
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            AppendOrderedGraphParams(activeVehicleGraph, baseDir, ordered, seen, visited);

            return ordered;
        }

        private void AppendOrderedGraphParams(GraphDefinition graph, string baseDir, List<string> ordered,
            HashSet<string> seen, HashSet<string> visited)
        {
            if (graph?.Nodes == null || graph.Nodes.Count == 0)
            {
                return;
            }

            var orderedNodes = graph.Nodes
                .OrderBy(n => n.Y)
                .ThenBy(n => n.X)
                .ThenBy(n => n.Id, StringComparer.OrdinalIgnoreCase);

            foreach (var node in orderedNodes)
            {
                if (node.Kind == GraphNodeKind.Param)
                {
                    foreach (var port in node.Ports.Where(p => p.Kind == GraphPortKind.Output))
                    {
                        string name = BuildParamName(node, port);
                        if (string.IsNullOrWhiteSpace(name) || seen.Contains(name))
                        {
                            continue;
                        }

                        seen.Add(name);
                        ordered.Add(name);
                    }
                }
                else if (node.Kind == GraphNodeKind.Include)
                {
                    string resolvedPath = ResolveIncludePath(baseDir, node.IncludePath);
                    if (string.IsNullOrEmpty(resolvedPath) || !File.Exists(resolvedPath))
                    {
                        continue;
                    }

                    if (!visited.Add(resolvedPath))
                    {
                        continue;
                    }

                    var includedGraph = TryLoadEditorGraph(resolvedPath);
                    if (includedGraph == null)
                    {
                        continue;
                    }

                    string includeDir = Path.GetDirectoryName(resolvedPath) ?? baseDir;
                    AppendOrderedGraphParams(includedGraph, includeDir, ordered, seen, visited);
                }
            }
        }

        private string GetActiveGraphBaseDirectory()
        {
            if (string.IsNullOrWhiteSpace(activeGraphPath))
            {
                return AppDomain.CurrentDomain.BaseDirectory;
            }

            string resolvedPath = ResolveGraphFilePath(activeGraphPath);
            string baseDir = Path.GetDirectoryName(resolvedPath);
            if (string.IsNullOrWhiteSpace(baseDir))
            {
                return AppDomain.CurrentDomain.BaseDirectory;
            }
            return baseDir;
        }

        private static string ResolveIncludePath(string baseDir, string includePath)
        {
            if (string.IsNullOrWhiteSpace(includePath))
            {
                return null;
            }

            if (Path.IsPathRooted(includePath))
            {
                return Path.GetFullPath(includePath);
            }

            if (!string.IsNullOrWhiteSpace(baseDir))
            {
                return Path.GetFullPath(Path.Combine(baseDir, includePath));
            }

            return includePath;
        }

        private static GraphDefinition TryLoadEditorGraph(string resolvedPath)
        {
            try
            {
                string json = File.ReadAllText(resolvedPath);
                var graph = GraphSerializer.Deserialize(json, out var validation);
                if (graph != null && validation != null && validation.IsValid)
                {
                    return graph;
                }
            }
            catch
            {
                return null;
            }

            return null;
        }

        private static string BuildParamName(GraphNode node, GraphPort port)
        {
            if (!string.IsNullOrEmpty(node.SignalGroup) && !string.IsNullOrEmpty(port.SignalSuffix))
            {
                return node.SignalGroup + "." + port.SignalSuffix;
            }
            return port.Name ?? "";
        }

        /// <summary>
        /// Detects the category of the active graph based on Output node SignalGroups.
        /// Priority: Collective (helicopter) > FlightStick/FlightPedals (aircraft) > Vehicle (default)
        /// </summary>
        public GraphCategory GetActiveGraphCategory()
        {
            if (activeVehicleGraph == null || activeVehicleGraph.Nodes == null)
            {
                return GraphCategory.Vehicle;
            }

            bool hasFlightOutput = false;

            foreach (var node in activeVehicleGraph.Nodes)
            {
                if (node.Kind != GraphNodeKind.Output || string.IsNullOrEmpty(node.SignalGroup))
                    continue;

                // Collective gets priority -> helicopter icon
                if (node.SignalGroup.StartsWith("FlightStickCollective", StringComparison.OrdinalIgnoreCase))
                {
                    return GraphCategory.Helicopter;
                }

                // Track if we have any flight-related outputs
                if (node.SignalGroup.StartsWith("FlightStick", StringComparison.OrdinalIgnoreCase) ||
                    node.SignalGroup.StartsWith("FlightPedals", StringComparison.OrdinalIgnoreCase))
                {
                    hasFlightOutput = true;
                }
            }

            return hasFlightOutput ? GraphCategory.Aircraft : GraphCategory.Vehicle;
        }

        public double GetGraphParamValue(string paramName)
        {
            if (graphParams.TryGetValue(paramName, out var value))
            {
                return value;
            }

            // Fallback: do three-tier resolution for params not yet in graphParams
            // Use GetActiveGraphParams() to find the default (reuses CollectAllGraphParams)
            var allParams = GetActiveGraphParams();
            double defaultValue = allParams.TryGetValue(paramName, out var param) ? param.DefaultValue : 0.0;

            return ResolveParamValue(paramName, defaultValue);
        }

        public void SetGraphParamValue(string paramName, double value)
        {
            // Update runtime param for immediate effect
            graphParams[paramName] = value;

            // Note: We intentionally do NOT write to activeVehicleGraph.ParamValues (Tier 2).
            // Tier 2 contains template defaults from the graph JSON file and is read-only at runtime.
            // All user edits go to Tier 3 (profile.GraphParamValues) which takes precedence.

            // Snapshot BEFORE modifying profile (so we can restore on discard)
            MarkProfileDirty();

            // Save to current aircraft profile (Tier 3, persists across sessions)
            var profile = GetCurrentAircraftProfile();
            if (profile != null)
            {
                if (profile.GraphParamValues == null)
                {
                    profile.GraphParamValues = new Dictionary<string, double>();
                }
                profile.GraphParamValues[paramName] = value;
            }

            // Notify listeners of parameter change
            GraphParamChanged?.Invoke(this, new GraphParamChangedEventArgs(paramName, value));
        }

        /// <summary>
        /// Resets a graph parameter to its default value by removing the vehicle override.
        /// </summary>
        /// <param name="paramName">Name of the parameter to reset.</param>
        public void ResetGraphParamValue(string paramName)
        {
            // Snapshot BEFORE modifying profile (so we can restore on discard)
            MarkProfileDirty();

            // Remove from profile (Tier 3)
            var profile = GetCurrentAircraftProfile();
            if (profile?.GraphParamValues != null)
            {
                profile.GraphParamValues.Remove(paramName);
            }

            // Get the default value (from graph Tier 2 or param definition)
            var allParams = GetActiveGraphParams();
            double defaultValue = allParams.TryGetValue(paramName, out var param) ? param.DefaultValue : 0.0;
            double effectiveValue = ResolveParamValue(paramName, defaultValue);

            // Update runtime param
            graphParams[paramName] = effectiveValue;

            // Notify listeners so UI updates
            GraphParamChanged?.Invoke(this, new GraphParamChangedEventArgs(paramName, effectiveValue));
        }

        /// <summary>
        /// Gets all graph parameters from the current active graph.
        /// </summary>
        /// <returns>List of graph parameters, or empty list if no graph is loaded.</returns>
        public List<GraphEditor.GraphParam> GetAllGraphParams()
        {
            if (activeVehicleGraph == null)
            {
                return new List<GraphEditor.GraphParam>();
            }
            return CollectAllGraphParams(activeVehicleGraph, activeGraphResolver);
        }

        private bool hasDirtyGraphParams = false;
        private Dictionary<string, double> _graphParamValuesSnapshot = null;

        private void MarkProfileDirty()
        {
            // On first dirty, snapshot the current profile's GraphParamValues
            if (!hasDirtyGraphParams)
            {
                var profile = GetCurrentAircraftProfile();
                if (profile?.GraphParamValues != null)
                {
                    _graphParamValuesSnapshot = new Dictionary<string, double>(profile.GraphParamValues);
                }
                else
                {
                    _graphParamValuesSnapshot = new Dictionary<string, double>();
                }
            }
            hasDirtyGraphParams = true;
        }

        /// <summary>
        /// Discards unsaved profile changes by restoring GraphParamValues from the snapshot.
        /// Called when user declines to save changes on vehicle switch.
        /// </summary>
        public void DiscardProfileChanges()
        {
            if (!hasDirtyGraphParams || _graphParamValuesSnapshot == null)
            {
                return;
            }

            var profile = GetCurrentAircraftProfile();
            if (profile != null)
            {
                // Restore from snapshot
                profile.GraphParamValues = new Dictionary<string, double>(_graphParamValuesSnapshot);
            }

            // Clear dirty state and snapshot
            hasDirtyGraphParams = false;
            _graphParamValuesSnapshot = null;

            // Rebuild runtime params from restored profile
            BuildGraphParams();
        }

        private void ClearDirtyState()
        {
            hasDirtyGraphParams = false;
            _graphParamValuesSnapshot = null;
        }

        public event EventHandler ActiveGraphChanged;
        public event EventHandler<GraphParamChangedEventArgs> GraphParamChanged;
        public event EventHandler<ParamMigrationResult> ParamMigrationDetected;

        /// <summary>
        /// Gets the current value of a graph output signal.
        /// </summary>
        /// <param name="outputName">Full output signal name (e.g., "FlightPedals.SpringGain")</param>
        /// <param name="defaultValue">Value to return if output is not available</param>
        /// <returns>The current output value or defaultValue if not found</returns>
        public double GetGraphOutputValue(string outputName, double defaultValue = 0.0)
        {
            if (lastGraphEvaluation?.Outputs?.TryGetValue(outputName, out var value) == true)
            {
                return value;
            }
            return defaultValue;
        }

        /// <summary>
        /// Gets all current graph output values.
        /// </summary>
        /// <returns>Dictionary of output name to value, or empty dictionary if no evaluation</returns>
        public IReadOnlyDictionary<string, double> GetAllGraphOutputs()
        {
            if (lastGraphEvaluation?.Outputs != null)
            {
                return lastGraphEvaluation.Outputs;
            }
            return new Dictionary<string, double>();
        }

        /// <summary>
        /// Called by graph editor when graph content changes (e.g., reload, edit).
        /// Fires ActiveGraphChanged event to refresh parameter UI in function tabs.
        /// </summary>
        public void OnGraphContentChanged()
        {
            ActiveGraphChanged?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Applies the given graph definition to runtime evaluation.
        /// Called from graph editor Apply button to push in-memory changes.
        /// </summary>
        public void ApplyGraphToRuntime(GraphDefinition graph, string basePath)
        {
            if (graph == null)
            {
                return;
            }

            activeVehicleGraph = graph;
            string baseDir = string.IsNullOrEmpty(basePath) ? "" : Path.GetDirectoryName(basePath);

            activeGraphRuntime = GraphRuntimeConverter.Convert(graph);
            activeGraphResolver = GraphRuntimeConverter.CreateResolver(baseDir);
            activeIncludeContextCache = new DiyFfb.GraphTest.IncludeContextCache();
            activeGraphEvaluator = new DiyFfb.GraphTest.GraphCompiledEvaluator(
                activeGraphRuntime, activeGraphResolver, activeIncludeContextCache, baseDir);

            ActiveGraphChanged?.Invoke(this, EventArgs.Empty);
        }

        public void SetVehicleGraphPath(string gameId, string carId, string path)
        {
            if (Settings == null)
            {
                return;
            }

            string key = BuildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(key))
            {
                return;
            }

            if (Settings.AircraftFfbProfiles == null)
            {
                Settings.AircraftFfbProfiles = new Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            }

            if (string.IsNullOrWhiteSpace(path))
            {
                // Remove profile if clearing path
                Settings.AircraftFfbProfiles.Remove(key);
            }
            else
            {
                // Create or update profile with graph path
                if (!Settings.AircraftFfbProfiles.TryGetValue(key, out var profile))
                {
                    profile = new DiyFfbPluginSettings.AircraftFfbProfile();
                    Settings.AircraftFfbProfiles[key] = profile;
                }
                profile.GraphPath = path;
            }

            ResolveActiveGraph(gameId, carId);
            BuildGraphParams();
        }

        public void ApplyAircraftFfbProfile(string carId, DiyFfbPluginSettings.AircraftFfbProfile profile)
        {
            if (Settings == null || string.IsNullOrWhiteSpace(carId) || profile == null)
            {
                return;
            }

            string profileKey = BuildProfileKey(activeGameId, carId);
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                return;
            }

            if (Settings.AircraftFfbProfiles == null)
            {
                Settings.AircraftFfbProfiles = new System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            }

            Settings.AircraftFfbProfiles[profileKey] = profile;
            ApplyAircraftProfile(activeGameId, carId);
            ApplyFfbProfileToCurrentSettings(profile);
            ui?.Dispatcher?.BeginInvoke(new Action(() =>
            {
                ui.RefreshGraphSelection();
            }));
        }

        public void ApplyFfbProfileToCurrentSettings(DiyFfbPluginSettings.AircraftFfbProfile profile)
        {
            if (Settings == null || profile == null)
            {
                return;
            }

            Settings.XPlaneRotorIndex = profile.XPlaneRotorIndex;
        }

        public void SetPendingFfbProfile(DiyFfbPluginSettings.AircraftFfbProfile profile)
        {
            pendingFfbProfile = profile;
            hasPendingFfbProfile = profile != null;
        }

        public void ReplaceAircraftFfbProfiles(System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile> profiles)
        {
            if (Settings == null)
            {
                return;
            }

            Settings.AircraftFfbProfiles = profiles ?? new System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();
            BackupAircraftProfiles();
            if (!string.IsNullOrWhiteSpace(activeCarId))
            {
                ApplyAircraftProfile(activeGameId, activeCarId);
            }
        }

        /// <summary>
        /// Safety net: backs up non-empty profiles, restores from backup if profiles are empty.
        /// Protects against JSON.NET silently dropping protobuf data on deserialize.
        /// </summary>
        private void BackupOrRestoreAircraftProfiles()
        {
            if (Settings.AircraftFfbProfiles != null && Settings.AircraftFfbProfiles.Count > 0)
            {
                BackupAircraftProfiles();
            }
            else
            {
                // Profiles empty — try to restore from backup
                var backup = this.ReadCommonSettings<Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>>(
                    "AircraftProfilesBackup", () => null);
                if (backup != null && backup.Count > 0)
                {
                    SimHub.Logging.Current.Warn(
                        $"[Profiles] AircraftFfbProfiles empty — restoring {backup.Count} profiles from backup");
                    Settings.AircraftFfbProfiles = backup;
                }
            }
        }

        private void BackupAircraftProfiles()
        {
            if (Settings.AircraftFfbProfiles != null && Settings.AircraftFfbProfiles.Count > 0)
            {
                this.SaveCommonSettings("AircraftProfilesBackup", Settings.AircraftFfbProfiles);
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
                if (ui != null)
                {
                    ui.NotifyGatewayPortAutoConnected(portName);
                }
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

            // Migrate VehicleGraphPaths to AircraftFfbProfiles.GraphPath
            MigrateVehicleGraphPaths();

            // Safety net: backup non-empty profiles, restore if empty
            BackupOrRestoreAircraftProfiles();

            // Create tiered config orchestrator
            _configOrchestrator = new TieredConfigOrchestrator(
                _functionConfigManager,
                _axisConfigManager,
                Settings,
                () => this.SaveCommonSettings("GeneralSettings", Settings),
                GetCurrentAircraftProfile,
                GetActiveGraphCategory,
                BuildProfileKey,
                () => activeGameId,
                () => activeCarId);

            // Initialize manager with stored baselines and overrides
            _configOrchestrator.InitializeManagerFromSettings();

            // Initialize DirectInput reader for grip button graph inputs
            _buttonInputReader = new ButtonInputReader();

            // Populate function and axis configs from manager
            if (ui != null)
            {
                ui.PopulateFunctionConfigsFromBaselines();
                ui.PopulateAxisConfigsFromBaselines();
            }

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
            StartXPlaneUdpReceiver();

        }
    }

    /// <summary>
    /// Event arguments for graph parameter changes
    /// </summary>
    public class GraphParamChangedEventArgs : EventArgs
    {
        public string ParamName { get; }
        public double Value { get; }

        public GraphParamChangedEventArgs(string paramName, double value)
        {
            ParamName = paramName;
            Value = value;
        }
    }

    /// <summary>
    /// Event args for override field changes.
    /// </summary>
    public class OverrideFieldChangedEventArgs : EventArgs
    {
        public int FunctionId { get; }
        public string FieldPath { get; }

        public OverrideFieldChangedEventArgs(int functionId, string fieldPath)
        {
            FunctionId = functionId;
            FieldPath = fieldPath;
        }
    }
}
