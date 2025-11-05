//using SimHub.Plugins.OutputPlugins.Dash.GLCDTemplating;
using Google.Protobuf;
using Newtonsoft.Json;
using ProtbufTest;
using SimHub.Plugins.Styles;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Media;
using System.Net.Http;
using System.Reflection;
//using vJoy.Wrapper;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Json;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Forms;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Effects;
//using System.Diagnostics;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
//using System.Drawing;

using vJoyInterfaceWrap;
using Windows.UI.Notifications;

// Win 11 install, see https://github.com/jshafer817/vJoy/releases
//using vJoy.Wrapper;



namespace User.PluginSdkDemo
{
    /// <summary>
    /// Logique d'interaction pour DiyFfbPluginUI.xaml
    /// </summary>
    public partial class DiyFfbPluginUI : System.Windows.Controls.UserControl
    {


        // payload revisiom
        //public uint pedalConfigPayload_version = 110;
        //public uint pedalConfigPayload_type = 100;
        //public uint pedalActionPayload_type = 110;

        public uint indexOfSelectedPedal_u = 1;
        public uint profile_select = 0;
        public DiyFfbPlugin Plugin { get; }

        private FunctionID selected_function_id = FunctionID.Undefined;
        private AxisID selected_axis_id = AxisID.AxisUndefined;

        public DAP_config_st[] dap_config_st = new DAP_config_st[8];
        public DAP_config_st dap_config_st_rudder;
        public DAP_bridge_state_st dap_bridge_state_st;
        public Basic_WIfi_info _basic_wifi_info;
        private string stringValue;


        public bool[] waiting_for_pedal_config = new bool[8];
        public System.Windows.Forms.Timer[] pedal_serial_read_timer = new System.Windows.Forms.Timer[8];
        public System.Windows.Forms.Timer connect_timer;
        public CancellationTokenSource ESP_host_serial_timer_cts = new CancellationTokenSource();
        public Task ESP_host_serial_timer;
        //public System.Timers.Timer[] pedal_serial_read_timer = new System.Timers.Timer[8];
        int printCtr = 0;

        public double[] Force_curve_Y = new double[100];
        public bool debug_flag = false;

        //public VirtualJoystick joystick;
        internal vJoyInterfaceWrap.vJoy joystick;

        private Profile_Online Online_profile;

        public bool[] dumpPedalToResponseFile = new bool[8];
        public bool[] dumpPedalToResponseFile_clearFile = new bool[8];

        private SolidColorBrush defaultcolor;
        private SolidColorBrush lightcolor;
        private SolidColorBrush redcolor;
        private SolidColorBrush color_RSSI_1;
        private SolidColorBrush color_RSSI_2;
        private SolidColorBrush color_RSSI_3;
        private SolidColorBrush color_RSSI_4;
        private string info_text_connection;
        private string system_info_text_connection;
        private int current_pedal_travel_state= 0;
        private double[] Pedal_position_reading=new double[8];
        private bool[] Serial_connect_status = new bool[8] { false, false, false, false, false, false, false, false };
        public byte Bridge_RSSI = 0;
        public bool[] axis_wireless_connection_state = new bool[8];
        public int Bridge_baudrate = 3000000;
        public bool Fanatec_mode = false;
        public bool Update_Profile_Checkbox_b = false;
        public bool Update_CV_textbox = false;
        public bool[] Version_error_warning_b = new bool[8] { false, false, false, false, false, false, false, false };
        public bool[] Version_warning_first_show_b= new bool[8] { false, false, false, false, false, false, false, false };
        public bool Version_warning_first_show_b_bridge = false;
        public byte[] Pedal_version = new byte[8];
        private SerialMonitor_Window _serial_monitor_window;
        public bool Pedal_Log_warning_1st_show_b = true;
        private double pedal_pos_min = 0.0;
        private double pedal_pos_max = 0.0;
        private double pedal_pos_range = 0.0;
        public SortedDictionary<AxisID,Axis> axes { get; } = new SortedDictionary<AxisID, Axis>();
        public SortedDictionary<FunctionID, Function> functions { get; } = new SortedDictionary<FunctionID, Function>();
        public Dictionary<AxisID, FunctionID> last_known_functions { get; } = new Dictionary<AxisID, FunctionID>();
        //public int Bridge_baudrate = 921600;
        /*
        private double kinematicDiagram_zeroPos_OX = 100;
        private double kinematicDiagram_zeroPos_OY = 20;
        private double kinematicDiagram_zeroPos_scale = 1;
        */



        // read config from JSON on startup
        //ReadStructFromJson();


        // read JSON config from JSON file
        //private void ReadStructFromJson()
        //{



        //    try
        //    {
        //        // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0
        //        // https://www.educative.io/answers/how-to-read-a-json-file-in-c-sharp

        //        string currentDirectory = Directory.GetCurrentDirectory();
        //        string dirName = currentDirectory + "\\PluginsData\\Common";
        //        //string jsonFileName = ComboBox_JsonFileSelected.Text;
        //        string jsonFileName = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();
        //        string fileName = dirName + "\\" + jsonFileName + ".json";

        //        string text = System.IO.File.ReadAllText(fileName);

        //        DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
        //        var ms = new MemoryStream(Encoding.UTF8.GetBytes(text));
        //        dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
        //        //TextBox_debugOutput.Text = "Config loaded!";
        //        //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.Text;
        //        //TextBox_debugOutput.Text += "    ";
        //        //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.SelectedIndex;

        //        updateTheGuiFromConfig();

        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }


        //}
        private void ToastNotification(string message1, string message2)
        {
            
            var xml = ToastNotificationManager.GetTemplateContent(ToastTemplateType.ToastText02);
            var text = xml.GetElementsByTagName("text");
            text[0].AppendChild(xml.CreateTextNode(message1));
            text[1].AppendChild(xml.CreateTextNode(message2));
            var toast = new ToastNotification(xml);
            toast.ExpirationTime = DateTime.Now.AddSeconds(1);
            toast.Tag = "Pedal_notification";
            ToastNotificationManager.CreateToastNotifier("FFB Pedal Dashboard").Show(toast);



        }

        private void vjoy_axis_initialize()
        {
            //center all axis/hats reader
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_X);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Y);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Z);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);
            //joystick.SetJoystickHat(0, Hats.Hat);
            //joystick.SetJoystickHat(0, Hats.HatExt1);
            //joystick.SetJoystickHat(0, Hats.HatExt2);
            //joystick.SetJoystickHat(0, Hats.HatExt3);

        }





        private void InitReadStructFromJson()
        {



            try
            {
                // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0
                // https://www.educative.io/answers/how-to-read-a-json-file-in-c-sharp
                string jsonFileName = "NA";

                string currentDirectory = Directory.GetCurrentDirectory();
                string dirName = currentDirectory + "\\PluginsData\\Common";
                //string jsonFileName = ComboBox_JsonFileSelected.Text;
                if (indexOfSelectedPedal_u == 0)
                {
                    jsonFileName = ("DiyPedalConfig_Clutch_Default");
                }
                else if (indexOfSelectedPedal_u == 1)
                {
                    jsonFileName = ("DiyPedalConfig_Brake_Default");
                }
                else if (indexOfSelectedPedal_u == 2)
                {
                    jsonFileName = ("DiyPedalConfig_Accelerator_Default");
                }

                string fileName = dirName + "\\" + jsonFileName + ".json";
                string text = System.IO.File.ReadAllText(fileName);



                DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                var ms = new MemoryStream(Encoding.UTF8.GetBytes(text));
                dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
                TextBox_debugOutput.Text = "Config loaded!" + jsonFileName;
                //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.Text;
                //TextBox_debugOutput.Text += "    ";
                //TextBox_debugOutput.Text += ComboBox_JsonFileSelected.SelectedIndex;

                updateTheGuiFromConfig();

            }
            catch (Exception caughtEx)
            {

                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }


        }

        private void UpdateSerialPortList_click()
        {

            var SerialPortSelectionArray = new List<SerialPortChoice>();
            string[] comPorts = SerialPort.GetPortNames();

            comPorts = comPorts.Distinct().ToArray(); // unique

            if (comPorts.Length > 0)
            {

                foreach (string portName in comPorts)
                {
                    SerialPortSelectionArray.Add(new SerialPortChoice(portName, portName));
                }
            }
            else
            {
                SerialPortSelectionArray.Add(new SerialPortChoice("NA", "NA"));
            }

            SerialPortSelection.DataContext = SerialPortSelectionArray;
            SerialPortSelection_ESPNow.DataContext = SerialPortSelectionArray;
        }

        private bool isDragging = false;
        private Point offset;

        public void DAP_config_set_default(uint pedalIdx)
        {
            dumpPedalToResponseFile[pedalIdx] = false;
            dumpPedalToResponseFile_clearFile[pedalIdx] = false;
            dap_config_st[pedalIdx].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            dap_config_st[pedalIdx].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            dap_config_st[pedalIdx].payloadPedalConfig_.pedalStartPosition = 35;
            dap_config_st[pedalIdx].payloadPedalConfig_.pedalEndPosition = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.maxForce = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.preloadForce = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p000 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p020 = 20;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p040 = 40;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p060 = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p080 = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.relativeForce_p100 = 100;
            dap_config_st[pedalIdx].payloadPedalConfig_.dampingPress = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.dampingPull = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.absFrequency = 5;
            dap_config_st[pedalIdx].payloadPedalConfig_.absAmplitude = 20;
            dap_config_st[pedalIdx].payloadPedalConfig_.absPattern = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.absForceOrTarvelBit = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_a = 205;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_b = 220;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_d = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_c_horizontal = 215;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_c_vertical = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.lengthPedal_travel = 100;

            dap_config_st[pedalIdx].payloadPedalConfig_.Simulate_ABS_trigger = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.Simulate_ABS_value = 80;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_max_freq = 40;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_min_freq = 10;
            dap_config_st[pedalIdx].payloadPedalConfig_.RPM_AMP = 30;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_trigger_value = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_amp = 1;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_freq = 15;
            dap_config_st[pedalIdx].payloadPedalConfig_.BP_trigger = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.G_multi = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.G_window = 10;
            dap_config_st[pedalIdx].payloadPedalConfig_.WS_amp = 1;
            dap_config_st[pedalIdx].payloadPedalConfig_.WS_freq = 15;
            dap_config_st[pedalIdx].payloadPedalConfig_.Impact_multi = 50;
            dap_config_st[pedalIdx].payloadPedalConfig_.Impact_window = 60;
            dap_config_st[pedalIdx].payloadPedalConfig_.CV_amp_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.CV_freq_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.CV_amp_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.CV_freq_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.maxGameOutput = 100;
            dap_config_st[pedalIdx].payloadPedalConfig_.kf_modelNoise = 128;
            dap_config_st[pedalIdx].payloadPedalConfig_.kf_modelOrder = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_0 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_3 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_a_4 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_0 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_1 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_2 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_3 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.cubic_spline_param_b_4 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.PID_p_gain = 0.1f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_i_gain = 1.0f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_d_gain = 0.0f;
            dap_config_st[pedalIdx].payloadPedalConfig_.PID_velocity_feedforward_gain = 0.0f;

            dap_config_st[pedalIdx].payloadPedalConfig_.MPC_0th_order_gain = 10.0f;
            dap_config_st[pedalIdx].payloadPedalConfig_.MPC_1st_order_gain = 0.0f;

            dap_config_st[pedalIdx].payloadPedalConfig_.control_strategy_b = 2;

            dap_config_st[pedalIdx].payloadPedalConfig_.loadcell_rating = 150;

            dap_config_st[pedalIdx].payloadPedalConfig_.travelAsJoystickOutput_u8 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.invertLoadcellReading_u8 = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.invertMotorDirection_u8 = 0;

            dap_config_st[pedalIdx].payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
            dap_config_st[pedalIdx].payloadPedalConfig_.pedal_type = (byte)pedalIdx;
            //dap_config_st[pedalIdx].payloadPedalConfig_.OTA_flag = 0;
            dap_config_st[pedalIdx].payloadPedalConfig_.stepLossFunctionFlags_u8 = 0b11;
        }

        public void DAP_config_set_default_rudder()
        {
            
            dap_config_st_rudder.payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            dap_config_st_rudder.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            dap_config_st_rudder.payloadPedalConfig_.pedalStartPosition = 5;
            dap_config_st_rudder.payloadPedalConfig_.pedalEndPosition = 95;
            dap_config_st_rudder.payloadPedalConfig_.maxForce = 10;
            dap_config_st_rudder.payloadPedalConfig_.preloadForce = 0;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p000 = 0;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p020 = 20;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p040 = 40;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p060 = 60;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p080 = 80;
            dap_config_st_rudder.payloadPedalConfig_.relativeForce_p100 = 100;
            dap_config_st_rudder.payloadPedalConfig_.dampingPress = 0;
            dap_config_st_rudder.payloadPedalConfig_.dampingPull = 0;
            dap_config_st_rudder.payloadPedalConfig_.absFrequency = 5;
            dap_config_st_rudder.payloadPedalConfig_.absAmplitude = 20;
            dap_config_st_rudder.payloadPedalConfig_.absPattern = 0;
            dap_config_st_rudder.payloadPedalConfig_.absForceOrTarvelBit = 0;

            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_a = 205;
            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_b = 220;
            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_d = 60;
            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_horizontal = 215;
            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_vertical = 60;
            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_travel = 60;

            dap_config_st_rudder.payloadPedalConfig_.Simulate_ABS_trigger = 0;
            dap_config_st_rudder.payloadPedalConfig_.Simulate_ABS_value = 80;
            dap_config_st_rudder.payloadPedalConfig_.RPM_max_freq = 45;
            dap_config_st_rudder.payloadPedalConfig_.RPM_min_freq = 15;
            dap_config_st_rudder.payloadPedalConfig_.RPM_AMP = 1;
            dap_config_st_rudder.payloadPedalConfig_.BP_trigger_value = 50;
            dap_config_st_rudder.payloadPedalConfig_.BP_amp = 1;
            dap_config_st_rudder.payloadPedalConfig_.BP_freq = 15;
            dap_config_st_rudder.payloadPedalConfig_.BP_trigger = 0;
            dap_config_st_rudder.payloadPedalConfig_.G_multi = 50;
            dap_config_st_rudder.payloadPedalConfig_.G_window = 10;
            dap_config_st_rudder.payloadPedalConfig_.WS_amp = 1;
            dap_config_st_rudder.payloadPedalConfig_.WS_freq = 15;
            dap_config_st_rudder.payloadPedalConfig_.Impact_multi = 50;
            dap_config_st_rudder.payloadPedalConfig_.Impact_window = 60;
            dap_config_st_rudder.payloadPedalConfig_.CV_amp_1 = 0;
            dap_config_st_rudder.payloadPedalConfig_.CV_freq_1 = 0;
            dap_config_st_rudder.payloadPedalConfig_.CV_amp_2 = 0;
            dap_config_st_rudder.payloadPedalConfig_.CV_freq_2 = 0;

            dap_config_st_rudder.payloadPedalConfig_.maxGameOutput = 100;
            dap_config_st_rudder.payloadPedalConfig_.kf_modelNoise = 10;
            dap_config_st_rudder.payloadPedalConfig_.kf_modelOrder = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_a_0 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_a_1 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_a_2 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_a_3 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_a_4 = 0;

            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_b_0 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_b_1 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_b_2 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_b_3 = 0;
            dap_config_st_rudder.payloadPedalConfig_.cubic_spline_param_b_4 = 0;

            dap_config_st_rudder.payloadPedalConfig_.PID_p_gain = 0.3f;
            dap_config_st_rudder.payloadPedalConfig_.PID_i_gain = 50.0f;
            dap_config_st_rudder.payloadPedalConfig_.PID_d_gain = 0.0f;
            dap_config_st_rudder.payloadPedalConfig_.PID_velocity_feedforward_gain = 0.0f;

            dap_config_st_rudder.payloadPedalConfig_.MPC_0th_order_gain = 6.0f;
            dap_config_st_rudder.payloadPedalConfig_.MPC_1st_order_gain = 0.0f;

            dap_config_st_rudder.payloadPedalConfig_.control_strategy_b = 2;

            dap_config_st_rudder.payloadPedalConfig_.loadcell_rating = 100;

            dap_config_st_rudder.payloadPedalConfig_.travelAsJoystickOutput_u8 = 1;

            dap_config_st_rudder.payloadPedalConfig_.invertLoadcellReading_u8 = 0;
            dap_config_st_rudder.payloadPedalConfig_.invertMotorDirection_u8 = 0;

            dap_config_st_rudder.payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
            dap_config_st_rudder.payloadPedalConfig_.pedal_type = (byte)4;
            //dap_config_st[pedalIdx].payloadPedalConfig_.OTA_flag = 0;
            dap_config_st_rudder.payloadPedalConfig_.stepLossFunctionFlags_u8 = 0b11;
        }
        System.Windows.Controls.CheckBox[,] Effect_status_profile=new System.Windows.Controls.CheckBox[8,8];
        unsafe public DiyFfbPluginUI()
        {
            
            DAP_config_set_default_rudder();
            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                DAP_config_set_default(pedalIdx);
                
            }
            for (uint i = 0; i < 30; i++)
            {
                _basic_wifi_info.WIFI_PASS[i] = 0;
                _basic_wifi_info.WIFI_SSID[i] = 0;
            }
            InitializeComponent();

            //initialize profile effect status
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 8; j++)
                {
                    Effect_status_profile[i, j] = new System.Windows.Controls.CheckBox();
                    Effect_status_profile[i, j].Width = 45;
                    Effect_status_profile[i, j].Height = 20;
                    Effect_status_profile[i, j].FontSize = 8;
                    switch (j)
                    {
                        case 0:
                            
                            Effect_status_profile[i, j].Margin = new Thickness(20, 0, 0, 0);
                            if (i == 0 || i == 2)
                            {
                                Effect_status_profile[i, j].Content = "TC";
                                
                            }
                            else
                            {
                                Effect_status_profile[i, j].Content = "ABS";
                            }
                            
                            break;
                        case 1:
                            Effect_status_profile[i, j].Content = "RPM";
                            break;
                        case 2:
                            Effect_status_profile[i, j].Content = "B.P";
                            Effect_status_profile[i, j].Width = 40;
                            break;
                        case 3:
                            Effect_status_profile[i, j].Content = "G-F";
                            Effect_status_profile[i, j].Width = 40;
                            if (i == 0 || i == 2)
                            {
                                Effect_status_profile[i, j].IsEnabled = false;
                            }
                            break;
                        case 4:
                            Effect_status_profile[i, j].Content = "W.S";
                            Effect_status_profile[i, j].Width = 40;
                            break;
                        case 5:
                            Effect_status_profile[i, j].Content = "IMAPCT";
                            Effect_status_profile[i, j].Width = 60;
                            break;
                        case 6:
                            Effect_status_profile[i, j].Content = "CUS-1";
                            Effect_status_profile[i, j].Width = 50;
                            break;
                        case 7:
                            Effect_status_profile[i, j].Content = "CUS-2";
                            Effect_status_profile[i, j].Width = 50;
                            break;
                    }
                    switch (i)
                    {
                        case 0:
                            StackPanel_Effects_Status_0.Children.Add(Effect_status_profile[i, j]);
                            break;
                        case 1:
                            StackPanel_Effects_Status_1.Children.Add(Effect_status_profile[i, j]);
                            break;
                        case 2:
                            StackPanel_Effects_Status_2.Children.Add(Effect_status_profile[i, j]);
                            break;
                    }
                }
            }

            // debug mode invisiable
            //text_debug_flag.Visibility = Visibility.Hidden;
            //text_serial.Visibility = Visibility.Hidden;
            //TextBox_serialMonitor.Visibility = System.Windows.Visibility.Hidden;
            //InvertLoadcellReading_check.Visibility = Visibility.Hidden;
            //InvertMotorDir_check.Visibility = Visibility.Hidden;
            //textBox_debug_Flag_0.Visibility = Visibility.Hidden;
            //Border_serial_monitor.Visibility=Visibility.Hidden;
            
           

            //Label_reverse_LC.Visibility=Visibility.Hidden;
            //Label_reverse_servo.Visibility=Visibility.Hidden;
            btn_test.Visibility=Visibility.Hidden;
            //setting drawing color with Simhub theme workaround
            SolidColorBrush buttonBackground_ = btn_update.Background as SolidColorBrush;

            Color color = Color.FromArgb(150, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            Color color_2 = Color.FromArgb(200, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            Color color_3 = Color.FromArgb(255, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B);
            Color RED_color = Color.FromArgb(60, 139, 0, 0);
            redcolor = new SolidColorBrush(RED_color);
            SolidColorBrush Line_fill = new SolidColorBrush(color_2);
            
            //SolidColorBrush rect_fill = new SolidColorBrush(color);
            defaultcolor = new SolidColorBrush(color);
            lightcolor = new SolidColorBrush(color_3);
            color_RSSI_1 = new SolidColorBrush(Color.FromArgb(150, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B));
            color_RSSI_2 = new SolidColorBrush(Color.FromArgb(180, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B));
            color_RSSI_3 = new SolidColorBrush(Color.FromArgb(210, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B));
            color_RSSI_4 = new SolidColorBrush(Color.FromArgb(255, buttonBackground_.Color.R, buttonBackground_.Color.G, buttonBackground_.Color.B));
            RSSI_1.Fill = color_RSSI_1;
            RSSI_2.Fill = color_RSSI_2;
            RSSI_3.Fill = color_RSSI_3;
            RSSI_4.Fill = color_RSSI_4;
            //Plugin.simhub_theme_color=defaultcolor.ToString();            
            // Call this method to generate gridlines on the Canvas
            //DrawGridLines();
            //DrawGridLines_kinematicCanvas(100,20,1.5);
            Label_RSSI.Visibility= Visibility.Hidden;
            TextBox_debug_count.Visibility= Visibility.Hidden;
            Online_profile = new Profile_Online();
            Online_profile.Basic_Config=new BasicConfig();








        }

        public SolidColorBrush MouseDownColor { get { return lightcolor; } }

        public SolidColorBrush MouseUpColor { get { return defaultcolor; } }

        public byte[] getBytesPayload(payloadPedalConfig aux)
        {
            int length = Marshal.SizeOf(aux);
            IntPtr ptr = Marshal.AllocHGlobal(length);
            byte[] myBuffer = new byte[length];

            Marshal.StructureToPtr(aux, ptr, true);
            Marshal.Copy(ptr, myBuffer, 0, length);
            Marshal.FreeHGlobal(ptr);

            return myBuffer;
        }


        public byte[] getBytes(DAP_config_st aux)
        {
            int length = Marshal.SizeOf(aux);
            IntPtr ptr = Marshal.AllocHGlobal(length);
            byte[] myBuffer = new byte[length];

            Marshal.StructureToPtr(aux, ptr, true);
            Marshal.Copy(ptr, myBuffer, 0, length);
            Marshal.FreeHGlobal(ptr);

            return myBuffer;
        }


        //public byte[] getBytes_Action(DAP_action_st aux)
        //{
        //    int length = Marshal.SizeOf(aux);
        //    IntPtr ptr = Marshal.AllocHGlobal(length);
        //    byte[] myBuffer = new byte[length];

        //    Marshal.StructureToPtr(aux, ptr, true);
        //    Marshal.Copy(ptr, myBuffer, 0, length);
        //    Marshal.FreeHGlobal(ptr);

        //    return myBuffer;
        //}


        public DAP_config_st getConfigFromBytes(byte[] myBuffer)
        {
            DAP_config_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_config_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_config_st)Marshal.PtrToStructure(ptr, typeof(DAP_config_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }


        public DAP_state_basic_st getStateFromBytes(byte[] myBuffer)
        {
            DAP_state_basic_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_state_basic_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_state_basic_st)Marshal.PtrToStructure(ptr, typeof(DAP_state_basic_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }

        public DAP_state_extended_st getStateExtFromBytes(byte[] myBuffer)
        {
            DAP_state_extended_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_state_extended_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_state_extended_st)Marshal.PtrToStructure(ptr, typeof(DAP_state_extended_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }
        public DAP_bridge_state_st getStateBridgeFromBytes(byte[] myBuffer)
        {
            DAP_bridge_state_st aux;

            // see https://stackoverflow.com/questions/31045358/how-do-i-copy-bytes-into-a-struct-variable-in-c
            int size = Marshal.SizeOf(typeof(DAP_bridge_state_st));
            IntPtr ptr = Marshal.AllocHGlobal(size);

            Marshal.Copy(myBuffer, 0, ptr, size);

            aux = (DAP_bridge_state_st)Marshal.PtrToStructure(ptr, typeof(DAP_bridge_state_st));
            Marshal.FreeHGlobal(ptr);

            return aux;
        }


        //unsafe private UInt16 checksumCalc(byte* data, int length)
        //{

        //    UInt16 curr_crc = 0x0000;
        //    byte sum1 = (byte)curr_crc;
        //    byte sum2 = (byte)(curr_crc >> 8);
        //    int index;
        //    for (index = 0; index < length; index = index + 1)
        //    {
        //        int v = (sum1 + (*data));
        //        sum1 = (byte)v;
        //        sum1 = (byte)(v % 255);

        //        int w = (sum1 + sum2) % 255;
        //        sum2 = (byte)w;

        //        data++;// = data++;
        //    }

        //    int x = (sum2 << 8) | sum1;
        //    return (UInt16)x;
        //}

        private void OnABSTestStateChange(bool state)
        {
            Plugin.sendAbsSignal = state;
        }

        public DiyFfbPluginUI(DiyFfbPlugin plugin) : this()
        {
            DataContext = this;
            this.Plugin = plugin;
            uc_function_config.ABSTestStateChange += OnABSTestStateChange;
            uc_axis_config.KinematicParametersChanged += OnKinematicParametersChanged;
            uc_function_config.SetGui(this, plugin);
            uc_axis_config.SetGui(this, plugin);
            //DiyPedalKinematicsControl.KinematicParametersChanged += OnKinematicParametersChanged;
            //DiyPedalKinematicsControl.KinematicParametersChanged += AutomotivePedalConfig.OnKinematicParametersChanged;
            //DiyPedalKinematicsControl.SetGui(this, plugin);
            for (FunctionID id = FunctionID.BrakePedal; id <= FunctionID.FlightPedals; id++)
            {
                functions[id] = new Function(id);
                functions[id].Config = FunctionConfigControl.GetDefaultConfig(id);
            }
            for (AxisID id = AxisID._1; id <= AxisID._8; id++)
            {
                axes[id] = new Axis(id);
                axes[id].Config = AxisConfigControl.GetDefaultConfig(id);
                axes[id].OnlineStateChanged += OnOnlineStateChange;
            }

            UpdateSerialPortList_click();
            //closeSerialAndStopReadCallback(1);

             

            // check if Json config files are present, otherwise create new ones
            //for (int jsonIndex = 0; jsonIndex < ComboBox_JsonFileSelected.Items.Count; jsonIndex++)
            //{

            //    ComboBox_JsonFileSelected.SelectedIndex = jsonIndex;

            //    // which config file is seleced
            //    string currentDirectory = Directory.GetCurrentDirectory();
            //    string dirName = currentDirectory + "\\PluginsData\\Common";
            //    //string jsonFileName = ComboBox_JsonFileSelected(ComboBox_JsonFileSelected.Items[jsonIndex]).Text;
            //    string jsonFileName = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();
            //    string fileName = dirName + "\\" + jsonFileName + ".json";


            //    // Check if file already exists, otherwise create    
            //    if (!File.Exists(fileName))
            //    {
            //        // create default config
            //        // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
            //        // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
            //        var stream1 = new MemoryStream();
            //        var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
            //        ser.WriteObject(stream1, Plugin.dap_config_initial_st);

            //        stream1.Position = 0;
            //        StreamReader sr = new StreamReader(stream1);
            //        string jsonString = sr.ReadToEnd();

            //        System.IO.File.WriteAllText(fileName, jsonString);
            //    }
            //}

            string currentDirectory = Directory.GetCurrentDirectory();
            string dirName = currentDirectory + "\\PluginsData\\Common";
            //string jsonFileName = ComboBox_JsonFileSelected(ComboBox_JsonFileSelected.Items[jsonIndex]).Text;
            string jsonFileNameA = "DiyPedalConfig_Accelerator_Default";
            string jsonFileNameB = "DiyPedalConfig_Brake_Default";
            string jsonFileNameC = "DiyPedalConfig_Clutch_Default";
            string fileNameA = dirName + "\\" + jsonFileNameA + ".json";
            string fileNameB = dirName + "\\" + jsonFileNameB + ".json";
            string fileNameC = dirName + "\\" + jsonFileNameC + ".json";
            /*
            if (!File.Exists(fileNameA))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameA, jsonString);
            }

            if (!File.Exists(fileNameB))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameB, jsonString);
            }
            if (!File.Exists(fileNameC))
            {
                // create default config
                // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                var stream1 = new MemoryStream();
                var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                ser.WriteObject(stream1, Plugin.dap_config_initial_st);

                stream1.Position = 0;
                StreamReader sr = new StreamReader(stream1);
                string jsonString = sr.ReadToEnd();

                System.IO.File.WriteAllText(fileNameC, jsonString);
            }
            */
            //InitReadStructFromJson();
            
            //for (uint pedalIndex = 0; pedalIndex < 3; pedalIndex++)
            //{
                //indexOfSelectedPedal_u = pedalIndex;
                //ComboBox_JsonFileSelected.SelectedIndex = Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u];
                //ComboBox_JsonFileSelected.SelectedIndex = Plugin.Settings.selectedJsonIndexLast[indexOfSelectedPedal_u];
                //InitReadStructFromJson();
                /*
                if (plugin.Settings.connect_status[pedalIndex] == 1)
                {
                    if (plugin.Settings.reading_config == 1)
                    {
                        if (plugin._serialPort[pedalIndex].IsOpen)
                        {
                            Reading_config_auto(pedalIndex);
                        }
                        else
                        {
                            plugin.Settings.connect_status[pedalIndex] = 0;
                        }
                        
                    }

                }
                */


                /*
                if (plugin.PortExists(plugin._serialPort[pedalIndex].PortName))
                {
                    if (plugin.Settings.connect_status[pedalIndex] == 1)
                    {
                        if (plugin.Settings.reading_config == 1)
                        {
                            Reading_config_auto(pedalIndex);
                        }

                    }
                    
                }
                else
                {
                    plugin.Settings.connect_status[pedalIndex] = 0;
                }
                */


                //updateTheGuiFromConfig();
            //}
        

            if (plugin.Settings.reading_config == 1)
            {
                checkbox_pedal_read.IsChecked = true;

            }
            else
            {
                checkbox_pedal_read.IsChecked = false;
            }

            indexOfSelectedPedal_u = plugin.Settings.function_tab_selected;
            tc_function_selection.SelectedIndex = (int)indexOfSelectedPedal_u;
            tc_axis_selection.SelectedIndex = (int)plugin.Settings.axis_tab_selected;

            //reconnect to com port
            if (plugin.Settings.axis_settings[indexOfSelectedPedal_u].auto_connect)
            {
                checkbox_auto_connect.IsChecked = true;
            }
            else
            {
                checkbox_auto_connect.IsChecked = false;
            }

            try_connect();

            /*
            // autoconnect serial
            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.connectSerialPort[pedalIdx] == true)
                {
                    if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                    {
                        if (Plugin._serialPort[pedalIdx].IsOpen == false)
                        {
                            if (Plugin.Settings.connect_status[pedalIdx] == 1)
                            {
                                openSerialAndAddReadCallback(pedalIdx);
                                Reading_config_auto(pedalIdx);
                            }

                        }
                    }
                    else
                    {
                        Plugin.connectSerialPort[pedalIdx] = false;
                        Plugin.Settings.connect_status[pedalIdx] = 0;
                    }

                }
            }
            */


            //vjoy initialized
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                Vjoy_out_check.IsChecked = true;
                uint vJoystickId = Plugin.Settings.vjoy_order;
                //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                joystick = new vJoyInterfaceWrap.vJoy();

                joystick.AcquireVJD(vJoystickId);
                //joystick.Aquire();
                vjoy_axis_initialize();
            }
            else
            {
                Vjoy_out_check.IsChecked = false;
            }


        }

        private void OnOnlineStateChange(AxisID axis_id, bool new_online_state)
        {
            string msg;
            if (new_online_state)
            {
                msg = String.Format("Axis {0} Connected", (int)axis_id);
            }
            else
            {
                msg = String.Format("Axis {0} Disconnected", (int)axis_id);
                if (last_known_functions.ContainsKey(axis_id))
                {
                    functions[last_known_functions[axis_id]].OnAxisRemoved(axis_id);
                    last_known_functions.Remove(axis_id);
                }
            }
            ToastNotification("Wireless Connection", msg);
        }

        public void updateTheGuiFromConfig()
        {
            // update the sliders

            info_label.Content = "State:\nDAP Version:\nPlugin Version:";
            info_label_system.Content = "Bridge:\nDAP Version:\nPlugin Version:";
            //RSSI canvas
            if (Plugin != null)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    RSSI_canvas.Visibility = Visibility.Visible;
                    //Label_RSSI.Visibility = Visibility.Visible;
                }
                else
                {
                    RSSI_canvas.Visibility = Visibility.Hidden;
                    Label_RSSI.Visibility = Visibility.Hidden;
                }
            }

            string plugin_version = Assembly.GetExecutingAssembly().GetName().Version.ToString();
            if (plugin_version == "1.0.0.0")
            {
                plugin_version = "Dev.";
            }
            string info_text;
            string system_info_text;
            info_text = "Waiting...";
            system_info_text = "Waiting...";
            if (Plugin != null)
            {
                
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
                {
                    info_text = "Connected";
                }
                else
                {
                    if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].auto_connect)
                    {
                        info_text = info_text_connection;
                    }
                }
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    system_info_text = "Connected";
                    if (axis_wireless_connection_state[indexOfSelectedPedal_u])
                    {
                        info_text = "Wireless";
                    }
                }
                else
                {
                    if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag)
                    {
                        system_info_text = system_info_text_connection;
                    }
                    if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
                    {
                        if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag)
                        {
                            info_text = info_text_connection;
                        }
                        
                    }

                }
                info_text += "\n" + Constants.pedalConfigPayload_version + "\n" + plugin_version;
                system_info_text += "\n" + Constants.pedalConfigPayload_version + "\n" + plugin_version;
                if (Plugin.Rudder_status)
                {
                    info_text += "\nIn Action";
                    system_info_text += "\nIn Action";
                    info_label.Content += "\nRudder:";
                    info_label_system.Content += "\nRudder:";
                }
                if (Fanatec_mode)
                {
                    info_text += "\nIn Action";
                    system_info_text += "\nIn Action";
                    info_label.Content += "\nFanatec:";
                    info_label_system.Content += "\nFanatec:";
                }
                info_label_2.Content = info_text;
                info_label_2_system.Content = system_info_text;
            }





            int debugFlagValue_0 = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0;
            textBox_debug_Flag_0.Text = debugFlagValue_0.ToString();

            //slider setting

            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition < 5)
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition = 5;
            }
            if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition > 95)
            {
                dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition=95;
            }
            
            if (Plugin != null)
            {
                Label_Pedal_interval_trigger.Content = "Action Interval: "+Plugin.Settings.function_settings[indexOfSelectedPedal_u].action_interval + "ms";
                Slider_Pedal_interval_trigger.Value = Plugin.Settings.function_settings[indexOfSelectedPedal_u].action_interval;

                if (Plugin.Sync_esp_connection_flag)
                {
                    btn_connect_espnow_port.Content = "Disconnect";
                }
                else
                {
                    btn_connect_espnow_port.Content = "Connect";
                }

                if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
                {
                    CheckBox_Pedal_ESPNow_SyncFlag.IsChecked = true;
                }
                else
                {
                    CheckBox_Pedal_ESPNow_SyncFlag.IsChecked = false;
                }

                if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag)
                {
                    CheckBox_Pedal_ESPNow_autoconnect.IsChecked = true;
                }
                else
                { 
                    CheckBox_Pedal_ESPNow_autoconnect.IsChecked= false;
                }

                if (Plugin.Settings.Serial_auto_clean)
                {
                    //Checkbox_auto_remove_serial_line.IsChecked = true;
                }
                else
                { 
                    //Checkbox_auto_remove_serial_line.IsChecked= false;
                }

                if (Plugin.Settings.Using_CDC_bridge)
                {
                    CheckBox_using_CDC_for_bridge.IsChecked = true;
                }
                else
                {
                    CheckBox_using_CDC_for_bridge.IsChecked = false;
                }
                if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].USING_ESP32S3 == true)
                {
                    CheckBox_USINGESP32S3.IsChecked = true;
                }
                else
                {
                    CheckBox_USINGESP32S3.IsChecked = false;
                }

                
                
            }

            //set for travel slider;
            double dx = 0;


            //// Select serial port accordingly
            string tmp = (string)Plugin._serialPort[indexOfSelectedPedal_u].PortName;
            try
            {
                SerialPortSelection.SelectedValue = tmp;
                TextBox_debugOutput.Text = "Serial port selected: " + SerialPortSelection.SelectedValue;

            }
            catch (Exception caughtEx)
            {
            }


            if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == true)
            {
                ConnectToPedal.IsChecked = true;
                btn_pedal_connect.Content = "Disconnect From Pedal";
            }
            else
            {
                ConnectToPedal.IsChecked = false;
                btn_pedal_connect.Content = "Connect To Pedal";
            }


            if (Plugin.Settings.file_enable_check[profile_select, 0] == 1)
            {
                Label_clutch_file.Content = Plugin.Settings.Pedal_file_string[profile_select,0];
                Clutch_file_check.IsChecked = true;
            }
            else 
            {
                Label_clutch_file.Content = "";
                Clutch_file_check.IsChecked = false;
            }
            if (Plugin.Settings.file_enable_check[profile_select, 1] == 1)
            {
                Label_brake_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 1];
                Brake_file_check.IsChecked = true;
            }
            else
            {
                Label_brake_file.Content = "";
                Brake_file_check.IsChecked = false;
            }

            if (Plugin.Settings.file_enable_check[profile_select,2] == 1)
            {
                Label_gas_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 2];
                Gas_file_check.IsChecked = true;
            }
            else
            {
                Label_gas_file.Content = "";
                Gas_file_check.IsChecked = false;
            }
            

            if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].RTSDTR_False == true)
            {
                CheckBox_RTSDTR.IsChecked = true;
            }
            else
            { 
                CheckBox_RTSDTR.IsChecked = false;
            }

            if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].auto_connect)
            {
                checkbox_auto_connect.IsChecked = true;
            }
            else
            {
                checkbox_auto_connect.IsChecked= false;
            }

            
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;
            textbox_profile_name.Text = Plugin.Settings.Profile_name[profile_select];



            if (Plugin != null)
            {
                if (Plugin.Settings.advanced_b)
                {
                    Debug_check.IsChecked = true;
                    debug_flag=Plugin.Settings.advanced_b;
                }
                else
                { 
                    Debug_check.IsChecked= false;
                    debug_flag = Plugin.Settings.advanced_b;
                }

                if (Plugin.Settings.Serial_auto_clean_bridge)
                {
                    Checkbox_auto_remove_serial_line_bridge.IsChecked = true;
                }
                else
                {
                    Checkbox_auto_remove_serial_line_bridge.IsChecked = false;
                }
                //effect profile reading
                if (Update_Profile_Checkbox_b)
                {
                    for (int j = 0; j < 8; j++)
                    {
                        for (int k = 0; k < 8; k++)
                        {
                            if (Plugin.Settings.function_settings[j].effect_status_profiles[profile_select, k])
                            {
                                Effect_status_profile[j, k].IsChecked = true;
                            }
                            else
                            {
                                Effect_status_profile[j, k].IsChecked = false;
                            }
                        }
                    }
                    Update_Profile_Checkbox_b = false;
                }

                textbox_SSID.Text = Plugin.Settings.SSID_string;
                textbox_PASS.Password = Plugin.Settings.PASS_string;
            }

        }






        public class SerialPortChoice
        {
            public SerialPortChoice(string display, string value)
            {
                Display = display;
                Value = value;
            }

            public string Value { get; set; }
            public string Display { get; set; }
        }



        // Select which pedal to config
        // see https://stackoverflow.com/questions/772841/is-there-selected-tab-changed-event-in-the-standard-wpf-tab-control
        private void FunctionSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin != null)
            {
                selected_function_id = (FunctionID)tc_function_selection.SelectedIndex + 1;
                Plugin.Settings.function_tab_selected = (uint)tc_function_selection.SelectedIndex;
                uc_function_config.SwitchFunction(functions[selected_function_id]);
            }
        }

        private void AxisSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin != null)
            {
                selected_axis_id = (AxisID)tc_axis_selection.SelectedIndex + 1;
                Plugin.Settings.axis_tab_selected = (uint)tc_axis_selection.SelectedIndex;
                AxisConfig axis_cfg = axes[selected_axis_id].Config;
                uc_axis_config.UpdateConfig(axis_cfg);
            }
        }

        public KinematicParameters GetKinematicParameters(AxisID axis_id)
        {
            if (axis_id != AxisID.AxisUndefined)
            {
                return axes[axis_id].KinematicParameters;
            }
            return null;
        }

        private void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            pedal_pos_min = parameters.ContactPointPosMinAbs / 10.0;
            pedal_pos_max = parameters.ContactPointPosMaxAbs / 10.0;
            if (selected_axis_id == AxisID.AxisUndefined) return;
            if (selected_function_id == FunctionID.Undefined) return;
            if (functions[selected_function_id].Config.Base.LinkedAxes.Count == 0) return;
            AxisID primary_axis = functions[selected_function_id].Config.Base.LinkedAxes[0];
            if (selected_axis_id == primary_axis)
            {
                uc_function_config.OnKinematicParametersChanged(parameters);
            }
        }



        /********************************************************************************************************************/
        /*							Slider callbacks																		*/
        /********************************************************************************************************************/









        /********************************************************************************************************************/
        /*							PID tuning                      														*/
        /********************************************************************************************************************/
        public void PID_tuning_P_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain = (float)e.NewValue;
        }

        public void PID_tuning_I_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain = (float)e.NewValue;
        }

        public void PID_tuning_D_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain = (float)e.NewValue;
        }

        public void PID_tuning_Feedforward_gain_changed(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain = (float)e.NewValue;
        }







        private void NumericTextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            //labelEingabe.Content = "Sie haben '" + textBox_debug_Flag_0.Text + "' eingegeben!";
            //TextBox_debugOutput.Text = textBox_debug_Flag_0.Text;

            if (int.TryParse(textBox_debug_Flag_0.Text, out int result))
            {
                if ((result >= 0) && (result <= 255))
                {
                    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)result;
                }
            }
        }
        private void NumericTextBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
        {

            //if ((e.NewValue >= 0) && (e.NewValue <= 255))
            //{
            //    dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)e.NewValue;
            //}

            // Use a regular expression to allow only numeric input
            Regex regex = new Regex("^[.][0-9]+$|^[0-9]*[.]{0,4}[0-9]*$");

            System.Windows.Controls.TextBox textBox = (System.Windows.Controls.TextBox)sender;

            e.Handled = !regex.IsMatch(textBox.Text + e.Text);

            ////if (!e.Handled)
            ////{
            ////    if (int.TryParse(textBox.Text + e.Text, out int result))
            ////    {
            ////        if ((result >= 0) && (result <= 255))
            ////        {
            ////            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.debug_flags_0 = (byte)result;
            ////        }
            ////    }
            ////}
        }


        /********************************************************************************************************************/
        /*							Write/read config to/from Json file														*/
        /********************************************************************************************************************/

        //private void ComboBox_SelectionChanged(object sender, EventArgs e)
        //{
        //    try
        //    {
        //        // https://stackoverflow.com/questions/3721430/what-is-the-simplest-way-to-get-the-selected-text-of-a-combo-box-containing-only

        //        string stringValue = ((ComboBoxItem)ComboBox_JsonFileSelected.SelectedItem).Content.ToString();


        //        // string stringValue = ComboBox_JsonFileSelected.SelectedValue.ToString();

        //        //TextBox_debugOutput.Text = stringValue;
        //        Plugin.Settings.selectedJsonFileNames[indexOfSelectedPedal_u] = stringValue;

        //        Plugin.Settings.selectedJsonIndexLast[indexOfSelectedPedal_u] = ComboBox_JsonFileSelected.SelectedIndex;



        //        //ReadStructFromJson();
        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }
        //}




        //public void SaveStructToJson_click(object sender, RoutedEventArgs e)
        //{
        //    // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/how-to?pivots=dotnet-8-0

        //    try
        //    {
        //        // which config file is seleced
        //        string currentDirectory = Directory.GetCurrentDirectory();
        //        string dirName = currentDirectory + "\\PluginsData\\Common";
        //        string jsonFileName = ComboBox_JsonFileSelected.Text;
        //        string fileName = dirName + "\\" + jsonFileName + ".json";

        //        this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)pedalConfigPayload_version;

        //        // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
        //        // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
        //        var stream1 = new MemoryStream();
        //        var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
        //        ser.WriteObject(stream1, dap_config_st[indexOfSelectedPedal_u]);

        //        stream1.Position = 0;
        //        StreamReader sr = new StreamReader(stream1);
        //        string jsonString = sr.ReadToEnd();

        //        // Check if file already exists. If yes, delete it.     
        //        if (File.Exists(fileName))
        //        {
        //            File.Delete(fileName);
        //        }


        //        System.IO.File.WriteAllText(fileName, jsonString);
        //        TextBox_debugOutput.Text = "Config exported!";

        //    }
        //    catch (Exception caughtEx)
        //    {

        //        string errorMessage = caughtEx.Message;
        //        TextBox_debugOutput.Text = errorMessage;
        //    }

        //}



        //public void ReadStructFromJson_click(object sender, RoutedEventArgs e)
        //{
        //    ReadStructFromJson();
        //}


        /********************************************************************************************************************/
        /*							Refind min endstop																		*/
        /********************************************************************************************************************/
       



    






        /********************************************************************************************************************/
        /*							Send config to pedal																	*/
        /********************************************************************************************************************/
        unsafe public void Sendconfig(uint pedalIdx, bool store=false)
        {
            // compute checksum
            //getBytes(this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_)
            this.dap_config_st[pedalIdx].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            this.dap_config_st[pedalIdx].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            this.dap_config_st[pedalIdx].payloadHeader_.PedalTag = (byte)pedalIdx;
            this.dap_config_st[pedalIdx].payloadHeader_.storeToEeprom = store ? (byte)1 : (byte)0;
            DAP_config_st tmp = this.dap_config_st[pedalIdx];
            //prevent read default config from pedal without assignement
            tmp.payloadPedalConfig_.pedal_type = (byte)pedalIdx;
            //payloadPedalConfig tmp = this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
            DAP_config_st* v = &tmp;

            byte* p = (byte*)v;
            this.dap_config_st[pedalIdx].payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalConfig));
            int length = sizeof(DAP_config_st);
            //int val = this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.checkSum;
            //string msg = "CRC value: " + val.ToString();
            byte[] newBuffer = new byte[length];
            newBuffer = getBytes(this.dap_config_st[pedalIdx]);

            //TextBox_debugOutput.Text = "CRC simhub calc: " + this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum + "    ";

            TextBox_debugOutput.Text = String.Empty;
            if (Plugin.Settings.axis_settings[pedalIdx].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    try
                    {
                        TextBox2.Text = "Buffer sent size:" + length;
                        //Plugin.ESPsync_serialPort.DiscardInBuffer();
                        //Plugin.ESPsync_serialPort.DiscardOutBuffer();
                         //send data
                        //Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                        //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                        System.Threading.Thread.Sleep(100);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }
            else
            {
                //int length2 = sizeof(DAP_config_st);
                if (Plugin._serialPort[pedalIdx].IsOpen)
                {

                    try
                    {
                        //TextBox_debugOutput.Text = "ConfigLength" + length;
                        // clear inbuffer 
                        Plugin._serialPort[pedalIdx].DiscardInBuffer();
                        Plugin._serialPort[pedalIdx].DiscardOutBuffer();
                        // send data
                        Plugin._serialPort[pedalIdx].Write(newBuffer, 0, newBuffer.Length);
                        //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }

                }
            }
        }

        unsafe public void Sendconfig_Rudder(uint pedalIdx)
        {

            this.dap_config_st_rudder.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            this.dap_config_st_rudder.payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            this.dap_config_st_rudder.payloadHeader_.PedalTag = (byte)pedalIdx;
            this.dap_config_st_rudder.payloadHeader_.storeToEeprom = 0;
            this.dap_config_st_rudder.payloadPedalConfig_.pedal_type = (byte)pedalIdx;
            DAP_config_st tmp = this.dap_config_st_rudder;

            DAP_config_st* v = &tmp;
            byte* p = (byte*)v;
            this.dap_config_st_rudder.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalConfig));
            int length = sizeof(DAP_config_st);
            //int val = this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.checkSum;
            //string msg = "CRC value: " + val.ToString();
            byte[] newBuffer = new byte[length];
            newBuffer = getBytes(this.dap_config_st_rudder);


            TextBox_debugOutput.Text = String.Empty;
            if (Plugin.Settings.axis_settings[pedalIdx].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    try
                    {
                        TextBox2.Text = "Buffer sent size:" + length;
                        //Plugin.ESPsync_serialPort.DiscardInBuffer();
                        //Plugin.ESPsync_serialPort.DiscardOutBuffer();
                        //// send data
                        //Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                        //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                        System.Threading.Thread.Sleep(100);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }

        }
        unsafe public void Sendconfigtopedal_shortcut()
        {

            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.Settings.file_enable_check[profile_select, pedalIdx] == 1)
                {
                    Sendconfig(pedalIdx);
                    TextBox_debugOutput.Text = "config was sent to pedal";
                }
            }

        }
        unsafe public void SendConfigToPedal_click(object sender, RoutedEventArgs e)
        {
            Sendconfig(indexOfSelectedPedal_u);
        }
        unsafe public void SendConfigToPedalAndStore_click(object sender, RoutedEventArgs e)
        {
            Sendconfig(indexOfSelectedPedal_u, true);
        }

        unsafe public void Reading_config_auto(uint i)
        {
            // compute checksum
            DAP_action_st tmp;
            tmp.payloadPedalAction_.returnPedalConfig_u8 = 1;
            tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
            tmp.payloadHeader_.PedalTag = (byte)i;
            DAP_action_st* v = &tmp;
            byte* p = (byte*)v;
            tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));
            int length = sizeof(DAP_action_st);
            byte[] newBuffer = new byte[length];
            newBuffer = Plugin.getBytes_Action(tmp);
            // tell the plugin that we expect config data
            waiting_for_pedal_config[i] = true;
            if (Plugin.Settings.axis_settings[i].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    // try N times and check whether config has been received
                    for (int rep = 0; rep < 1; rep++)
                    {
                        // send query command
                        //Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);

                        // wait some time and check whether data has been received
                        System.Threading.Thread.Sleep(50);

                        if (waiting_for_pedal_config[i] == false)
                        {
                            break;
                        }
                    }
                }
            }
            else
            {
                if (Plugin._serialPort[i].IsOpen)
                {
                    // try N times and check whether config has been received
                    for (int rep = 0; rep < 1; rep++)
                    {
                        // send query command
                        Plugin._serialPort[i].Write(newBuffer, 0, newBuffer.Length);

                        // wait some time and check whether data has been received
                        System.Threading.Thread.Sleep(50);

                        if (waiting_for_pedal_config[i] == false)
                        {
                            break;
                        }
                    }
                }
            }

        }

        public string[] STOPCHAR = { "\r\n" };
        public bool EndsWithStop(string incomingData)
        {
            for (int i = 0; i < STOPCHAR.Length; i++)
            {
                if (incomingData.EndsWith(STOPCHAR[i]))
                {
                    return true;
                }
            }
            return false;
        }

        /********************************************************************************************************************/
        /*							Read config from pedal																	*/
        /********************************************************************************************************************/
        unsafe public void ReadConfigFromPedal_click(object sender, RoutedEventArgs e)
        {
            Reading_config_auto(indexOfSelectedPedal_u);
        }


        public string[] _data = { "", "", "" };// = "";

        //unsafe private void sp_DataReceived(object sender, object e)
        unsafe private void sp_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            SerialPort sp = (SerialPort)sender;
            //string _type = (string)e;

            //if (Plugin._serialPort[indexOfSelectedPedal_u].PortName = sp.PortName)

            // identify which pedal has send the data
            int pedalSelected = 255;
            for (int pedalIdx_i = 0; pedalIdx_i < 3; pedalIdx_i++)
            {
                if ((Plugin._serialPort[pedalIdx_i].PortName == sp.PortName) && (Plugin._serialPort[pedalIdx_i].IsOpen))
                {
                    pedalSelected = pedalIdx_i;
                }
            }

            // once the pedal has identified, go ahead
            if (pedalSelected < 3)
            //if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {
                // https://stackoverflow.com/questions/9732709/the-calling-thread-cannot-access-this-object-because-a-different-thread-owns-it


                int length = sizeof(DAP_config_st);
                byte[] newBuffer_config = new byte[length];

                int receivedLength = sp.BytesToRead;


                string incomingData = sp.ReadExisting();
                //if the data doesn't end with a stop char this will signal to keep it in _data 
                //for appending to the following read of data
                bool endsWithStop = EndsWithStop(incomingData);

                //each array object will be sent separately to the callback
                string[] dataArray = incomingData.Split(STOPCHAR, StringSplitOptions.None);

                for (int i = 0; i < dataArray.Length; i++)
                {
                    string newData = dataArray[i];

                    //if you are at the last object in the array and this hasn't got a stopchar after
                    //it will be saved in _data
                    if (!endsWithStop && i == dataArray.Length - 1)
                    {
                        _data[pedalSelected] += newData;
                    }
                    else
                    {
                        string dataToSend = _data[pedalSelected] + newData;
                        _data[pedalSelected] = "";


                        // decode into config struct
                        if (dataToSend.Length == length)
                        {
                            DAP_config_st tmp;

                            // transform string into byte
                            fixed (byte* p = Encoding.ASCII.GetBytes(dataToSend))
                            {
                                // create a fixed size buffer
                                length = sizeof(DAP_config_st);
                                byte[] newBuffer_config_2 = new byte[length];

                                // copy the received bytes into byte array
                                for (int j = 0; j < length; j++)
                                {
                                    newBuffer_config_2[j] = p[j];
                                }

                                // parse byte array as config struct
                                DAP_config_st pedalConfig_read_st = getConfigFromBytes(newBuffer_config_2);

                                // check whether receive struct is plausible
                                DAP_config_st* v_config = &pedalConfig_read_st;
                                byte* p_config = (byte*)v_config;

                                // payload type check
                                bool check_payload_config_b = false;
                                if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                {
                                    check_payload_config_b = true;
                                }

                                // CRC check
                                bool check_crc_config_b = false;
                                if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_config_b = true;
                                }


                                // when all checks are passed, accept the config. Otherwise discard and trow error
                                Dispatcher.Invoke(
                                new Action<DAP_config_st>((t) => this.dap_config_st[pedalSelected] = t),
                                pedalConfig_read_st);


                                this.Dispatcher.Invoke(() =>
                                {
                                    // update pedal config
                                    if (check_payload_config_b)
                                    {
                                        //this.dap_config_st[indexOfSelectedPedal_u] = pedalConfig_read_st;
                                        updateTheGuiFromConfig();
                                    }

                                    TextBox_debugOutput.Text = "Payload config test 1: " + check_payload_config_b;
                                    TextBox_debugOutput.Text += "Payload config test 2: " + check_crc_config_b;
                                });

                            }

                        }
                        else
                        {
                            this.Dispatcher.Invoke(() =>
                            {
                                //TextBox_serialMonitor.Text += "DataArrayLength: " + dataArray.Length + "\n";
                                //TextBox_serialMonitor.Text += "DataLength: " + dataToSend.Length + "\n";
                                if (_serial_monitor_window != null)
                                {
                                    _serial_monitor_window.TextBox_SerialMonitor.Text += dataToSend + "\n";
                                    _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                                }
                                //TextBox_serialMonitor.Text += dataToSend + "\n";
                                //TextBox_serialMonitor.ScrollToEnd();
                                //TextBox_serialMonitor.Text += receivedLength + "\n";
                            });
                        }


                    }











                    //limits the data stored to 1000 to avoid using up all the memory in case of 
                    //failure to register callback or include stopchar

                    if (_data[pedalSelected].Length > 1000)
                    {
                        _data[pedalSelected] = "";
                    }


                    //////this.Dispatcher.Invoke(() =>
                    //////    {
                    //////        TextBox_serialMonitor.Text += incomingData;

                    //////        TextBox_serialMonitor.ScrollToEnd();
                    //////        //TextBox_serialMonitor.Text += receivedLength + "\n";
                    //////    });
                }

                // obtain data and check whether it is from known payload type or just debug info

            }
        }



        /********************************************************************************************************************/
        /*							read serial stream																		*/
        /********************************************************************************************************************/
        public void openSerialAndAddReadCallback(uint pedalIdx)
        {
            try
            {
                DiyFfbPluginSettings.AxisSettings axis_settings = Plugin.Settings.axis_settings[pedalIdx];
                // serial port settings
                Plugin._serialPort[pedalIdx].Handshake = Handshake.None;
                Plugin._serialPort[pedalIdx].Parity = Parity.None;
                //_serialPort[pedalIdx].StopBits = StopBits.None;


                Plugin._serialPort[pedalIdx].ReadTimeout = 2000;
                Plugin._serialPort[pedalIdx].WriteTimeout = 500;

                // https://stackoverflow.com/questions/7178655/serialport-encoding-how-do-i-get-8-bit-ascii
                Plugin._serialPort[pedalIdx].Encoding = System.Text.Encoding.GetEncoding(28591);

                // regular ESP
                //Plugin._serialPort[pedalIdx].DtrEnable = false;

                // ESP32 S3
                //Plugin._serialPort[pedalIdx].RtsEnable = false;
                //Plugin._serialPort[pedalIdx].DtrEnable = true;


                Plugin._serialPort[pedalIdx].NewLine = "\r\n";
                Plugin._serialPort[pedalIdx].ReadBufferSize = 10000;
                Plugin._serialPort[pedalIdx].PortName = axis_settings.com_port_name;

                if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                {
                    try
                    {
                        Plugin._serialPort[pedalIdx].Open();

                        // ESP32 S3
                        if (Plugin.Settings.axis_settings[pedalIdx].RTSDTR_False == true)
                        {
                            Plugin._serialPort[pedalIdx].RtsEnable = false;
                            Plugin._serialPort[pedalIdx].DtrEnable = false;
                        }
                        //

                        if (Plugin.Settings.axis_settings[pedalIdx].USING_ESP32S3 == true)
                        {
                            // ESP32 S3
                            Plugin._serialPort[pedalIdx].RtsEnable = false;
                            Plugin._serialPort[pedalIdx].DtrEnable = true;
                        }
                        
                        

                        System.Threading.Thread.Sleep(200);

                       
                        // read callback
                        if (pedal_serial_read_timer[pedalIdx] != null)
                        {
                            pedal_serial_read_timer[pedalIdx].Stop();
                            pedal_serial_read_timer[pedalIdx].Dispose();
                        }
                        pedal_serial_read_timer[pedalIdx] = new System.Windows.Forms.Timer();
                        pedal_serial_read_timer[pedalIdx].Tick += new EventHandler(timerCallback_serial);
                        pedal_serial_read_timer[pedalIdx].Tag = pedalIdx;
                        pedal_serial_read_timer[pedalIdx].Interval = 16; // in miliseconds
                        pedal_serial_read_timer[pedalIdx].Start();
                        System.Threading.Thread.Sleep(100);
                        Serial_connect_status[pedalIdx] = true;
                    }
                    catch(Exception ex)
                    { 
                        TextBox2.Text = ex.Message;
                        Serial_connect_status[pedalIdx] = false;
                    }
                    

                }
                else
                {
                    Plugin.connectSerialPort[pedalIdx] = false;
                    Serial_connect_status[pedalIdx] = false;

                }
            }
            catch (Exception ex)
            { }




        }
        private uint count_timmer_count = 0;
        private string Toast_tmp;
        public void try_connect()
        {
            //simhub action for debug
            Simhub_action_update();

            if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag)
            {
                if (Plugin.PortExists(Plugin.Settings.ESPNow_port))
                {
                    if (Plugin.ESPsync_serialPort.IsOpen == false)
                    {
                        Plugin.ESPsync_serialPort = new ProtobufSerial<Message>(Plugin.Settings.ESPNow_port, 3000000);
                        try
                        {
                            // serial port settings
                            //Plugin.ESPsync_serialPort.Handshake = Handshake.None;
                            //Plugin.ESPsync_serialPort.Parity = Parity.None;
                            ////_serialPort[pedalIdx].StopBits = StopBits.None;
                            //Plugin.ESPsync_serialPort.ReadTimeout = 2000;
                            //Plugin.ESPsync_serialPort.WriteTimeout = 500;
                            //Plugin.ESPsync_serialPort.BaudRate = Bridge_baudrate;
                            //// https://stackoverflow.com/questions/7178655/serialport-encoding-how-do-i-get-8-bit-ascii
                            //Plugin.ESPsync_serialPort.Encoding = System.Text.Encoding.GetEncoding(28591);
                            //Plugin.ESPsync_serialPort.NewLine = "\r\n";
                            //Plugin.ESPsync_serialPort.ReadBufferSize = 40960;
                            try
                            {
                                Plugin.ESPsync_serialPort.Open();
                                System.Threading.Thread.Sleep(200);
                                // ESP32 S3
                                if (Plugin.Settings.Using_CDC_bridge)
                                {
                                    Plugin.ESPsync_serialPort.RtsEnable = false;
                                    Plugin.ESPsync_serialPort.DtrEnable = true;
                                }
                                //SystemSounds.Beep.Play();
                                Plugin.Sync_esp_connection_flag = true;
                                btn_connect_espnow_port.Content = "Disconnect";
                                Plugin.ESPsync_serialPort.OnMessage += OnMessage;
                                ToastNotification("Pedal Wireless Bridge", "Connected");
                                updateTheGuiFromConfig();
                            }
                            catch (Exception ex)
                            {
                                TextBox2.Text = ex.Message;
                                //Serial_connect_status[3] = false;
                            }
                        }
                        catch (Exception ex)
                        {
                            TextBox2.Text = ex.Message;
                        }
                    }
                }
                else
                {
                    if (Plugin.Sync_esp_connection_flag)
                    {
                        Plugin.Sync_esp_connection_flag = false;
                        dap_bridge_state_st.payloadBridgeState_.Pedal_availability_0 = 0;
                        dap_bridge_state_st.payloadBridgeState_.Pedal_availability_1 = 0;
                        dap_bridge_state_st.payloadBridgeState_.Pedal_availability_2 = 0;
                    }

                    btn_connect_espnow_port.Content = "Connect";
                    if (ESP_host_serial_timer_cts != null)
                    {
                        ESP_host_serial_timer_cts.Cancel();
                        updateTheGuiFromConfig();
                    }

                }

            }

            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.Settings.axis_settings[pedalIdx].auto_connect)
                {

                    if (Plugin.PortExists(Plugin._serialPort[pedalIdx].PortName))
                    {
                        if (Plugin._serialPort[pedalIdx].IsOpen == false)
                        {
                            //UpdateSerialPortList_click();
                            openSerialAndAddReadCallback(pedalIdx);
                            //Plugin.Settings.autoconnectComPortNames[pedalIdx] = Plugin._serialPort[pedalIdx].PortName;
                            System.Threading.Thread.Sleep(200);
                            if (Serial_connect_status[pedalIdx])
                            {
                                if (Plugin.Settings.reading_config == 1)
                                {
                                    Reading_config_auto(pedalIdx);
                                }
                                System.Threading.Thread.Sleep(100);
                                //add toast notificaiton
                                switch (pedalIdx)
                                {
                                    case 0:
                                        Toast_tmp = "Clutch Pedal:" + Plugin.Settings.axis_settings[pedalIdx].com_port_name;
                                        break;
                                    case 1:
                                        Toast_tmp = "Brake Pedal:" + Plugin.Settings.axis_settings[pedalIdx].com_port_name;
                                        break;
                                    case 2:
                                        Toast_tmp = "Throttle Pedal:" + Plugin.Settings.axis_settings[pedalIdx].com_port_name;
                                        break;
                                }
                                ToastNotification(Toast_tmp, "Connected");
                                updateTheGuiFromConfig();
                                //System.Threading.Thread.Sleep(2000);
                                //ToastNotificationManager.History.Clear("FFB Pedal Dashboard");
                            }



                        }
                    }
                    else
                    {
                        Plugin.connectSerialPort[pedalIdx] = false;
                        updateTheGuiFromConfig();
                    }
                }
            }
            Task.Delay(500).ContinueWith(t => this.Dispatcher.Invoke(() => try_connect()));
        }

        public void CloseSerialPorts()
        {
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                //Plugin.ESPsync_serialPort.DiscardInBuffer();
                //Plugin.ESPsync_serialPort.DiscardOutBuffer();
                Plugin.ESPsync_serialPort.OnMessage -= OnMessage;
                Plugin.ESPsync_serialPort.Close();
                Plugin.Sync_esp_connection_flag = false;
            }
        }

        public void closeSerialAndStopReadCallback(uint pedalIdx)
        {
            
            if (pedal_serial_read_timer[pedalIdx] != null)
            {
                pedal_serial_read_timer[pedalIdx].Stop();
                pedal_serial_read_timer[pedalIdx].Dispose();
            }
            if (ESP_host_serial_timer_cts != null)
            {
                ESP_host_serial_timer_cts.Cancel();

            }
            System.Threading.Thread.Sleep(300);
            
            
            if (Plugin._serialPort[pedalIdx].IsOpen)
            {
                // ESP32 S3
                if (Plugin.Settings.axis_settings[pedalIdx].RTSDTR_False == true)
                {
                    Plugin._serialPort[pedalIdx].RtsEnable = false;
                    Plugin._serialPort[pedalIdx].DtrEnable = false;
                }


                Plugin._serialPort[pedalIdx].DiscardInBuffer();
                Plugin._serialPort[pedalIdx].DiscardOutBuffer();
                Plugin._serialPort[pedalIdx].Close();
            }
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                //Plugin.ESPsync_serialPort.DiscardInBuffer();
                //Plugin.ESPsync_serialPort.DiscardOutBuffer();
                Plugin.ESPsync_serialPort.OnMessage -= OnMessage;
                Plugin.ESPsync_serialPort.Close();
                Plugin.Sync_esp_connection_flag = false;
            }
        }

        Int64 writeCntr = 0;

        int[] timeCntr = { 0, 0, 0,0 };

        double[] timeCollector = { 0, 0, 0,0 };


        static List<int> FindAllOccurrences(byte[] source, byte[] sequence, int maxLength)
        {
            List<int> indices = new List<int>();

            int len = source.Length - sequence.Length;
            if (len > maxLength)
            {
                len = maxLength;
            }

            for (int i = 0; i <= len; i++)
            {
                bool found = true;
                for (int j = 0; j < sequence.Length; j++)
                {
                    if (source[i + j] != sequence[j])
                    {
                        found = false;
                        break;
                    }
                }
                if (found)
                {
                    indices.Add(i); // Sequence found, add index to the list
                }
            }



            //int i = 0;
            //while (i < len)
            //{
            //    bool found = true;
            //    for (int j = 0; j < sequence.Length; j++)
            //    {
            //        if (source[i + j] != sequence[j])
            //        {
            //            found = false;
            //            break;
            //        }
            //    }
            //    if (found)
            //    {
            //        indices.Add(i); // Sequence found, add index to the list
            //        i += sequence.Length;
            //    }
            //    else { i++; } 
            //}



            return indices;
        }

        public void Simhub_action_update()
        {
            if (Plugin.Page_update_flag == true)
            {
                Profile_change(Plugin.profile_index);
                Plugin.Page_update_flag = false;
                tc_function_selection.SelectedIndex = (int)Plugin.Settings.function_tab_selected;
                Plugin.pedal_select_update_flag = false;
                Plugin.simhub_theme_color = defaultcolor.ToString();
                switch (Plugin.Settings.function_tab_selected)
                {
                    case 0:
                        Plugin.current_pedal = "Clutch";
                        break;
                    case 1:
                        Plugin.current_pedal = "Brake";
                        break;
                    case 2:
                        Plugin.current_pedal = "Throttle";
                        break;
                }
                updateTheGuiFromConfig();
            }

            if (Plugin.sendconfig_flag == 1)
            {
                Sendconfigtopedal_shortcut();
                Plugin.sendconfig_flag = 0;
            }
        }

        int[] appendedBufferOffset = { 0, 0, 0,0 };

        static int bufferSize = 10000;
        static int destBufferSize = 1000;
        byte[][] buffer_appended = { new byte[bufferSize], new byte[bufferSize], new byte[bufferSize], new byte[bufferSize] };

        unsafe public void timerCallback_serial(object sender, EventArgs e)
        {

            //action here 
            Simhub_action_update();
            
            


            int pedalSelected = Int32.Parse((sender as System.Windows.Forms.Timer).Tag.ToString());
            //int pedalSelected = (int)(sender as System.Windows.Forms.Timer).Tag;

            bool pedalStateHasAlreadyBeenUpdated_b = false;

            // once the pedal has identified, go ahead
            if (pedalSelected < 3)
            //if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
            {



                // Create a Stopwatch instance
                Stopwatch stopwatch = new Stopwatch();

                // Start the stopwatch
                stopwatch.Start();



                SerialPort sp = Plugin._serialPort[pedalSelected];



                // https://stackoverflow.com/questions/9732709/the-calling-thread-cannot-access-this-object-because-a-different-thread-owns-it


                //int length = sizeof(DAP_config_st);




                if (sp.IsOpen)
                {
                    if (Plugin.Settings.Serial_auto_clean)
                    {
                        /*
                        if (TextBox_serialMonitor.LineCount > 300)
                        {
                            TextBox_serialMonitor.Clear();
                        }
                        */
                        if (_serial_monitor_window != null && _serial_monitor_window.TextBox_SerialMonitor.LineCount > 300)
                        {
                            _serial_monitor_window.TextBox_SerialMonitor.Clear();
                        }
                    }

                    int receivedLength = 0;
                    try 
                    {
                        receivedLength = sp.BytesToRead;
                    }
                    catch (Exception ex)
                    {
                        TextBox_debugOutput.Text = ex.Message;
                        //ConnectToPedal.IsChecked = false;
                        return;
                    }

                

                    if (receivedLength > 0)
                    {

                        //TextBox_serialMonitor.Text += "Received:" + receivedLength + "\n";
                        //TextBox_serialMonitor.ScrollToEnd();


                        timeCntr[pedalSelected] += 1;


                        // determine byte sequence which is defined as message end --> crlf
                        byte[] byteToFind = System.Text.Encoding.GetEncoding(28591).GetBytes(STOPCHAR[0].ToCharArray());
                        int stop_char_length = byteToFind.Length;


                        // calculate current buffer length
                        int currentBufferLength = appendedBufferOffset[pedalSelected] + receivedLength;


                        // check if buffer is large enough otherwise discard in buffer and set offset to 0
                        if ((bufferSize > currentBufferLength) && (appendedBufferOffset[pedalSelected] >= 0))
                        {
                            sp.Read(buffer_appended[pedalSelected], appendedBufferOffset[pedalSelected], receivedLength);
                        }
                        else
                        {
                            sp.DiscardInBuffer();
                            appendedBufferOffset[pedalSelected] = 0;
                            return;
                        }


                        


                        // copy to local buffer
                        //byte[] localBuffer = new byte[currentBufferLength];
                        
                        //Buffer.BlockCopy(buffer_appended[pedalSelected], 0, localBuffer, 0, currentBufferLength);


                        // find all occurences of crlf as they indicate message end
                        List<int> indices = FindAllOccurrences(buffer_appended[pedalSelected], byteToFind, currentBufferLength);




                        // Destination array
                        byte[] destinationArray = new byte[destBufferSize];

                        





                        int srcBufferOffset = 0;
                        // decode every message
                        //foreach (int number in indices)
                        for (int msgId = 0; msgId < indices.Count; msgId++)
                        {
                            // computes the length of bytes to read
                            int destBuffLength = 0; //number - srcBufferOffset;

                            if (msgId == 0)
                            {
                                srcBufferOffset = 0;
                                destBuffLength = indices.ElementAt(msgId);
                            }
                            else 
                            {
                                srcBufferOffset = indices.ElementAt(msgId - 1) + stop_char_length;
                                destBuffLength = indices.ElementAt(msgId) - srcBufferOffset;
                            }

                            // check if dest buffer length is within valid length
                            if ( (destBuffLength <= 0) | (destBuffLength > destBufferSize) )
                            {
                                continue;
                            }


                 


                            // copy bytes to subarray
                            Buffer.BlockCopy(buffer_appended[pedalSelected], srcBufferOffset, destinationArray, 0, destBuffLength);


                            // check for pedal state struct
                            if ((destBuffLength == sizeof(DAP_state_basic_st)))
                            {

                                // parse byte array as config struct
                                DAP_state_basic_st pedalState_read_st = getStateFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_state_basic_st* v_state = &pedalState_read_st;
                                byte* p_state = (byte*)v_state;
                                
                                // payload type check
                                bool check_payload_state_b = false;
                                if (pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                {
                                    check_payload_state_b = true;
                                }

                                //Pedal version and Plugin DAP version check
                                Pedal_version[pedalSelected] = pedalState_read_st.payloadHeader_.version;
                                if (Pedal_version[pedalSelected] != Constants.pedalConfigPayload_version && pedalState_read_st.payloadHeader_.payloadType == Constants.pedalStateBasicPayload_type)
                                {
                                    if (!Version_warning_first_show_b[pedalSelected])
                                    {
                                        Version_warning_first_show_b[pedalSelected] = true;
                                        if (Pedal_version[pedalSelected] > Constants.pedalConfigPayload_version)
                                        {
                                            String MSG_tmp;
                                            MSG_tmp = "Pedal: " + pedalState_read_st.payloadHeader_.PedalTag + " Pedal Dap version: " + Pedal_version[pedalSelected] + ", Plugin DAP version: " + Constants.pedalConfigPayload_version + ". Please update Simhub Plugin.";
                                            System.Windows.MessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                                        }
                                        else
                                        {
                                            String MSG_tmp;
                                            MSG_tmp = "Pedal: " + pedalState_read_st.payloadHeader_.PedalTag + " Pedal Dap version: " + Pedal_version[pedalSelected] + ", Plugin DAP version: " + Constants.pedalConfigPayload_version + ". Please update Pedal Firmware.";
                                            System.Windows.MessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                                        }
                                    }
                                }


                                // CRC check
                                bool check_crc_state_b = false;
                                if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Basic)) == pedalState_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_state_b = true;
                                }

                                if ((check_payload_state_b) && check_crc_state_b)
                                {

                                    // write vJoy data
                                    Pedal_position_reading[pedalSelected] = pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16;
                                    //if (Plugin.Rudder_enable_flag == false)
                                    //{
                                        if (Plugin.Settings.vjoy_output_flag == 1)
                                        {
                                            switch (pedalSelected)
                                            {

                                                case 0:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RX);  // Center X axis
                                                    joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);   // HID_USAGES Enums
                                                    break;
                                                case 1:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RY);  // Center X axis
                                                    joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);   // HID_USAGES Enums
                                                    break;
                                                case 2:
                                                    //joystick.SetJoystickAxis(pedalState_read_st.payloadPedalState_.joystickOutput_u16, Axis.HID_USAGE_RZ);  // Center X axis
                                                    joystick.SetAxis(pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);   // HID_USAGES Enums
                                                    break;
                                                default:
                                                    break;
                                            }

                                        }
                                   



                                    // GUI update
                                    if ((pedalStateHasAlreadyBeenUpdated_b == false) && (indexOfSelectedPedal_u == pedalSelected))
                                    {
                                        //TextBox_debugOutput.Text = "Pedal pos: " + pedalState_read_st.payloadPedalState_.pedalPosition_u16;
                                        //TextBox_debugOutput.Text += "Pedal force: " + pedalState_read_st.payloadPedalState_.pedalForce_u16;
                                        //TextBox_debugOutput.Text += ",  Servo pos targe: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;
                                        //TextBox_debugOutput.Text += ",  Servo pos: " + pedalState_read_st.payloadPedalState_.servoPosition_i16;



                                        pedalStateHasAlreadyBeenUpdated_b = true;

                                        //text_point_pos.Visibility = Visibility.Hidden;
                                        //double control_rect_value_max = 65535;
                                        //double dyy = canvas.Height / control_rect_value_max;
                                        //double dxx = canvas.Width / control_rect_value_max;

                                        //if (debug_flag)
                                        //{
                                        //    Canvas.SetLeft(rect_State, dxx * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 - rect_State.Width / 2 );
                                        //    Canvas.SetTop(rect_State, canvas.Height - dyy * pedalState_read_st.payloadPedalBasicState_.pedalForce_u16 - rect_State.Height / 2);

                                        //    Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
                                        //    Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
                                        //    text_state.Text = Math.Round(pedalState_read_st.payloadPedalBasicState_.pedalForce_u16 / control_rect_value_max * 100) + "%";
                                        //    int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                        //    int x_showed = round_x + 1;
                                            
                                        //    current_pedal_travel_state = x_showed;
                                        //    Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;
                                        //}
                                        //else
                                        //{
                                        //    Canvas.SetLeft(rect_State, dxx * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 - rect_State.Width / 2 );
                                        //    int round_x = (int)(100 * pedalState_read_st.payloadPedalBasicState_.pedalPosition_u16 / control_rect_value_max) - 1;
                                        //    int x_showed = round_x + 1;
                                        //    round_x = Math.Max(0, Math.Min(round_x, 99));
                                        //    current_pedal_travel_state = x_showed;
                                        //    Plugin.pedal_state_in_ratio = (byte)current_pedal_travel_state;
                                        //    Canvas.SetTop(rect_State, canvas.Height - Force_curve_Y[round_x] - rect_State.Height / 2);
                                        //    Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
                                        //    Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
                                        //    text_state.Text = x_showed + "%";
                                        //    Pedal_joint_draw();
                                        //}

                                    }


                                    continue;
                                }

                            }






                            // check for pedal extended state struct
                            if ((destBuffLength == sizeof(DAP_state_extended_st)))
                            {

                                // parse byte array as config struct
                                DAP_state_extended_st pedalState_ext_read_st = getStateExtFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_state_extended_st* v_state = &pedalState_ext_read_st;
                                byte* p_state = (byte*)v_state;

                                // payload type check
                                bool check_payload_state_b = false;
                                if (pedalState_ext_read_st.payloadHeader_.payloadType == Constants.pedalStateExtendedPayload_type)
                                {
                                    check_payload_state_b = true;
                                }

                                // CRC check
                                bool check_crc_state_b = false;
                                if (Plugin.checksumCalc(p_state, sizeof(payloadHeader) + sizeof(payloadPedalState_Extended)) == pedalState_ext_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_state_b = true;
                                }

                                if ((check_payload_state_b) && check_crc_state_b)
                                {




                                    if (indexOfSelectedPedal_u == pedalSelected)
                                    {
                                        if (dumpPedalToResponseFile[indexOfSelectedPedal_u])
                                        {
                                            // Specify the path to the file
                                            string currentDirectory = Directory.GetCurrentDirectory();
                                            string filePath = currentDirectory + "\\PluginsData\\Common" + "\\DiyFfbPedalStateLog_" + indexOfSelectedPedal_u.ToString() + ".txt";


                                            // delete file 
                                            if (true == dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u])
                                            {
                                                dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u] = false;
                                                File.Delete(filePath);
                                            }


                                            // write header
                                            if (!File.Exists(filePath))
                                            {
                                                using (StreamWriter writer = new StreamWriter(filePath, true))
                                                {
                                                    // Write the content to the file
                                                    writer.Write("cycleCtr, ");
                                                    writer.Write("time_InMs, ");
                                                    writer.Write("forceRaw_InKg, ");
                                                    writer.Write("forceFiltered_InKg, ");
                                                    writer.Write("forceVelocity_InKgPerSec, ");
                                                    writer.Write("servoPos_InSteps, ");
                                                    writer.Write("servoPosEsp_InSteps, ");
                                                    writer.Write("servoCurrent_InPercent, ");
                                                    writer.Write("servoVoltage_InV");
                                                    writer.Write("\n");
                                                }

                                            }


                                            // Use StreamWriter to write to the file
                                            using (StreamWriter writer = new StreamWriter(filePath, true))
                                            {
                                                // Write the content to the file
                                                writeCntr++;
                                                writer.Write(writeCntr);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.timeInMs_u32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_raw_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_filtered_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.forceVel_est_fl32);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPosition_i16);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionTarget_i16);
                                                writer.Write(", ");
                                                writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_current_percent_i16);
                                                writer.Write(", ");
                                                writer.Write(((float)pedalState_ext_read_st.payloadPedalExtendedState_.servo_voltage_0p1V_i16) / 10.0);
                                                writer.Write("\n");
                                            }
                                        }
                                    }




                                    continue;
                                }
                            }








                            // decode into config struct
                            if ((waiting_for_pedal_config[pedalSelected]) && (destBuffLength == sizeof(DAP_config_st)))
                            {

                                // parse byte array as config struct
                                DAP_config_st pedalConfig_read_st = getConfigFromBytes(destinationArray);

                                // check whether receive struct is plausible
                                DAP_config_st* v_config = &pedalConfig_read_st;
                                byte* p_config = (byte*)v_config;

                                // payload type check
                                bool check_payload_config_b = false;
                                if (pedalConfig_read_st.payloadHeader_.payloadType == Constants.pedalConfigPayload_type)
                                {
                                    check_payload_config_b = true;
                                }

                                // CRC check
                                bool check_crc_config_b = false;
                                if (Plugin.checksumCalc(p_config, sizeof(payloadHeader) + sizeof(payloadPedalConfig)) == pedalConfig_read_st.payloadFooter_.checkSum)
                                {
                                    check_crc_config_b = true;
                                }

                                if ((check_payload_config_b) && check_crc_config_b)
                                {
                                    waiting_for_pedal_config[pedalSelected] = false;
                                    dap_config_st[pedalSelected] = pedalConfig_read_st;
                                    updateTheGuiFromConfig();

                                    continue;
                                }
                                else
                                {
                                    TextBox_debugOutput.Text = "Payload config test 1: " + check_payload_config_b;
                                    TextBox_debugOutput.Text += "Payload config test 2: " + check_crc_config_b;
                                }

                            }


                            // If non known array datatype was received, assume a text message was received and print it
                            // only print debug messages when debug mode is active as it degrades performance
                            if (/*Debug_check.IsChecked == true|| */_serial_monitor_window != null)
                            {
                                byte[] destinationArray_sub = new byte[destBuffLength];
                                Buffer.BlockCopy(destinationArray, 0, destinationArray_sub, 0, destBuffLength);
                                string resultString = Encoding.GetEncoding(28591).GetString(destinationArray_sub);
                                if (_serial_monitor_window != null)
                                {
                                    _serial_monitor_window.TextBox_SerialMonitor.Text += resultString + "\n";
                                    _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                                }
                                /*
                                TextBox_serialMonitor.Text += resultString + "\n";
                                TextBox_serialMonitor.ScrollToEnd();
                                */
                            }

                            




                            // When only a few messages are received, make the counter greater than N thus every message is printed
                            //if (destBuffLength < 100)
                            //{
                            //    printCtr = 600;
                            //}

                            //if (printCtr++ > 200)
                            //{
                            //    printCtr = 0;
                            //    TextBox_serialMonitor.Text += dataToSend + "\n";
                            //    TextBox_serialMonitor.ScrollToEnd();
                            //}





                        }







                        // copy the last not finished buffer element to begining of next cycles buffer
                        // and determine buffer offset
                        if (indices.Count > 0)
                        {
                            // If at least one crlf was detected, check whether it arrieved at the last bytes
                            int lastElement = indices.Last<int>();
                            int remainingMessageLength = currentBufferLength - (lastElement + stop_char_length);
                            if (remainingMessageLength > 0)
                            {
                                appendedBufferOffset[pedalSelected] = remainingMessageLength;

                                Buffer.BlockCopy(buffer_appended[pedalSelected], lastElement + stop_char_length, buffer_appended[pedalSelected], 0, remainingMessageLength);
                            }
                            else
                            {
                                appendedBufferOffset[pedalSelected] = 0;
                            }
                        }
                        else
                        {
                            appendedBufferOffset[pedalSelected] += receivedLength;
                        }






                        // Stop the stopwatch
                        stopwatch.Stop();

                        // Get the elapsed time
                        TimeSpan elapsedTime = stopwatch.Elapsed;

                        timeCollector[pedalSelected] += elapsedTime.TotalMilliseconds;

                        if (timeCntr[pedalSelected] >= 50)
                        {


                            double avgTime = timeCollector[pedalSelected] / timeCntr[pedalSelected];
                            if (debug_flag)
                            {
                                TextBox_debugOutput.Text = "Serial callback time in ms: " + avgTime.ToString();
                            }
                            timeCntr[pedalSelected] = 0;
                            timeCollector[pedalSelected] = 0;
                        }
                    }

                }
            }
        }



        /********************************************************************************************************************/
        /*							Connect to pedal																		*/
        /********************************************************************************************************************/


        unsafe public void ConnectToPedal_click(object sender, RoutedEventArgs e)
        {

            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedal_type = (byte)indexOfSelectedPedal_u;
            if (ConnectToPedal.IsChecked == false)
            {
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == false)
                {
                    try
                    {
                        openSerialAndAddReadCallback(indexOfSelectedPedal_u);
                        TextBox_debugOutput.Text = "Serialport open";
                        ConnectToPedal.IsChecked = true;
                        btn_pedal_connect.Content = "Disconnect From Pedal";

                        // register a callback that is triggered when serial data is received
                        // see https://gist.github.com/mini-emmy/9617732
                        //Plugin._serialPort[indexOfSelectedPedal_u].DataReceived += new SerialDataReceivedEventHandler(sp_DataReceived);

                        System.Threading.Thread.Sleep(100);


                    }
                    catch (Exception ex)
                    {
                        TextBox_debugOutput.Text = ex.Message;
                        ConnectToPedal.IsChecked = false;
                    }

                }
                else
                {
                    closeSerialAndStopReadCallback(indexOfSelectedPedal_u);

                    //Plugin._serialPort[indexOfSelectedPedal_u].DataReceived -= sp_DataReceived;

                    ConnectToPedal.IsChecked = false;
                    TextBox_debugOutput.Text = "Serialport already open, close it";
                    Plugin.connectSerialPort[indexOfSelectedPedal_u] = false;
                    btn_pedal_connect.Content = "Connect To Pedal";
                }
            }
            else
            {
                ConnectToPedal.IsChecked = false;
                closeSerialAndStopReadCallback(indexOfSelectedPedal_u);
                TextBox_debugOutput.Text = "Serialport close";
                Plugin.connectSerialPort[indexOfSelectedPedal_u] = false;
                btn_pedal_connect.Content = "Connect To Pedal";

            }

            ////reading config from pedal

            if (checkbox_pedal_read.IsChecked == true)
            {
                Reading_config_auto(indexOfSelectedPedal_u);
            }
            updateTheGuiFromConfig();
        }

        /********************************************************************************************************************/
        /*							Serial port selection																	*/
        /********************************************************************************************************************/
        public void UpdateSerialPortList_click(object sender, RoutedEventArgs e)
        {
            UpdateSerialPortList_click();
        }

        public void SerialPortSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            string tmp = (string)SerialPortSelection.SelectedValue;
            //Plugin._serialPort[indexOfSelectedPedal_u].PortName = tmp;


            //try 
            //{
            //    TextBox_debugOutput.Text = "Debug: " + Plugin.Settings.selectedComPortNames[indexOfSelectedPedal_u];
            //}
            //catch (Exception caughtEx)
            //{
            //    string errorMessage = caughtEx.Message;
            //    TextBox_debugOutput.Text = errorMessage;
            //}

            try
            {
                //if (Plugin.Settings.connect_status[indexOfSelectedPedal_u] == 0)
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen == false)
                {
                    Plugin.Settings.axis_settings[indexOfSelectedPedal_u].com_port_name = tmp;
                    Plugin._serialPort[indexOfSelectedPedal_u].PortName = tmp;
                }
                TextBox_debugOutput.Text = "COM port selected: " + Plugin.Settings.axis_settings[indexOfSelectedPedal_u].com_port_name;

            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }



        }











        unsafe private void RestartPedal_click(object sender, RoutedEventArgs e)
        {
            Plugin._serialPort[indexOfSelectedPedal_u].DtrEnable = true;
            Plugin._serialPort[indexOfSelectedPedal_u].RtsEnable = true;
            System.Threading.Thread.Sleep(100);
            Plugin._serialPort[indexOfSelectedPedal_u].DtrEnable = false;
            Plugin._serialPort[indexOfSelectedPedal_u].RtsEnable = false;
            if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    try
                    {
                        // compute checksum
                        DAP_action_st tmp;
                        tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
                        tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
                        tmp.payloadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
                        tmp.payloadPedalAction_.system_action_u8 = 2; //1=reset pedal position, 2 =restart esp.

                        DAP_action_st* v = &tmp;
                        byte* p = (byte*)v;
                        tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));
                        int length = sizeof(DAP_action_st);
                        byte[] newBuffer = new byte[length];
                        newBuffer = Plugin.getBytes_Action(tmp);
                        // clear inbuffer 
                        //Plugin.ESPsync_serialPort.DiscardInBuffer();

                        // send query command
                        //Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }
            
        }

        public void Read_for_slot(object sender, EventArgs e)
        {
            var Button = sender as SHButtonPrimary;
            
            using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
            {
                openFileDialog.Title = "Datei auswählen";
                openFileDialog.Filter = "Configdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                openFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    string content = (string)openFileDialog.FileName;


                    string filePath = openFileDialog.FileName;
                    //TextBox_debugOutput.Text =  Button.Name;
                    //
                    uint i = profile_select;
                    uint j = 0;
                    if (Button.Name == "Reading_clutch")
                    {

                        Plugin.Settings.Pedal_file_string[profile_select,0] = filePath;
                        Label_clutch_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 0];
                        Plugin.Settings.file_enable_check[profile_select, 0] = 1;
                        Clutch_file_check.IsChecked = true;                       
                        j = 0;                        
                    }
                    if (Button.Name == "Reading_brake")
                    {
                        Plugin.Settings.Pedal_file_string[profile_select, 1] = filePath;
                        Label_brake_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 1];
                        Plugin.Settings.file_enable_check[profile_select, 1] = 1;
                        Brake_file_check.IsChecked = true;
                        j = 1;
                    }
                    if (Button.Name == "Reading_gas")
                    {
                        Plugin.Settings.Pedal_file_string[profile_select, 2] = filePath;
                        Label_gas_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 2];
                        Plugin.Settings.file_enable_check[profile_select, 2] = 1;
                        Gas_file_check.IsChecked = true;
                        j = 2;
                    }

                    //write to setting
                    for (int k = 0; k < 8; k++)
                    {
                        if (Effect_status_profile[j, k].IsChecked == true)
                        {
                            Plugin.Settings.function_settings[j].effect_status_profiles[i, k] = true;
                        }
                        else
                        {
                            Plugin.Settings.function_settings[j].effect_status_profiles[i, k] = false;
                        }

                    }


                }
            }
        }

        public void Clear_slot(object sender, EventArgs e)
        {
            var Button = sender as SHButtonPrimary;
            uint i = profile_select;
            uint j = 0;
            if (Button.Name == "Clear_clutch")
            {

                Plugin.Settings.Pedal_file_string[profile_select, 0] = "";
                Label_clutch_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 0];
                Plugin.Settings.file_enable_check[profile_select, 0] = 0;
                Clutch_file_check.IsChecked = false;
                j = 0;

            }
            if (Button.Name == "Clear_brake")
            {

                Plugin.Settings.Pedal_file_string[profile_select, 1] = "";
                Label_brake_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 1];
                Plugin.Settings.file_enable_check[profile_select, 1] = 0;
                Brake_file_check.IsChecked = false;
                j = 1;

            }
            if (Button.Name == "Clear_gas")
            {

                Plugin.Settings.Pedal_file_string[profile_select, 2] = "";
                Label_gas_file.Content = Plugin.Settings.Pedal_file_string[profile_select, 2];
                Plugin.Settings.file_enable_check[profile_select, 2] = 0;
                Gas_file_check.IsChecked = false;
                j = 2;

            }
            //write to setting
            for (int k = 0; k < 8; k++)
            {
                Plugin.Settings.function_settings[j].effect_status_profiles[i, k] = false;
                Effect_status_profile[j, k].IsChecked = false;
            }
            //updateTheGuiFromConfig();
        }
        void Parsefile(uint profile_index)
        {
            // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/deserialization


            // c# code to iterate over all fields of struct and set values from json file
            for (uint pedalIdx = 0; pedalIdx < 3; pedalIdx++)
            {
                if (Plugin.Settings.file_enable_check[profile_select, pedalIdx] == 1)
                {
                    payloadPedalConfig payloadPedalConfig_fromJson_st = dap_config_st[pedalIdx].payloadPedalConfig_;
                    // Read the entire JSON file
                    string jsonString = File.ReadAllText(Plugin.Settings.Pedal_file_string[profile_index, pedalIdx]);
                    // Parse all of the JSON.
                    //JsonNode forecastNode = JsonNode.Parse(jsonString);
                    dynamic data = JsonConvert.DeserializeObject(jsonString);
                    //var s = default(payloadPedalConfig);
                    Object obj = payloadPedalConfig_fromJson_st;// s;
                    FieldInfo[] fi = payloadPedalConfig_fromJson_st.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance);
                    // Iterate over each field and print its name and value
                    foreach (var field in fi)
                    {

                        if (data["payloadPedalConfig_"][field.Name] != null)
                        //if (forecastNode["payloadPedalConfig_"][field.Name] != null)
                        {
                            try
                            {
                                if (field.FieldType == typeof(float))
                                {
                                    //float value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<float>();
                                    float value = (float)data["payloadPedalConfig_"][field.Name];
                                    field.SetValue(obj, value);
                                }

                                if (field.FieldType == typeof(byte))
                                {
                                    //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                    byte value = (byte)data["payloadPedalConfig_"][field.Name];
                                    field.SetValue(obj, value);
                                }
                                if (field.FieldType == typeof(Int16))
                                {
                                    //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                    Int16 value = (Int16)data["payloadPedalConfig_"][field.Name];
                                    field.SetValue(obj, value);
                                }

                            }
                            catch (Exception)
                            {

                            }

                        }
                    }

                    // set values in global structure
                    dap_config_st[pedalIdx].payloadPedalConfig_ = (payloadPedalConfig)obj;// payloadPedalConfig_fromJson_st;
                    if (dap_config_st[pedalIdx].payloadPedalConfig_.spindlePitch_mmPerRev_u8 == 0)
                    {
                        dap_config_st[pedalIdx].payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
                    }
                    if (dap_config_st[pedalIdx].payloadPedalConfig_.kf_modelNoise == 0)
                    {
                        dap_config_st[pedalIdx].payloadPedalConfig_.kf_modelNoise = 5;
                    }
                    dap_config_st[pedalIdx].payloadPedalConfig_.pedal_type = (byte)pedalIdx;

                }
                
            }
            


            updateTheGuiFromConfig();
        }
    

        

        
        private void OpenButton_Click(object sender, EventArgs e)
        {
            using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
            {
                openFileDialog.Title = "Datei auswählen";
                openFileDialog.Filter = "Configdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                openFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    string content = (string)openFileDialog.FileName;
                    TextBox_debugOutput.Text = content;

                    string filePath = openFileDialog.FileName;


                    if (false)
                    {
                        string text1 = System.IO.File.ReadAllText(filePath);
                        DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                        var ms = new MemoryStream(Encoding.UTF8.GetBytes(text1));
                        dap_config_st[indexOfSelectedPedal_u] = (DAP_config_st)deserializer.ReadObject(ms);
                    }
                    else
                    {
                        // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/deserialization


                        // c# code to iterate over all fields of struct and set values from json file

                        // Read the entire JSON file
                        string jsonString = File.ReadAllText(filePath);

                        // Parse all of the JSON.
                        //JsonNode forecastNode = JsonNode.Parse(jsonString);
                        dynamic data = JsonConvert.DeserializeObject(jsonString);



                        payloadPedalConfig payloadPedalConfig_fromJson_st = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
                        //var s = default(payloadPedalConfig);
                        Object obj = payloadPedalConfig_fromJson_st;// s;



                        FieldInfo[] fi = payloadPedalConfig_fromJson_st.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance);

                        // Iterate over each field and print its name and value
                        foreach (var field in fi)
                        {

                            if (data["payloadPedalConfig_"][field.Name] != null)
                            //if (forecastNode["payloadPedalConfig_"][field.Name] != null)
                            {
                                try
                                {
                                    if (field.FieldType == typeof(float))
                                    {
                                        //float value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<float>();
                                        float value = (float)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }

                                    if (field.FieldType == typeof(byte))
                                    {
                                        //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                        byte value = (byte)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }

                                    if (field.FieldType == typeof(Int16))
                                    {
                                        //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                        Int16 value = (Int16)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }


                                }
                                catch (Exception)
                                {

                                }

                            }
                        }

                        // set values in global structure
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_ = (payloadPedalConfig)obj;// payloadPedalConfig_fromJson_st;
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.spindlePitch_mmPerRev_u8 == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise = 5;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedal_type != indexOfSelectedPedal_u)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedal_type = (byte)indexOfSelectedPedal_u;

                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_a==0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_a = 205;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_b == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_b = 220;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_d == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_d = 60;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_c_horizontal == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_c_horizontal = 215;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_c_vertical == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_c_vertical = 60;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_travel == 0)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_travel = 100;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition < 5)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition = 5;
                        }
                        if (dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition > 95)
                        {
                            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition = 95;
                        }
                    }

                    updateTheGuiFromConfig();
                    TextBox_debugOutput.Text = "Config new imported!";
                    TextBox2.Text = "Open " + openFileDialog.FileName;
                }
            }

        }

        private void SaveButton_Click(object sender, RoutedEventArgs e)
        {
            using (System.Windows.Forms.SaveFileDialog saveFileDialog = new System.Windows.Forms.SaveFileDialog())
            {
                saveFileDialog.Title = "Datei speichern";
                saveFileDialog.Filter = "Textdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                saveFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (saveFileDialog.ShowDialog() == DialogResult.OK)
                {
                     string fileName = saveFileDialog.FileName;


                    this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;

                    // https://stackoverflow.com/questions/3275863/does-net-4-have-a-built-in-json-serializer-deserializer
                    // https://learn.microsoft.com/en-us/dotnet/framework/wcf/feature-details/how-to-serialize-and-deserialize-json-data?redirectedfrom=MSDN
                    var stream1 = new MemoryStream();
                    //var ser = new DataContractJsonSerializer(typeof(DAP_config_st));
                    //ser.WriteObject(stream1, dap_config_st[indexOfSelectedPedal_u]);


                    // formatted JSON see https://stackoverflow.com/a/38538454
                    var writer = JsonReaderWriterFactory.CreateJsonWriter(stream1, Encoding.UTF8, true, true, "  ");
                    var serializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                    serializer.WriteObject(writer, dap_config_st[indexOfSelectedPedal_u]);
                    writer.Flush();

                    stream1.Position = 0;
                    StreamReader sr = new StreamReader(stream1);
                    string jsonString = sr.ReadToEnd();

                    // Check if file already exists. If yes, delete it.     
                    if (File.Exists(fileName))
                    {
                        File.Delete(fileName);
                    }


                    System.IO.File.WriteAllText(fileName, jsonString);
                    TextBox_debugOutput.Text = "Config new exported!";
                    TextBox2.Text = "Save " + saveFileDialog.FileName;
                    }
            }
        }

        
        private void DisconnectToPedal_click(object sender, RoutedEventArgs e)
        {

            closeSerialAndStopReadCallback(indexOfSelectedPedal_u);

            if (ConnectToPedal.IsChecked == true)
            {
                ConnectToPedal.IsChecked = false;
                TextBox_debugOutput.Text = "Serialport close";
            }           
            else
            {
                ConnectToPedal.IsChecked = false;
                TextBox_debugOutput.Text = "Not Checked Serialport close";
            }
            updateTheGuiFromConfig();

        }

        private void dump_pedal_response_to_file_checked(object sender, RoutedEventArgs e)
        {
            dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u] = true;
            dumpPedalToResponseFile[indexOfSelectedPedal_u] = true;
        }

        private void dump_pedal_response_to_file_unchecked(object sender, RoutedEventArgs e)
        {
            dumpPedalToResponseFile[indexOfSelectedPedal_u] = false;
        }




        //dragable control rect.

        /*private void InitializeRectanglePositions()
        {
            rectanglePositions.Add("rect1", new Point(75, 75));
            rectanglePositions.Add("rect2", new Point(155, 55));
            rectanglePositions.Add("rect3", new Point(235, 35));
            rectanglePositions.Add("rect4", new Point(315, 15));
        }*/

        private void Rectangle_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            isDragging = true;
            var rectangle = sender as Rectangle;
            offset = e.GetPosition(rectangle);
            rectangle.CaptureMouse();                      
            if (rectangle.Name != "rect_SABS_Control" & rectangle.Name != "rect_BP_Control")
            {
                var dropShadowEffect = new DropShadowEffect
                {
                    ShadowDepth = 0,
                    BlurRadius = 15,
                    Color = Colors.White,
                    Opacity = 1
                };
                rectangle.Fill = lightcolor;
                rectangle.Effect = dropShadowEffect;
            }

        }

        private void Debug_checkbox_Checked(object sender, RoutedEventArgs e)
        {

            //text_debug_flag.Visibility = Visibility.Visible; 
            //text_serial.Visibility = Visibility.Visible;
            //TextBox_serialMonitor.Visibility = System.Windows.Visibility.Visible;
            //textBox_debug_Flag_0.Visibility = Visibility.Visible;

            debug_flag = true;
            if (Plugin != null)
            {
                Plugin.Settings.advanced_b = debug_flag;
            }
            //Border_serial_monitor.Visibility = Visibility.Visible;
            
            
           // Label_reverse_LC.Visibility = Visibility.Visible;
            //Label_reverse_servo.Visibility = Visibility.Visible;
            btn_test.Visibility = Visibility.Visible;
            //Line_H_HeaderTab.X2 = 1128;

            TextBox_debug_count.Visibility=Visibility.Visible;


        }
        private void Debug_checkbox_Unchecked(object sender, RoutedEventArgs e)
        {
            //text_debug_flag.Visibility = Visibility.Hidden; ;
            //text_serial.Visibility = Visibility.Hidden;
            //TextBox_serialMonitor.Visibility = System.Windows.Visibility.Hidden;
            //textBox_debug_Flag_0.Visibility = Visibility.Hidden;

            debug_flag = false;
            if (Plugin != null)
            {
                Plugin.Settings.advanced_b = debug_flag;
            }
            //Border_serial_monitor.Visibility = Visibility.Hidden;
            
            
            //Label_reverse_LC.Visibility = Visibility.Hidden;
            //Label_reverse_servo.Visibility = Visibility.Hidden;
            btn_test.Visibility = Visibility.Hidden;
            //Line_H_HeaderTab.X2 = 763;

            TextBox_debug_count.Visibility = Visibility.Hidden;
        }


        private void JoystickOutput_checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 = 1;

        }
        private void JoystickOutput_unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.travelAsJoystickOutput_u8 = 0;
        }



        private void CheckBox_Reading_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.reading_config = 1;
        }
        private void CheckBox_Reading_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.reading_config = 0;
        }

        private void checkbox_auto_connect_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.axis_settings[indexOfSelectedPedal_u].auto_connect = true;
        }

        private void checkbox_auto_connect_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.axis_settings[indexOfSelectedPedal_u].auto_connect = false;
        }

        private void Vjoy_out_check_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_output_flag = 1;
            ////// vJoy c# wrapper, see https://github.com/bobhelander/vJoy.Wrapper
            ////uint vJoystickId = Plugin.Settings.vjoy_order;
            ////joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
            ////joystick.Aquire();
            ////vjoy_axis_initialize();

            uint vJoystickId = Plugin.Settings.vjoy_order;
            //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
            joystick = new vJoyInterfaceWrap.vJoy();

            joystick.AcquireVJD(vJoystickId);
            //joystick.Aquire();
            vjoy_axis_initialize();
            //CheckBox_rudder.IsEnabled = true;

        }


        private void Vjoy_out_check_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_output_flag = 0;
            //joystick.Release();
            joystick.RelinquishVJD(Plugin.Settings.vjoy_order);
            //CheckBox_rudder.IsEnabled = false;
        }


        private void vjoy_plus_click(object sender, RoutedEventArgs e)
        {
            // release old joystick
            joystick.RelinquishVJD(Plugin.Settings.vjoy_order);

            Plugin.Settings.vjoy_order += 1;
            uint max = 16;
            uint min = 1;
            Plugin.Settings.vjoy_order = Math.Max(min, Math.Min(Plugin.Settings.vjoy_order, max));
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                //joystick.Release();
                
                //VjdStat status;
                VjdStat status = joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                //status = joystick.Joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                switch (status)
                {
                    case VjdStat.VJD_STAT_OWN:
                        TextBox_debugOutput.Text = "vjoy already aquaried";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        break;
                    case VjdStat.VJD_STAT_FREE:

                        TextBox_debugOutput.Text = "vjoy aquaried";
                        //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                        //joystick.Aquire();
                        joystick.AcquireVJD(Plugin.Settings.vjoy_order);
                        if (Vjoy_out_check.IsChecked == false)
                        {
                            Vjoy_out_check.IsChecked = true;
                        }
                        //Console.WriteLine("vJoy Device {0} is free\n", id);
                        break;
                    case VjdStat.VJD_STAT_BUSY:
                        TextBox_debugOutput.Text = "vjoy was aquaried by other program";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is already owned by another feeder\nCannot continue\n", id);
                        return;
                    case VjdStat.VJD_STAT_MISS:
                        TextBox_debugOutput.Text = "the selected vjoy device not enabled";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is not installed or disabled\nCannot continue\n", id);
                        return;
                    default:
                        TextBox_debugOutput.Text = "vjoy device error";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} general error\nCannot continue\n", id);
                        return;
                };
            }
            

        }

        private void vjoy_minus_click(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.vjoy_order -= 1;
            uint max = 16;
            uint min = 1;
            Plugin.Settings.vjoy_order = Math.Max(min, Math.Min(Plugin.Settings.vjoy_order, max));
            Label_vjoy_order.Content = Plugin.Settings.vjoy_order;
            if (Plugin.Settings.vjoy_output_flag == 1)
            {
                //joystick.Release();
                joystick.RelinquishVJD(Plugin.Settings.vjoy_order);
                VjdStat status;
                status = joystick.GetVJDStatus(Plugin.Settings.vjoy_order);
                switch (status)
                {
                    case VjdStat.VJD_STAT_OWN:
                        TextBox_debugOutput.Text = "vjoy already aquaried";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        break;
                    case VjdStat.VJD_STAT_FREE:

                        TextBox_debugOutput.Text = "vjoy aquaried";
                        //joystick = new VirtualJoystick(Plugin.Settings.vjoy_order);
                        joystick.AcquireVJD(Plugin.Settings.vjoy_order);
                        //joystick.Aquire();
                        if (Vjoy_out_check.IsChecked == false)
                        {
                            Vjoy_out_check.IsChecked = true;
                        }
                        //Console.WriteLine("vJoy Device {0} is free\n", id);
                        break;
                    case VjdStat.VJD_STAT_BUSY:
                        TextBox_debugOutput.Text = "vjoy was aquaried by other program";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is already owned by another feeder\nCannot continue\n", id);
                        return;
                    case VjdStat.VJD_STAT_MISS:
                        TextBox_debugOutput.Text = "the selected vjoy device not enabled";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} is not installed or disabled\nCannot continue\n", id);
                        return;
                    default:
                        TextBox_debugOutput.Text = "vjoy device error";
                        Plugin.Settings.vjoy_output_flag = 0;
                        Vjoy_out_check.IsChecked = false;
                        //Console.WriteLine("vJoy Device {0} general error\nCannot continue\n", id);
                        return;
                };
            }




        }
        private void btn_reset_default_Click(object sender, RoutedEventArgs e)
        {
            DAP_config_set_default(indexOfSelectedPedal_u);
            updateTheGuiFromConfig();
        }

        private void TabControl_file_path(object sender, SelectionChangedEventArgs e)
        {
            profile_select = (uint)ProfileTab.SelectedIndex;
            Plugin.profile_index = profile_select;
            //Profile_change(profile_select);
            //Plugin.Settings.table_selected = (uint)MyTab.SelectedIndex;
            // update the sliders & serial port selection accordingly
            Update_Profile_Checkbox_b = true;
            updateTheGuiFromConfig();
            
        }

        private void file_check_Checked(object sender, RoutedEventArgs e)
        {
            var checkbox = sender as System.Windows.Controls.CheckBox;
            if (checkbox.Name == "Clutch_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 0] = 1;
            }

            if (checkbox.Name == "Brake_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 1] = 1;
            }

            if (checkbox.Name == "Gas_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 2] = 1;
            }
        }

        private void file_check_Unchecked(object sender, RoutedEventArgs e)
        {
            var checkbox = sender as System.Windows.Controls.CheckBox;
            if (checkbox.Name == "Clutch_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 0] = 0;
            }

            if (checkbox.Name == "Brake_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 1] = 0;
            }

            if (checkbox.Name == "Gas_file_check")
            {
                Plugin.Settings.file_enable_check[profile_select, 2] = 0;
            }
        }

        public void Profile_change(uint profile_index)
        {
            profile_select = profile_index;
            ProfileTab.SelectedIndex = (int)profile_index;
            //if (Plugin.Settings.file_enable_check[profile_select])
            Parsefile(profile_index);
            string tmp;
            switch (profile_index)
            {
                case 0:
                    tmp = "A:" +Plugin.Settings.Profile_name[profile_index];
                    break;
                case 1:
                    tmp = "B:" + Plugin.Settings.Profile_name[profile_index];
                    break;
                case 2:
                    tmp = "C:" + Plugin.Settings.Profile_name[profile_index];
                    break;
                case 3:
                    tmp = "D:" + Plugin.Settings.Profile_name[profile_index];
                    break;
                case 4:
                    tmp = "E:" + Plugin.Settings.Profile_name[profile_index];
                    break;
                case 5:
                    tmp = "F:" + Plugin.Settings.Profile_name[profile_index];
                    break;
                default:
                    tmp = "No Profile";
                    break;
            }
            Plugin.current_profile = tmp;
            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 8; k++)
                {
                    if (Plugin.Settings.function_settings[j].effect_status_profiles[profile_select, k])
                    {
                        switch (k)
                        {
                            case 0:
                                Plugin.Settings.function_settings[j].ABS_enabled = true;
                                break;
                            case 1:
                                Plugin.Settings.function_settings[j].RPM_enabled = true;
                                break;
                            case 2:
                                //Plugin.Settings. = 1;
                                break;
                            case 3:
                                Plugin.Settings.function_settings[j].G_force_enabled = true;
                                break;
                            case 4:
                                Plugin.Settings.function_settings[j].WS_enabled = true;
                                break;
                            case 5:
                                Plugin.Settings.function_settings[j].Road_impact_enabled = true;
                                break;
                            case 6:
                                Plugin.Settings.function_settings[j].CV1_enabled = true;
                                break;
                            case 7:
                                Plugin.Settings.function_settings[j].CV2_enabled = true;
                                break;
                        }
                    }
                    else
                    {
                        switch (k)
                        {
                            case 0:
                                Plugin.Settings.function_settings[j].ABS_enabled = false;
                                break;
                            case 1:
                                Plugin.Settings.function_settings[j].RPM_enabled = false;
                                break;
                            case 2:
                                //Plugin.Settings. = 1;
                                break;
                            case 3:
                                Plugin.Settings.function_settings[j].G_force_enabled = false;
                                break;
                            case 4:
                                Plugin.Settings.function_settings[j].WS_enabled = false;
                                break;
                            case 5:
                                Plugin.Settings.function_settings[j].Road_impact_enabled = false;
                                break;
                            case 6:
                                Plugin.Settings.function_settings[j].CV1_enabled = false;
                                break;
                            case 7:
                                Plugin.Settings.function_settings[j].CV2_enabled = false;
                                break;
                        }
                    }

                }
            }
            //effect profile change

        }

        private void btn_apply_profile_Click(object sender, RoutedEventArgs e)
        {
            Profile_change((uint)ProfileTab.SelectedIndex);
            Parsefile((uint)ProfileTab.SelectedIndex);
        }

        private void btn_send_profile_Click(object sender, RoutedEventArgs e)
        {
            Sendconfigtopedal_shortcut();
        }

        private void textbox_profile_name_TextChanged(object sender, TextChangedEventArgs e)
        {
            var textbox = sender as System.Windows.Controls.TextBox;
            Plugin.Settings.Profile_name[profile_select]= textbox.Text;
        }

        unsafe private void btn_toast_Click(object sender, RoutedEventArgs e)
        {
            
            ToastNotification("Rudder Brake","Test");
            Plugin.Rudder_brake_enable_flag = true;
            
            
        }
        private void Hyperlink_RequestNavigate(object sender, RequestNavigateEventArgs e)
        {
            // for .NET Core you need to add UseShellExecute = true
            // see https://learn.microsoft.com/dotnet/api/system.diagnostics.processstartinfo.useshellexecute#property-value
            Process.Start(new ProcessStartInfo(e.Uri.AbsoluteUri));
            e.Handled = true;
        }
        /*
        private void OTA_update_check_Unchecked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.OTA_flag = 0;
        }

        private void OTA_update_check_Checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.OTA_flag = 1;
        }
        */

    



        private void EnableStepLossRecov_check_Unchecked(object sender, RoutedEventArgs e)
        {
            byte tmp = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8;
            tmp = (byte)(tmp & ~(1 << 0));
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8 = tmp;

            tmp = 5;
        }
        private void EnableStepLossRecov_check_Checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8 |= (1 << 0);

            byte tmp = 5;
        }

        private void EnableCrashDetection_check_Unchecked(object sender, RoutedEventArgs e)
        {
            byte tmp = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8;
            tmp = (byte)(tmp & ~(1 << 1));
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8 = tmp;

            tmp = 5;
        }
        private void EnableCrashDetection_check_Checked(object sender, RoutedEventArgs e)
        {
            dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.stepLossFunctionFlags_u8 |= (1 << 1);

            byte tmp = 5;
        }





        private void btn_serial_clear_Click(object sender, RoutedEventArgs e)
        {
            //TextBox_serialMonitor.Clear();
        }

        private void Tab_main_1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            updateTheGuiFromConfig();
        }


        private void CheckBox_rudder_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Rudder_enable_flag = true;
            

        }

        private void CheckBox_rudder_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Rudder_enable_flag = true;
        }

        private void CheckBox_RTSDTR_Checked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.axis_settings[indexOfSelectedPedal_u].RTSDTR_False = true;
        }

        private void CheckBox_RTSDTR_Unchecked(object sender, RoutedEventArgs e)
        {
            Plugin.Settings.axis_settings[indexOfSelectedPedal_u].RTSDTR_False = false;
        }

        public void ESPNow_SerialPortSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            string tmp = (string)SerialPortSelection_ESPNow.SelectedValue;
            //Plugin._serialPort[indexOfSelectedPedal_u].PortName = tmp;


            //try 
            //{
            //    TextBox_debugOutput.Text = "Debug: " + Plugin.Settings.selectedComPortNames[indexOfSelectedPedal_u];
            //}
            //catch (Exception caughtEx)
            //{
            //    string errorMessage = caughtEx.Message;
            //    TextBox_debugOutput.Text = errorMessage;
            //}

            try
            {
                //if (Plugin.Settings.connect_status[indexOfSelectedPedal_u] == 0)
                if (Plugin.ESPsync_serialPort.IsOpen == false)
                {
                    Plugin.Settings.ESPNow_port = tmp;
                    Plugin.ESPsync_serialPort.PortName = tmp;
                }
                TextBox_debugOutput.Text = "COM port selected: " + Plugin.Settings.ESPNow_port;

            }
            catch (Exception caughtEx)
            {
                string errorMessage = caughtEx.Message;
                TextBox_debugOutput.Text = errorMessage;
            }



        }

        private void btn_connect_espnow_port_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin.Sync_esp_connection_flag)
            {
                Plugin.ESPsync_serialPort.OnMessage -= OnMessage;
                Plugin.ESPsync_serialPort.Close();
                Plugin.Sync_esp_connection_flag = false;
                btn_connect_espnow_port.Content = "Connect";
                SystemSounds.Beep.Play();
                Plugin.Settings.Pedal_ESPNow_auto_connect_flag = false;
                updateTheGuiFromConfig();
            }
            else
            {
                try
                {
                    if (Plugin.PortExists(Plugin.Settings.ESPNow_port))
                    {
                        Plugin.ESPsync_serialPort = new ProtobufSerial<Message>(Plugin.Settings.ESPNow_port, 3000000);
                        try
                        {
                            Plugin.ESPsync_serialPort.OnMessage += OnMessage;
                            Plugin.ESPsync_serialPort.Open(true);
                            System.Threading.Thread.Sleep(200);
                            // ESP32 S3
                            if (Plugin.Settings.Using_CDC_bridge)
                            {
                                Plugin.ESPsync_serialPort.RtsEnable = false;
                                Plugin.ESPsync_serialPort.DtrEnable = true;
                            }
                            Plugin.ESPsync_serialPort.RtsEnable = false;
                            Plugin.ESPsync_serialPort.DtrEnable = false;

                            SystemSounds.Beep.Play();
                            Plugin.Sync_esp_connection_flag = true;
                            btn_connect_espnow_port.Content = "Disconnect";
                            if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag)
                            {
                                Plugin.Settings.ESPNow_port = Plugin.ESPsync_serialPort.PortName;
                            }
                            updateTheGuiFromConfig();
                        }
                        catch (Exception ex)
                        {
                            TextBox2.Text = ex.Message;
                        }
                    }
                }
                catch (Exception ex)
                {
                    TextBox2.Text = ex.Message;
                }
            }
        }



        private void CheckBox_Pedal_ESPNow_SyncFlag_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway = true;
            }
        }

        private void CheckBox_Pedal_ESPNow_SyncFlag_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway = false;
            }
        }

        private void ProccessAxisState(global::AxisState axis_state)
        {
            // write vJoy data
            //Pedal_position_reading[pedalSelected] = state. pedalState_read_st.payloadPedalBasicState_.joystickOutput_u16;
            // GUI update
            //if (pedalState_read_st.payloadPedalBasicState_.error_code_u8 != 0)
            //{
            //    Plugin.PedalErrorCode = pedalState_read_st.payloadPedalBasicState_.error_code_u8;
            //    Plugin.PedalErrorIndex = pedalState_read_st.payloadHeader_.PedalTag;
            //    TextBox2.Text = "Pedal:" + pedalState_read_st.payloadHeader_.PedalTag + " ErrorCode" + pedalState_read_st.payloadPedalBasicState_.error_code_u8;

            //}
            uc_function_config.OnAxisStateUpdate(axis_state);
            uc_axis_config.OnAxisStateUpdate(axis_state);
        }

        private void ProcessExtendedState()
        {
            //if (dumpPedalToResponseFile[indexOfSelectedPedal_u])
            //{
            //    // Specify the path to the file
            //    string currentDirectory = Directory.GetCurrentDirectory();
            //    string filePath = currentDirectory + "\\PluginsData\\Common" + "\\output_" + pedalSelected + ".txt";

            //    // delete file 
            //    if (true == dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u])
            //    {
            //        dumpPedalToResponseFile_clearFile[indexOfSelectedPedal_u] = false;
            //        File.Delete(filePath);
            //    }

            //    // write header
            //    if (!File.Exists(filePath))
            //    {
            //        using (StreamWriter writer = new StreamWriter(filePath, true))
            //        {
            //            // Write the content to the file
            //            writer.Write("cycleCtr, ");
            //            writer.Write("time_InMs, ");
            //            writer.Write("forceRaw_InKg, ");
            //            writer.Write("forceFiltered_InKg, ");
            //            writer.Write("forceVelocity_InKgPerSec, ");
            //            writer.Write("servoPos_InSteps, ");
            //            writer.Write("servoPosEsp_InSteps, ");
            //            writer.Write("servoCurrent_InPercent, ");
            //            writer.Write("servoVoltage_InV");
            //            writer.Write("\n");
            //        }

            //    }
            //    // Use StreamWriter to write to the file
            //    using (StreamWriter writer = new StreamWriter(filePath, true))
            //    {
            //        // Write the content to the file
            //        writeCntr++;
            //        writer.Write(writeCntr);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.timeInMs_u32);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_raw_fl32);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.pedalForce_filtered_fl32);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.forceVel_est_fl32);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPosition_i16);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servoPositionTarget_i16);
            //        writer.Write(", ");
            //        writer.Write(pedalState_ext_read_st.payloadPedalExtendedState_.servo_current_percent_i16);
            //        writer.Write(", ");
            //        writer.Write(((float)pedalState_ext_read_st.payloadPedalExtendedState_.servo_voltage_0p1V_i16) / 10.0);
            //        writer.Write("\n");
            //    }
            //}
        }

        private void ProcessGatewayState(ProtobufSerial<Message> port, GatewayState state)
        {
            var Bridge_RSSI = state.Rssi;
            Label_RSSI.Content = "" + (Bridge_RSSI - 100) + "dBm";
            if (Bridge_RSSI < 25)
            {
                RSSI_1.Visibility = Visibility.Visible;
                RSSI_1.Fill = redcolor;
                RSSI_2.Visibility = Visibility.Hidden;
                RSSI_3.Visibility = Visibility.Hidden;
                RSSI_4.Visibility = Visibility.Hidden;
            }
            if (Bridge_RSSI > 25 && Bridge_RSSI < 30)
            {
                RSSI_1.Visibility = Visibility.Visible;
                RSSI_1.Fill = color_RSSI_1;
                RSSI_2.Visibility = Visibility.Visible;

                RSSI_3.Visibility = Visibility.Hidden;
                RSSI_4.Visibility = Visibility.Hidden;
            }
            if (Bridge_RSSI > 30 && Bridge_RSSI < 35)
            {
                RSSI_1.Visibility = Visibility.Visible;
                RSSI_1.Fill = color_RSSI_1;
                RSSI_2.Visibility = Visibility.Visible;

                RSSI_3.Visibility = Visibility.Visible;
                RSSI_4.Visibility = Visibility.Hidden;
            }
            if (Bridge_RSSI > 35)
            {
                RSSI_1.Visibility = Visibility.Visible;
                RSSI_1.Fill = defaultcolor;
                RSSI_2.Visibility = Visibility.Visible;
                RSSI_3.Visibility = Visibility.Visible;
                RSSI_4.Visibility = Visibility.Visible;
            }
            if (debug_flag)
            {
                Label_RSSI.Visibility = Visibility.Visible;
            }
            else
            {
                Label_RSSI.Visibility = Visibility.Hidden;
            }

            for (AxisID axis_id = AxisID._1; axis_id <= AxisID._8; axis_id++)
            {
                int axis_flag = 1 << ((int)axis_id - 1);
                if ((state.AxesPresent & axis_flag) != 0)
                {
                    axes[axis_id].SerialChannel = port;
                }
                else
                {
                    axes[axis_id].SerialChannel = null;
                }
            }
        }

        public void OnMessage(object sender, object message)
        {
            ProtobufSerial<Message> sp = sender as ProtobufSerial<Message>;
            Message msg = message as Message;
            this.Dispatcher.Invoke(() => OnMessage(sp, msg));
        }

        private void OnMessage(ProtobufSerial<Message> port, Message msg) {
            //action here 
            Simhub_action_update();

            //int pedalSelected = Int32.Parse((sender as System.Windows.Forms.Timer).Tag.ToString());
            //int pedalSelected = (int)(sender as System.Windows.Forms.Timer).Tag;

            if (Plugin.Settings.Serial_auto_clean_bridge)
            {
                if (TextBox_serialMonitor_bridge.LineCount > 100)
                {
                    TextBox_serialMonitor_bridge.Clear();
                }
                /*
                if (TextBox_serialMonitor.LineCount > 100)
                {
                    TextBox_serialMonitor.Clear();
                }*/
                if (_serial_monitor_window != null && _serial_monitor_window.TextBox_SerialMonitor.LineCount > 100)
                {
                    _serial_monitor_window.TextBox_SerialMonitor.Clear();
                }
            }
            try
            {
                // Create a Stopwatch instance
                Stopwatch stopwatch = new Stopwatch();
                // Start the stopwatch
                stopwatch.Start();
                switch (msg.PayloadCase)
                {
                    case Message.PayloadOneofCase.AxisState:
                        ProccessAxisState(msg.AxisState);
                        break;
                    case Message.PayloadOneofCase.GatewayState:
                        ProcessGatewayState(port, msg.GatewayState);
                        break;
                    case Message.PayloadOneofCase.AxisConfig:
                        OnAxisConfigUpdate(msg.AxisConfig);
                        break;
                    case Message.PayloadOneofCase.FunctionConfig:
                        OnFunctionConfigUpdate(msg.FunctionConfig);
                        break;
                    case Message.PayloadOneofCase.AxisLogMessage:
                        string axis_log_line;
                        if (msg.AxisLogMessage.AxisId != AxisID.AxisUndefined)
                        {
                            axis_log_line = String.Format("A{0} : {1}", (int)msg.AxisLogMessage.AxisId, msg.AxisLogMessage.Msg);
                        } else
                        {
                            axis_log_line = String.Format("A? : {0}", msg.AxisLogMessage.Msg);
                        }
                        TextBox_serialMonitor_bridge.Text += axis_log_line + "\n";
                        TextBox_serialMonitor_bridge.ScrollToEnd();
                        SimHub.Logging.Current.Info(String.Format("DIY_FFB : {0}", axis_log_line));
                        if (_serial_monitor_window != null)
                        {
                            _serial_monitor_window.TextBox_SerialMonitor.Text += axis_log_line;
                            _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                        }
                        break;
                    case Message.PayloadOneofCase.GatewayLogMessage:
                        string gateway_log_line;
                        if (msg.GatewayLogMessage.GatewayId != GatewayID.GatewayUndefined)
                        {
                            gateway_log_line = String.Format("G{0} : {1}", (int)msg.GatewayLogMessage.GatewayId, msg.GatewayLogMessage.Msg);
                        }
                        else
                        {
                            gateway_log_line = String.Format("G? : {0}", msg.GatewayLogMessage.Msg);
                        }
                        TextBox_serialMonitor_bridge.Text += gateway_log_line + "\n";
                        TextBox_serialMonitor_bridge.ScrollToEnd();
                        SimHub.Logging.Current.Info(String.Format("DIY_FFB : {0}", gateway_log_line));
                        if (_serial_monitor_window != null)
                        {
                            _serial_monitor_window.TextBox_SerialMonitor.Text += gateway_log_line;
                            _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                        }
                        break;
                    case Message.PayloadOneofCase.ActiveFunction:
                        if (last_known_functions.ContainsKey(msg.ActiveFunction.AxisId))
                        {
                            functions[last_known_functions[msg.ActiveFunction.AxisId]].OnAxisRemoved(msg.ActiveFunction.AxisId);
                        }
                        last_known_functions[msg.ActiveFunction.AxisId] = msg.ActiveFunction.FunctionId;
                        functions[msg.ActiveFunction.FunctionId].OnAxisAdded(msg.ActiveFunction.AxisId);
                        break;
                    default:
                        break;
                }

                // Stop the stopwatch
                stopwatch.Stop();

                // Get the elapsed time
                TimeSpan elapsedTime = stopwatch.Elapsed;

                timeCollector[3] += elapsedTime.TotalMilliseconds;

                if (timeCntr[3] >= 50)
                {


                    double avgTime = timeCollector[3] / timeCntr[3];
                    if (debug_flag)
                    {
                        TextBox_debugOutput.Text = "Serial callback time in ms: " + avgTime.ToString();
                    }
                    timeCntr[3] = 0;
                    timeCollector[3] = 0;
                }
            }
            catch (Exception caughtEx)
            {

                string errorMessage = caughtEx.Message;
                TextBox_debug_count.Text += errorMessage;
                SimHub.Logging.Current.Error(errorMessage);
            }
        }

        private void CheckBox_Pedal_ESPNow_autoconnect_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            { 
                Plugin.Settings.Pedal_ESPNow_auto_connect_flag = true;
            }
        }

        private void CheckBox_Pedal_ESPNow_autoconnect_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Pedal_ESPNow_auto_connect_flag = false;
            }
        }

        private void Checkbox_auto_remove_serial_line_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Serial_auto_clean = true;
            }
        }

        private void Checkbox_auto_remove_serial_line_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Serial_auto_clean = false;
            }
        }

 unsafe private void btn_OTA_enable_Click(object sender, RoutedEventArgs e)
        {
            // compute checksum
            /*
            DAP_action_st tmp;
            tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
            tmp.payloadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            tmp.payloadPedalAction_.system_action_u8 = 3; //1=reset pedal position, 2 =restart esp, 3=enable wifi OTA

            DAP_action_st* v = &tmp;
            byte* p = (byte*)v;
            tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));
            int length = sizeof(DAP_action_st);
            byte[] newBuffer = new byte[length];
            newBuffer = Plugin.getBytes_Action(tmp);
            if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    try
                    {
                        // clear inbuffer 
                        Plugin.ESPsync_serialPort.DiscardInBuffer();

                        // send query command
                        Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }
            else
            {
                if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
                {
                    try
                    {
                        // clear inbuffer 
                        Plugin._serialPort[indexOfSelectedPedal_u].DiscardInBuffer();

                        // send query command
                        Plugin._serialPort[indexOfSelectedPedal_u].Write(newBuffer, 0, newBuffer.Length);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }
            string MSG_tmp = "Connect to ";
            if (indexOfSelectedPedal_u == 0)
            {
                MSG_tmp += "FFBPedalClutch";
            }
            if (indexOfSelectedPedal_u == 1)
            {
                MSG_tmp += "FFBPedalBrake";
            }
            if (indexOfSelectedPedal_u == 2)
            {
                MSG_tmp += "FFBPedalGas";
            }
            MSG_tmp += " wifi hotspot, then go to 192.168.2.1 to upload firmware.bin";

            System.Windows.MessageBox.Show(MSG_tmp, "OTA warning", MessageBoxButton.OK, MessageBoxImage.Warning);
            */
            Basic_WIfi_info tmp_2;
            int length;
            string SSID = textbox_SSID.Text;
            string PASS = textbox_PASS.Password;
            string MSG_tmp="";
            bool SSID_PASS_check = true;
            if (Checkbox_Force_flash.IsChecked == true)
            {
                tmp_2.wifi_action = 1;
            }
            if (OTAChannel_Sel_1.IsChecked == true)
            {
                tmp_2.mode_select = 1;
            }
            if (OTAChannel_Sel_2.IsChecked == true)
            {
                tmp_2.mode_select = 2;
            }
            if (SSID.Length > 30 || PASS.Length > 30)
            {
                SSID_PASS_check = false;
                
                MSG_tmp = "ERROR! SSID or Password length must less than 30 bytes";
                System.Windows.MessageBox.Show(MSG_tmp, "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            MSG_tmp += "OTA-Pull function only support V4/Gilphilbert board, for V3/Speedcrafter board user, please connect";
            if (indexOfSelectedPedal_u == 0)
            {
                MSG_tmp += "FFBPedalClutch";
            }
            if (indexOfSelectedPedal_u == 1)
            {
                MSG_tmp += "FFBPedalBrake";
            }
            if (indexOfSelectedPedal_u == 2)
            {
                MSG_tmp += "FFBPedalGas";
            }
            MSG_tmp += " wifi hotspot, open 192.168.2.1 in web browser to upload firmware.bin";

            System.Windows.MessageBox.Show(MSG_tmp, "OTA warning", MessageBoxButton.OK, MessageBoxImage.Warning);

            if (SSID_PASS_check)
            {
                tmp_2.SSID_Length = (byte)SSID.Length;
                tmp_2.PASS_Length = (byte)PASS.Length;
                tmp_2.device_ID = (byte)indexOfSelectedPedal_u;
                tmp_2.payload_Type = (Byte)Constants.Basic_Wifi_info_type;

                byte[] array_ssid = Encoding.ASCII.GetBytes(SSID);
                //TextBox_serialMonitor_bridge.Text += "SSID:";
                for (int i = 0; i < SSID.Length; i++)
                {
                    tmp_2.WIFI_SSID[i] = array_ssid[i];
                    //TextBox_serialMonitor_bridge.Text += tmp_2.WIFI_SSID[i] + ",";
                }
                //TextBox_serialMonitor_bridge.Text += "\nPASS:";
                byte[] array_pass = Encoding.ASCII.GetBytes(PASS);
                for (int i = 0; i < PASS.Length; i++)
                {
                    tmp_2.WIFI_PASS[i] = array_pass[i];
                    //TextBox_serialMonitor_bridge.Text += tmp_2.WIFI_PASS[i] + ",";
                }

                Basic_WIfi_info* v_2 = &tmp_2;
                byte* p_2 = (byte*)v_2;
                TextBox_serialMonitor_bridge.Text += "\nwifi info sent\n\r";

                length = sizeof(Basic_WIfi_info);
                TextBox_serialMonitor_bridge.Text += "\nLength:" + length;
                byte[] newBuffer_2 = new byte[length];
                newBuffer_2 = Plugin.getBytes_Basic_Wifi_info(tmp_2);
                if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
                {
                    if (Plugin.ESPsync_serialPort.IsOpen)
                    {
                        try
                        {
                            // clear inbuffer 
                            //Plugin.ESPsync_serialPort.DiscardInBuffer();

                            // send query command
                            //Plugin.ESPsync_serialPort.Write(newBuffer_2, 0, newBuffer_2.Length);
                        }
                        catch (Exception caughtEx)
                        {
                            string errorMessage = caughtEx.Message;
                            //TextBox_debugOutput.Text = errorMessage;
                            if (_serial_monitor_window != null)
                            {
                                _serial_monitor_window.TextBox_SerialMonitor.Text += errorMessage + "\n";
                                _serial_monitor_window.TextBox_SerialMonitor.ScrollToEnd();
                            }
                            //TextBox_serialMonitor.Text+= errorMessage+"\n";
                        }
                    }
                }
                else
                {
                    if (Plugin._serialPort[indexOfSelectedPedal_u].IsOpen)
                    {
                        try
                        {
                            // clear inbuffer 
                            Plugin._serialPort[indexOfSelectedPedal_u].DiscardInBuffer();

                            // send query command
                            Plugin._serialPort[indexOfSelectedPedal_u].Write(newBuffer_2, 0, newBuffer_2.Length);
                        }
                        catch (Exception caughtEx)
                        {
                            string errorMessage = caughtEx.Message;
                            //TextBox_debugOutput.Text = errorMessage;
                            if (_serial_monitor_window != null)
                            {
                                _serial_monitor_window.TextBox_SerialMonitor.Text += errorMessage + "\n";
                            }
                            //TextBox_serialMonitor.Text += errorMessage + "\n";
                        }
                    }
                }
            }
        }

        unsafe private void btn_Bridge_restart_Click(object sender, RoutedEventArgs e)
        {
            /*
            Plugin.ESPsync_serialPort.DtrEnable = true;
            Plugin.ESPsync_serialPort.RtsEnable = true;
            System.Threading.Thread.Sleep(100);
            Plugin.ESPsync_serialPort.DtrEnable = false;
            Plugin.ESPsync_serialPort.RtsEnable = false;
            */
            //write to bridge
            DAP_bridge_state_st tmp_2;
            int length;
            tmp_2.payLoadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp_2.payLoadHeader_.payloadType = (byte)Constants.bridgeStatePayloadType;
            tmp_2.payLoadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            tmp_2.payloadBridgeState_.Pedal_RSSI = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_0 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_1 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_2 = 0;
            tmp_2.payloadBridgeState_.Bridge_action = 2; //restart bridge
            DAP_bridge_state_st* v_2 = &tmp_2;
            byte* p_2 = (byte*)v_2;
            tmp_2.payloadFooter_.checkSum = Plugin.checksumCalc(p_2, sizeof(payloadHeader) + sizeof(payloadBridgeState));
            length = sizeof(DAP_bridge_state_st);
            byte[] newBuffer_2 = new byte[length];
            newBuffer_2 = Plugin.getBytes_Bridge(tmp_2);
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                try
                {
                    // clear inbuffer 
                    //Plugin.ESPsync_serialPort.DiscardInBuffer();
                    // send query command
                    //Plugin.ESPsync_serialPort.Write(newBuffer_2, 0, newBuffer_2.Length);
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }
            }
        }

        unsafe private void btn_PedalID_Reset_Click(object sender, RoutedEventArgs e)
        {
            // compute checksum
            //getBytes(this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_)
            this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.payloadType = (byte)Constants.pedalConfigPayload_type;
            this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.storeToEeprom = 1;
            this.dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedal_type = 4;//back to default value 4.
            DAP_config_st tmp = this.dap_config_st[indexOfSelectedPedal_u];
            DAP_config_st* v = &tmp;
            byte* p = (byte*)v;
            this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalConfig));
            int length = sizeof(DAP_config_st);
            //int val = this.dap_config_st[indexOfSelectedPedal_u].payloadHeader_.checkSum;
            //string msg = "CRC value: " + val.ToString();
            byte[] newBuffer = new byte[length];
            newBuffer = getBytes(this.dap_config_st[indexOfSelectedPedal_u]);

            //TextBox_debugOutput.Text = "CRC simhub calc: " + this.dap_config_st[indexOfSelectedPedal_u].payloadFooter_.checkSum + "    ";

            TextBox_debugOutput.Text = String.Empty;
            if (Plugin.Settings.axis_settings[indexOfSelectedPedal_u].via_gateway)
            {
                if (Plugin.ESPsync_serialPort.IsOpen)
                {
                    try
                    {
                        TextBox2.Text = "Buffer sent size:" + length;
                        //Plugin.ESPsync_serialPort.DiscardInBuffer();
                        //Plugin.ESPsync_serialPort.DiscardOutBuffer();
                        // send data
                        //Plugin.ESPsync_serialPort.Write(newBuffer, 0, newBuffer.Length);
                        //Plugin._serialPort[indexOfSelectedPedal_u].Write("\n");
                        System.Threading.Thread.Sleep(100);
                        string MSG_tmp = "Pedal:"+indexOfSelectedPedal_u+" ID is reset, please adjust jumpper on the control board then re-send config in.";
                        System.Windows.MessageBox.Show(MSG_tmp, "OTA warning", MessageBoxButton.OK, MessageBoxImage.Warning);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }
        }

        private void CheckBox_using_CDC_for_bridge_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            { 
                Plugin.Settings.Using_CDC_bridge = true;
            }
        }

        private void CheckBox_using_CDC_for_bridge_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Using_CDC_bridge = false;
            }
        }

        private void Slider_Pedal_interval_trigger_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.function_settings[indexOfSelectedPedal_u].action_interval = (byte)e.NewValue;
                Label_Pedal_interval_trigger.Content = "Action Interval: "+ Plugin.Settings.function_settings[indexOfSelectedPedal_u].action_interval+"ms";
            }
        }

        unsafe private void btn_Pairing_Click(object sender, RoutedEventArgs e)
        {
            //write to pedal
            DAP_action_st tmp;
            tmp.payloadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp.payloadHeader_.payloadType = (byte)Constants.pedalActionPayload_type;
            tmp.payloadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            tmp.payloadPedalAction_.system_action_u8 = 4; //1=reset pedal position, 2 =restart esp, 3=enable wifi OTA, 4= pairing

            DAP_action_st* v = &tmp;
            byte* p = (byte*)v;
            tmp.payloadFooter_.checkSum = Plugin.checksumCalc(p, sizeof(payloadHeader) + sizeof(payloadPedalAction));
            int length = sizeof(DAP_action_st);
            byte[] newBuffer = new byte[length];
            newBuffer = Plugin.getBytes_Action(tmp);
            for (uint pedalIDX = 0; pedalIDX < 3; pedalIDX++)
            {
                if (Plugin._serialPort[pedalIDX].IsOpen)
                {
                    try
                    {
                        // clear inbuffer 
                        Plugin._serialPort[pedalIDX].DiscardInBuffer();

                        // send query command
                        Plugin._serialPort[pedalIDX].Write(newBuffer, 0, newBuffer.Length);
                    }
                    catch (Exception caughtEx)
                    {
                        string errorMessage = caughtEx.Message;
                        TextBox_debugOutput.Text = errorMessage;
                    }
                }
            }

            //write to bridge
            DAP_bridge_state_st tmp_2;
            tmp_2.payLoadHeader_.version = (byte)Constants.pedalConfigPayload_version;           
            tmp_2.payLoadHeader_.payloadType = (byte)Constants.bridgeStatePayloadType;
            tmp_2.payLoadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            tmp_2.payloadBridgeState_.Pedal_RSSI = 0; 
            tmp_2.payloadBridgeState_.Pedal_availability_0 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_1 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_2 = 0;
            tmp_2.payloadBridgeState_.Bridge_action = 1; //enable pairing
            DAP_bridge_state_st* v_2 = &tmp_2;
            byte* p_2 = (byte*)v_2;
            tmp_2.payloadFooter_.checkSum = Plugin.checksumCalc(p_2, sizeof(payloadHeader) + sizeof(payloadBridgeState));
            length = sizeof(DAP_bridge_state_st);
            byte[] newBuffer_2 = new byte[length];
            newBuffer_2 = Plugin.getBytes_Bridge(tmp_2);
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                try
                {
                    // clear inbuffer 
                    //Plugin.ESPsync_serialPort.DiscardInBuffer();
                    // send query command
                    //Plugin.ESPsync_serialPort.Write(newBuffer_2, 0, newBuffer_2.Length);
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }
            }
            string MSG_tmp = "Please restart all pedals and bridge after pairing complete, related message will be shown in serial monitor.";
            System.Windows.MessageBox.Show(MSG_tmp, "Pairing", MessageBoxButton.OK, MessageBoxImage.Warning);


        }

        private void CheckBox_USINGESP32S3_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.axis_settings[indexOfSelectedPedal_u].USING_ESP32S3 = true;
            }
        }

        private void CheckBox_USINGESP32S3_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.axis_settings[indexOfSelectedPedal_u].USING_ESP32S3 = false;
            }
        }

        //Rudder initialize procee
        public void DelayCall(int msec, Action fn)
        {
            // Grab the dispatcher from the current executing thread
            Dispatcher d = Dispatcher.CurrentDispatcher;

            // Tasks execute in a thread pool thread
            new System.Threading.Tasks.Task(() =>
            {
                System.Threading.Thread.Sleep(msec);   // delay

                // use the dispatcher to asynchronously invoke the action 
                // back on the original thread
                d.BeginInvoke(fn);
            }).Start();
        }

        private void SHButtonPrimary_Click(object sender, RoutedEventArgs e)
        {
            using (System.Windows.Forms.OpenFileDialog openFileDialog = new System.Windows.Forms.OpenFileDialog())
            {
                openFileDialog.Title = "Datei auswählen";
                openFileDialog.Filter = "Configdateien (*.json)|*.json";
                string currentDirectory = Directory.GetCurrentDirectory();
                openFileDialog.InitialDirectory = currentDirectory + "\\PluginsData\\Common";

                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    string content = (string)openFileDialog.FileName;
                    TextBox_debugOutput.Text = content;

                    string filePath = openFileDialog.FileName;


                    if (false)
                    {
                        string text1 = System.IO.File.ReadAllText(filePath);
                        DataContractJsonSerializer deserializer = new DataContractJsonSerializer(typeof(DAP_config_st));
                        var ms = new MemoryStream(Encoding.UTF8.GetBytes(text1));
                        dap_config_st_rudder = (DAP_config_st)deserializer.ReadObject(ms);
                    }
                    else
                    {
                        // https://learn.microsoft.com/en-us/dotnet/standard/serialization/system-text-json/deserialization


                        // c# code to iterate over all fields of struct and set values from json file

                        // Read the entire JSON file
                        string jsonString = File.ReadAllText(filePath);

                        // Parse all of the JSON.
                        //JsonNode forecastNode = JsonNode.Parse(jsonString);
                        dynamic data = JsonConvert.DeserializeObject(jsonString);



                        payloadPedalConfig payloadPedalConfig_fromJson_st = dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_;
                        //var s = default(payloadPedalConfig);
                        Object obj = payloadPedalConfig_fromJson_st;// s;



                        FieldInfo[] fi = payloadPedalConfig_fromJson_st.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance);

                        // Iterate over each field and print its name and value
                        foreach (var field in fi)
                        {

                            if (data["payloadPedalConfig_"][field.Name] != null)
                            //if (forecastNode["payloadPedalConfig_"][field.Name] != null)
                            {
                                try
                                {
                                    if (field.FieldType == typeof(float))
                                    {
                                        //float value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<float>();
                                        float value = (float)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }

                                    if (field.FieldType == typeof(byte))
                                    {
                                        //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                        byte value = (byte)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }

                                    if (field.FieldType == typeof(Int16))
                                    {
                                        //byte value = forecastNode["payloadPedalConfig_"][field.Name].GetValue<byte>();
                                        Int16 value = (Int16)data["payloadPedalConfig_"][field.Name];
                                        field.SetValue(obj, value);
                                    }


                                }
                                catch (Exception)
                                {

                                }

                            }
                        }

                        // set values in global structure
                        dap_config_st_rudder.payloadPedalConfig_ = (payloadPedalConfig)obj;// payloadPedalConfig_fromJson_st;
                        if (dap_config_st_rudder.payloadPedalConfig_.spindlePitch_mmPerRev_u8 == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.spindlePitch_mmPerRev_u8 = 5;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.kf_modelNoise == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.kf_modelNoise = 5;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_a == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_a = 205;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_b == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_b = 220;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_d == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_d = 60;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_horizontal == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_horizontal = 215;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_vertical == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_c_vertical = 60;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.lengthPedal_travel == 0)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.lengthPedal_travel = 100;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.pedalStartPosition < 5)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.pedalStartPosition = 5;
                        }
                        if (dap_config_st_rudder.payloadPedalConfig_.pedalEndPosition > 95)
                        {
                            dap_config_st_rudder.payloadPedalConfig_.pedalEndPosition = 95;
                        }
                    }

                    updateTheGuiFromConfig();
                    /*
                    TextBox_debugOutput.Text = "Config new imported!";
                    TextBox2.Text = "Open " + openFileDialog.FileName;
                    */
                }
            }
        }


        private void btn_serial_clear_bridge_Click(object sender, RoutedEventArgs e)
        {
            TextBox_serialMonitor_bridge.Clear();
        }

        private void Checkbox_auto_remove_serial_line_bridge_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Serial_auto_clean_bridge = true;
            }
        }

        private void Checkbox_auto_remove_serial_line_bridge_Unchecked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.Serial_auto_clean_bridge = false;
            }
        }

        unsafe private void btn_Bridge_boot_restart_Click(object sender, RoutedEventArgs e)
        {
            DAP_bridge_state_st tmp_2;
            int length;
            tmp_2.payLoadHeader_.version = (byte)Constants.pedalConfigPayload_version;
            tmp_2.payLoadHeader_.payloadType = (byte)Constants.bridgeStatePayloadType;
            tmp_2.payLoadHeader_.PedalTag = (byte)indexOfSelectedPedal_u;
            tmp_2.payloadBridgeState_.Pedal_RSSI = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_0 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_1 = 0;
            tmp_2.payloadBridgeState_.Pedal_availability_2 = 0;
            tmp_2.payloadBridgeState_.Bridge_action = 3; //restart bridge into boot mode
            DAP_bridge_state_st* v_2 = &tmp_2;
            byte* p_2 = (byte*)v_2;
            tmp_2.payloadFooter_.checkSum = Plugin.checksumCalc(p_2, sizeof(payloadHeader) + sizeof(payloadBridgeState));
            length = sizeof(DAP_bridge_state_st);
            byte[] newBuffer_2 = new byte[length];
            newBuffer_2 = Plugin.getBytes_Bridge(tmp_2);
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                try
                {
                    // clear inbuffer 
                    //Plugin.ESPsync_serialPort.DiscardInBuffer();
                    // send query command
                    //Plugin.ESPsync_serialPort.Write(newBuffer_2, 0, newBuffer_2.Length);
                }
                catch (Exception caughtEx)
                {
                    string errorMessage = caughtEx.Message;
                    TextBox_debugOutput.Text = errorMessage;
                }
            }
        }

        

        unsafe private void btn_Bridge_OTA_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                Message msg = new Message();
                msg.StartOtaUpdate = new StartOtaUpdate();
                msg.StartOtaUpdate.WifiInfo = new WifiInfo();
                msg.StartOtaUpdate.WifiInfo.Ssid = textbox_SSID.Text;
                msg.StartOtaUpdate.WifiInfo.Password = textbox_PASS.Password;
                msg.StartOtaUpdate.AllowDowngrades = (bool)Checkbox_Force_flash.IsChecked;
                msg.StartOtaUpdate.InfoJsonUrl = "https://github.com/CK-AT/DIY-Sim-Racing-FFB-Pedal/raw/refs/heads/ck_comm_rework/OTA/update_info.json";

                Plugin.ESPsync_serialPort.WriteMessage(msg);
            }
        }

        private void textbox_SSID_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (textbox_SSID.Text.Length > 30)
            {
                Label_SSID.Content = "Error! SSID length >30";
            }
            else
            {
                if (Plugin != null)
                { 
                    Plugin.Settings.SSID_string = textbox_SSID.Text;
                }
                Label_SSID.Content = "";
            }
        }


        private void textbox_PASS_PasswordChanged(object sender, RoutedEventArgs e)
        {
            if (textbox_PASS.Password.Length > 30)
            {
                Label_PASS.Content = "Error! Password length >30";
            }
            else
            {
                if (Plugin != null)
                {
                    Plugin.Settings.PASS_string = textbox_PASS.Password;
                }
                Label_PASS.Content = "";
            }
        }

        private void Function_Tab_seleciton_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Function_Tab_seleciton.SelectedIndex==2)
            {
                Update_Profile_Checkbox_b = true;
                updateTheGuiFromConfig();
            }
        }


        private void OpenProfileWindow_Click(object sender, RoutedEventArgs e)
        {

        }

        private async void btn_OnlineProfile_Click(object sender, RoutedEventArgs e)
        {
            System.Windows.MessageBox.Show("Please make sure you already set the correct Pedal kinematics and Pedal start position(min pos)", "Warning", MessageBoxButton.OK, MessageBoxImage.Warning);
            OnlineProfile sideWindow = new OnlineProfile();
            double screenWidth = SystemParameters.PrimaryScreenWidth;
            double screenHeight = SystemParameters.PrimaryScreenHeight;
            sideWindow.Left=screenWidth/2-sideWindow.Width/2;
            sideWindow.Top=screenHeight/2-sideWindow.Height/2;
            if (sideWindow.ShowDialog() == true)
            {

                string jsonUrl = "https://raw.githubusercontent.com/tcfshcrw/FFB_PEDAL_PROFILE/master/Profiles/"+sideWindow.SelectedFileName;

                try
                {
                    DAP_config_st tmp_config;
                    tmp_config = await GetProfileDataAsync(jsonUrl);
                    float travel = (tmp_config.payloadPedalConfig_.pedalEndPosition - tmp_config.payloadPedalConfig_.pedalStartPosition) / 100.0f * (float)tmp_config.payloadPedalConfig_.lengthPedal_travel;
                    byte max_pos= (byte)(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition + (travel / (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_travel * 100.0f));
                    if (max_pos > 95)
                    {
                        System.Windows.MessageBox.Show("Pedal max position calculation error(max position out of travel), please adjust Pedal min position.", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                    }
                    else
                    {
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce = tmp_config.payloadPedalConfig_.maxForce;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce = tmp_config.payloadPedalConfig_.preloadForce;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p000 = tmp_config.payloadPedalConfig_.relativeForce_p000;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p020 = tmp_config.payloadPedalConfig_.relativeForce_p020;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p040 = tmp_config.payloadPedalConfig_.relativeForce_p040;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p060 = tmp_config.payloadPedalConfig_.relativeForce_p060;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p080 = tmp_config.payloadPedalConfig_.relativeForce_p080;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p100 = tmp_config.payloadPedalConfig_.relativeForce_p100;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress = tmp_config.payloadPedalConfig_.dampingPress;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPull = tmp_config.payloadPedalConfig_.dampingPull;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition = max_pos;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_0th_order_gain = tmp_config.payloadPedalConfig_.MPC_0th_order_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_1st_order_gain = tmp_config.payloadPedalConfig_.MPC_1st_order_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.MPC_2nd_order_gain = tmp_config.payloadPedalConfig_.MPC_2nd_order_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_d_gain = tmp_config.payloadPedalConfig_.PID_d_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_i_gain = tmp_config.payloadPedalConfig_.PID_i_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_p_gain = tmp_config.payloadPedalConfig_.PID_p_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.PID_velocity_feedforward_gain = tmp_config.payloadPedalConfig_.PID_velocity_feedforward_gain;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.control_strategy_b = tmp_config.payloadPedalConfig_.control_strategy_b;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelNoise = tmp_config.payloadPedalConfig_.kf_modelNoise;
                        dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.kf_modelOrder = tmp_config.payloadPedalConfig_.kf_modelOrder;
                        updateTheGuiFromConfig();
                    }

                }
                catch (Exception ex)
                {
                    System.Windows.MessageBox.Show($"Error loading JSON: {ex.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                    
                }
            }
        }

        private void btn_Export_OnlineProfile_Click(object sender, RoutedEventArgs e)
        {
            Online_profile.Basic_Config.Travel = (int)((float)(dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalEndPosition - dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.pedalStartPosition)/100.0f* (float)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.lengthPedal_travel);
            Online_profile.Basic_Config.MaxForce = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.maxForce;
            Online_profile.Basic_Config.PreloadForce = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.preloadForce;
            Online_profile.Basic_Config.Damping = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.dampingPress;
            Online_profile.Basic_Config.relativeForce_p000 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p000;
            Online_profile.Basic_Config.relativeForce_p020 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p020;
            Online_profile.Basic_Config.relativeForce_p040 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p040;
            Online_profile.Basic_Config.relativeForce_p060 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p060;
            Online_profile.Basic_Config.relativeForce_p080 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p080;
            Online_profile.Basic_Config.relativeForce_p100 = (int)dap_config_st[indexOfSelectedPedal_u].payloadPedalConfig_.relativeForce_p100;
            
            if (Online_profile != null)
            {
                Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
                {
                    Filter = "JSON files (*.json)|*.json",
                    DefaultExt = "json",
                    FileName = "Profile.json"
                };

                if (saveFileDialog.ShowDialog() == true)
                {
                    string jsonString = JsonConvert.SerializeObject(Online_profile, Formatting.Indented);
                    File.WriteAllText(saveFileDialog.FileName, jsonString);
                    System.Windows.MessageBox.Show("File saved successfully.");
                    
                }
            }
            else
            {
                System.Windows.MessageBox.Show("No profile data to save.");
            }
        }
        private async Task<DAP_config_st> GetProfileDataAsync(string url)
        {
            using (HttpClient client = new HttpClient())
            {
                string jsonString = await client.GetStringAsync(url);
                //return JsonConvert.DeserializeObject<Profile_Online>(jsonString);
                return JsonConvert.DeserializeObject<DAP_config_st>(jsonString);
            }
        }
        private void DisplayProfileData(Profile_Online profile)
        {
            if (profile?.Basic_Config != null)
            {
                TextBox_debug_count.Text = $"Max Force: {profile.Basic_Config.MaxForce}\n" +
                                     $"Preload Force: {profile.Basic_Config.PreloadForce}\n" +
                                     $"Damping: {profile.Basic_Config.Damping}\n" +
                                     $"Travel: {profile.Basic_Config.Travel}\n" +
                                     $"Relative Force (0%): {profile.Basic_Config.relativeForce_p000}\n" +
                                     $"Relative Force (20%): {profile.Basic_Config.relativeForce_p020}\n" +
                                     $"Relative Force (40%): {profile.Basic_Config.relativeForce_p040}\n" +
                                     $"Relative Force (60%): {profile.Basic_Config.relativeForce_p060}\n" +
                                     $"Relative Force (80%): {profile.Basic_Config.relativeForce_p080}\n" +
                                     $"Relative Force (100%): {profile.Basic_Config.relativeForce_p100}";
            }
            else
            {
                TextBox_debug_count.Text = "No data available.";
            }
        }
        private void btn_SerialMonitorWindow_Click(object sender, RoutedEventArgs e)
        {
            if (_serial_monitor_window == null || !_serial_monitor_window.IsVisible)
            {
                if (Pedal_Log_warning_1st_show_b)
                {
                    System.Windows.MessageBox.Show("Please connect Pedal via USB to Simhub to get Logs");
                    Pedal_Log_warning_1st_show_b = false;
                }
                
                _serial_monitor_window = new SerialMonitor_Window(this); // Create a new side window
                double screenWidth = SystemParameters.PrimaryScreenWidth;
                double screenHeight = SystemParameters.PrimaryScreenHeight;
                _serial_monitor_window.Left = screenWidth / 2 - _serial_monitor_window.Width / 2;
                _serial_monitor_window.Top = screenHeight / 2 - _serial_monitor_window.Height / 2;
                _serial_monitor_window.Show(); // Show the side window

            }
        }

        private void OnUploadFunctionConfigClicked(object sender, RoutedEventArgs e)
        {
            FunctionConfig function_config = functions[selected_function_id].Config;
            UploadFunctionConfig(function_config, PersistConfig);
        }

        private void UploadFunctionConfig(FunctionConfig function_config, bool store)
        {
            function_config.Base.Store = store;
            Message msg = new Message();
            msg.FunctionConfig = function_config;
            bool broadcast_required = false;
            foreach (var linked_axis_id in function_config.Base.LinkedAxes)
            {
                var axis_id = linked_axis_id & AxisID.Mask;
                if (axis_id != AxisID.AxisUndefined)
                {
                    var serial_channel = axes[axis_id].SerialChannel;
                    if (serial_channel != null && serial_channel != Plugin.ESPsync_serialPort)
                    {
                        serial_channel.WriteMessage(msg);
                    }
                    else
                    {
                        broadcast_required = true;
                    }
                }
            }
            if (broadcast_required && Plugin.ESPsync_serialPort.IsOpen)
            {
                Plugin.ESPsync_serialPort.WriteMessage(msg);
            }
        }

        private void OnFunctionConfigUpdate(FunctionConfig new_function_config)
        {
            FunctionID new_function_id = new_function_config.Base.FunctionId;
            if (new_function_id != FunctionID.Undefined)
            {
                functions[new_function_id].Config = new_function_config;
                if (new_function_id == selected_function_id)
                {
                    uc_function_config.SwitchFunction(functions[new_function_id]);
                }
            }
            else
            {
                TextBox_debugOutput.Text = $"invalid function ID ({(int)new_function_id})";
            }
        }

        private void OnAxisConfigUpdate(AxisConfig new_axis_config)
        {
            AxisID new_axis_id = new_axis_config.AxisId;
            if (new_axis_id != AxisID.AxisUndefined && new_axis_id <= AxisID._8)
            {
                axes[new_axis_id].Config = new_axis_config;
                if (new_axis_id == selected_axis_id)
                {
                    uc_axis_config.UpdateConfig(axes[new_axis_id].Config);
                }
            }
            else
            {
                TextBox_debugOutput.Text = $"invalid axis ID ({(int)new_axis_id})";
            }
        }

        private void OnLoadConfigClick(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
            };

            if (openFileDialog.ShowDialog() == true)
            {
                try
                {
                    var content = File.ReadAllText(openFileDialog.FileName);
                    var json_parser = new JsonParser(JsonParser.Settings.Default);
                    ConfigItemsList msg = (ConfigItemsList)json_parser.Parse(content, ConfigItemsList.Descriptor);
                    Dictionary<AxisID, AxisConfig> axis_configs = new Dictionary<AxisID, AxisConfig>();
                    Dictionary<FunctionID, FunctionConfig> function_configs = new Dictionary<FunctionID, FunctionConfig>();
                    foreach (var item in msg.ConfigItems)
                    {
                        switch (item.ItemCase)
                        {
                            case ConfigItem.ItemOneofCase.AxisConfig:
                                axis_configs[item.AxisConfig.AxisId] = item.AxisConfig;
                                axes[item.AxisConfig.AxisId].SelectedToLoad = true;
                                axes[item.AxisConfig.AxisId].SelectableToLoad = true;
                                break;
                            case ConfigItem.ItemOneofCase.FunctionConfig:
                                function_configs[item.FunctionConfig.Base.FunctionId] = item.FunctionConfig;
                                functions[item.FunctionConfig.Base.FunctionId].SelectedToLoad = true;
                                functions[item.FunctionConfig.Base.FunctionId].SelectableToLoad = true;
                                break;
                        }
                    }
                    loadSelectionDialog = new LoadSelectionDialog(this, axis_configs, function_configs);
                    loadSelectionDialog.Closed += OnLoadSelectionClosed;
                    btn_load_axis_config_from_file.IsEnabled = false;
                    btn_load_function_config_from_file.IsEnabled = false;
                    btn_store_function_config_to_file.IsEnabled = false;
                    btn_store_axis_config_to_file.IsEnabled = false;
                    loadSelectionDialog.Show();
                }
                catch (Exception caughtEx)
                {
                    System.Windows.MessageBox.Show($"Error loading {openFileDialog.FileName}: {caughtEx.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
        }

        private void OnLoadSelectionClosed(object sender, EventArgs e)
        {
            foreach (var item in axes.Values)
            {
                if (loadSelectionDialog.LoadRequested && item.SelectedToLoad)
                {
                    OnAxisConfigUpdate(loadSelectionDialog.axis_configs[item.ID]);
                }
                if (loadSelectionDialog.UploadRequested && item.SelectedToLoad)
                {
                    axes[item.ID].UploadConfig(false);
                }
                item.SelectedToLoad = false;
                item.SelectableToLoad = false;
            }
            foreach (var item in functions.Values)
            {
                if (loadSelectionDialog.LoadRequested && item.SelectedToLoad)
                {
                    OnFunctionConfigUpdate(loadSelectionDialog.function_configs[item.ID]);
                }
                if (loadSelectionDialog.UploadRequested && item.SelectedToLoad)
                {
                    UploadFunctionConfig(loadSelectionDialog.function_configs[item.ID], false);
                }
                item.SelectedToLoad = false;
                item.SelectableToLoad = false;
            }
            btn_load_axis_config_from_file.IsEnabled = true;
            btn_load_function_config_from_file.IsEnabled = true;
            btn_store_function_config_to_file.IsEnabled = true;
            btn_store_axis_config_to_file.IsEnabled = true;
        }

        SaveSelectionDialog saveSelectionDialog;
        LoadSelectionDialog loadSelectionDialog;

        private void OnSaveSelectionClosed(object sender, EventArgs e)
        {
            ConfigItemsList items = new ConfigItemsList();
            foreach (var axis in axes.Values)
            {
                if (axis.SelectedToStore)
                {
                    items.ConfigItems.Add(new ConfigItem { AxisConfig = axis.Config });
                    axis.SelectedToStore = false;
                }
            }
            foreach (var function in functions.Values)
            {
                if (function.SelectedToStore)
                {
                    items.ConfigItems.Add(new ConfigItem { FunctionConfig = function.Config });
                    function.SelectedToStore = false;
                }
            }
            if (saveSelectionDialog.SaveRequested)
            {
                JsonFormatter formatter = new JsonFormatter(JsonFormatter.Settings.Default.WithIndentation());
                var output = formatter.Format(items);
                File.WriteAllText(saveSelectionDialog.FileName, output);
            }
            btn_load_axis_config_from_file.IsEnabled = true;
            btn_load_function_config_from_file.IsEnabled = true;
            btn_store_function_config_to_file.IsEnabled = true;
            btn_store_axis_config_to_file.IsEnabled = true;
        }

        private void btn_store_function_config_to_file_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = "config.json"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                functions[selected_function_id].SelectedToStore = true;
                saveSelectionDialog = new SaveSelectionDialog(this, saveFileDialog.FileName);
                saveSelectionDialog.Closed += OnSaveSelectionClosed;
                btn_load_axis_config_from_file.IsEnabled = false;
                btn_load_function_config_from_file.IsEnabled = false;
                btn_store_function_config_to_file.IsEnabled = false;
                btn_store_axis_config_to_file.IsEnabled = false;
                saveSelectionDialog.Show();
            }
        }

        private void OnUploadAxisConfigClicked(object sender, RoutedEventArgs e)
        {
            axes[selected_axis_id].UploadConfig(PersistConfig);
        }

        private void btn_store_axis_config_to_file_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = "config.json"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                axes[selected_axis_id].SelectedToStore = true;
                saveSelectionDialog = new SaveSelectionDialog(this, saveFileDialog.FileName);
                saveSelectionDialog.Closed += OnSaveSelectionClosed;
                btn_load_axis_config_from_file.IsEnabled = false;
                btn_load_function_config_from_file.IsEnabled = false;
                btn_store_function_config_to_file.IsEnabled = false;
                btn_store_axis_config_to_file.IsEnabled = false;
                saveSelectionDialog.Show();
            }
        }
        bool PersistConfig = false;
        private void OnPreviewKeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                btn_upload_axis_config.Content = "Upload and Persist";
                btn_upload_function_config.Content = "Upload and Persist";
                PersistConfig = true;
            }
        }

        private void OnPreviewKeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                btn_upload_axis_config.Content = "Upload";
                btn_upload_function_config.Content = "Upload";
                PersistConfig = false;
            }
        }
    }
    
}
