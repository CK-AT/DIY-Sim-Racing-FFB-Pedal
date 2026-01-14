using Google.Protobuf;
using Newtonsoft.Json;
using ProtbufTest;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using vJoyInterfaceWrap;
using Windows.UI.Notifications;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for DiyFfbPluginUI.xaml
    /// </summary>
    public partial class DiyFfbPluginUI : UserControl
    {
        public uint indexOfSelectedPedal_u = 0;
        public DiyFfbPlugin Plugin { get; }

        private AxisID selected_axis_id = AxisID.AxisUndefined;
        private FunctionID selected_function_id = FunctionID.Undefined;

        public SortedDictionary<AxisID, Axis> axes { get; } = new SortedDictionary<AxisID, Axis>();
        public SortedDictionary<FunctionID, Function> functions { get; } = new SortedDictionary<FunctionID, Function>();
        public Dictionary<AxisID, FunctionID> last_known_functions { get; } = new Dictionary<AxisID, FunctionID>();

        internal vJoyInterfaceWrap.vJoy joystick;

        private readonly SolidColorBrush defaultcolor = new SolidColorBrush(Colors.White);
        private readonly SolidColorBrush lightcolor = new SolidColorBrush(Color.FromRgb(0xD8, 0xE6, 0xFF));

        public SolidColorBrush MouseDownColor { get { return lightcolor; } }
        public SolidColorBrush MouseUpColor { get { return defaultcolor; } }

        private bool persistConfigModifier;

        private bool PersistConfig => persistConfigModifier;

        private const int MaxWifiCredentialLength = 30;
        private const string OtaInfoUrlRelease = "https://github.com/CK-AT/DIY-Sim-Racing-FFB-Pedal/raw/refs/heads/main/OTA/update_info.json";
        private const string OtaInfoUrlDev = "https://github.com/CK-AT/DIY-Sim-Racing-FFB-Pedal/raw/refs/heads/ck_comm_rework/OTA/update_info.json";

        private SaveSelectionDialog saveSelectionDialog;
        private LoadSelectionDialog loadSelectionDialog;

        public DiyFfbPluginUI()
        {
            InitializeComponent();
        }

        public DiyFfbPluginUI(DiyFfbPlugin plugin) : this()
        {
            Plugin = plugin;
            DataContext = this;

            uc_function_config.ABSTestStateChange += OnABSTestStateChange;
            uc_function_config.DebugMessage += OnDebugMessage;
            uc_axis_config.DebugMessage += OnDebugMessage;
            uc_axis_config.KinematicParametersChanged += OnKinematicParametersChanged;
            uc_function_config.SetGui(this, plugin);
            uc_axis_config.SetGui(this, plugin);

            for (FunctionID id = FunctionID.BrakePedal; id <= FunctionID.Shifter; id++)
            {
                Function function = new Function(id);
                function.Config = FunctionConfigControl.GetDefaultConfig(id);
                functions[id] = function;
            }

            for (AxisID id = AxisID._1; id <= AxisID._8; id++)
            {
                Axis axis = new Axis(id);
                axis.Config = AxisConfigControl.GetDefaultConfig(id);
                axis.OnlineStateChanged += OnOnlineStateChange;
                axes[id] = axis;
            }

            UpdateUploadButtonLabels();

            UpdateSerialPortList();
            InitializeSystemSettings();

            if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag && !string.IsNullOrWhiteSpace(Plugin.Settings.ESPNow_port))
            {
                ConnectToPort(Plugin.Settings.ESPNow_port);
            }

            SetInitialSelections();

            InitializeVjoyIfEnabled();
        }

        public KinematicParameters GetKinematicParameters(AxisID axis_id)
        {
            if (axes.TryGetValue(axis_id, out Axis axis))
            {
                return axis.Config?.KinematicParameters;
            }
            return null;
        }

        private void InitializeVjoyIfEnabled()
        {
            if (Plugin.Settings.vjoy_output_flag != 1)
            {
                return;
            }

            joystick = new vJoyInterfaceWrap.vJoy();
            uint vJoystickId = Plugin.Settings.vjoy_order;
            joystick.AcquireVJD(vJoystickId);
            CenterVjoyAxes();
        }

        private void CenterVjoyAxes()
        {
            if (joystick == null)
            {
                return;
            }

            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_X);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Y);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_Z);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);
            joystick.SetAxis(16384, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);
        }

        private void SetInitialSelections()
        {
            int functionIndex = 0;
            int axisIndex = 0;
            if (Plugin != null)
            {
                functionIndex = (int)Plugin.Settings.function_tab_selected;
                axisIndex = (int)Plugin.Settings.axis_tab_selected;
            }

            if (tc_function_selection != null)
            {
                tc_function_selection.SelectedIndex = Math.Max(0, Math.Min(functions.Count - 1, functionIndex));
            }
            if (tc_axis_selection != null)
            {
                tc_axis_selection.SelectedIndex = Math.Max(0, Math.Min(axes.Count - 1, axisIndex));
            }

            UpdateFunctionSelection();
            UpdateAxisSelection();
        }

        private void UpdateSerialPortList()
        {
            List<string> ports = SerialPort.GetPortNames().Distinct().OrderBy(port => port).ToList();
            if (ports.Count == 0)
            {
                ports.Add("NA");
            }

            if (SerialPortSelection_ESPNow == null)
            {
                return;
            }

            SerialPortSelection_ESPNow.ItemsSource = ports;

            if (Plugin != null && !string.IsNullOrWhiteSpace(Plugin.Settings.ESPNow_port) && ports.Contains(Plugin.Settings.ESPNow_port))
            {
                SerialPortSelection_ESPNow.SelectedItem = Plugin.Settings.ESPNow_port;
            }
            else
            {
                SerialPortSelection_ESPNow.SelectedIndex = 0;
            }
        }

        private void InitializeSystemSettings()
        {
            if (Plugin == null)
            {
                return;
            }

            if (CheckBox_Pedal_ESPNow_autoconnect != null)
            {
                CheckBox_Pedal_ESPNow_autoconnect.IsChecked = Plugin.Settings.Pedal_ESPNow_auto_connect_flag;
            }

            if (textbox_SSID != null)
            {
                textbox_SSID.Text = Plugin.Settings.SSID_string ?? string.Empty;
            }

            if (textbox_PASS != null)
            {
                textbox_PASS.Password = Plugin.Settings.PASS_string ?? string.Empty;
            }

            if (CheckBox_XPlaneUdpEnabled != null)
            {
                CheckBox_XPlaneUdpEnabled.IsChecked = Plugin.Settings.XPlaneUdpEnabled;
            }

            if (TextBox_XPlanePort != null)
            {
                TextBox_XPlanePort.Text = Plugin.Settings.XPlaneUdpPort.ToString();
            }

            UpdateActiveAircraftLabel(null, null);
        }

        private void UpdateSerialPortList_click(object sender, RoutedEventArgs e)
        {
            UpdateSerialPortList();
        }

        private void ESPNow_SerialPortSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            if (SerialPortSelection_ESPNow.SelectedItem is string portName)
            {
                Plugin.Settings.ESPNow_port = portName;
                TextBox_debugOutput.Text = $"Gateway port selected: {portName}";
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

        private void CheckBox_XPlaneUdpEnabled_Checked(object sender, RoutedEventArgs e)
        {
            UpdateXPlaneUdpSettings(true);
        }

        private void CheckBox_XPlaneUdpEnabled_Unchecked(object sender, RoutedEventArgs e)
        {
            UpdateXPlaneUdpSettings(false);
        }

        private void TextBox_XPlanePort_TextChanged(object sender, TextChangedEventArgs e)
        {
            UpdateXPlaneUdpSettings(CheckBox_XPlaneUdpEnabled?.IsChecked == true);
        }

        private void UpdateXPlaneUdpSettings(bool enabled)
        {
            if (Plugin == null)
            {
                return;
            }

            int port = Plugin.Settings.XPlaneUdpPort;
            if (TextBox_XPlanePort != null && int.TryParse(TextBox_XPlanePort.Text, out int parsedPort))
            {
                port = parsedPort;
            }

            Plugin.ApplyXPlaneUdpSettings(enabled, port);
        }

        private void btn_save_aircraft_ffb_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            string carId = Plugin.GetActiveCarId();
            if (string.IsNullOrWhiteSpace(carId))
            {
                MessageBox.Show("No active aircraft detected.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = $"{carId}_ffb.json"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                if (!Plugin.Settings.AircraftFfbProfiles.TryGetValue(carId, out var profile))
                {
                    MessageBox.Show("No stored profile for current aircraft.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                    return;
                }

                string json = JsonConvert.SerializeObject(profile, Formatting.Indented);
                System.IO.File.WriteAllText(saveFileDialog.FileName, json);
            }
        }

        private void btn_load_aircraft_ffb_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            string carId = Plugin.GetActiveCarId();
            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
            };

            if (openFileDialog.ShowDialog() == true)
            {
                string json = System.IO.File.ReadAllText(openFileDialog.FileName);
                var profile = JsonConvert.DeserializeObject<DiyFfbPluginSettings.AircraftFfbProfile>(json);
                if (profile == null)
                {
                    MessageBox.Show("Invalid profile JSON.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                if (string.IsNullOrWhiteSpace(carId))
                {
                    Plugin.ApplyFfbProfileToCurrentSettings(profile);
                    Plugin.SetPendingFfbProfile(profile);
                    MessageBox.Show("Loaded profile into current settings (no active aircraft).", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                }
                else
                {
                    Plugin.ApplyAircraftFfbProfile(carId, profile);
                }
                RefreshXPlaneFfbSettings();
            }
        }

        private void btn_save_ffb_map_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = "ffb_profiles.json"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                string json = JsonConvert.SerializeObject(Plugin.Settings.AircraftFfbProfiles, Formatting.Indented);
                System.IO.File.WriteAllText(saveFileDialog.FileName, json);
            }
        }

        public bool ConfirmApplyPendingProfile(string carName, string carId)
        {
            string label = string.IsNullOrWhiteSpace(carName) ? carId : $"{carName} ({carId})";
            string message = $"A pending FFB profile is loaded without an active aircraft.\n\n" +
                             $"Apply it to {label} or discard and use the stored profile?";
            var result = MessageBox.Show(message, "FFB Profiles", MessageBoxButton.YesNo, MessageBoxImage.Question);
            return result == MessageBoxResult.Yes;
        }

        private void btn_load_ffb_map_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
            };

            if (openFileDialog.ShowDialog() == true)
            {
                string json = System.IO.File.ReadAllText(openFileDialog.FileName);
                var profiles = JsonConvert.DeserializeObject<System.Collections.Generic.Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>>(json);
                if (profiles == null)
                {
                    MessageBox.Show("Invalid profile map JSON.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                Plugin.ReplaceAircraftFfbProfiles(profiles);
                RefreshXPlaneFfbSettings();
            }
        }

        private void textbox_SSID_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (Plugin != null && textbox_SSID != null)
            {
                Plugin.Settings.SSID_string = textbox_SSID.Text ?? string.Empty;
            }
        }

        private void textbox_PASS_PasswordChanged(object sender, RoutedEventArgs e)
        {
            if (Plugin != null && textbox_PASS != null)
            {
                Plugin.Settings.PASS_string = textbox_PASS.Password ?? string.Empty;
            }
        }

        private void btn_connect_espnow_port_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin != null && Plugin.ESPsync_serialPort != null && Plugin.ESPsync_serialPort.IsOpen)
            {
                DisconnectFromPort();
                return;
            }

            if (SerialPortSelection_ESPNow.SelectedItem is string portName)
            {
                ConnectToPort(portName);
            }
        }

        private string GetOtaInfoUrl()
        {
            if (OTAChannel_Sel_2 != null && OTAChannel_Sel_2.IsChecked == true)
            {
                return OtaInfoUrlDev;
            }

            return OtaInfoUrlRelease;
        }

        private void btn_start_ota_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null || Plugin.ESPsync_serialPort == null || !Plugin.ESPsync_serialPort.IsOpen)
            {
                TextBox_debugOutput.Text = "Connect the gateway before starting OTA.";
                return;
            }

            string ssid = Plugin.Settings.SSID_string ?? string.Empty;
            string pass = Plugin.Settings.PASS_string ?? string.Empty;
            if (ssid.Length > MaxWifiCredentialLength || pass.Length > MaxWifiCredentialLength)
            {
                TextBox_debugOutput.Text = $"SSID and password must be {MaxWifiCredentialLength} characters or less.";
                return;
            }

            StartOtaUpdate startOtaUpdate = new StartOtaUpdate
            {
                WifiInfo = new WifiInfo
                {
                    Ssid = ssid,
                    Password = pass
                },
                InfoJsonUrl = GetOtaInfoUrl(),
                AllowDowngrades = Checkbox_Force_flash != null && Checkbox_Force_flash.IsChecked == true,
                Target = OtaTarget.GatewayOnly
            };

            Message msg = new Message { StartOtaUpdate = startOtaUpdate };
            Plugin.ESPsync_serialPort.WriteMessage(msg);
            TextBox_debugOutput.Text = "Gateway OTA update request sent.";
        }

        private void ConnectToPort(string portName)
        {
            if (Plugin == null)
            {
                return;
            }

            if (string.IsNullOrWhiteSpace(portName) || portName == "NA")
            {
                TextBox_debugOutput.Text = "Select a valid port before connecting.";
                return;
            }

            DisconnectFromPort();

            try
            {
                Plugin.ESPsync_serialPort = new ProtobufSerial<Message>(portName, 3000000);
                Plugin.ESPsync_serialPort.OnMessage += OnMessage;
                Plugin.ESPsync_serialPort.Open();
                Label_Status.Text = "Connected";
                btn_connect_espnow_port.Content = "Disconnect";
                TextBox_debugOutput.Text = $"Connected to {portName}.";
            }
            catch (Exception ex)
            {
                Label_Status.Text = "Disconnected";
                btn_connect_espnow_port.Content = "Connect";
                TextBox_debugOutput.Text = $"Failed to connect to {portName}: {ex.Message}";
            }
        }

        private void DisconnectFromPort()
        {
            if (Plugin == null)
            {
                return;
            }

            try
            {
                if (Plugin.ESPsync_serialPort != null)
                {
                    Plugin.ESPsync_serialPort.OnMessage -= OnMessage;
                    Plugin.ESPsync_serialPort.Close();
                }
            }
            catch (Exception ex)
            {
                TextBox_debugOutput.Text = $"Disconnect failed: {ex.Message}";
            }

            foreach (Axis axis in axes.Values)
            {
                axis.SerialChannel = null;
            }

            Label_Status.Text = "Disconnected";
            if (btn_connect_espnow_port != null)
            {
                btn_connect_espnow_port.Content = "Connect";
            }
        }

        public void CloseSerialPorts()
        {
            DisconnectFromPort();
        }

        private void UpdateUploadButtonLabels()
        {
            string label = PersistConfig ? "Upload and Persist" : "Upload";
            btn_upload_axis_config.Content = label;
            btn_upload_function_config.Content = label;
        }

        private void OnPreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                persistConfigModifier = true;
                UpdateUploadButtonLabels();
            }
        }

        private void OnPreviewKeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                persistConfigModifier = false;
                UpdateUploadButtonLabels();
            }
        }

        private void OnAxisSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            UpdateAxisSelection();
        }

        private void OnFunctionSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            UpdateFunctionSelection();
        }

        private void UpdateAxisSelection()
        {
            if (tc_axis_selection == null)
            {
                return;
            }

            if (tc_axis_selection.SelectedItem is KeyValuePair<AxisID, Axis> axisEntry)
            {
                selected_axis_id = axisEntry.Key;
                if (Plugin != null)
                {
                    Plugin.Settings.axis_tab_selected = (uint)Math.Max(0, (int)selected_axis_id - 1);
                }
                uc_axis_config.UpdateConfig(axisEntry.Value.Config);
            }
        }

        private void UpdateFunctionSelection()
        {
            if (tc_function_selection == null)
            {
                return;
            }

            if (tc_function_selection.SelectedItem is KeyValuePair<FunctionID, Function> functionEntry)
            {
                selected_function_id = functionEntry.Key;
                if (Plugin != null)
                {
                    Plugin.Settings.function_tab_selected = (uint)Math.Max(0, (int)selected_function_id - 1);
                    indexOfSelectedPedal_u = Plugin.Settings.function_tab_selected;
                }
                uc_function_config.SwitchFunction(functionEntry.Value);
            }
        }

        private void Function_Tab_seleciton_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
        }

        private void OnABSTestStateChange(bool state)
        {
            if (Plugin != null)
            {
                Plugin.sendAbsSignal = state;
            }
        }

        private void OnDebugMessage(string message)
        {
            TextBox_debugOutput.Text = message;
        }

        private void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            uc_function_config.OnKinematicParametersChanged(parameters);
        }

        private void OnOnlineStateChange(AxisID axis_id, bool new_online_state)
        {
            string msg = new_online_state
                ? $"Axis {(int)axis_id} Connected"
                : $"Axis {(int)axis_id} Disconnected";

            if (!new_online_state && last_known_functions.ContainsKey(axis_id))
            {
                functions[last_known_functions[axis_id]].OnAxisRemoved(axis_id);
                last_known_functions.Remove(axis_id);
            }

            ToastNotification("Axis Connection", msg);
        }

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

        private void OnMessage(object sender, object message)
        {
            ProtobufSerial<Message> port = sender as ProtobufSerial<Message>;
            Message msg = message as Message;
            if (port == null || msg == null)
            {
                return;
            }

            Dispatcher.Invoke(() => HandleMessage(port, msg));
        }

        private void HandleMessage(ProtobufSerial<Message> port, Message msg)
        {
            switch (msg.PayloadCase)
            {
                case Message.PayloadOneofCase.AxisState:
                    HandleAxisState(port, msg.AxisState);
                    break;
                case Message.PayloadOneofCase.GatewayState:
                    ProcessGatewayState(port, msg.GatewayState);
                    break;
                case Message.PayloadOneofCase.AxisConfig:
                    RegisterAxisChannel(msg.AxisConfig.AxisId, port);
                    OnAxisConfigUpdate(msg.AxisConfig);
                    break;
                case Message.PayloadOneofCase.FunctionConfig:
                    OnFunctionConfigUpdate(msg.FunctionConfig);
                    break;
                case Message.PayloadOneofCase.AxisLogMessage:
                    AppendLog(BuildAxisLogLine(msg.AxisLogMessage));
                    break;
                case Message.PayloadOneofCase.GatewayLogMessage:
                    AppendLog(BuildGatewayLogLine(msg.GatewayLogMessage));
                    break;
                case Message.PayloadOneofCase.ActiveFunction:
                    RegisterAxisChannel(msg.ActiveFunction.AxisId, port);
                    UpdateActiveFunction(msg.ActiveFunction);
                    break;
                default:
                    break;
            }
        }

        private void HandleAxisState(ProtobufSerial<Message> port, AxisState axisState)
        {
            RegisterAxisChannel(axisState.AxisId, port);
            uc_function_config.OnAxisStateUpdate(axisState);
            uc_axis_config.OnAxisStateUpdate(axisState);
            UpdateVjoy(axisState);
        }

        public void RefreshXPlaneFfbSettings()
        {
            uc_function_config.RefreshXPlaneFfbSettings();
        }

        public void UpdateActiveAircraftLabel(string carName, string carId)
        {
            if (TextBlock_ActiveAircraft == null)
            {
                return;
            }

            string label = string.IsNullOrWhiteSpace(carName) ? "-" : carName;
            TextBlock_ActiveAircraft.Text = label;
            TextBlock_ActiveAircraft.ToolTip = string.IsNullOrWhiteSpace(carId) ? null : carId;
        }

        private void UpdateVjoy(AxisState axisState)
        {
            if (joystick == null || Plugin == null || Plugin.Settings.vjoy_output_flag != 1)
            {
                return;
            }

            float position = axisState.Position;
            if (position < 0f)
            {
                position = 0f;
            }
            else if (position > 1f)
            {
                position = 1f;
            }

            int value = (int)Math.Round(position * 65535f);
            switch (axisState.AxisId)
            {
                case AxisID._1:
                    joystick.SetAxis(value, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RX);
                    break;
                case AxisID._2:
                    joystick.SetAxis(value, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RY);
                    break;
                case AxisID._3:
                    joystick.SetAxis(value, Plugin.Settings.vjoy_order, HID_USAGES.HID_USAGE_RZ);
                    break;
            }
        }

        private void RegisterAxisChannel(AxisID axisId, ProtobufSerial<Message> port)
        {
            if (axisId == AxisID.AxisUndefined)
            {
                return;
            }

            if (axes.TryGetValue(axisId, out Axis axis))
            {
                axis.SerialChannel = port;
            }
        }

        private void ProcessGatewayState(ProtobufSerial<Message> port, GatewayState state)
        {
            for (AxisID axisId = AxisID._1; axisId <= AxisID._8; axisId++)
            {
                int flag = 1 << ((int)axisId - 1);
                if ((state.AxesPresent & flag) != 0)
                {
                    axes[axisId].SerialChannel = port;
                }
                else
                {
                    axes[axisId].SerialChannel = null;
                }
            }
        }

        private string BuildAxisLogLine(AxisLogMessage msg)
        {
            if (msg.AxisId != AxisID.AxisUndefined)
            {
                return $"A{(int)msg.AxisId} : {msg.Msg}";
            }

            return $"A? : {msg.Msg}";
        }

        private string BuildGatewayLogLine(GatewayLogMessage msg)
        {
            if (msg.GatewayId != GatewayID.GatewayUndefined)
            {
                return $"G{(int)msg.GatewayId} : {msg.Msg}";
            }

            return $"G? : {msg.Msg}";
        }

        private void AppendLog(string line)
        {
            TextBox_Log.AppendText(line + Environment.NewLine);
            TextBox_Log.ScrollToEnd();
        }

        private void UpdateActiveFunction(ActiveFunction msg)
        {
            if (last_known_functions.ContainsKey(msg.AxisId))
            {
                functions[last_known_functions[msg.AxisId]].OnAxisRemoved(msg.AxisId);
                last_known_functions.Remove(msg.AxisId);
            }

            if (msg.FunctionId != FunctionID.Undefined)
            {
                last_known_functions[msg.AxisId] = msg.FunctionId;
                functions[msg.FunctionId].OnAxisAdded(msg.AxisId);
            }
        }

        private void OnUploadFunctionConfigClicked(object sender, RoutedEventArgs e)
        {
            if (selected_function_id == FunctionID.Undefined)
            {
                TextBox_debugOutput.Text = "No function selected.";
                return;
            }

            FunctionConfig functionConfig = functions[selected_function_id].Config;
            UploadFunctionConfig(functionConfig, PersistConfig);
        }

        private void UploadFunctionConfig(FunctionConfig functionConfig, bool store)
        {
            functionConfig.Base.Store = store;
            Message msg = new Message { FunctionConfig = functionConfig };
            bool broadcastRequired = false;
            foreach (var linkedAxisId in functionConfig.Base.LinkedAxes)
            {
                var axisId = linkedAxisId & AxisID.Mask;
                if (axisId != AxisID.AxisUndefined)
                {
                    var serialChannel = axes[axisId].SerialChannel;
                    if (serialChannel != null && serialChannel != Plugin.ESPsync_serialPort)
                    {
                        serialChannel.WriteMessage(msg);
                    }
                    else
                    {
                        broadcastRequired = true;
                    }
                }
            }

            if (broadcastRequired && Plugin.ESPsync_serialPort != null && Plugin.ESPsync_serialPort.IsOpen)
            {
                Plugin.ESPsync_serialPort.WriteMessage(msg);
            }
        }

        private void OnFunctionConfigUpdate(FunctionConfig newFunctionConfig)
        {
            FunctionID newFunctionId = newFunctionConfig.Base.FunctionId;
            if (newFunctionId == FunctionID.Undefined)
            {
                TextBox_debugOutput.Text = "Function ID undefined (ignored)";
                return;
            }

            functions[newFunctionId].Config = newFunctionConfig;
            if (newFunctionId == selected_function_id)
            {
                uc_function_config.SwitchFunction(functions[newFunctionId]);
            }
        }

        private void OnAxisConfigUpdate(AxisConfig newAxisConfig)
        {
            AxisID newAxisId = newAxisConfig.AxisId;
            if (newAxisId != AxisID.AxisUndefined && newAxisId <= AxisID._8)
            {
                axes[newAxisId].Config = newAxisConfig;
                if (newAxisId == selected_axis_id)
                {
                    uc_axis_config.UpdateConfig(axes[newAxisId].Config);
                }
            }
            else
            {
                TextBox_debugOutput.Text = $"Invalid axis ID ({(int)newAxisId})";
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
                    var jsonParser = new JsonParser(JsonParser.Settings.Default);
                    ConfigItemsList msg = (ConfigItemsList)jsonParser.Parse(content, ConfigItemsList.Descriptor);
                    Dictionary<AxisID, AxisConfig> axisConfigs = new Dictionary<AxisID, AxisConfig>();
                    Dictionary<FunctionID, FunctionConfig> functionConfigs = new Dictionary<FunctionID, FunctionConfig>();

                    foreach (var item in msg.ConfigItems)
                    {
                        switch (item.ItemCase)
                        {
                            case ConfigItem.ItemOneofCase.AxisConfig:
                                axisConfigs[item.AxisConfig.AxisId] = item.AxisConfig;
                                if (axes.TryGetValue(item.AxisConfig.AxisId, out Axis axis))
                                {
                                    axis.SelectedToLoad = true;
                                    axis.SelectableToLoad = true;
                                }
                                break;
                            case ConfigItem.ItemOneofCase.FunctionConfig:
                                functionConfigs[item.FunctionConfig.Base.FunctionId] = item.FunctionConfig;
                                if (functions.TryGetValue(item.FunctionConfig.Base.FunctionId, out Function function))
                                {
                                    function.SelectedToLoad = true;
                                    function.SelectableToLoad = true;
                                }
                                break;
                        }
                    }

                    loadSelectionDialog = new LoadSelectionDialog(this, axisConfigs, functionConfigs);
                    loadSelectionDialog.Closed += OnLoadSelectionClosed;
                    btn_load_axis_config_from_file.IsEnabled = false;
                    btn_load_function_config_from_file.IsEnabled = false;
                    btn_store_function_config_to_file.IsEnabled = false;
                    btn_store_axis_config_to_file.IsEnabled = false;
                    loadSelectionDialog.Show();
                }
                catch (Exception caughtEx)
                {
                    MessageBox.Show($"Error loading {openFileDialog.FileName}: {caughtEx.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
                }
            }
        }

        private void OnLoadSelectionClosed(object sender, EventArgs e)
        {
            if (loadSelectionDialog == null)
            {
                return;
            }

            foreach (var axis in axes.Values)
            {
                if (loadSelectionDialog.LoadRequested && axis.SelectedToLoad && loadSelectionDialog.axis_configs.TryGetValue(axis.ID, out AxisConfig cfg))
                {
                    OnAxisConfigUpdate(cfg);
                }
                if (loadSelectionDialog.UploadRequested && axis.SelectedToLoad)
                {
                    axis.UploadConfig(false);
                }
                axis.SelectedToLoad = false;
                axis.SelectableToLoad = false;
            }
            foreach (var function in functions.Values)
            {
                if (loadSelectionDialog.LoadRequested && function.SelectedToLoad && loadSelectionDialog.function_configs.TryGetValue(function.ID, out FunctionConfig cfg))
                {
                    OnFunctionConfigUpdate(cfg);
                }
                if (loadSelectionDialog.UploadRequested && function.SelectedToLoad && loadSelectionDialog.function_configs.TryGetValue(function.ID, out FunctionConfig uploadCfg))
                {
                    UploadFunctionConfig(uploadCfg, false);
                }
                function.SelectedToLoad = false;
                function.SelectableToLoad = false;
            }
            btn_load_axis_config_from_file.IsEnabled = true;
            btn_load_function_config_from_file.IsEnabled = true;
            btn_store_function_config_to_file.IsEnabled = true;
            btn_store_axis_config_to_file.IsEnabled = true;
        }

        private void OnSaveSelectionClosed(object sender, EventArgs e)
        {
            if (saveSelectionDialog == null)
            {
                return;
            }

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
            if (selected_axis_id == AxisID.AxisUndefined)
            {
                TextBox_debugOutput.Text = "No axis selected.";
                return;
            }

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
    }
}
