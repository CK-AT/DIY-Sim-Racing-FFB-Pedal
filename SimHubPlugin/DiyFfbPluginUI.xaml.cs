using Google.Protobuf;
using Newtonsoft.Json;
using ProtbufTest;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.IO.Compression;
using System.IO.Ports;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using DiyFfb.Controls;
using DiyFfb.GraphEditor;
using DiyFfb.ProfileBrowser;
using DiyFfb.TieredConfig;
using System.Windows.Data;
using vJoyInterfaceWrap;
using Windows.UI.Notifications;

namespace DiyFfb
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
        public System.Collections.ObjectModel.ObservableCollection<OtaTargetEntry> ota_targets { get; } = new System.Collections.ObjectModel.ObservableCollection<OtaTargetEntry>();
        public System.Collections.ObjectModel.ObservableCollection<UiLogEntry> UiLogEntries { get; } = new System.Collections.ObjectModel.ObservableCollection<UiLogEntry>();
        public System.Collections.ObjectModel.ObservableCollection<LogSourceFilterItem> LogSourceFilters { get; } = new System.Collections.ObjectModel.ObservableCollection<LogSourceFilterItem>();
        public ICollectionView UiLogView { get; }
        private readonly Dictionary<AxisID, OtaTargetEntry> ota_axis_targets = new Dictionary<AxisID, OtaTargetEntry>();
        private readonly Dictionary<GatewayID, OtaTargetEntry> ota_gateway_targets = new Dictionary<GatewayID, OtaTargetEntry>();
        private readonly Dictionary<AxisID, string> ota_axis_logs = new Dictionary<AxisID, string>();
        private readonly Dictionary<GatewayID, string> ota_gateway_logs = new Dictionary<GatewayID, string>();
        private readonly Dictionary<string, LogSourceFilterItem> logSourceFilterLookup = new Dictionary<string, LogSourceFilterItem>();
        private readonly Dictionary<AxisID, string> ota_axis_versions = new Dictionary<AxisID, string>();
        private readonly Dictionary<GatewayID, string> ota_gateway_versions = new Dictionary<GatewayID, string>();
        private GatewayID last_gateway_id = GatewayID.GatewayUndefined;
        private readonly HashSet<AxisID> seenAxisSources = new HashSet<AxisID>();
        private readonly HashSet<GatewayID> seenGatewaySources = new HashSet<GatewayID>();
        private OtaSelectionDialog otaSelectionDialog;
        private CancellationTokenSource otaUpdateCancellation;
        private bool otaAclWarned;
        private bool suppressSerialPortSelectionChange;
        private ProtobufSerial<Message> attachedGatewayPort;

        internal vJoyInterfaceWrap.vJoy joystick;

        private readonly SolidColorBrush defaultcolor = new SolidColorBrush(Colors.White);
        private readonly SolidColorBrush lightcolor = new SolidColorBrush(Color.FromRgb(0xD8, 0xE6, 0xFF));

        public SolidColorBrush MouseDownColor { get { return lightcolor; } }
        public SolidColorBrush MouseUpColor { get { return defaultcolor; } }

        private bool persistConfigModifier;
        private bool updatingXPlaneUdp;

        private bool PersistConfig => persistConfigModifier;

        private const int UiLogMaxEntries = 200;
        private const int MaxWifiCredentialLength = 63;
        private const string OtaInfoUrlDefault = "https://github.com/CK-AT/DIY-Sim-Racing-FFB-Pedal/raw/refs/heads/main/OTA/update_info.json";

        private LocalOtaServer otaServer;

        private SaveSelectionDialog saveSelectionDialog;
        private LoadSelectionDialog loadSelectionDialog;
        private AxisRequestQueue axisRequestQueue;
        private GraphEditorWindow graphEditorWindow;
        private string lastGraphEditorPath;
        private bool isUpdatingOverrideUi;
        private bool suppressUserProfileSelectionChange;

        private sealed class OverrideFieldTag
        {
            public OverrideFieldTag(FunctionID functionId, string fieldName, TextBox textBox, Button clearButton, TextBlock badge)
            {
                FunctionId = functionId;
                FieldName = fieldName;
                TextBox = textBox;
                ClearButton = clearButton;
                Badge = badge;
            }

            public FunctionID FunctionId { get; }
            public string FieldName { get; }
            public TextBox TextBox { get; }
            public Button ClearButton { get; }
            public TextBlock Badge { get; }
        }

        private sealed class OverrideCheckboxTag
        {
            public OverrideCheckboxTag(FunctionID functionId, string fieldName, CheckBox checkbox, TextBlock stateLabel, TextBlock badge)
            {
                FunctionId = functionId;
                FieldName = fieldName;
                Checkbox = checkbox;
                StateLabel = stateLabel;
                Badge = badge;
            }

            public FunctionID FunctionId { get; }
            public string FieldName { get; }
            public CheckBox Checkbox { get; }
            public TextBlock StateLabel { get; }
            public TextBlock Badge { get; }
        }

        private enum UiLogLevel
        {
            Info,
            Warning,
            Error,
            Debug
        }

        public enum UiLogSourceKind
        {
            Plugin,
            Gateway,
            Axis
        }

        public sealed class LogSourceFilterItem : INotifyPropertyChanged
        {
            private bool isSelected;

            public UiLogSourceKind Kind { get; }
            public int? Id { get; }
            public string Label { get; }

            public bool IsSelected
            {
                get => isSelected;
                set
                {
                    if (isSelected == value)
                    {
                        return;
                    }
                    isSelected = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsSelected)));
                }
            }

            public event PropertyChangedEventHandler PropertyChanged;

            public LogSourceFilterItem(UiLogSourceKind kind, int? id, string label, bool isSelected)
            {
                Kind = kind;
                Id = id;
                Label = label;
                this.isSelected = isSelected;
            }
        }

        public sealed class UiLogEntry
        {
            public string Time { get; set; } = string.Empty;
            public string Level { get; set; } = string.Empty;
            public string Source { get; set; } = string.Empty;
            public string Message { get; set; } = string.Empty;
            public UiLogSourceKind SourceKind { get; set; }
            public int? SourceId { get; set; }
        }

        public DiyFfbPluginUI()
        {
            InitializeComponent();
            UiLogView = CollectionViewSource.GetDefaultView(UiLogEntries);
            UiLogView.Filter = FilterLogEntry;
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

            for (FunctionID id = FunctionID.BrakePedal; id <= FunctionID.FlightStickCollective; id++)
            {
                Function function = new Function(id);
                // Config will be populated later in PopulateFunctionConfigsFromBaselines() after manager is initialized
                function.Config = FunctionConfigControl.GetDefaultConfig(id);
                functions[id] = function;
            }

            for (AxisID id = AxisID._1; id <= AxisID._8; id++)
            {
                Axis axis = new Axis(id);
                axis.Config = AxisConfigControl.GetDefaultConfig(id);
                axis.OnlineStateChanged += OnOnlineStateChange;
                axis.RequestDispatcher = EnqueueAxisRequest;
                axes[id] = axis;
            }

            axisRequestQueue = new AxisRequestQueue(this);

            UpdateUploadButtonLabels();
            InitializeLogSourceFilters();

            UpdateSerialPortList();
            InitializeSystemSettings();

            if (plugin != null)
            {
                plugin.ActiveGraphChanged += OnActiveGraphChanged_UI;
                plugin.GraphParamChanged += OnGraphParamChanged_Vehicle;
                plugin.ParamMigrationDetected += OnParamMigrationDetected;
                plugin.FunctionConfigManager.FunctionConfigChanged += OnMergedFunctionConfigChanged;
                plugin.AxisConfigManager.AxisConfigChanged += OnMergedAxisConfigChanged;
            }
            UpdateVehicleTabHeader();
            RefreshVehicleParams();

            if (Plugin.Settings.Pedal_ESPNow_auto_connect_flag
                && !string.IsNullOrWhiteSpace(Plugin.Settings.ESPNow_port)
                && SerialPort.GetPortNames().Any(port => string.Equals(port, Plugin.Settings.ESPNow_port, StringComparison.OrdinalIgnoreCase)))
            {
                ConnectToPort(Plugin.Settings.ESPNow_port);
            }

            // Populate function configs from stored baselines BEFORE selecting initial function,
            // so SwitchFunction displays merged values instead of defaults
            PopulateFunctionConfigsFromBaselines();

            SetInitialSelections();

            InitializeVjoyIfEnabled();
        }

        /// <summary>
        /// Populate Function.Config from stored baselines in the manager.
        /// Called after InitializeManagerFromSettings() has loaded baselines.
        /// </summary>
        public void PopulateFunctionConfigsFromBaselines()
        {
            if (Plugin == null)
                return;

            foreach (var kvp in functions)
            {
                var functionId = (int)kvp.Key;
                var function = kvp.Value;

                // Get baseline + overrides from manager
                var config = Plugin.GetInitialFunctionConfig(functionId);
                if (config != null)
                {
                    function.Config = config;
                }
            }
        }

        public KinematicParameters GetKinematicParameters(AxisID axis_id)
        {
            if (axes.TryGetValue(axis_id, out Axis axis))
            {
                if (!axis.HasAxisConfig)
                {
                    return null;
                }
                return axis.Config?.KinematicParameters;
            }
            return null;
        }

        public void NotifyGatewayPortAutoConnected(string portName)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() => NotifyGatewayPortAutoConnected(portName));
                return;
            }

            UpdateSerialPortList();

            if (Plugin?.ESPsync_serialPort == null)
            {
                return;
            }

            if (attachedGatewayPort != Plugin.ESPsync_serialPort)
            {
                if (attachedGatewayPort != null)
                {
                    attachedGatewayPort.OnMessage -= OnMessage;
                }
                Plugin.ESPsync_serialPort.OnMessage += OnMessage;
                attachedGatewayPort = Plugin.ESPsync_serialPort;
            }

            if (SerialPortSelection_ESPNow != null && !string.IsNullOrWhiteSpace(portName))
            {
                suppressSerialPortSelectionChange = true;
                SerialPortSelection_ESPNow.SelectedItem = portName;
                suppressSerialPortSelectionChange = false;
            }

            if (Plugin.ESPsync_serialPort.IsOpen)
            {
                Label_Status.Text = "Connected";
                btn_connect_espnow_port.Content = "Disconnect";
                SetDebugOutput($"Auto-connected to {portName}.");
            }
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

            suppressSerialPortSelectionChange = true;
            SerialPortSelection_ESPNow.ItemsSource = ports;

            if (Plugin != null && !string.IsNullOrWhiteSpace(Plugin.Settings.ESPNow_port) && ports.Contains(Plugin.Settings.ESPNow_port))
            {
                SerialPortSelection_ESPNow.SelectedItem = Plugin.Settings.ESPNow_port;
            }
            else if (ports.Count == 1 && ports[0] == "NA")
            {
                SerialPortSelection_ESPNow.SelectedIndex = 0;
            }
            else
            {
                SerialPortSelection_ESPNow.SelectedIndex = -1;
            }
            suppressSerialPortSelectionChange = false;
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

            if (TextBox_OtaCustomUrl != null)
            {
                TextBox_OtaCustomUrl.Text = Plugin.Settings.OtaCustomUrl ?? string.Empty;
            }

            if (TextBox_OtaFfbotaPath != null)
            {
                TextBox_OtaFfbotaPath.Text = Plugin.Settings.OtaLocalFfbotaPath ?? string.Empty;
            }

            if (TextBox_OtaLocalPort != null)
            {
                TextBox_OtaLocalPort.Text = Plugin.Settings.OtaLocalPort.ToString();
            }

            if (OTAUrl_Custom != null)
            {
                OTAUrl_Custom.IsChecked = Plugin.Settings.OtaUseCustomUrl;
            }
            if (OTAUrl_Default != null && OTAUrl_Custom?.IsChecked != true)
            {
                OTAUrl_Default.IsChecked = true;
            }

            if (OTASource_Local != null)
            {
                OTASource_Local.IsChecked = Plugin.Settings.OtaUseLocalSource;
            }
            if (OTASource_Public != null && OTASource_Local?.IsChecked != true)
            {
                OTASource_Public.IsChecked = true;
            }

            UpdateOtaSourceUi();
            UpdateOtaInfoReadout("-", "-", "-");
            if (Plugin.Settings.OtaUseLocalSource && !string.IsNullOrWhiteSpace(Plugin.Settings.OtaLocalFfbotaPath))
            {
                if (TryLoadFfbotaManifest(Plugin.Settings.OtaLocalFfbotaPath, out OtaManifest manifest, out _))
                {
                    UpdateOtaInfoReadout(manifest.Board, manifest.Version, manifest.Md5);
                }
            }

            UpdateActiveAircraftLabel(null, null);
            RefreshGraphSelectionUI();
            RefreshXPlaneUdpSettings();
            RefreshUserProfileUi();
        }

        public void RefreshGraphSelection()
        {
            RefreshGraphSelectionUI();
        }

        private void RefreshXPlaneUdpSettings()
        {
            if (Plugin == null)
            {
                return;
            }

            updatingXPlaneUdp = true;
            if (CheckBox_XPlaneUdpEnabled != null)
            {
                CheckBox_XPlaneUdpEnabled.IsChecked = Plugin.Settings.XPlaneUdpEnabled;
            }
            if (TextBox_XPlanePort != null)
            {
                TextBox_XPlanePort.Text = Plugin.Settings.XPlaneUdpPort.ToString();
            }
            updatingXPlaneUdp = false;
        }

        private void RefreshUserProfileUi()
        {
            if (Plugin == null || ComboBox_UserProfile == null)
            {
                return;
            }

            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            if (Plugin.Settings.UserPreferencesProfiles != null)
            {
                foreach (var key in Plugin.Settings.UserPreferencesProfiles.Keys)
                {
                    if (!string.IsNullOrWhiteSpace(key))
                    {
                        names.Add(key);
                    }
                }
            }

            if (!string.IsNullOrWhiteSpace(Plugin.Settings.CurrentUserProfile))
            {
                names.Add(Plugin.Settings.CurrentUserProfile);
            }

            var windowsUser = System.Environment.UserName;
            if (!string.IsNullOrWhiteSpace(windowsUser))
            {
                names.Add(windowsUser);
            }

            var ordered = names.OrderBy(n => n, StringComparer.OrdinalIgnoreCase).ToList();
            suppressUserProfileSelectionChange = true;
            ComboBox_UserProfile.ItemsSource = ordered;
            if (!string.IsNullOrWhiteSpace(Plugin.Settings.CurrentUserProfile))
            {
                ComboBox_UserProfile.SelectedItem = Plugin.Settings.CurrentUserProfile;
            }
            else if (ordered.Count > 0)
            {
                ComboBox_UserProfile.SelectedIndex = 0;
            }
            suppressUserProfileSelectionChange = false;

            if (TextBlock_UserProfileInfo != null)
            {
                TextBlock_UserProfileInfo.Text = $"Profiles: {ordered.Count}";
            }
        }

        private void SetCurrentUserProfile(string userProfile)
        {
            if (Plugin == null)
            {
                return;
            }

            Plugin.SetCurrentUserProfile(userProfile);
            RefreshUserProfileUi();
            RefreshVehicleParams();
        }

        private void RefreshGraphSelectionUI()
        {
            if (Plugin == null)
            {
                return;
            }

            // Update Vehicle tab info display
            if (TextBlock_VehicleTabActiveGraph != null)
            {
                TextBlock_VehicleTabActiveGraph.Text = Plugin.GetActiveGraphStatus();
            }

            // Note: Do not auto-sync the graph editor window here.
            // The editor should be allowed to browse includes without being forced back to the vehicle graph.
            // Auto-sync only happens when opening the editor window initially.
        }

        private void UpdateSerialPortList_click(object sender, RoutedEventArgs e)
        {
            UpdateSerialPortList();
        }

        private void btn_restart_all_axes_Click(object sender, RoutedEventArgs e)
        {
            int sentCount = 0;
            int totalCount = 0;
            foreach (var axis in axes.Values)
            {
                if (axis.ID == AxisID.AxisUndefined)
                {
                    continue;
                }
                if (!axis.IsOnline)
                {
                    continue;
                }
                totalCount++;
                if (SendAxisRequest(axis.ID, AxisRequestType.Restart, null))
                {
                    sentCount++;
                }
            }

            if (totalCount == 0)
            {
                SetDebugOutput("Restart: No axes configured.", UiLogLevel.Warning);
            }
            else if (sentCount == 0)
            {
                SetDebugOutput("Restart: No axes reachable.", UiLogLevel.Warning);
            }
            else if (sentCount == totalCount)
            {
                SetDebugOutput($"Restart: Sent to {sentCount} axes.");
            }
            else
            {
                SetDebugOutput($"Restart: Sent to {sentCount} of {totalCount} axes.");
            }
        }

        private void ESPNow_SerialPortSelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            if (SerialPortSelection_ESPNow.SelectedItem is string portName)
            {
                if (suppressSerialPortSelectionChange)
                {
                    return;
                }
                if (string.IsNullOrWhiteSpace(portName) || portName == "NA")
                {
                    return;
                }
                Plugin.Settings.ESPNow_port = portName;
                SetDebugOutput($"Gateway port selected: {portName}");
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

        private void btn_save_aircraft_ffb_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            string profileKey = Plugin.GetActiveProfileKey();
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                ThemedMessageBox.Show("No active aircraft detected.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            // Use a safe filename based on the profile key
            string safeFileName = string.Join("_", profileKey.Split(System.IO.Path.GetInvalidFileNameChars()));
            Microsoft.Win32.SaveFileDialog saveFileDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = $"{safeFileName}_ffb.json"
            };

            if (saveFileDialog.ShowDialog() == true)
            {
                if (!Plugin.Settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile))
                {
                    ThemedMessageBox.Show("No stored profile for current aircraft.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                    return;
                }

                var exported = new DiyFfbPluginSettings.ExportedProfile
                {
                    ProfileKey = profileKey,
                    GraphPath = Plugin.GetActiveGraphPath(),
                    ExportedAt = System.DateTime.UtcNow.ToString("o"),
                    Profile = profile
                };
                string json = JsonConvert.SerializeObject(exported, Formatting.Indented);
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
                DiyFfbPluginSettings.AircraftFfbProfile profile;
                string sourceGraphPath = null;

                // Try new ExportedProfile format first
                var exported = JsonConvert.DeserializeObject<DiyFfbPluginSettings.ExportedProfile>(json);
                if (exported?.Profile != null && exported.Version >= 1)
                {
                    profile = exported.Profile;
                    sourceGraphPath = exported.GraphPath;
                }
                else
                {
                    // Fall back to legacy format (just AircraftFfbProfile)
                    profile = JsonConvert.DeserializeObject<DiyFfbPluginSettings.AircraftFfbProfile>(json);
                }

                if (profile == null)
                {
                    ThemedMessageBox.Show("Invalid profile JSON.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                // Check for graph path mismatch
                string currentGraphPath = Plugin.GetActiveGraphPath();
                if (!string.IsNullOrWhiteSpace(sourceGraphPath) && !string.IsNullOrWhiteSpace(currentGraphPath))
                {
                    if (!string.Equals(sourceGraphPath, currentGraphPath, System.StringComparison.OrdinalIgnoreCase))
                    {
                        var mismatchResult = ThemedMessageBox.Show(
                            $"This profile was created for a different graph:\n\n" +
                            $"Profile graph: {sourceGraphPath}\n" +
                            $"Current graph: {currentGraphPath}\n\n" +
                            $"Parameters may not match. Continue anyway?",
                            "Graph Mismatch",
                            MessageBoxButton.YesNo,
                            MessageBoxImage.Warning);
                        if (mismatchResult != MessageBoxResult.Yes)
                        {
                            return;
                        }
                    }
                }

                // Overwrite confirmation
                string profileKey = Plugin.GetActiveProfileKey();
                if (!string.IsNullOrWhiteSpace(profileKey) && Plugin.Settings.AircraftFfbProfiles.ContainsKey(profileKey))
                {
                    var overwriteResult = ThemedMessageBox.Show(
                        $"Overwrite existing profile for '{profileKey}'?",
                        "Confirm Overwrite",
                        MessageBoxButton.YesNo,
                        MessageBoxImage.Question);
                    if (overwriteResult != MessageBoxResult.Yes)
                    {
                        return;
                    }
                }

                if (string.IsNullOrWhiteSpace(carId))
                {
                    Plugin.ApplyFfbProfileToCurrentSettings(profile);
                    Plugin.SetPendingFfbProfile(profile);
                    ThemedMessageBox.Show("Loaded profile into current settings (no active aircraft).", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                }
                else
                {
                    Plugin.ApplyAircraftFfbProfile(carId, profile);
                }
                UpdateFunctionSelection();
            }
        }

        private void btn_manage_profiles_Click(object sender, RoutedEventArgs e)
        {
            ShowProfileBrowser(ProfileBrowserMode.ManageProfiles);
        }

        private void btn_review_params_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            try
            {
                // Use pending migration result if available, otherwise create empty result
                var result = _pendingMigrationResult;
                if (result == null)
                {
                    string graphPath = Plugin.GetActiveGraphPath();
                    if (string.IsNullOrWhiteSpace(graphPath))
                    {
                        ThemedMessageBox.Show("No graph is currently loaded.", "Review Params", MessageBoxButton.OK, MessageBoxImage.Information);
                        return;
                    }

                    result = new ParamMigrationResult
                    {
                        NewHash = GraphHashComputer.ComputeGraphTreeHash(
                            DiyFfbPlugin.ResolveGraphFilePath(graphPath),
                            Plugin.GetActiveVehicleGraph()),
                        CurrentSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>()
                    };
                }

                var dialog = new GraphEditor.ParamReviewWindow(Plugin, result);
                dialog.Owner = Window.GetWindow(this);
                dialog.ShowDialog();

                // Clear pending result after review
                _pendingMigrationResult = null;
            }
            catch (System.Exception ex)
            {
                ThemedMessageBox.Show($"Error opening review window: {ex.Message}\n\n{ex.StackTrace}", "Error", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void btn_store_profile_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            string profileKey = Plugin.GetActiveProfileKey();
            if (string.IsNullOrWhiteSpace(profileKey))
            {
                ThemedMessageBox.Show("No active vehicle detected.", "Store Profile", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            if (Plugin.StoreCurrentProfile())
            {
                ThemedMessageBox.Show($"Profile stored for {profileKey}.", "Store Profile", MessageBoxButton.OK, MessageBoxImage.Information);
            }
        }

        private void ShowProfileBrowser(ProfileBrowserMode mode)
        {
            if (Plugin == null)
            {
                return;
            }

            try
            {
                var dialog = new ProfileBrowserDialog(
                    Plugin,
                    mode,
                    Plugin.GetActiveGameId(),
                    Plugin.GetActiveCarId());

                var parentWindow = Window.GetWindow(this);
                if (parentWindow != null)
                {
                    dialog.Owner = parentWindow;
                }

                if (dialog.ShowDialog() == true && dialog.SelectedEntry != null)
                {
                    var entry = dialog.SelectedEntry;
                    string graphPath = entry.GraphPath;

                    // For templates, resolve the template path
                    if (entry.TemplateEntry != null)
                    {
                        graphPath = GraphTemplateRegistry.ResolveTemplatePath(
                            entry.TemplateEntry.TemplatePath,
                            AppDomain.CurrentDomain.BaseDirectory);
                    }

                    Plugin.ApplyProfileFromBrowser(graphPath, entry.Profile, dialog.UseTuning);
                    RefreshGraphSelection();
                }
            }
            catch (Exception ex)
            {
                ThemedMessageBox.Show($"Error opening Profile Browser:\n\n{ex.Message}\n\n{ex.StackTrace}",
                    "Profile Browser Error", MessageBoxButton.OK, MessageBoxImage.Error);
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
            var result = ThemedMessageBox.Show(message, "FFB Profiles", MessageBoxButton.YesNo, MessageBoxImage.Question);
            return result == MessageBoxResult.Yes;
        }

        public bool ConfirmSaveCurrentProfile(string carName, string carId)
        {
            string label = string.IsNullOrWhiteSpace(carName) ? carId : $"{carName} ({carId})";
            string message = $"Save FFB changes for {label} before switching aircraft?\n\n" +
                             "Choose Yes to update the stored profile or No to discard these changes.";
            var result = ThemedMessageBox.Show(message, "FFB Profiles", MessageBoxButton.YesNo, MessageBoxImage.Question);
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
                    ThemedMessageBox.Show("Invalid profile map JSON.", "FFB Profiles", MessageBoxButton.OK, MessageBoxImage.Warning);
                    return;
                }

                Plugin.ReplaceAircraftFfbProfiles(profiles);
                RefreshFunctionSelection();
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
            if (Plugin == null || updatingXPlaneUdp)
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

        private void ComboBox_UserProfile_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (suppressUserProfileSelectionChange)
            {
                return;
            }

            if (ComboBox_UserProfile.SelectedItem is string profileName)
            {
                SetCurrentUserProfile(profileName);
            }
        }

        private void btn_user_profile_refresh_Click(object sender, RoutedEventArgs e)
        {
            RefreshUserProfileUi();
        }

        private void btn_user_profile_create_Click(object sender, RoutedEventArgs e)
        {
            if (TextBox_UserProfileName == null)
            {
                return;
            }

            var profileName = TextBox_UserProfileName.Text?.Trim();
            if (string.IsNullOrWhiteSpace(profileName))
            {
                ThemedMessageBox.Show("Enter a user profile name first.", "User Profiles", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            SetCurrentUserProfile(profileName);
            TextBox_UserProfileName.Text = string.Empty;
        }

        private void TextBox_UserProfileName_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Enter)
            {
                return;
            }

            btn_user_profile_create_Click(sender, e);
            e.Handled = true;
        }

        private void btn_user_profile_delete_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            var profileName = ComboBox_UserProfile?.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(profileName))
            {
                return;
            }

            var result = ThemedMessageBox.Show(
                $"Delete user profile '{profileName}'? This removes stored user preference overrides.",
                "User Profiles",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);

            if (result != MessageBoxResult.Yes)
            {
                return;
            }

            if (Plugin.Settings.UserPreferencesProfiles != null)
            {
                Plugin.Settings.UserPreferencesProfiles.Remove(profileName);
            }

            var fallback = System.Environment.UserName;
            if (string.Equals(Plugin.Settings.CurrentUserProfile, profileName, StringComparison.OrdinalIgnoreCase))
            {
                SetCurrentUserProfile(fallback);
            }
            else
            {
                RefreshUserProfileUi();
                RefreshVehicleParams();
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
            if (OTAUrl_Custom != null && OTAUrl_Custom.IsChecked == true)
            {
                return TextBox_OtaCustomUrl?.Text?.Trim() ?? string.Empty;
            }

            return OtaInfoUrlDefault;
        }

        private void UpdateOtaSourceUi()
        {
            bool useLocal = OTASource_Local != null && OTASource_Local.IsChecked == true;
            if (Panel_OtaPublic != null)
            {
                Panel_OtaPublic.IsEnabled = !useLocal;
                Panel_OtaPublic.Opacity = useLocal ? 0.5 : 1.0;
            }
            if (Panel_OtaCustomUrl != null)
            {
                Panel_OtaCustomUrl.IsEnabled = !useLocal && OTAUrl_Custom?.IsChecked == true;
                Panel_OtaCustomUrl.Opacity = Panel_OtaCustomUrl.IsEnabled ? 1.0 : 0.5;
            }
            if (Panel_OtaLocal != null)
            {
                Panel_OtaLocal.IsEnabled = useLocal;
                Panel_OtaLocal.Opacity = useLocal ? 1.0 : 0.5;
            }
            if (Panel_OtaLocalPort != null)
            {
                Panel_OtaLocalPort.IsEnabled = useLocal;
                Panel_OtaLocalPort.Opacity = useLocal ? 1.0 : 0.5;
            }
        }

        private void OTAUrl_Default_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.OtaUseCustomUrl = false;
            }
            UpdateOtaSourceUi();
        }

        private void OTAUrl_Custom_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.OtaUseCustomUrl = true;
            }
            UpdateOtaSourceUi();
        }

        private void OTASource_Public_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.OtaUseLocalSource = false;
            }
            UpdateOtaSourceUi();
        }

        private void OTASource_Local_Checked(object sender, RoutedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.OtaUseLocalSource = true;
            }
            UpdateOtaSourceUi();
            string ffbotaPath = TextBox_OtaFfbotaPath?.Text ?? string.Empty;
            if (!string.IsNullOrWhiteSpace(ffbotaPath) && File.Exists(ffbotaPath))
            {
                if (TryLoadFfbotaManifest(ffbotaPath, out OtaManifest manifest, out string error))
                {
                    UpdateOtaInfoReadout(manifest.Board, manifest.Version, manifest.Md5);
                }
                else
                {
                    UpdateOtaInfoReadout("-", "-", "-");
                    SetDebugOutput(error, UiLogLevel.Error);
                }
            }
        }

        private void TextBox_OtaCustomUrl_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (Plugin != null)
            {
                Plugin.Settings.OtaCustomUrl = TextBox_OtaCustomUrl?.Text ?? string.Empty;
            }
        }

        private void TextBox_OtaLocalPort_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (Plugin == null)
            {
                return;
            }

            if (int.TryParse(TextBox_OtaLocalPort?.Text, out int port) && port > 0 && port <= 65535)
            {
                Plugin.Settings.OtaLocalPort = port;
            }
        }

        private void btn_browse_ota_ffbota_Click(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.OpenFileDialog openFileDialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "FFB OTA container (*.ffbota)|*.ffbota",
                DefaultExt = "ffbota",
            };

            if (openFileDialog.ShowDialog() == true)
            {
                TextBox_OtaFfbotaPath.Text = openFileDialog.FileName;
                if (Plugin != null)
                {
                    Plugin.Settings.OtaLocalFfbotaPath = openFileDialog.FileName;
                }
                if (TryLoadFfbotaManifest(openFileDialog.FileName, out OtaManifest manifest, out string error))
                {
                    UpdateOtaInfoReadout(manifest.Board, manifest.Version, manifest.Md5);
                }
                else
                {
                    UpdateOtaInfoReadout("-", "-", "-");
                    SetDebugOutput(error, UiLogLevel.Error);
                }
            }
        }

        private void UpdateOtaInfoReadout(string board, string version, string md5)
        {
            if (TextBlock_OtaBoard != null)
            {
                TextBlock_OtaBoard.Text = string.IsNullOrWhiteSpace(board) ? "-" : board;
            }
            if (TextBlock_OtaVersion != null)
            {
                TextBlock_OtaVersion.Text = string.IsNullOrWhiteSpace(version) ? "-" : version;
            }
            if (TextBlock_OtaMd5 != null)
            {
                TextBlock_OtaMd5.Text = string.IsNullOrWhiteSpace(md5) ? "-" : md5;
            }
        }

        private bool TryLoadFfbotaManifest(string path, out OtaManifest manifest, out string error)
        {
            manifest = new OtaManifest();
            error = string.Empty;
            try
            {
                using (FileStream stream = File.OpenRead(path))
                using (ZipArchive archive = new ZipArchive(stream, ZipArchiveMode.Read))
                {
                    ZipArchiveEntry manifestEntry = archive.GetEntry("manifest.json");
                    ZipArchiveEntry firmwareEntry = archive.GetEntry("firmware.bin");
                    if (manifestEntry == null || firmwareEntry == null)
                    {
                        error = "ffbota must contain manifest.json and firmware.bin.";
                        return false;
                    }

                    using (StreamReader reader = new StreamReader(manifestEntry.Open()))
                    {
                        string json = reader.ReadToEnd();
                        manifest = JsonConvert.DeserializeObject<OtaManifest>(json) ?? new OtaManifest();
                    }

                    string md5 = manifest.Md5?.Trim();
                    if (string.IsNullOrWhiteSpace(md5))
                    {
                        error = "ffbota manifest.json is missing md5.";
                        return false;
                    }

                    using (Stream firmwareStream = firmwareEntry.Open())
                    using (MD5 md5Hasher = MD5.Create())
                    {
                        byte[] hash = md5Hasher.ComputeHash(firmwareStream);
                        string computed = BitConverter.ToString(hash).Replace("-", "").ToLowerInvariant();
                        if (!string.Equals(md5, computed, StringComparison.OrdinalIgnoreCase))
                        {
                            error = $"ffbota MD5 mismatch (manifest {md5}, computed {computed}).";
                            return false;
                        }
                    }
                }
                return true;
            }
            catch (Exception ex)
            {
                error = $"ffbota load failed: {ex.Message}";
                return false;
            }
        }

        private bool TryBuildLocalOtaPayload(string ffbotaPath, out byte[] infoJson, out byte[] firmwareBytes, out string infoUrl, out string expectedVersion, out string error)
        {
            infoJson = null;
            firmwareBytes = null;
            infoUrl = string.Empty;
            expectedVersion = string.Empty;
            error = string.Empty;

            if (!TryLoadFfbotaManifest(ffbotaPath, out OtaManifest manifest, out error))
            {
                return false;
            }

            try
            {
                using (FileStream stream = File.OpenRead(ffbotaPath))
                using (ZipArchive archive = new ZipArchive(stream, ZipArchiveMode.Read))
                {
                    ZipArchiveEntry firmwareEntry = archive.GetEntry("firmware.bin");
                    using (Stream firmwareStream = firmwareEntry.Open())
                    using (MemoryStream buffer = new MemoryStream())
                    {
                        firmwareStream.CopyTo(buffer);
                        firmwareBytes = buffer.ToArray();
                    }
                }
            }
            catch (Exception ex)
            {
                error = $"ffbota read failed: {ex.Message}";
                return false;
            }

            string hostIp = ResolveHostIp();
            int port = Plugin?.Settings?.OtaLocalPort ?? 8000;
            infoUrl = $"http://{hostIp}:{port}/update_info.json";
            string firmwareUrl = $"http://{hostIp}:{port}/firmware.bin";
            var infoPayload = new
            {
                Configurations = new[]
                {
                    new
                    {
                        Board = manifest.Board ?? string.Empty,
                        Version = manifest.Version ?? string.Empty,
                        URL = firmwareUrl,
                        MD5 = manifest.Md5 ?? string.Empty
                    }
                }
            };
            infoJson = Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(infoPayload));
            UpdateOtaInfoReadout(manifest.Board, manifest.Version, manifest.Md5);
            expectedVersion = manifest.Version ?? string.Empty;
            return true;
        }

        private string ResolveHostIp()
        {
            try
            {
                using (Socket socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
                {
                    socket.Connect("8.8.8.8", 80);
                    if (socket.LocalEndPoint is IPEndPoint endPoint)
                    {
                        return endPoint.Address.ToString();
                    }
                }
            }
            catch
            {
            }
            return "127.0.0.1";
        }

        private void WarnIfOtaUrlAclMissing()
        {
            int port = Plugin?.Settings?.OtaLocalPort ?? 8000;
            string host = ResolveHostIp();
            string prefix = $"http://{host}:{port}/";

            try
            {
                TcpListener listener = new TcpListener(IPAddress.Any, port);
                try
                {
                    listener.Start();
                    listener.Stop();
                }
                finally
                {
                    listener.Stop();
                }
                otaSelectionDialog?.SetBindingUrl(prefix);
            }
            catch (SocketException ex) when (ex.SocketErrorCode == SocketError.AddressAlreadyInUse)
            {
                if (!otaAclWarned)
                {
                    LogOta($"OTA local binding failed (port {port} already in use).");
                    otaAclWarned = true;
                }
            }
            catch
            {
                // Ignore other listener issues; OTA start will surface them.
            }
        }

        private void LogOta(string message)
        {
            if (string.IsNullOrWhiteSpace(message))
            {
                return;
            }

            if (otaSelectionDialog != null)
            {
                otaSelectionDialog.AppendLog(message);
            }

            SetDebugOutput(message);
        }

        private void ShowCopyableMessage(string title, string message)
        {
            Window dialog = new Window
            {
                Title = title,
                Width = 520,
                Height = 220,
                WindowStartupLocation = WindowStartupLocation.CenterScreen,
                ResizeMode = ResizeMode.NoResize,
                Background = new SolidColorBrush(Color.FromRgb(0x25, 0x25, 0x25)),
                Foreground = Brushes.White,
                Content = new Grid()
            };

            Grid grid = (Grid)dialog.Content;
            grid.RowDefinitions.Add(new RowDefinition { Height = new GridLength(1, GridUnitType.Star) });
            grid.RowDefinitions.Add(new RowDefinition { Height = GridLength.Auto });

            TextBox textBox = new TextBox
            {
                Text = message,
                IsReadOnly = true,
                TextWrapping = TextWrapping.Wrap,
                VerticalScrollBarVisibility = ScrollBarVisibility.Auto,
                Background = new SolidColorBrush(Color.FromRgb(0x1F, 0x25, 0x25)),
                BorderThickness = new Thickness(1),
                BorderBrush = new SolidColorBrush(Color.FromRgb(0x4E, 0x4E, 0x4E)),
                Margin = new Thickness(12),
                Foreground = Brushes.White
            };
            Grid.SetRow(textBox, 0);
            grid.Children.Add(textBox);

            Button okButton = new Button
            {
                Content = "OK",
                Width = 80,
                Height = 28,
                Margin = new Thickness(0, 0, 12, 12),
                HorizontalAlignment = HorizontalAlignment.Right
            };
            okButton.Click += (_, __) => dialog.Close();
            Grid.SetRow(okButton, 1);
            grid.Children.Add(okButton);

            dialog.ShowDialog();
        }

        private void btn_start_ota_Click(object sender, RoutedEventArgs e)
        {
            if (Plugin == null || Plugin.ESPsync_serialPort == null || !Plugin.ESPsync_serialPort.IsOpen)
            {
                LogOta("Connect the gateway before starting OTA.");
                return;
            }

            BuildOtaTargets();
            RequestOtaDeviceInfo();

            if (otaSelectionDialog != null)
            {
                otaSelectionDialog.Close();
                otaSelectionDialog = null;
            }

            otaSelectionDialog = new OtaSelectionDialog(ota_targets);
            otaSelectionDialog.StartRequested += OnOtaStartRequested;
            otaSelectionDialog.Closed += (s, args) =>
            {
                otaUpdateCancellation?.Cancel();
                StopLocalOtaServer();
                otaSelectionDialog = null;
            };
            WarnIfOtaUrlAclMissing();
            otaSelectionDialog.Show();
        }

        private void StopLocalOtaServer()
        {
            if (otaServer != null)
            {
                otaServer.Dispose();
                otaServer = null;
            }
        }

        private async void OnOtaStartRequested(object sender, EventArgs e)
        {
            if (Plugin == null || Plugin.ESPsync_serialPort == null || !Plugin.ESPsync_serialPort.IsOpen)
            {
                SetDebugOutput("Connect the gateway before starting OTA.", UiLogLevel.Warning);
                return;
            }

            string ssid = Plugin.Settings.SSID_string ?? string.Empty;
            string pass = Plugin.Settings.PASS_string ?? string.Empty;
            if (ssid.Length > MaxWifiCredentialLength || pass.Length > MaxWifiCredentialLength)
            {
                LogOta($"SSID and password must be {MaxWifiCredentialLength} characters or less.");
                return;
            }

            string infoUrl = GetOtaInfoUrl();
            string expectedVersion = string.Empty;
            if (OTASource_Local != null && OTASource_Local.IsChecked == true)
            {
                string ffbotaPath = TextBox_OtaFfbotaPath?.Text ?? string.Empty;
                if (string.IsNullOrWhiteSpace(ffbotaPath) || !File.Exists(ffbotaPath))
                {
                    LogOta("Select a valid .ffbota file.");
                    return;
                }

                if (!TryBuildLocalOtaPayload(ffbotaPath, out byte[] infoJson, out byte[] firmwareBytes, out string localInfoUrl, out expectedVersion, out string error))
                {
                    LogOta(error);
                    return;
                }

                StopLocalOtaServer();
                try
                {
                    otaServer = new LocalOtaServer(localInfoUrl, infoJson, firmwareBytes, LogOta);
                    otaServer.Start();
                    infoUrl = otaServer.InfoUrl;
                    otaSelectionDialog?.SetBindingUrl(infoUrl);
                    otaSelectionDialog?.AppendLog($"Binding URL: {infoUrl}");
                }
                catch (Exception ex)
                {
                    LogOta($"Failed to start local OTA server: {ex.Message}");
                    StopLocalOtaServer();
                    return;
                }
            }
            else
            {
                StopLocalOtaServer();
                if (string.IsNullOrWhiteSpace(infoUrl))
                {
                    LogOta("Enter a valid OTA URL.");
                    return;
                }

                if (!TryLoadOtaInfoFromUrl(infoUrl, out string board, out string version, out string md5, out expectedVersion, out string error))
                {
                    LogOta(error);
                    return;
                }
                UpdateOtaInfoReadout(board, version, md5);
                otaSelectionDialog?.SetBindingUrl(infoUrl);
            }

            bool allowDowngrades = Checkbox_Force_flash != null && Checkbox_Force_flash.IsChecked == true;
            var selectedAxes = ota_targets.Where(t => t.Selected && t.IsOnline && t.AxisId != AxisID.AxisUndefined).ToList();
            var selectedGateways = ota_targets.Where(t => t.Selected && t.IsOnline && t.GatewayId != GatewayID.GatewayUndefined).ToList();

            otaUpdateCancellation?.Cancel();
            otaUpdateCancellation = new CancellationTokenSource();

            OtaUpdateCoordinator coordinator = new OtaUpdateCoordinator(
                axisId => ota_axis_versions.TryGetValue(axisId, out string version) ? version : "-",
                gatewayId => ota_gateway_versions.TryGetValue(gatewayId, out string version) ? version : "-",
                axisId =>
                {
                    StartOtaUpdate startOtaUpdate = new StartOtaUpdate
                    {
                        WifiInfo = new WifiInfo
                        {
                            Ssid = ssid,
                            Password = pass
                        },
                        InfoJsonUrl = infoUrl,
                        AllowDowngrades = allowDowngrades,
                        Target = OtaTarget.AxesOnly,
                        TargetAxisId = axisId
                    };
                    Message msg = new Message { StartOtaUpdate = startOtaUpdate };
                    Plugin.ESPsync_serialPort.WriteMessage(msg);
                },
                gatewayId =>
                {
                    StartOtaUpdate startOtaUpdate = new StartOtaUpdate
                    {
                        WifiInfo = new WifiInfo
                        {
                            Ssid = ssid,
                            Password = pass
                        },
                        InfoJsonUrl = infoUrl,
                        AllowDowngrades = allowDowngrades,
                        Target = OtaTarget.GatewayOnly
                    };
                    Message msg = new Message { StartOtaUpdate = startOtaUpdate };
                    Plugin.ESPsync_serialPort.WriteMessage(msg);
                },
                message => LogOta(message));

            try
            {
                await coordinator.RunAsync(
                    selectedAxes.Select(entry => entry.AxisId).ToList(),
                    selectedGateways.Select(entry => entry.GatewayId).ToList(),
                    expectedVersion,
                    otaUpdateCancellation.Token);
                LogOta("OTA update flow complete.");
            }
            catch (OperationCanceledException)
            {
                LogOta("OTA update cancelled.");
            }
        }

        private void BuildOtaTargets()
        {
            ota_targets.Clear();
            ota_axis_targets.Clear();
            ota_gateway_targets.Clear();

            if (Plugin?.ESPsync_serialPort != null && Plugin.ESPsync_serialPort.IsOpen)
            {
                GatewayID gatewayId = last_gateway_id == GatewayID.GatewayUndefined ? GatewayID._1 : last_gateway_id;
                var gatewayEntry = new OtaTargetEntry(gatewayId)
                {
                    IsOnline = true,
                    Selected = true
                };
                if (ota_gateway_logs.TryGetValue(gatewayId, out string gatewayLog))
                {
                    gatewayEntry.LatestLog = gatewayLog;
                }
                if (ota_gateway_versions.TryGetValue(gatewayId, out string gatewayVersion))
                {
                    gatewayEntry.LatestVersion = gatewayVersion;
                }
                ota_gateway_targets[gatewayId] = gatewayEntry;
                ota_targets.Add(gatewayEntry);
            }

            foreach (var axis in axes.Values)
            {
                if (!axis.IsOnline)
                {
                    continue;
                }
                var axisEntry = new OtaTargetEntry(axis.ID)
                {
                    IsOnline = true,
                    Selected = true
                };
                if (ota_axis_logs.TryGetValue(axis.ID, out string axisLog))
                {
                    axisEntry.LatestLog = axisLog;
                }
                if (ota_axis_versions.TryGetValue(axis.ID, out string axisVersion))
                {
                    axisEntry.LatestVersion = axisVersion;
                }
                ota_axis_targets[axis.ID] = axisEntry;
                ota_targets.Add(axisEntry);
            }
        }

        private void RequestOtaDeviceInfo()
        {
            if (Plugin?.ESPsync_serialPort == null || !Plugin.ESPsync_serialPort.IsOpen)
            {
                return;
            }

            foreach (var entry in ota_targets)
            {
                Message msg = new Message();
                msg.DeviceInfoRequest = new DeviceInfoRequest();
                if (entry.AxisId != AxisID.AxisUndefined)
                {
                    msg.DeviceInfoRequest.AxisId = entry.AxisId;
                }
                else if (entry.GatewayId != GatewayID.GatewayUndefined)
                {
                    msg.DeviceInfoRequest.GatewayId = entry.GatewayId;
                }
                Plugin.ESPsync_serialPort.WriteMessage(msg);
            }
        }

        private void UpdateOtaAxisLog(AxisLogMessage msg)
        {
            if (msg.AxisId != AxisID.AxisUndefined && ota_axis_targets.TryGetValue(msg.AxisId, out var entry))
            {
                entry.LatestLog = msg.Msg ?? string.Empty;
            }
            if (msg.AxisId != AxisID.AxisUndefined)
            {
                ota_axis_logs[msg.AxisId] = msg.Msg ?? string.Empty;
            }
        }

        private void UpdateOtaGatewayLog(GatewayLogMessage msg)
        {
            if (msg.GatewayId != GatewayID.GatewayUndefined && ota_gateway_targets.TryGetValue(msg.GatewayId, out var entry))
            {
                entry.LatestLog = msg.Msg ?? string.Empty;
            }
            if (msg.GatewayId != GatewayID.GatewayUndefined)
            {
                ota_gateway_logs[msg.GatewayId] = msg.Msg ?? string.Empty;
            }
        }

        private void UpdateOtaDeviceInfo(DeviceInfo info)
        {
            if (info.AxisId != AxisID.AxisUndefined && ota_axis_targets.TryGetValue(info.AxisId, out var axisEntry))
            {
                axisEntry.LatestVersion = string.IsNullOrWhiteSpace(info.FwVersion) ? "-" : info.FwVersion;
                axisEntry.IsOnline = true;
            }
            else if (info.GatewayId != GatewayID.GatewayUndefined && ota_gateway_targets.TryGetValue(info.GatewayId, out var gatewayEntry))
            {
                gatewayEntry.LatestVersion = string.IsNullOrWhiteSpace(info.FwVersion) ? "-" : info.FwVersion;
                gatewayEntry.IsOnline = true;
            }
            if (info.AxisId != AxisID.AxisUndefined)
            {
                ota_axis_versions[info.AxisId] = string.IsNullOrWhiteSpace(info.FwVersion) ? "-" : info.FwVersion;
            }
            else if (info.GatewayId != GatewayID.GatewayUndefined)
            {
                ota_gateway_versions[info.GatewayId] = string.IsNullOrWhiteSpace(info.FwVersion) ? "-" : info.FwVersion;
            }
        }

        private bool TryLoadOtaInfoFromUrl(string url, out string board, out string version, out string md5, out string expectedVersion, out string error)
        {
            board = "-";
            version = "-";
            md5 = "-";
            expectedVersion = string.Empty;
            error = string.Empty;
            try
            {
                using (WebClient client = new WebClient())
                {
                    string json = client.DownloadString(url);
                    var payload = JsonConvert.DeserializeObject<OtaInfoPayload>(json);
                    if (payload?.Configurations == null || payload.Configurations.Count == 0)
                    {
                        error = "OTA JSON has no configurations.";
                        return false;
                    }

                    var boards = payload.Configurations.Select(cfg => cfg.Board).Distinct().ToList();
                    var versions = payload.Configurations.Select(cfg => cfg.Version).Distinct().ToList();
                    var md5s = payload.Configurations.Select(cfg => cfg.Md5).Distinct().ToList();

                    board = boards.Count == 1 ? boards[0] : "Multiple";
                    version = versions.Count == 1 ? versions[0] : "Multiple";
                    md5 = md5s.Count == 1 ? md5s[0] : "Multiple";
                    if (versions.Count > 0)
                    {
                        expectedVersion = versions[0];
                    }
                    return true;
                }
            }
            catch (Exception ex)
            {
                error = $"Failed to load OTA JSON: {ex.Message}";
                return false;
            }
        }

        private void ConnectToPort(string portName)
        {
            if (Plugin == null)
            {
                return;
            }

            if (string.IsNullOrWhiteSpace(portName) || portName == "NA")
            {
                SetDebugOutput("Select a valid port before connecting.", UiLogLevel.Warning);
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
                SetDebugOutput($"Connected to {portName}.");
            }
            catch (Exception ex)
            {
                Label_Status.Text = "Disconnected";
                btn_connect_espnow_port.Content = "Connect";
                SetDebugOutput($"Failed to connect to {portName}: {ex.Message}", UiLogLevel.Error);
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
                SetDebugOutput($"Disconnect failed: {ex.Message}", UiLogLevel.Error);
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

        public void RefreshFunctionSelection()
        {
            UpdateFunctionSelection();
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
            SetDebugOutput(message, UiLogLevel.Debug);
        }

        private void btn_clear_log_Click(object sender, RoutedEventArgs e)
        {
            ClearUiLog();
        }

        private void InitializeLogSourceFilters()
        {
            AddLogSourceFilter(UiLogSourceKind.Plugin, null, "Plugin");
        }

        private void AddLogSourceFilter(UiLogSourceKind kind, int? id, string label)
        {
            string key = GetSourceKey(kind, id);
            if (logSourceFilterLookup.ContainsKey(key))
            {
                return;
            }

            var item = new LogSourceFilterItem(kind, id, label, true);
            item.PropertyChanged += OnLogSourceFilterChanged;
            logSourceFilterLookup[key] = item;
            int insertIndex = 0;
            int itemRank = GetLogSourceSortRank(kind, id);
            while (insertIndex < LogSourceFilters.Count)
            {
                var existing = LogSourceFilters[insertIndex];
                if (GetLogSourceSortRank(existing.Kind, existing.Id) > itemRank)
                {
                    break;
                }
                insertIndex++;
            }
            LogSourceFilters.Insert(insertIndex, item);
        }

        private void OnLogSourceFilterChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(LogSourceFilterItem.IsSelected))
            {
                UiLogView?.Refresh();
            }
        }

        private bool FilterLogEntry(object entry)
        {
            if (entry is UiLogEntry logEntry)
            {
                if (LogSourceFilters.Count == 0)
                {
                    return true;
                }

                string key = GetSourceKey(logEntry.SourceKind, logEntry.SourceId);
                if (logSourceFilterLookup.TryGetValue(key, out LogSourceFilterItem filterItem))
                {
                    return filterItem.IsSelected;
                }
            }

            return true;
        }

        private static string GetSourceKey(UiLogSourceKind kind, int? id)
        {
            return id.HasValue ? $"{kind}:{id.Value}" : $"{kind}";
        }

        private static int GetLogSourceSortRank(UiLogSourceKind kind, int? id)
        {
            int kindRank = kind == UiLogSourceKind.Plugin ? 0 : kind == UiLogSourceKind.Gateway ? 1 : 2;
            int idRank = id ?? -1;
            return (kindRank * 100) + idRank;
        }

        private void ClearUiLog()
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(ClearUiLog);
                return;
            }

            UiLogEntries.Clear();
        }

        private void SetDebugOutput(string message, UiLogLevel level = UiLogLevel.Info)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() => SetDebugOutput(message, level));
                return;
            }

            string resolvedMessage = (message ?? string.Empty).TrimEnd('\r', '\n');
            if (TextBlock_Status != null)
            {
                TextBlock_Status.Text = resolvedMessage;
            }

            if (string.IsNullOrWhiteSpace(resolvedMessage))
            {
                return;
            }

            AppendUiLog(resolvedMessage, level, UiLogSourceKind.Plugin, null);
        }

        private void AppendUiLog(string message, UiLogLevel level, UiLogSourceKind sourceKind, int? sourceId)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(() => AppendUiLog(message, level, sourceKind, sourceId));
                return;
            }

            var entry = new UiLogEntry
            {
                Time = DateTime.Now.ToString("HH:mm:ss"),
                Level = level.ToString().ToUpperInvariant(),
                Source = BuildSourceLabel(sourceKind, sourceId),
                Message = (message ?? string.Empty).TrimEnd('\r', '\n')
            };
            entry.SourceKind = sourceKind;
            entry.SourceId = sourceId;

            UiLogEntries.Add(entry);
            while (UiLogEntries.Count > UiLogMaxEntries)
            {
                UiLogEntries.RemoveAt(0);
            }

            ListBox_UiLog?.ScrollIntoView(entry);
        }

        private static string BuildSourceLabel(UiLogSourceKind kind, int? id)
        {
            switch (kind)
            {
                case UiLogSourceKind.Gateway:
                    return id.HasValue ? $"Gateway {id.Value}" : "Gateway";
                case UiLogSourceKind.Axis:
                    return id.HasValue ? $"Axis {id.Value}" : "Axis";
                default:
                    return "Plugin";
            }
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

            if (ota_axis_targets.TryGetValue(axis_id, out OtaTargetEntry entry))
            {
                entry.IsOnline = new_online_state;
            }

            if (new_online_state && axis_id != AxisID.AxisUndefined && seenAxisSources.Add(axis_id))
            {
                AddLogSourceFilter(UiLogSourceKind.Axis, (int)axis_id, $"Axis {(int)axis_id}");
            }
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
            axisRequestQueue?.HandleResponse(msg);
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
                    int? axisSourceId = msg.AxisLogMessage.AxisId != AxisID.AxisUndefined ? (int)msg.AxisLogMessage.AxisId : (int?)null;
                    AppendUiLog(BuildAxisLogLine(msg.AxisLogMessage), MapLogLevel(msg.AxisLogMessage.Level), UiLogSourceKind.Axis, axisSourceId);
                    UpdateOtaAxisLog(msg.AxisLogMessage);
                    break;
                case Message.PayloadOneofCase.GatewayLogMessage:
                    int? gatewaySourceId = msg.GatewayLogMessage.GatewayId != GatewayID.GatewayUndefined ? (int)msg.GatewayLogMessage.GatewayId : (int?)null;
                    AppendUiLog(BuildGatewayLogLine(msg.GatewayLogMessage), MapLogLevel(msg.GatewayLogMessage.Level), UiLogSourceKind.Gateway, gatewaySourceId);
                    UpdateOtaGatewayLog(msg.GatewayLogMessage);
                    break;
                case Message.PayloadOneofCase.DeviceInfo:
                    UpdateOtaDeviceInfo(msg.DeviceInfo);
                    break;
                case Message.PayloadOneofCase.ActiveFunction:
                    RegisterAxisChannel(msg.ActiveFunction.AxisId, port);
                    UpdateActiveFunction(msg.ActiveFunction);
                    break;
                case Message.PayloadOneofCase.StaticBalanceResult:
                    RegisterAxisChannel(msg.StaticBalanceResult.AxisId, port);
                    if (axes.TryGetValue(msg.StaticBalanceResult.AxisId, out Axis axis))
                    {
                        AxisConfigControl.ApplyStaticBalanceResult(axis.Config, msg.StaticBalanceResult);
                        if (msg.StaticBalanceResult.AxisId == selected_axis_id)
                        {
                            uc_axis_config.OnStaticBalanceResult(msg.StaticBalanceResult);
                        }
                    }
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

        public void RequestStaticBalanceCalibration(AxisID axisId)
        {
            if (axisId == AxisID.AxisUndefined)
            {
                SetDebugOutput("Static balance: Axis ID undefined", UiLogLevel.Warning);
                return;
            }
            if (!axes.TryGetValue(axisId, out Axis axis))
            {
                SetDebugOutput($"Static balance: Axis {axisId} not found", UiLogLevel.Warning);
                return;
            }
            EnqueueAxisRequest(axisId, AxisRequestType.StaticBalanceCalibration, null);
        }

        private bool EnqueueAxisRequest(AxisID axisId, AxisRequestType type, Message payload)
        {
            axisRequestQueue?.Enqueue(axisId, type, payload);
            return true;
        }

        public bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload)
        {
            Message msg = payload ?? new Message();
            switch (type)
            {
                case AxisRequestType.AxisConfig:
                    msg.AxisAction = new AxisAction { AxisId = axisId, ReturnAxisConfig = true };
                    break;
                case AxisRequestType.FunctionConfig:
                    msg.AxisAction = new AxisAction { AxisId = axisId, ReturnFunctionConfig = true };
                    break;
                case AxisRequestType.ActiveFunction:
                    msg.AxisAction = new AxisAction { AxisId = axisId, ReturnActiveFunction = true };
                    break;
                case AxisRequestType.DeviceInfo:
                    msg.DeviceInfoRequest = new DeviceInfoRequest { AxisId = axisId };
                    break;
                case AxisRequestType.Restart:
                    msg.AxisAction = new AxisAction { AxisId = axisId, Restart = true };
                    break;
                case AxisRequestType.Homing:
                    msg.AxisAction = new AxisAction { AxisId = axisId, StartHoming = true };
                    break;
                case AxisRequestType.StaticBalanceCalibration:
                    msg.AxisAction = new AxisAction { AxisId = axisId, StartStaticBalanceCalibration = true };
                    break;
                case AxisRequestType.AxisConfigUpload:
                case AxisRequestType.FunctionConfigUpload:
                    break;
                default:
                    return false;
            }

            if (axes.TryGetValue(axisId, out Axis axis))
            {
                var serialChannel = axis.SerialChannel;
                if (serialChannel != null && serialChannel != Plugin.ESPsync_serialPort)
                {
                    serialChannel.WriteMessage(msg);
                    return true;
                }
            }
            if (Plugin?.ESPsync_serialPort != null && Plugin.ESPsync_serialPort.IsOpen)
            {
                Plugin.ESPsync_serialPort.WriteMessage(msg);
                return true;
            }
            return false;
        }

        private void EnqueueAxisConfigUpload(AxisID axisId, AxisConfig axisConfig, bool store)
        {
            if (axisId == AxisID.AxisUndefined)
            {
                return;
            }
            AxisConfig configToSend = axisConfig.Clone();
            configToSend.Store = store;
            Message msg = new Message { AxisConfig = configToSend };
            axisRequestQueue?.Enqueue(axisId, AxisRequestType.AxisConfigUpload, msg);
        }

        private void EnqueueFunctionConfigUpload(FunctionConfig functionConfig, bool store)
        {
            FunctionConfig configToSend = functionConfig.Clone();
            configToSend.Base.Store = store;
            Message msg = new Message { FunctionConfig = configToSend };
            bool broadcastRequired = false;
            HashSet<AxisID> queuedAxes = new HashSet<AxisID>();
            foreach (var linkedAxisId in configToSend.Base.LinkedAxes)
            {
                var axisId = linkedAxisId & AxisID.Mask;
                if (axisId == AxisID.AxisUndefined)
                {
                    continue;
                }
                if (!axes.TryGetValue(axisId, out Axis axis))
                {
                    continue;
                }
                var serialChannel = axis.SerialChannel;
                if (serialChannel != null && serialChannel != Plugin.ESPsync_serialPort)
                {
                    if (queuedAxes.Add(axisId))
                    {
                        axisRequestQueue?.Enqueue(axisId, AxisRequestType.FunctionConfigUpload, msg);
                    }
                }
                else
                {
                    broadcastRequired = true;
                }
            }

            if (broadcastRequired)
            {
                axisRequestQueue?.Enqueue(AxisID.AxisUndefined, AxisRequestType.FunctionConfigUpload, msg);
            }
        }

        public void UpdateActiveAircraftLabel(string carName, string carId, string gameId = null)
        {
            if (TextBlock_VehicleTabVehicleId == null)
            {
                return;
            }

            string label = string.IsNullOrWhiteSpace(carName) ? "-" : carName;
            if (!string.IsNullOrWhiteSpace(gameId))
            {
                label += $" ({gameId})";
            }
            TextBlock_VehicleTabVehicleId.Text = label;
            TextBlock_VehicleTabVehicleId.ToolTip = string.IsNullOrWhiteSpace(carId) ? null : carId;
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
            if (state.GatewayId != GatewayID.GatewayUndefined)
            {
                last_gateway_id = state.GatewayId;
                if (seenGatewaySources.Add(state.GatewayId))
                {
                    AddLogSourceFilter(UiLogSourceKind.Gateway, (int)state.GatewayId, $"Gateway {(int)state.GatewayId}");
                }
            }
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

            if (ota_gateway_targets.TryGetValue(last_gateway_id, out OtaTargetEntry gatewayEntry))
            {
                gatewayEntry.IsOnline = true;
            }
        }

        private string BuildAxisLogLine(AxisLogMessage msg)
        {
            return msg.Msg ?? string.Empty;
        }

        private string BuildGatewayLogLine(GatewayLogMessage msg)
        {
            return msg.Msg ?? string.Empty;
        }

        private UiLogLevel MapLogLevel(LogLevel level)
        {
            switch (level)
            {
                case LogLevel.Debug:
                    return UiLogLevel.Debug;
                case LogLevel.Info:
                    return UiLogLevel.Info;
                case LogLevel.Warning:
                    return UiLogLevel.Warning;
                case LogLevel.Error:
                    return UiLogLevel.Error;
                default:
                    return UiLogLevel.Info;
            }
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
                SetDebugOutput("No function selected.", UiLogLevel.Warning);
                return;
            }

            FunctionConfig functionConfig = functions[selected_function_id].Config;
            EnqueueFunctionConfigUpload(functionConfig, PersistConfig);
        }

        private void OnDownloadFunctionConfigClicked(object sender, RoutedEventArgs e)
        {
            if (selected_function_id == FunctionID.Undefined)
            {
                SetDebugOutput("No function selected.", UiLogLevel.Warning);
                return;
            }

            FunctionConfig functionConfig = functions[selected_function_id].Config;
            AxisID targetAxis = AxisID.AxisUndefined;
            foreach (var linkedAxis in functionConfig.Base.LinkedAxes)
            {
                AxisID axisId = linkedAxis & AxisID.Mask;
                if (axisId == AxisID.AxisUndefined)
                {
                    continue;
                }
                if (axes.TryGetValue(axisId, out Axis axis) && axis.IsOnline)
                {
                    targetAxis = axisId;
                    break;
                }
                if (targetAxis == AxisID.AxisUndefined)
                {
                    targetAxis = axisId;
                }
            }

            if (targetAxis == AxisID.AxisUndefined && selected_axis_id != AxisID.AxisUndefined)
            {
                targetAxis = selected_axis_id;
            }

            if (targetAxis == AxisID.AxisUndefined)
            {
                SetDebugOutput("No axis available for function config download.", UiLogLevel.Warning);
                return;
            }
            if (!axes.TryGetValue(targetAxis, out Axis targetAxisInfo) || !targetAxisInfo.IsOnline)
            {
                SetDebugOutput($"Axis {(int)targetAxis} is offline.", UiLogLevel.Warning);
                return;
            }
            EnqueueAxisRequest(targetAxis, AxisRequestType.FunctionConfig, null);
        }

        private void UploadFunctionConfig(FunctionConfig functionConfig, bool store)
        {
            EnqueueFunctionConfigUpload(functionConfig, store);
        }

        private void OnFunctionConfigUpdate(FunctionConfig newFunctionConfig)
        {
            FunctionID newFunctionId = newFunctionConfig.Base.FunctionId;
            if (newFunctionId == FunctionID.Undefined)
            {
                SetDebugOutput("Function ID undefined (ignored)", UiLogLevel.Warning);
                return;
            }

            int funcId = (int)newFunctionId;

            // Store base config from ESP32
            Plugin.FunctionConfigManager.SetBaseConfig(funcId, newFunctionConfig);

            // Check if we should apply profile overrides
            if (Plugin.ShouldApplyProfileOverride(funcId))
            {
                // ApplyProfileOverridesToFunction will fire FunctionConfigChanged event
                // which will update UI and send merged config to ESP32
                Plugin.ApplyProfileOverridesToFunction(funcId);
            }
            else
            {
                // No profile override - but still apply user overrides if any exist
                var userOverrides = Plugin.GetUserFunctionOverrides(funcId);
                if (userOverrides != null && !userOverrides.IsEmpty)
                {
                    var merged = TieredConfig.ConfigMerger.MergeFunctionConfig(newFunctionConfig, userOverrides);
                    functions[newFunctionId].Config = merged;
                }
                else
                {
                    functions[newFunctionId].Config = newFunctionConfig;
                }
                if (newFunctionId == selected_function_id)
                {
                    uc_function_config.SwitchFunction(functions[newFunctionId]);
                }
            }
        }

        private void OnAxisConfigUpdate(AxisConfig newAxisConfig)
        {
            AxisID newAxisId = newAxisConfig.AxisId;
            if (newAxisId != AxisID.AxisUndefined && newAxisId <= AxisID._8)
            {
                axes[newAxisId].Config = newAxisConfig;
                axes[newAxisId].HasAxisConfig = true;
                if (newAxisId == selected_axis_id)
                {
                    uc_axis_config.UpdateConfig(axes[newAxisId].Config);
                }
                if (selected_function_id != FunctionID.Undefined && functions.TryGetValue(selected_function_id, out Function function))
                {
                    FunctionConfig cfg = function.Config;
                    bool affectsSelected = false;
                    if (cfg?.Base != null)
                    {
                        foreach (var axis in cfg.Base.LinkedAxes)
                        {
                            if ((axis & AxisID.Mask) == newAxisId)
                            {
                                affectsSelected = true;
                                break;
                            }
                        }
                    }
                    if (!affectsSelected && cfg?.AuxFunction != null)
                    {
                        foreach (var axis in cfg.AuxFunction.LinkedAxes)
                        {
                            if ((axis & AxisID.Mask) == newAxisId)
                            {
                                affectsSelected = true;
                                break;
                            }
                        }
                    }
                    if (affectsSelected)
                    {
                        uc_function_config.OnKinematicParametersChanged(newAxisConfig.KinematicParameters);
                    }
                }
            }
            else
            {
                SetDebugOutput($"Invalid axis ID ({(int)newAxisId})", UiLogLevel.Warning);
            }
        }

        /// <summary>
        /// Called when FunctionConfigManager fires after merging profile/user overrides.
        /// Sends the merged config to ESP32 and updates UI.
        /// </summary>
        private void OnMergedFunctionConfigChanged(object sender, FunctionConfigChangedEventArgs e)
        {
            FunctionID funcId = (FunctionID)e.FunctionId;
            if (funcId == FunctionID.Undefined || !functions.ContainsKey(funcId))
                return;

            // DON'T overwrite function.Config - it contains direct edits to non-wrapped fields
            // The merged config only includes override-tracked fields and would lose those edits
            // functions[funcId].Config = e.NewConfig; // <-- REMOVED: causes loss of non-wrapped field edits

            // Send to ESP32 (don't store to EEPROM - these are runtime overrides)
            EnqueueFunctionConfigUpload(e.NewConfig, store: false);

            // Update UI if this is the selected function
            if (funcId == selected_function_id)
            {
                var func = functions[funcId];
                this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    uc_function_config.SwitchFunction(func);
                }));
            }
        }

        /// <summary>
        /// Called when AxisConfigManager fires after applying function overrides.
        /// Sends the merged config to ESP32 and updates UI.
        /// </summary>
        private void OnMergedAxisConfigChanged(object sender, AxisConfigChangedEventArgs e)
        {
            AxisID axisId = (AxisID)e.AxisId;
            if (axisId == AxisID.AxisUndefined || !axes.ContainsKey(axisId))
                return;

            // Update local cache with merged config
            axes[axisId].Config = e.NewConfig;

            // Send to ESP32 (don't store to EEPROM - these are runtime overrides)
            EnqueueAxisConfigUpload(axisId, e.NewConfig, store: false);

            // Update UI if this is the selected axis
            if (axisId == selected_axis_id)
            {
                var config = axes[axisId].Config;
                this.Dispatcher.BeginInvoke(new Action(() =>
                {
                    uc_axis_config.UpdateConfig(config);
                }));
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
                    ThemedMessageBox.Show($"Error loading {openFileDialog.FileName}: {caughtEx.Message}", "Error", MessageBoxButton.OK, MessageBoxImage.Warning);
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
                if (loadSelectionDialog.UploadRequested && axis.SelectedToLoad && loadSelectionDialog.axis_configs.TryGetValue(axis.ID, out AxisConfig uploadCfg))
                {
                    EnqueueAxisConfigUpload(axis.ID, uploadCfg, false);
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
                    EnqueueFunctionConfigUpload(uploadCfg, false);
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

        private void OnSaveFunctionBaselineClicked(object sender, RoutedEventArgs e)
        {
            if (Plugin == null || !functions.TryGetValue(selected_function_id, out var function) || function == null)
            {
                return;
            }

            // Use function.Config which has ALL current edits (override-tracked + direct edits)
            // The manager's merged config only includes override-tracked fields, losing direct edits to non-wrapped fields
            var configToSave = function.Config?.Clone();

            if (configToSave != null)
            {
                // Save as baseline (this "bakes" all overrides AND direct edits into the new baseline)
                Plugin.SetFunctionBaseline((int)function.ID, configToSave);

                // Clear all overrides since they're now part of the baseline
                Plugin.ClearAllFunctionOverrides((int)function.ID);

                // Update manager with new baseline (no overrides)
                Plugin.FunctionConfigManager.SetBaseConfig((int)function.ID, configToSave);
                Plugin.ApplyProfileOverridesToFunction((int)function.ID);

                // Refresh badges to remove [U] badge after clearing overrides
                Dispatcher.BeginInvoke(new System.Action(() =>
                {
                    uc_function_config.RefreshAllBadges();
                }), System.Windows.Threading.DispatcherPriority.Background);

                // Show confirmation
                ThemedMessageBox.Show(
                    $"Saved {function.Name} configuration as hardware baseline.\n\nAll overrides have been baked into the baseline and cleared.",
                    "Baseline Saved",
                    MessageBoxButton.OK,
                    MessageBoxImage.Information);
            }
        }

        private void OnOpenGraphEditorClicked(object sender, RoutedEventArgs e)
        {
            if (graphEditorWindow == null)
            {
                graphEditorWindow = new GraphEditorWindow();
                graphEditorWindow.SetPlugin(Plugin);
                graphEditorWindow.SetLiveInputProvider(() => Plugin != null ? Plugin.GetLiveGraphInputs() : null);
                string activeGraphPath = Plugin?.GetActiveGraphPath();
                if (!string.IsNullOrWhiteSpace(activeGraphPath))
                {
                    graphEditorWindow.LoadGraphFromPath(activeGraphPath);
                    lastGraphEditorPath = activeGraphPath;
                }
                graphEditorWindow.Closed += (_, __) => graphEditorWindow = null;
                graphEditorWindow.Show();
            }
            else
            {
                graphEditorWindow.SetPlugin(Plugin);
                graphEditorWindow.SetLiveInputProvider(() => Plugin != null ? Plugin.GetLiveGraphInputs() : null);
                string activeGraphPath = Plugin?.GetActiveGraphPath();
                if (!string.IsNullOrWhiteSpace(activeGraphPath))
                {
                    graphEditorWindow.LoadGraphFromPath(activeGraphPath);
                    lastGraphEditorPath = activeGraphPath;
                }
                graphEditorWindow.Activate();
            }
        }

        private void OnUploadAxisConfigClicked(object sender, RoutedEventArgs e)
        {
            if (selected_axis_id == AxisID.AxisUndefined)
            {
                SetDebugOutput("No axis selected.", UiLogLevel.Warning);
                return;
            }

            EnqueueAxisConfigUpload(selected_axis_id, axes[selected_axis_id].Config, PersistConfig);
        }

        private void OnDownloadAxisConfigClicked(object sender, RoutedEventArgs e)
        {
            if (selected_axis_id == AxisID.AxisUndefined)
            {
                SetDebugOutput("No axis selected.", UiLogLevel.Warning);
                return;
            }
            if (!axes.TryGetValue(selected_axis_id, out Axis axis) || !axis.IsOnline)
            {
                SetDebugOutput($"Axis {(int)selected_axis_id} is offline.", UiLogLevel.Warning);
                return;
            }
            EnqueueAxisRequest(selected_axis_id, AxisRequestType.AxisConfig, null);
        }

        private void btn_home_axis_Click(object sender, RoutedEventArgs e)
        {
            if (selected_axis_id == AxisID.AxisUndefined)
            {
                SetDebugOutput("No axis selected.", UiLogLevel.Warning);
                return;
            }
            if (!axes.TryGetValue(selected_axis_id, out Axis axis))
            {
                SetDebugOutput($"Axis {(int)selected_axis_id} not found.", UiLogLevel.Warning);
                return;
            }
            if (!axis.IsOnline)
            {
                SetDebugOutput($"Axis {(int)selected_axis_id} is offline.", UiLogLevel.Warning);
                return;
            }
            if (!SendAxisRequest(selected_axis_id, AxisRequestType.Homing, null))
            {
                SetDebugOutput($"Axis {(int)selected_axis_id}: homing send failed.", UiLogLevel.Error);
                return;
            }
            SetDebugOutput($"Axis {(int)selected_axis_id}: homing started.");
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

        private sealed class OtaManifest
        {
            [JsonProperty("version")]
            public string Version { get; set; } = string.Empty;

            [JsonProperty("board")]
            public string Board { get; set; } = string.Empty;

            [JsonProperty("md5")]
            public string Md5 { get; set; } = string.Empty;
        }

        private sealed class OtaInfoPayload
        {
            [JsonProperty("Configurations")]
            public List<OtaInfoEntry> Configurations { get; set; } = new List<OtaInfoEntry>();
        }

        private sealed class OtaInfoEntry
        {
            [JsonProperty("Board")]
            public string Board { get; set; } = string.Empty;

            [JsonProperty("Version")]
            public string Version { get; set; } = string.Empty;

            [JsonProperty("MD5")]
            public string Md5 { get; set; } = string.Empty;
        }

        private sealed class LocalOtaServer : IDisposable
        {
            private readonly byte[] infoJson;
            private readonly byte[] firmwareBytes;
            private readonly TcpListener listener;
            private readonly Action<string> logger;
            private CancellationTokenSource cancellation;
            private Task loopTask;

            public string InfoUrl { get; }
            public string BindPrefixes { get; }

            public LocalOtaServer(string infoUrl, byte[] infoJson, byte[] firmwareBytes, Action<string> logger)
            {
                if (string.IsNullOrWhiteSpace(infoUrl))
                {
                    throw new ArgumentException("infoUrl must be set.");
                }

                this.infoJson = infoJson ?? Array.Empty<byte>();
                this.firmwareBytes = firmwareBytes ?? Array.Empty<byte>();
                this.logger = logger;
                InfoUrl = infoUrl;

                Uri infoUri = new Uri(infoUrl);
                BindPrefixes = $"{infoUri.Host}:{infoUri.Port}";
                listener = new TcpListener(IPAddress.Any, infoUri.Port);
            }

            public void Start()
            {
                if (cancellation != null)
                {
                    return;
                }

                listener.Start();
                cancellation = new CancellationTokenSource();
                loopTask = Task.Run(() => ListenAsync(cancellation.Token));
            }

            private async Task ListenAsync(CancellationToken token)
            {
                while (!token.IsCancellationRequested)
                {
                    TcpClient client = null;
                    try
                    {
                        client = await listener.AcceptTcpClientAsync().ConfigureAwait(false);
                        _ = Task.Run(() => HandleClientAsync(client, token), token);
                    }
                    catch (ObjectDisposedException)
                    {
                        break;
                    }
                    catch (InvalidOperationException)
                    {
                        break;
                    }
                }
            }

            public void Dispose()
            {
                cancellation?.Cancel();
                try
                {
                    listener.Stop();
                }
                catch
                {
                }
                try
                {
                    loopTask?.Wait(500);
                }
                catch
                {
                }
                cancellation?.Dispose();
                cancellation = null;
                loopTask = null;
            }

            private async Task HandleClientAsync(TcpClient client, CancellationToken token)
            {
                string logPath = "/";
                int responseBytes = 0;
                using (client)
                using (NetworkStream stream = client.GetStream())
                using (StreamReader reader = new StreamReader(stream, Encoding.ASCII, false, 1024, true))
                {
                    string requestLine = await reader.ReadLineAsync().ConfigureAwait(false);
                    if (string.IsNullOrWhiteSpace(requestLine))
                    {
                        return;
                    }

                    string[] parts = requestLine.Split(' ');
                    string path = parts.Length >= 2 ? parts[1] : "/";
                    logPath = path;

                    while (true)
                    {
                        string line = await reader.ReadLineAsync().ConfigureAwait(false);
                        if (string.IsNullOrEmpty(line))
                        {
                            break;
                        }
                    }

                    byte[] body;
                    string contentType;
                    int statusCode = 200;
                    int bytesWritten = 0;

                    if (path == "/" || path.Equals("/update_info.json", StringComparison.OrdinalIgnoreCase))
                    {
                        body = infoJson;
                        contentType = "application/json";
                    }
                    else if (path.Equals("/firmware.bin", StringComparison.OrdinalIgnoreCase))
                    {
                        body = firmwareBytes;
                        contentType = "application/octet-stream";
                    }
                    else
                    {
                        statusCode = 404;
                        body = Encoding.UTF8.GetBytes("Not Found");
                        contentType = "text/plain";
                    }

                    StringBuilder header = new StringBuilder();
                    header.Append($"HTTP/1.1 {statusCode} {(statusCode == 200 ? "OK" : "Not Found")}\r\n");
                    header.Append($"Content-Type: {contentType}\r\n");
                    header.Append($"Content-Length: {body.Length}\r\n");
                    header.Append("Connection: close\r\n");
                    header.Append("\r\n");

                    byte[] headerBytes = Encoding.ASCII.GetBytes(header.ToString());
                    await stream.WriteAsync(headerBytes, 0, headerBytes.Length, token).ConfigureAwait(false);
                    await stream.WriteAsync(body, 0, body.Length, token).ConfigureAwait(false);
                    await stream.FlushAsync(token).ConfigureAwait(false);
                    responseBytes = body.Length;
                }
                // Quiet by default; keep dialog log focused on OTA flow events.
            }
        }

        private void OnActiveGraphChanged_UI(object sender, EventArgs e)
        {
            Dispatcher.Invoke(() =>
            {
                UpdateVehicleTabHeader();
                RefreshVehicleParams();
            });
        }

        private void OnParamMigrationDetected(object sender, ParamMigrationResult e)
        {
            Dispatcher.Invoke(() =>
            {
                string summary = BuildMigrationSummary(e);
                ShowParamMigrationNotification(summary, e);
            });
        }

        private string BuildMigrationSummary(ParamMigrationResult result)
        {
            var parts = new List<string>();
            if (result.ChangedDefaults.Count > 0)
                parts.Add($"{result.ChangedDefaults.Count} default(s) changed");
            if (result.ClampedValues.Count > 0)
                parts.Add($"{result.ClampedValues.Count} value(s) clamped");
            if (result.NewOrphans.Count > 0)
                parts.Add($"{result.NewOrphans.Count} new orphan(s)");

            return parts.Count > 0
                ? string.Join(", ", parts)
                : "Graph updated";
        }

        private void ShowParamMigrationNotification(string summary, ParamMigrationResult result)
        {
            // Store result for review window access
            _pendingMigrationResult = result;

            // Show notification in status bar or message box
            ThemedMessageBox.Show(
                $"FFB Graph Changed\n\n{summary}\n\nClick 'Review Params' in the Vehicle tab to see details.",
                "Parameter Migration",
                MessageBoxButton.OK,
                MessageBoxImage.Information);
        }

        private ParamMigrationResult _pendingMigrationResult;

        private void UpdateVehicleTabHeader()
        {
            if (Plugin == null)
                return;

            var category = Plugin.GetActiveGraphCategory();

            switch (category)
            {
                case GraphCategory.Helicopter:
                    VehicleTabIcon.Source = (ImageSource)FindResource("HelicopterSymbol");
                    VehicleTabLabel.Content = "AIRCRAFT";
                    break;
                case GraphCategory.Aircraft:
                    VehicleTabIcon.Source = (ImageSource)FindResource("AirplaneSymbol");
                    VehicleTabLabel.Content = "AIRCRAFT";
                    break;
                case GraphCategory.Vehicle:
                default:
                    VehicleTabIcon.Source = (ImageSource)FindResource("VehicleSymbol");
                    VehicleTabLabel.Content = "VEHICLE";
                    break;
            }
        }

        #region Vehicle Parameters

        private Dictionary<string, FrameworkElement> vehicleParamControls = new Dictionary<string, FrameworkElement>();
        private Dictionary<string, Label> vehicleParamLabels = new Dictionary<string, Label>();
        private bool isUpdatingVehicleParams = false;

        private bool IsVehicleParam(GraphParam p)
        {
            var group = p.Ui?.Group;

            // Null/empty -> goes to "<unknown>" panel
            if (string.IsNullOrEmpty(group))
                return true;

            // Exclude System
            if ("System".Equals(group, StringComparison.OrdinalIgnoreCase))
                return false;

            return true;
        }

        private void RefreshVehicleParams()
        {
            VehicleParamsContainer.Children.Clear();
            vehicleParamControls.Clear();
            vehicleParamLabels.Clear();

            if (Plugin == null)
            {
                return;
            }

            // Add Active Functions section at the top
            var activeFunctionsExpander = CreateActiveFunctionsExpander();
            if (activeFunctionsExpander != null)
            {
                VehicleParamsContainer.Children.Add(activeFunctionsExpander);
            }

            var allParams = Plugin.GetActiveGraphParams();
            if (allParams == null || allParams.Count == 0)
            {
                if (activeFunctionsExpander == null)
                {
                    ShowVehicleEmptyState();
                }
                return;
            }

            // Filter to vehicle params only
            var vehicleParams = allParams.Values
                .Where(IsVehicleParam)
                .ToList();

            if (vehicleParams.Count == 0)
            {
                if (activeFunctionsExpander == null)
                {
                    ShowVehicleEmptyState();
                }
                return;
            }

            var orderedNames = Plugin.GetActiveGraphParamOrder();

            // Group by Group property, "<unknown>" for null/empty
            var grouped = vehicleParams
                .GroupBy(p => string.IsNullOrEmpty(p.Ui?.Group) ? "<unknown>" : p.Ui.Group)
                .OrderBy(g => g.Key == "<unknown>" ? "\uFFFF" : g.Key); // <unknown> sorts last

            foreach (var group in grouped)
            {
                var orderedParams = OrderParamsByGraph(orderedNames, group);
                var expander = CreateVehicleGroupExpander(group.Key, orderedParams);
                VehicleParamsContainer.Children.Add(expander);
            }
        }

        private Expander CreateActiveFunctionsExpander()
        {
            // Only show if we have known functions
            if (functions.Count == 0)
                return null;

            var expander = new Expander
            {
                Header = "Active Functions",
                IsExpanded = false,
                Foreground = Brushes.White,
                FontFamily = new FontFamily("Arial Black"),
                FontSize = 12,
                Margin = new Thickness(0, 0, 0, 5)
            };

            var border = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(0x7F, 0x4E, 0x4E, 0x4E)),
                CornerRadius = new CornerRadius(5),
                Padding = new Thickness(10)
            };

            var panel = new StackPanel { Orientation = Orientation.Vertical };

            // Add description
            var description = new TextBlock
            {
                Text = "Select which functions should use profile-specific overrides for this vehicle.",
                Foreground = Brushes.Gray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                TextWrapping = TextWrapping.Wrap,
                Margin = new Thickness(0, 0, 0, 10)
            };
            panel.Children.Add(description);

            // Add a checkbox for each known function (skip Undefined)
            foreach (var kvp in functions)
            {
                if (kvp.Key == FunctionID.Undefined)
                    continue;

                var functionPanel = CreateActiveFunctionRow(kvp.Key, kvp.Value);
                panel.Children.Add(functionPanel);
            }

            border.Child = panel;
            expander.Content = border;
            return expander;
        }

        private StackPanel CreateActiveFunctionRow(FunctionID functionId, Function function)
        {
            var container = new StackPanel
            {
                Orientation = Orientation.Vertical,
                Margin = new Thickness(0, 2, 0, 2)
            };

            var row = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                Height = 24
            };

            bool isActive = Plugin.IsFunctionActive((int)functionId);

            var checkbox = new CheckBox
            {
                IsChecked = isActive,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(0, 0, 8, 0)
            };
            checkbox.Tag = functionId;
            checkbox.Checked += OnActiveFunctionChecked;
            checkbox.Unchecked += OnActiveFunctionUnchecked;

            var nameLabel = new TextBlock
            {
                Text = function.Name,
                Foreground = Brushes.White,
                FontFamily = new FontFamily("Arial"),
                FontSize = 11,
                VerticalAlignment = VerticalAlignment.Center,
                Width = 140
            };

            var statusLabel = new TextBlock
            {
                Text = function.IsOnline ? "(online)" : "(offline)",
                Foreground = function.IsOnline ? Brushes.LightGreen : Brushes.Gray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                FontStyle = FontStyles.Italic,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(5, 0, 0, 0)
            };

            // Add override indicator badge (always create, update visibility dynamically)
            var badge = new TextBlock
            {
                Tag = $"functionBadge_{(int)functionId}",
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                FontWeight = FontWeights.Bold,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(5, 0, 0, 0)
            };
            UpdateFunctionLevelBadge(badge, functionId);

            // Edit button (only for active functions)
            Button editButton = null;
            if (isActive)
            {
                editButton = new Button
                {
                    Content = "▼",
                    Width = 20,
                    Height = 18,
                    FontSize = 8,
                    Padding = new Thickness(0),
                    Margin = new Thickness(10, 0, 0, 0),
                    VerticalAlignment = VerticalAlignment.Center,
                    ToolTip = "Edit overrides"
                };
                editButton.Tag = functionId;
            }

            row.Children.Add(checkbox);
            row.Children.Add(nameLabel);
            row.Children.Add(badge);
            row.Children.Add(statusLabel);
            if (editButton != null) row.Children.Add(editButton);

            container.Children.Add(row);

            // Override editor panel (initially collapsed)
            if (isActive)
            {
                var editorPanel = CreateOverrideEditorPanel(functionId);
                editorPanel.Visibility = Visibility.Collapsed;
                editorPanel.Tag = $"editor_{(int)functionId}";
                container.Children.Add(editorPanel);

                // Wire up edit button toggle
                editButton.Click += (s, e) =>
                {
                    if (editorPanel.Visibility == Visibility.Collapsed)
                    {
                        editorPanel.Visibility = Visibility.Visible;
                        editButton.Content = "▲";
                    }
                    else
                    {
                        editorPanel.Visibility = Visibility.Collapsed;
                        editButton.Content = "▼";
                    }
                };
            }

            return container;
        }

        private Border CreateOverrideEditorPanel(FunctionID functionId)
        {
            var border = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(0x40, 0x30, 0x30, 0x30)),
                BorderBrush = new SolidColorBrush(Color.FromArgb(0x60, 0x60, 0x60, 0x60)),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(3),
                Margin = new Thickness(20, 5, 0, 5),
                Padding = new Thickness(10)
            };

            var panel = new StackPanel { Orientation = Orientation.Vertical };

            var header = new TextBlock
            {
                Text = "Overrides",
                Foreground = Brushes.LightGray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                FontWeight = FontWeights.Bold,
                Margin = new Thickness(0, 0, 0, 8)
            };
            panel.Children.Add(header);

            // Output scaling section
            panel.Children.Add(CreateOverrideFieldRow(functionId, "OutputMin", "Output Min",
                GetOverrideFloatValue(functionId, "OutputMin"), 0f, 1f, "Minimum output value (0-1)"));
            panel.Children.Add(CreateOverrideFieldRow(functionId, "OutputMax", "Output Max",
                GetOverrideFloatValue(functionId, "OutputMax"), 0f, 1f, "Maximum output value (0-1)"));

            // Physics parameters section
            var physicsHeader = new TextBlock
            {
                Text = "Physics",
                Foreground = Brushes.Gray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 9,
                Margin = new Thickness(0, 8, 0, 4)
            };
            panel.Children.Add(physicsHeader);

            panel.Children.Add(CreateOverrideFieldRow(functionId, "SimulatedMass", "Simulated Mass",
                GetOverrideFloatValue(functionId, "SimulatedMass"), 0f, 100f, "Simulated mass in kg"));
            panel.Children.Add(CreateOverrideFieldRow(functionId, "Friction", "Friction",
                GetOverrideFloatValue(functionId, "Friction"), 0f, 10f, "Friction coefficient"));

            // Static balance section
            var balanceHeader = new TextBlock
            {
                Text = "Static Balance Tuning",
                Foreground = Brushes.Gray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 9,
                Margin = new Thickness(0, 8, 0, 4)
            };
            panel.Children.Add(balanceHeader);

            panel.Children.Add(CreateOverrideCheckboxRow(functionId, "StaticBalanceEnabled", "Enabled",
                GetOverrideBoolValue(functionId, "StaticBalanceEnabled"), "Enable static balance compensation"));
            panel.Children.Add(CreateOverrideFieldRow(functionId, "StaticBalanceGain", "Gain",
                GetOverrideFloatValue(functionId, "StaticBalanceGain"), 0f, 2f, "Static balance gain multiplier"));

            border.Child = panel;
            return border;
        }

        private StackPanel CreateOverrideFieldRow(FunctionID functionId, string fieldName, string label,
            float? currentValue, float min, float max, string tooltip)
        {
            var row = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                Height = 26,
                Margin = new Thickness(0, 2, 0, 2)
            };

            var labelBlock = new TextBlock
            {
                Text = label,
                Foreground = Brushes.LightGray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                Width = 100,
                VerticalAlignment = VerticalAlignment.Center,
                ToolTip = tooltip
            };

            var textBox = new TextBox
            {
                Width = 60,
                Height = 20,
                FontSize = 10,
                Text = currentValue?.ToString("F2") ?? "",
                VerticalContentAlignment = VerticalAlignment.Center
            };
            textBox.LostFocus += OnOverrideFieldLostFocus;
            textBox.KeyDown += OnOverrideFieldKeyDown;

            var clearButton = new Button
            {
                Content = "×",
                Width = 18,
                Height = 18,
                FontSize = 10,
                Padding = new Thickness(0),
                Margin = new Thickness(4, 0, 0, 0),
                VerticalAlignment = VerticalAlignment.Center,
                ToolTip = "Clear override",
                Tag = new Tuple<FunctionID, string, TextBox>(functionId, fieldName, textBox),
                Visibility = currentValue.HasValue ? Visibility.Visible : Visibility.Collapsed
            };
            clearButton.Click += OnClearOverrideClick;

            // Badge showing which layer this value comes from
            var badge = CreateLayerBadge((int)functionId, fieldName);
            var tag = new OverrideFieldTag(functionId, fieldName, textBox, clearButton, badge);
            textBox.Tag = tag;
            clearButton.Tag = tag;

            row.Children.Add(labelBlock);
            row.Children.Add(textBox);
            row.Children.Add(clearButton);
            row.Children.Add(badge);

            return row;
        }

        /// <summary>
        /// Create a layer badge that shows [P] for Profile or [U] for User layer.
        /// </summary>
        private TextBlock CreateLayerBadge(int functionId, string fieldName)
        {
            var badge = new TextBlock
            {
                FontFamily = new FontFamily("Arial"),
                FontSize = 8,
                FontWeight = FontWeights.Bold,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(4, 0, 0, 0)
            };

            UpdateLayerBadge(badge, functionId, fieldName);

            return badge;
        }

        private StackPanel CreateOverrideCheckboxRow(FunctionID functionId, string fieldName, string label,
            bool? currentValue, string tooltip)
        {
            var row = new StackPanel
            {
                Orientation = Orientation.Horizontal,
                Height = 26,
                Margin = new Thickness(0, 2, 0, 2)
            };

            var labelBlock = new TextBlock
            {
                Text = label,
                Foreground = Brushes.LightGray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 10,
                Width = 100,
                VerticalAlignment = VerticalAlignment.Center,
                ToolTip = tooltip
            };

            var checkbox = new CheckBox
            {
                IsChecked = currentValue,
                IsThreeState = true, // null = no override, true/false = override value
                VerticalAlignment = VerticalAlignment.Center,
                Tag = new Tuple<FunctionID, string>(functionId, fieldName)
            };
            checkbox.Checked += OnOverrideCheckboxChanged;
            checkbox.Unchecked += OnOverrideCheckboxChanged;
            checkbox.Indeterminate += OnOverrideCheckboxChanged;

            var stateLabel = new TextBlock
            {
                Text = currentValue.HasValue ? (currentValue.Value ? "On" : "Off") : "(default)",
                Foreground = currentValue.HasValue ? Brushes.LightGray : Brushes.Gray,
                FontFamily = new FontFamily("Arial"),
                FontSize = 9,
                FontStyle = currentValue.HasValue ? FontStyles.Normal : FontStyles.Italic,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(8, 0, 0, 0)
            };

            // Badge showing which layer this value comes from
            var badge = CreateLayerBadge((int)functionId, fieldName);
            var tag = new OverrideCheckboxTag(functionId, fieldName, checkbox, stateLabel, badge);
            checkbox.Tag = tag;

            row.Children.Add(labelBlock);
            row.Children.Add(checkbox);
            row.Children.Add(stateLabel);
            row.Children.Add(badge);

            return row;
        }

        private void OnOverrideFieldLostFocus(object sender, RoutedEventArgs e)
        {
            if (!(sender is TextBox textBox)) return;
            if (!(textBox.Tag is OverrideFieldTag tag)) return;

            var functionId = tag.FunctionId;
            var fieldName = tag.FieldName;

            if (string.IsNullOrWhiteSpace(textBox.Text))
            {
                // Empty = clear the override
                ClearOverrideFieldAndRefresh(tag);
                return;
            }

            if (!float.TryParse(textBox.Text, out float value))
            {
                // Invalid input - restore previous value
                var currentValue = GetOverrideFloatValue(functionId, fieldName);
                textBox.Text = currentValue?.ToString("F2") ?? "";
                return;
            }

            // Update the override
            Plugin.UpdateFunctionOverrideField((int)functionId, fieldName, overrides =>
            {
                switch (fieldName)
                {
                    case "OutputMin": overrides.OutputMin = value; break;
                    case "OutputMax": overrides.OutputMax = value; break;
                    case "SimulatedMass": overrides.SimulatedMass = value; break;
                    case "Friction": overrides.Friction = value; break;
                    case "StaticBalanceGain":
                        if (overrides.StaticBalanceTuning == null)
                            overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                        overrides.StaticBalanceTuning.Gain = value;
                        break;
                }
            });

            RefreshOverrideFieldRow(tag);
            // Note: Removed RefreshVehicleParams() to preserve editor panel state
            // The per-field badge and value are already updated by RefreshOverrideFieldRow
        }

        private void OnOverrideFieldKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key != Key.Enter)
                return;

            OnOverrideFieldLostFocus(sender, e);
            e.Handled = true;
        }

        private void OnClearOverrideClick(object sender, RoutedEventArgs e)
        {
            if (!(sender is Button button)) return;
            if (!(button.Tag is OverrideFieldTag tag)) return;

            ClearOverrideFieldAndRefresh(tag);
        }

        private void OnOverrideCheckboxChanged(object sender, RoutedEventArgs e)
        {
            if (isUpdatingOverrideUi)
                return;
            if (!(sender is CheckBox checkbox)) return;
            if (!(checkbox.Tag is OverrideCheckboxTag tag)) return;

            var functionId = tag.FunctionId;
            var fieldName = tag.FieldName;

            if (checkbox.IsChecked == null)
            {
                // Indeterminate = clear override (use default)
                ClearOverrideCheckboxAndRefresh(tag);
            }
            else
            {
                bool value = checkbox.IsChecked.Value;
                Plugin.UpdateFunctionOverrideField((int)functionId, fieldName, overrides =>
                {
                    if (fieldName == "StaticBalanceEnabled")
                    {
                        if (overrides.StaticBalanceTuning == null)
                            overrides.StaticBalanceTuning = new TieredConfig.StaticBalanceTuningOverrides();
                        overrides.StaticBalanceTuning.Enabled = value;
                    }
                });

                RefreshOverrideCheckboxRow(tag);
            }
        }

        private void ClearOverrideFieldAndRefresh(OverrideFieldTag tag)
        {
            var layer = GetOverrideSourceLayer(tag.FunctionId, tag.FieldName);
            Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, layer);

            // Clear both User and Profile layers to fully remove the override
            // Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, ConfigLayer.User);
            // Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, ConfigLayer.Profile);
            RefreshOverrideFieldRow(tag);
            // Note: Not calling RefreshVehicleParams() to preserve editor panel state
        }

        private void ClearOverrideCheckboxAndRefresh(OverrideCheckboxTag tag)
        {
            var layer = GetOverrideSourceLayer(tag.FunctionId, tag.FieldName);
            Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, layer);
            // Clear both User and Profile layers to fully remove the override
            // Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, ConfigLayer.User);
            // Plugin.ClearFunctionOverrideField((int)tag.FunctionId, tag.FieldName, ConfigLayer.Profile);
            RefreshOverrideCheckboxRow(tag);
            // Note: Not calling RefreshVehicleParams() to preserve editor panel state
        }

        private void RefreshOverrideFieldRow(OverrideFieldTag tag)
        {
            var currentValue = GetOverrideFloatValue(tag.FunctionId, tag.FieldName);
            tag.TextBox.Text = currentValue?.ToString("F2") ?? "";
            tag.ClearButton.Visibility = currentValue.HasValue ? Visibility.Visible : Visibility.Collapsed;
            UpdateLayerBadge(tag.Badge, (int)tag.FunctionId, tag.FieldName);
            RefreshFunctionLevelBadge(tag.FunctionId);
        }

        private void RefreshOverrideCheckboxRow(OverrideCheckboxTag tag)
        {
            var currentValue = GetOverrideBoolValue(tag.FunctionId, tag.FieldName);
            isUpdatingOverrideUi = true;
            tag.Checkbox.IsChecked = currentValue;
            isUpdatingOverrideUi = false;

            tag.StateLabel.Text = currentValue.HasValue ? (currentValue.Value ? "On" : "Off") : "(default)";
            tag.StateLabel.Foreground = currentValue.HasValue ? Brushes.LightGray : Brushes.Gray;
            tag.StateLabel.FontStyle = currentValue.HasValue ? FontStyles.Normal : FontStyles.Italic;

            UpdateLayerBadge(tag.Badge, (int)tag.FunctionId, tag.FieldName);
            RefreshFunctionLevelBadge(tag.FunctionId);
        }

        private ConfigLayer? GetOverrideSourceLayer(FunctionID functionId, string fieldName)
        {
            var layerProvider = Plugin.CreateConfigLayerProvider();
            return layerProvider.GetFieldSourceLayer((int)functionId, fieldName);
        }

        private float? GetOverrideFloatValue(FunctionID functionId, string fieldName)
        {
            var userOverrides = Plugin.GetUserFunctionOverrides((int)functionId);
            var profileOverrides = Plugin.GetFunctionOverrides((int)functionId);

            switch (fieldName)
            {
                case "OutputMin":
                    return userOverrides?.OutputMin ?? profileOverrides?.OutputMin;
                case "OutputMax":
                    return userOverrides?.OutputMax ?? profileOverrides?.OutputMax;
                case "SimulatedMass":
                    return userOverrides?.SimulatedMass ?? profileOverrides?.SimulatedMass;
                case "Friction":
                    return userOverrides?.Friction ?? profileOverrides?.Friction;
                case "StaticBalanceGain":
                    return userOverrides?.StaticBalanceTuning?.Gain ?? profileOverrides?.StaticBalanceTuning?.Gain;
                default:
                    return null;
            }
        }

        private bool? GetOverrideBoolValue(FunctionID functionId, string fieldName)
        {
            var userOverrides = Plugin.GetUserFunctionOverrides((int)functionId);
            var profileOverrides = Plugin.GetFunctionOverrides((int)functionId);

            switch (fieldName)
            {
                case "StaticBalanceEnabled":
                    return userOverrides?.StaticBalanceTuning?.Enabled ?? profileOverrides?.StaticBalanceTuning?.Enabled;
                default:
                    return null;
            }
        }

        private void UpdateLayerBadge(TextBlock badge, int functionId, string fieldName)
        {
            var layerProvider = Plugin.CreateConfigLayerProvider();
            var layer = layerProvider.GetFieldSourceLayer(functionId, fieldName);

            if (layer == ConfigLayer.User)
            {
                badge.Text = "[U]";
                badge.Foreground = new SolidColorBrush(Color.FromRgb(0x64, 0xB5, 0xF6));
                badge.ToolTip = "User preference override";
                badge.Visibility = Visibility.Visible;
            }
            else if (layer == ConfigLayer.Profile)
            {
                badge.Text = "[P]";
                badge.Foreground = new SolidColorBrush(Color.FromRgb(0x4C, 0xAF, 0x50));
                badge.ToolTip = "Vehicle profile override";
                badge.Visibility = Visibility.Visible;
            }
            else
            {
                badge.Text = "";
                badge.Visibility = Visibility.Collapsed;
            }
        }

        /// <summary>
        /// Update the function-level badge that shows [U] or [P] next to the function name.
        /// Shows [U] if any field has User override, [P] if any field has Profile override, hidden otherwise.
        /// </summary>
        private void UpdateFunctionLevelBadge(TextBlock badge, FunctionID functionId)
        {
            var userOverrides = Plugin.GetUserFunctionOverrides((int)functionId);
            var profileOverrides = Plugin.GetFunctionOverrides((int)functionId);
            bool hasUserOverrides = userOverrides != null && !userOverrides.IsEmpty;
            bool hasProfileOverrides = profileOverrides != null && !profileOverrides.IsEmpty;
            bool isActive = Plugin.IsFunctionActive((int)functionId);

            if (hasUserOverrides && isActive)
            {
                badge.Text = "[U]";
                badge.Foreground = new SolidColorBrush(Color.FromRgb(0x64, 0xB5, 0xF6));
                badge.ToolTip = "Has user preference overrides";
                badge.Visibility = Visibility.Visible;
            }
            else if (hasProfileOverrides && isActive)
            {
                badge.Text = "[P]";
                badge.Foreground = new SolidColorBrush(Color.FromRgb(0x4C, 0xAF, 0x50));
                badge.ToolTip = "Has vehicle profile overrides";
                badge.Visibility = Visibility.Visible;
            }
            else
            {
                badge.Text = "";
                badge.Visibility = Visibility.Collapsed;
            }
        }

        /// <summary>
        /// Find and update the function-level badge for a given function ID.
        /// Called after field-level changes to keep the function badge in sync.
        /// </summary>
        private void RefreshFunctionLevelBadge(FunctionID functionId)
        {
            var tagName = $"functionBadge_{(int)functionId}";
            var badge = FindElementByTag<TextBlock>(VehicleParamsContainer, tagName);
            if (badge != null)
            {
                UpdateFunctionLevelBadge(badge, functionId);
            }
        }

        /// <summary>
        /// Find a UI element by its Tag value within a container.
        /// </summary>
        private T FindElementByTag<T>(DependencyObject parent, string tag) where T : FrameworkElement
        {
            if (parent == null) return null;

            int childCount = VisualTreeHelper.GetChildrenCount(parent);
            for (int i = 0; i < childCount; i++)
            {
                var child = VisualTreeHelper.GetChild(parent, i);
                if (child is T element && element.Tag?.ToString() == tag)
                    return element;

                var result = FindElementByTag<T>(child, tag);
                if (result != null)
                    return result;
            }
            return null;
        }

        private void OnActiveFunctionChecked(object sender, RoutedEventArgs e)
        {
            if (sender is CheckBox checkbox && checkbox.Tag is FunctionID functionId)
            {
                Plugin.SetFunctionActive((int)functionId, true);
            }
        }

        private void OnActiveFunctionUnchecked(object sender, RoutedEventArgs e)
        {
            if (sender is CheckBox checkbox && checkbox.Tag is FunctionID functionId)
            {
                Plugin.SetFunctionActive((int)functionId, false);
            }
        }

        private void ShowVehicleEmptyState()
        {
            var message = new TextBlock
            {
                Text = "No vehicle parameters defined.\n\nVehicle parameters can be added via graph Param nodes with custom Group values.",
                Foreground = Brushes.Gray,
                FontStyle = FontStyles.Italic,
                Margin = new Thickness(10),
                TextWrapping = TextWrapping.Wrap,
                Width = 400
            };
            VehicleParamsContainer.Children.Add(message);
        }

        private Expander CreateVehicleGroupExpander(string groupName, List<GraphParam> parameters)
        {
            var expander = new Expander
            {
                Header = groupName,
                IsExpanded = true,
                Foreground = Brushes.White,
                FontFamily = new FontFamily("Arial Black"),
                FontSize = 12,
                Margin = new Thickness(0, 0, 0, 0)
            };

            var border = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(0x7F, 0x4E, 0x4E, 0x4E)),
                CornerRadius = new CornerRadius(5),
                Padding = new Thickness(10)
            };

            var panel = new StackPanel { Orientation = Orientation.Vertical };

            foreach (var param in parameters)
            {
                var paramPanel = CreateVehicleParamPanel(param);
                panel.Children.Add(paramPanel);
            }

            border.Child = panel;
            expander.Content = border;
            return expander;
        }

        private static List<GraphParam> OrderParamsByGraph(IReadOnlyList<string> orderedNames, IEnumerable<GraphParam> parameters)
        {
            var map = new Dictionary<string, GraphParam>(StringComparer.OrdinalIgnoreCase);
            foreach (var param in parameters)
            {
                if (!string.IsNullOrWhiteSpace(param?.Name))
                {
                    map[param.Name] = param;
                }
            }

            var ordered = new List<GraphParam>();
            if (orderedNames != null)
            {
                foreach (var name in orderedNames)
                {
                    if (map.TryGetValue(name, out var param))
                    {
                        ordered.Add(param);
                        map.Remove(name);
                    }
                }
            }

            ordered.AddRange(map.Values.OrderBy(p => p.Ui?.Label ?? p.Name));
            return ordered;
        }

        private StackPanel CreateVehicleParamPanel(GraphParam param)
        {
            var panel = new StackPanel
            {
                Width = 400,
                Height = 40,
                Orientation = Orientation.Vertical
            };

            // Get current value (GetGraphParamValue now does full three-tier resolution)
            double currentValue = Plugin.GetGraphParamValue(param.Name);

            var label = new Label
            {
                Foreground = Brushes.White,
                FontSize = 10,
                FontFamily = new FontFamily("Arial"),
                Content = FormatVehicleParamLabel(param, currentValue),
                Padding = new Thickness(0, 0, 0, 8)
            };
            var control = GraphParamControlBuilder.BuildControl(
                param,
                value =>
                {
                    if (!isUpdatingVehicleParams)
                    {
                        Plugin.SetGraphParamValue(param.Name, value);
                        // Update label to show new value
                        if (vehicleParamLabels.TryGetValue(param.Name, out var lbl))
                        {
                            lbl.Content = FormatVehicleParamLabel(param, value);
                        }
                    }
                },
                width: 400,
                initialValue: currentValue
            );

            panel.Children.Add(label);
            panel.Children.Add(control);

            vehicleParamControls[param.Name] = control;
            vehicleParamLabels[param.Name] = label;

            return panel;
        }

        private void OnGraphParamChanged_Vehicle(object sender, GraphParamChangedEventArgs e)
        {
            if (isUpdatingVehicleParams)
                return;

            Dispatcher.Invoke(() =>
            {
                isUpdatingVehicleParams = true;
                try
                {
                    if (vehicleParamControls.TryGetValue(e.ParamName, out var control))
                    {
                        if (control is Slider slider)
                        {
                            slider.Value = e.Value;
                        }
                        else if (control is TextBox textBox)
                        {
                            var allParams = Plugin?.GetActiveGraphParams();
                            int precision = 3;
                            if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                            {
                                precision = param.Ui?.Precision ?? 3;
                            }
                            textBox.Text = e.Value.ToString($"F{precision}");
                        }
                        else if (control is CheckBox checkBox)
                        {
                            checkBox.IsChecked = e.Value > 0.5;
                        }
                    }

                    // Update label with new value
                    if (vehicleParamLabels.TryGetValue(e.ParamName, out var label))
                    {
                        var allParams = Plugin?.GetActiveGraphParams();
                        if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                        {
                            label.Content = FormatVehicleParamLabel(param, e.Value);
                        }
                    }
                }
                finally
                {
                    isUpdatingVehicleParams = false;
                }
            });
        }

        private string FormatVehicleParamLabel(GraphParam param, double currentValue)
        {
            string label = param.Ui?.Label ?? param.Name;

            // Format value with appropriate precision
            int precision = param.Ui?.Precision ?? 3;
            string valueStr = currentValue.ToString($"F{precision}");

            // Build label as: <name>: <value><unit>
            if (!string.IsNullOrWhiteSpace(param.Ui?.Units))
            {
                return $"{label}: {valueStr}{param.Ui.Units}";
            }
            else
            {
                return $"{label}: {valueStr}";
            }
        }

        #endregion
    }
}
