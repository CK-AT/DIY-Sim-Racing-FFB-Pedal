using System.Collections.Generic;
using System.Linq.Expressions;
using System.Windows.Media.Converters;
using DiyFfb.TieredConfig;

namespace DiyFfb
{
    /// <summary>
    /// Settings class, make sure it can be correctly serialized using JSON.net
    /// </summary>

    public class DiyFfbPluginSettings
    {
        /// <summary>
        /// Snapshot of a graph parameter's definition at time of last review.
        /// Used to detect when defaults or ranges change.
        /// </summary>
        public class ParamSnapshot
        {
            public double DefaultValue;
            public double Min;
            public double Max;
        }

        public class AircraftFfbProfile
        {
            public string GraphPath;
            public int XPlaneRotorIndex = -1;
            public Dictionary<string, double> GraphParamValues = new Dictionary<string, double>();

            /// <summary>
            /// Hash of graph + includes content at last review.
            /// Null means profile predates hash tracking (will initialize on next load).
            /// </summary>
            public string LastReviewedGraphHash;

            /// <summary>
            /// Snapshot of param definitions at last review.
            /// Key: param name, Value: default/min/max at that time.
            /// </summary>
            public Dictionary<string, ParamSnapshot> LastReviewedParamSnapshots = new Dictionary<string, ParamSnapshot>();

            /// <summary>
            /// Vehicle-specific function config overrides.
            /// Key: function ID, Value: delta overlay for that function.
            /// </summary>
            public Dictionary<int, FunctionConfigOverrides> FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>();

            /// <summary>
            /// Which functions are active for this profile.
            /// On profile load, only these functions have their overrides applied.
            /// </summary>
            public HashSet<int> ActiveFunctionIds = new HashSet<int>();
        }

        /// <summary>
        /// Wrapper for exporting/importing aircraft FFB profiles.
        /// Includes the profile data plus metadata about which graph it was created for.
        /// </summary>
        public class ExportedProfile
        {
            public int Version = 1;
            public string ProfileKey;
            public string GraphPath;
            public string ExportedAt;
            public AircraftFfbProfile Profile;
        }

        public class AxisSettings
        {
            public string com_port_name = "COM1";
            public bool auto_connect = false;
            public bool RTSDTR_False = true;
            public bool USING_ESP32S3 = true;
            public bool via_gateway = false;
        }

        public class FunctionSettings
        {
            public bool ABS_enabled = false;
            public bool RPM_enabled = false;
            public bool WS_enabled = false;
            public bool G_force_enabled = false;
            public bool Road_impact_enabled = false;
            public bool CV1_enabled = false;
            public bool CV2_enabled = false;
            public int CV1_trigger_level = 0;
            public string CV1_binding = "";
            public int CV2_trigger_level = 0;
            public string CV2_binding = "";
            public int action_interval = 30;
            public bool[,] effect_status_profiles = new bool[6, 8] { { false, false, false, false, false, false, false, false }, { false, false, false, false, false, false, false, false }, { false, false, false, false, false, false, false, false } , { false, false, false, false, false, false, false, false }, { false, false, false, false, false, false, false, false }, { false, false, false, false, false, false, false, false } };
        }

        public DiyFfbPluginSettings()
        {
            for (int i = 0; i < 8; i++)
            {
                axis_settings[i] = new AxisSettings();
                function_settings[i] = new FunctionSettings();
            }
        }

        //should change the variable name after array size change to updtae the config setting
        public AxisSettings[] axis_settings = new AxisSettings[8];
        public FunctionSettings[] function_settings = new FunctionSettings[8];
        public uint axis_tab_selected = 0;
        public uint function_tab_selected = 0;
        public int XPlaneRotorIndex = -1;
        // Obsolete: kept for migration. Use AircraftFfbProfiles[key].GraphPath instead.
        public Dictionary<string, string> VehicleGraphPaths = new Dictionary<string, string>();
        // Obsolete: game-level graph fallback removed. Profile Browser handles new vehicles.
        public Dictionary<string, string> GameGraphPaths = new Dictionary<string, string>();

        public string[] selectedJsonFileNames = { "1", "2", "3" };
        public int reading_config = 0;
        public uint RPM_effect_type = 0;
        public int vjoy_output_flag = 0;
        public uint vjoy_order = 1;
        public string[,] Pedal_file_string = new string[6, 3] { { "NA", "NA", "NA" }, { "NA", "NA", "NA" }, { "NA", "NA", "NA" }, { "NA", "NA", "NA" }, { "NA", "NA", "NA" }, { "NA", "NA", "NA" } };
        public int[,] file_enable_check = new int[6, 3] { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
        public string WSeffect_bind = "";
        public string Road_impact_bind = "";
        public int WS_trigger = 30;
        public string[] Profile_name = new string[6] { "", "", "", "", "", "" };
        public double kinematicDiagram_zeroPos_OX = 100;
        public double kinematicDiagram_zeroPos_OY = 20;
        public double kinematicDiagram_zeroPos_scale = 1.5;
        public string ESPNow_port = "";
        public bool Pedal_ESPNow_auto_connect_flag = false;
        public bool Serial_auto_clean = false; //clean serial monitor
        public bool Serial_auto_clean_bridge = false; //clean serial monitor bridge
        public bool Using_CDC_bridge = false;
        public bool Rudder_RPM_effect_b = false;
        public bool Rudder_ACC_effect_b = false;
        public bool Rudder_ACC_WindForce = false;
        public bool advanced_b = false;
        public string SSID_string = "";
        public string PASS_string = "";
        public string OtaCustomUrl = "";
        public bool OtaUseCustomUrl = false;
        public bool OtaUseLocalSource = false;
        public string OtaLocalFfbotaPath = "";
        public int OtaLocalPort = 8000;
        public bool XPlaneUdpEnabled = true;
        public int XPlaneUdpPort = 27015;
        public Dictionary<string, AircraftFfbProfile> AircraftFfbProfiles = new Dictionary<string, AircraftFfbProfile>();

        // Tiered Config Override System
        /// <summary>
        /// Current user profile name. Defaults to Windows username.
        /// User preferences are stored per-profile to support multiple users on shared rigs.
        /// </summary>
        public string CurrentUserProfile = System.Environment.UserName;

        /// <summary>
        /// Per-user preferences keyed by user profile name.
        /// Contains function config overrides that follow the user across vehicles.
        /// </summary>
        public Dictionary<string, UserPreferences> UserPreferencesProfiles = new Dictionary<string, UserPreferences>();

        /// <summary>
        /// Per-function axis parameter overrides (function_id → axis_id → overrides).
        /// Defined globally; only applied when function is active for current profile.
        /// </summary>
        public Dictionary<int, Dictionary<int, AxisParameterOverrides>> FunctionAxisOverrides = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>();

        /// <summary>
        /// Function baselines (complete FunctionConfig snapshots) stored in SimHub.
        /// These serve as the "Baseline layer" for tiered config merge operations.
        /// Updated only by: importing compound configs, or explicit "Save to Baseline" action.
        /// Key: function ID, Value: complete FunctionConfig snapshot.
        /// </summary>
        // Store as JSON strings because FunctionConfig (protobuf) doesn't serialize correctly with JSON.NET
        public Dictionary<int, string> FunctionBaselines = new Dictionary<int, string>();

        /// <summary>
        /// Axis baselines (complete AxisConfig snapshots) stored in SimHub.
        /// Prevents ESP32 reconnect from overwriting axis geometry with overridden values.
        /// Key: axis ID, Value: complete AxisConfig snapshot as JSON.
        /// </summary>
        public Dictionary<int, string> AxisBaselines = new Dictionary<int, string>();

        /// <summary>
        /// Maps logical grip signal names (e.g., "Grip.TrimHat.Up") to physical
        /// joystick button or keyboard key bindings for graph input.
        /// </summary>
        public Dictionary<string, ButtonBinding> GripButtonBindings = new Dictionary<string, ButtonBinding>();
    }


}
