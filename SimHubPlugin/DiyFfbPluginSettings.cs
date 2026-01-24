using System.Collections.Generic;
using System.Linq.Expressions;
using System.Windows.Media.Converters;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Settings class, make sure it can be correctly serialized using JSON.net
    /// </summary>

    public class DiyFfbPluginSettings
    {
        public const float DefaultXPlaneFfbKq = 1.0f;
        public const float DefaultXPlaneFfbKrate = 1.0f;
        public const float DefaultXPlaneTrimMmPerDeg = 0.1f;
        public const float DefaultXPlaneBuffetStartDeg = 10.0f;
        public const float DefaultXPlaneBuffetFullDeg = 18.0f;
        public const float DefaultXPlaneBuffetGain = 0.05f;
        public const float DefaultXPlaneVrefKts = 60.0f;
        public const float DefaultXPlaneNominalRpm = 400.0f;
        public const float DefaultXPlaneAeroMomentGain = 0.0f;
        public const float DefaultXPlaneTorqueRefNm = 0.0f;
        public const float DefaultXPlaneFfbKcenter = DefaultXPlaneFfbKq;
        public const float DefaultXPlaneFrictionQ = 0.0f;
        public const float DefaultXPlaneFrictionTorque = 0.0f;
        public const float DefaultXPlaneFrictionLowRpm = 0.0f;
        public const float DefaultXPlaneRpmBlend = 0.5f;
        public const float DefaultXPlaneLoadTorqueGain = DefaultXPlaneFfbKq;
        public const float DefaultXPlaneMrTorqueRefNm = 0.0f;
        public const float DefaultXPlaneLoadForceClamp = 1000.0f;

        public class FunctionFfbSettings
        {
            public bool XPlaneFfbEnabled = true;
            public float XPlaneFfbKq = DefaultXPlaneFfbKq;
            public float XPlaneFfbKrate = DefaultXPlaneFfbKrate;
            public float XPlaneTrimMmPerDeg = DefaultXPlaneTrimMmPerDeg;
            public float XPlaneBuffetStartDeg = DefaultXPlaneBuffetStartDeg;
            public float XPlaneBuffetFullDeg = DefaultXPlaneBuffetFullDeg;
            public float XPlaneBuffetGain = DefaultXPlaneBuffetGain;
            public float XPlaneWeathervaneGain = 0.0f;
            public float XPlaneAeroMomentGain = DefaultXPlaneAeroMomentGain;
            public float XPlaneTorqueRefNm = DefaultXPlaneTorqueRefNm;
            public float XPlaneFfbKcenter = DefaultXPlaneFfbKcenter;
            public float XPlaneFrictionQ = DefaultXPlaneFrictionQ;
            public float XPlaneFrictionTorque = DefaultXPlaneFrictionTorque;
            public float XPlaneFrictionLowRpm = DefaultXPlaneFrictionLowRpm;
            public float XPlaneRpmBlend = DefaultXPlaneRpmBlend;
            public float XPlaneLoadTorqueGain = DefaultXPlaneLoadTorqueGain;
            public bool XPlaneReferenceFlightMode = false;
            public float XPlaneLoadForceClamp = DefaultXPlaneLoadForceClamp;

            public void CopyFrom(FunctionSettings source)
            {
                if (source == null)
                {
                    return;
                }

                XPlaneFfbEnabled = source.XPlaneFfbEnabled;
                XPlaneFfbKq = source.XPlaneFfbKq;
                XPlaneFfbKrate = source.XPlaneFfbKrate;
                XPlaneTrimMmPerDeg = source.XPlaneTrimMmPerDeg;
                XPlaneBuffetStartDeg = source.XPlaneBuffetStartDeg;
                XPlaneBuffetFullDeg = source.XPlaneBuffetFullDeg;
                XPlaneBuffetGain = source.XPlaneBuffetGain;
                XPlaneWeathervaneGain = source.XPlaneWeathervaneGain;
                XPlaneAeroMomentGain = source.XPlaneAeroMomentGain;
                XPlaneTorqueRefNm = source.XPlaneTorqueRefNm;
                XPlaneFfbKcenter = source.XPlaneFfbKcenter;
                XPlaneFrictionQ = source.XPlaneFrictionQ;
                XPlaneFrictionTorque = source.XPlaneFrictionTorque;
                XPlaneFrictionLowRpm = source.XPlaneFrictionLowRpm;
                XPlaneRpmBlend = source.XPlaneRpmBlend;
                XPlaneLoadTorqueGain = source.XPlaneLoadTorqueGain;
                XPlaneReferenceFlightMode = source.XPlaneReferenceFlightMode;
                XPlaneLoadForceClamp = source.XPlaneLoadForceClamp;
            }

            public void ApplyTo(FunctionSettings target)
            {
                if (target == null)
                {
                    return;
                }

                target.XPlaneFfbEnabled = XPlaneFfbEnabled;
                target.XPlaneFfbKq = XPlaneFfbKq;
                target.XPlaneFfbKrate = XPlaneFfbKrate;
                target.XPlaneTrimMmPerDeg = XPlaneTrimMmPerDeg;
                target.XPlaneBuffetStartDeg = XPlaneBuffetStartDeg;
                target.XPlaneBuffetFullDeg = XPlaneBuffetFullDeg;
                target.XPlaneBuffetGain = XPlaneBuffetGain;
                target.XPlaneWeathervaneGain = XPlaneWeathervaneGain;
                target.XPlaneAeroMomentGain = XPlaneAeroMomentGain;
                target.XPlaneTorqueRefNm = XPlaneTorqueRefNm;
                target.XPlaneFfbKcenter = XPlaneFfbKcenter;
                target.XPlaneFrictionQ = XPlaneFrictionQ;
                target.XPlaneFrictionTorque = XPlaneFrictionTorque;
                target.XPlaneFrictionLowRpm = XPlaneFrictionLowRpm;
                target.XPlaneRpmBlend = XPlaneRpmBlend;
                target.XPlaneLoadTorqueGain = XPlaneLoadTorqueGain;
                target.XPlaneReferenceFlightMode = XPlaneReferenceFlightMode;
                target.XPlaneLoadForceClamp = XPlaneLoadForceClamp;
            }
        }

        public class AircraftFfbProfile
        {
            public FunctionFfbSettings FlightStickPitch = new FunctionFfbSettings();
            public FunctionFfbSettings FlightStickRoll = new FunctionFfbSettings();
            public FunctionFfbSettings FlightStickCollective = new FunctionFfbSettings();
            public FunctionFfbSettings FlightPedals = new FunctionFfbSettings();
            public int XPlaneRotorIndex = -1;
            public bool XPlaneAircraftIsHelicopter = false;
            public float XPlaneVrefKts = DefaultXPlaneVrefKts;
            public float XPlaneNominalRpm = DefaultXPlaneNominalRpm;
            public float XPlaneMainRotorTorqueRefNm = DefaultXPlaneMrTorqueRefNm;
            public Dictionary<string, double> GraphParamValues = new Dictionary<string, double>();
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
            public bool XPlaneFfbEnabled = true;
            public float XPlaneFfbKq = DefaultXPlaneFfbKq;
            public float XPlaneFfbKrate = DefaultXPlaneFfbKrate;
            public float XPlaneTrimMmPerDeg = DefaultXPlaneTrimMmPerDeg;
            public float XPlaneBuffetStartDeg = DefaultXPlaneBuffetStartDeg;
            public float XPlaneBuffetFullDeg = DefaultXPlaneBuffetFullDeg;
            public float XPlaneBuffetGain = DefaultXPlaneBuffetGain;
            public float XPlaneWeathervaneGain = 0.0f;
            public float XPlaneVrefKts = DefaultXPlaneVrefKts; // legacy per-function value; migrated to system setting
            public float XPlaneAeroMomentGain = DefaultXPlaneAeroMomentGain;
            public float XPlaneTorqueRefNm = DefaultXPlaneTorqueRefNm;
            public float XPlaneFfbKcenter = DefaultXPlaneFfbKcenter;
            public float XPlaneFrictionQ = DefaultXPlaneFrictionQ;
            public float XPlaneFrictionTorque = DefaultXPlaneFrictionTorque;
            public float XPlaneFrictionLowRpm = DefaultXPlaneFrictionLowRpm;
            public float XPlaneRpmBlend = DefaultXPlaneRpmBlend;
            public float XPlaneLoadTorqueGain = DefaultXPlaneLoadTorqueGain;
            public bool XPlaneReferenceFlightMode = false;
            public float XPlaneLoadForceClamp = DefaultXPlaneLoadForceClamp;
            public bool XPlaneUsingVrefScaling = false;
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
        public bool XPlaneAircraftIsHelicopter = false;
        public float XPlaneVrefKtsSystem = DefaultXPlaneVrefKts;
        public float XPlaneNominalRpmSystem = DefaultXPlaneNominalRpm;
        public float XPlaneMainRotorTorqueRefNmSystem = DefaultXPlaneMrTorqueRefNm;

        public Dictionary<string, string> VehicleGraphPaths = new Dictionary<string, string>();
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
        public bool XPlaneUdpEnabled = false;
        public int XPlaneUdpPort = 27015;
        public bool XPlaneTorqueCaptureEnabled = true;
        public float XPlaneFfbKq = DefaultXPlaneFfbKq;
        public float XPlaneFfbKrate = DefaultXPlaneFfbKrate;
        public float XPlaneTrimMmPerDeg = DefaultXPlaneTrimMmPerDeg;
        public float XPlaneBuffetStartDeg = DefaultXPlaneBuffetStartDeg;
        public float XPlaneBuffetFullDeg = DefaultXPlaneBuffetFullDeg;
        public float XPlaneBuffetGain = DefaultXPlaneBuffetGain;
        public Dictionary<string, AircraftFfbProfile> AircraftFfbProfiles = new Dictionary<string, AircraftFfbProfile>();
    }
        

}
