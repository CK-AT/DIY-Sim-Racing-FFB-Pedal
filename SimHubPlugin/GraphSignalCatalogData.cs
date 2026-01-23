using System.Collections.Generic;

namespace User.PluginSdkDemo
{
    public static class GraphSignalCatalogData
    {
        public static readonly IReadOnlyList<string> InputNames = new[]
        {
            "XPlane.IAS_kts",
            "XPlane.Alpha_deg",
            "XPlane.Beta_deg",
            "XPlane.PRate",
            "XPlane.QRate",
            "XPlane.RRate",
            "XPlane.GNrml",
            "XPlane.AeroTorque.RollNm",
            "XPlane.AeroTorque.PitchNm",
            "XPlane.AeroTorque.YawNm",
            "XPlane.Vref_kts",
            "XPlane.NominalRpm",
            "XPlane.MrTorqueRefNm",
            "XPlane.MainRotorTorqueNm",
            "XPlane.MainRotorRpm",
            "XPlane.OnGround"
        };

        public static readonly IReadOnlyList<string> OutputNames = new[]
        {
            "FlightStickPitch.SpringGain",
            "FlightStickPitch.DamperGain",
            "FlightStickPitch.Friction",
            "FlightStickPitch.LoadForce",
            "FlightStickPitch.TrimOffset",
            "FlightStickRoll.SpringGain",
            "FlightStickRoll.DamperGain",
            "FlightStickRoll.Friction",
            "FlightStickRoll.LoadForce",
            "FlightStickRoll.TrimOffset",
            "FlightPedals.SpringGain",
            "FlightPedals.DamperGain",
            "FlightPedals.Friction",
            "FlightPedals.LoadForce",
            "FlightPedals.TrimOffset",
            "FlightStickCollective.SpringGain",
            "FlightStickCollective.DamperGain",
            "FlightStickCollective.Friction",
            "FlightStickCollective.LoadForce",
            "FlightStickCollective.TrimOffset"
        };
    }
}
