using System;
using System.Collections.Generic;
using GameReaderCommon;

namespace User.PluginSdkDemo
{
    public static class GraphSignalCatalog
    {
        public static readonly IReadOnlyList<string> InputNames = GraphSignalCatalogData.InputNames;
        public static readonly IReadOnlyList<string> OutputNames = GraphSignalCatalogData.OutputNames;
        public static readonly IReadOnlyList<string> InputGroups = GraphSignalCatalogData.InputGroups;
        public static readonly IReadOnlyList<string> OutputGroups = GraphSignalCatalogData.OutputGroups;
        public static readonly IReadOnlyList<string> ParamGroups = GraphSignalCatalogData.ParamGroups;

        public static IReadOnlyList<string> GetInputSignalsForGroup(string group) =>
            GraphSignalCatalogData.GetInputSignalsForGroup(group);

        public static IReadOnlyList<string> GetOutputSignalsForGroup(string group) =>
            GraphSignalCatalogData.GetOutputSignalsForGroup(group);

        public static bool IsValidInputSignal(string fullName) =>
            GraphSignalCatalogData.IsValidInputSignal(fullName);

        public static bool IsValidOutputSignal(string fullName) =>
            GraphSignalCatalogData.IsValidOutputSignal(fullName);

        public static string BuildSignalName(string group, string suffix) =>
            GraphSignalCatalogData.BuildSignalName(group, suffix);

        public static void BuildXPlaneInputs(DiyFfbPlugin plugin, GameData data, IDictionary<string, double> inputs)
        {
            if (plugin == null || inputs == null)
            {
                return;
            }

            var packet = plugin.GetLatestXPlanePacket();
            if (packet == null)
            {
                return;
            }

            int rotorIndex = plugin.ResolveXPlaneRotorIndexForGraph(packet);
            double torqueNm = 0.0;
            double omegaRad = 0.0;
            if (rotorIndex >= 0 && rotorIndex < packet.TorqueNm.Length)
            {
                torqueNm = packet.TorqueNm[rotorIndex];
                omegaRad = packet.OmegaRad[rotorIndex];
            }

            // Telemetry inputs using new hierarchical naming
            inputs["XPlane.Speed.IAS"] = packet.IasKts;
            inputs["XPlane.Angle.Alpha"] = packet.AlphaDeg;
            inputs["XPlane.Angle.Beta"] = packet.BetaDeg;
            inputs["XPlane.Rate.Roll"] = packet.PRate;   // p = roll rate
            inputs["XPlane.Rate.Pitch"] = packet.QRate;  // q = pitch rate
            inputs["XPlane.Rate.Yaw"] = packet.RRate;    // r = yaw rate
            inputs["XPlane.G_Nrml"] = packet.GNrml;
            inputs["XPlane.AeroTorque.Roll"] = packet.LAero;
            inputs["XPlane.AeroTorque.Pitch"] = packet.MAero;
            inputs["XPlane.AeroTorque.Yaw"] = packet.NAero;
            inputs["XPlane.MainRotor.Torque"] = torqueNm;
            inputs["XPlane.MainRotor.Speed"] = DiyFfbPlugin.ToRpm((float)omegaRad);
            inputs["XPlane.OnGround"] = packet.OnGround ? 1.0 : 0.0;
        }
    }
}
