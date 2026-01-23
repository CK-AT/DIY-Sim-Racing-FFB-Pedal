using System;
using System.Collections.Generic;
using GameReaderCommon;

namespace User.PluginSdkDemo
{
    public static class GraphSignalCatalog
    {
        public static readonly IReadOnlyList<string> InputNames = GraphSignalCatalogData.InputNames;
        public static readonly IReadOnlyList<string> OutputNames = GraphSignalCatalogData.OutputNames;

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

            double iasKts = packet.IasKts;
            double alphaDeg = packet.AlphaDeg;
            double betaDeg = packet.BetaDeg;
            double pRate = packet.PRate;
            double qRate = packet.QRate;
            double rRate = packet.RRate;
            double gNrml = packet.GNrml;
            double lAero = packet.LAero;
            double mAero = packet.MAero;
            double nAero = packet.NAero;
            bool onGround = packet.OnGround;

            inputs["XPlane.IAS_kts"] = iasKts;
            inputs["XPlane.Alpha_deg"] = alphaDeg;
            inputs["XPlane.Beta_deg"] = betaDeg;
            inputs["XPlane.PRate"] = pRate;
            inputs["XPlane.QRate"] = qRate;
            inputs["XPlane.RRate"] = rRate;
            inputs["XPlane.GNrml"] = gNrml;
            inputs["XPlane.AeroTorque.RollNm"] = lAero;
            inputs["XPlane.AeroTorque.PitchNm"] = mAero;
            inputs["XPlane.AeroTorque.YawNm"] = nAero;
            inputs["XPlane.Vref_kts"] = plugin.GetXPlaneVrefKts();
            inputs["XPlane.NominalRpm"] = plugin.GetXPlaneNominalRpm();
            inputs["XPlane.MrTorqueRefNm"] = plugin.Settings?.XPlaneMainRotorTorqueRefNmSystem ?? 0.0f;
            inputs["XPlane.MainRotorTorqueNm"] = torqueNm;
            inputs["XPlane.MainRotorRpm"] = DiyFfbPlugin.ToRpm((float)omegaRad);
            inputs["XPlane.OnGround"] = onGround ? 1.0 : 0.0;
        }
    }
}
