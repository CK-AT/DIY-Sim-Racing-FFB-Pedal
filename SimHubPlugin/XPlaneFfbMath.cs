using System;

namespace User.PluginSdkDemo
{
    internal static class XPlaneFfbMath
    {
        public const float MaxQScale = 2.0f;
        private const float KtsToMps = 0.514444f;

        public static float ComputeQScaleFromIasKts(float iasKts, float vrefKts)
        {
            float iasMps = iasKts * KtsToMps;
            float qHat = iasMps * iasMps;
            return ComputeQScaleFromQHat(qHat, vrefKts);
        }

        public static float ComputeQScaleFromQHat(float qHat, float vrefKts)
        {
            float vrefMps = vrefKts * KtsToMps;
            float vref2 = vrefMps * vrefMps;
            if (qHat <= 0.0f)
            {
                return 0.0f;
            }

            float scale = (2.0f * qHat) / (qHat + vref2);
            return Math.Min(MaxQScale, scale);
        }

        public static float ComputeBuffet(float alphaDeg, float buffetStartDeg, float buffetFullDeg, float buffetGain, float qScale)
        {
            if (buffetFullDeg <= buffetStartDeg || buffetGain <= 0.0f)
            {
                return 0.0f;
            }
            if (alphaDeg <= buffetStartDeg)
            {
                return 0.0f;
            }

            float t = (alphaDeg - buffetStartDeg) / (buffetFullDeg - buffetStartDeg);
            t = Math.Max(0.0f, Math.Min(1.0f, t));
            return t * buffetGain * qScale;
        }
    }
}
