// Plan 19 - SimVar registration table. Order is the contract: the client
// registers SimVars in this order, MSFS streams doubles back in the same
// order, and the plugin reads sample[(int)MsfsSampleIndex.X] to populate
// MsfsUdpPacket fields.

namespace DiyFfb.Msfs
{
    internal static class MsfsSimVarTable
    {
        public struct Entry
        {
            public string Name;
            public string Units;
            public Entry(string name, string units) { Name = name; Units = units; }
        }

        // Order matches MsfsSampleIndex below. 36 entries. Editing this
        // table without also editing MsfsSampleIndex breaks the contract.
        public static readonly Entry[] Entries = new[]
        {
            new Entry("AIRSPEED INDICATED",                       "Knots"),
            new Entry("AIRSPEED TRUE",                            "Knots"),
            new Entry("INCIDENCE ALPHA",                          "Degrees"),
            new Entry("INCIDENCE BETA",                           "Degrees"),
            new Entry("ROTATION VELOCITY BODY X",                 "Radians per second"),
            new Entry("ROTATION VELOCITY BODY Y",                 "Radians per second"),
            new Entry("ROTATION VELOCITY BODY Z",                 "Radians per second"),
            new Entry("G FORCE",                                  "GForce"),
            new Entry("VELOCITY WORLD Y",                         "Feet per second"),
            new Entry("VELOCITY BODY X",                          "Feet per second"),
            new Entry("VELOCITY BODY Y",                          "Feet per second"),
            new Entry("VELOCITY BODY Z",                          "Feet per second"),
            new Entry("PLANE PITCH DEGREES",                      "Radians"),
            new Entry("PLANE BANK DEGREES",                       "Radians"),
            new Entry("TOTAL WEIGHT",                             "Pounds"),
            new Entry("AMBIENT DENSITY",                          "Slugs per cubic feet"),
            new Entry("ROTOR RPM:1",                              "Rpm"),
            new Entry("ROTOR RPM:2",                              "Rpm"),
            new Entry("ENG TORQUE PERCENT:1",                     "Percent"),
            new Entry("COLLECTIVE POSITION",                      "Percent"),
            new Entry("TAIL ROTOR PEDAL POSITION",                "Percent"),
            new Entry("TAIL ROTOR BLADE PITCH PCT",               "Percent"),
            new Entry("ROTOR COLLECTIVE BLADE PITCH PCT",         "Percent"),
            new Entry("ROTOR CYCLIC BLADE PITCH PCT",             "Percent"),
            new Entry("ROTOR CYCLIC BLADE MAX PITCH POSITION",    "Radians"),
            new Entry("DISK PITCH ANGLE",                         "Radians"),
            new Entry("DISK BANK ANGLE",                          "Radians"),
            new Entry("DISK CONING PCT",                          "Percent"),
            new Entry("ROTOR LATERAL TRIM PCT",                   "Percent"),
            new Entry("ROTOR LONGITUDINAL TRIM PCT",              "Percent"),
            new Entry("ROTOR ROTATION ANGLE:1",                   "Radians"),
            new Entry("ELEVATOR TRIM PCT",                        "Percent"),
            new Entry("AILERON TRIM PCT",                         "Percent"),
            new Entry("RUDDER TRIM PCT",                          "Percent"),
            new Entry("SIM ON GROUND",                            "Bool"),
            new Entry("GROUND VELOCITY",                          "Knots"),
        };

        public const int SampleCount = 36;
    }

    // Index enum keeps the plugin's sample[(int)X] reads self-documenting.
    // Slot numbers MUST stay aligned with MsfsSimVarTable.Entries above.
    internal enum MsfsSampleIndex
    {
        IasKts = 0,
        TasKts,
        AlphaDeg,
        BetaDeg,
        PRateRadS,
        QRateRadS,
        RRateRadS,
        GForce,
        VviWorldFps,
        VelocityBodyXFps,
        VelocityBodyYFps,
        VelocityBodyZFps,
        PitchRad,
        BankRad,
        TotalWeightLb,
        AmbientDensitySlugsFt3,
        MainRotorRpm,
        TailRotorRpm,
        EngTorquePct,
        CollectivePosPct,
        TailRotorPedalPct,
        TailRotorBladePitchPct,
        RotorCollectiveBladePitchPct,
        RotorCyclicBladePitchPct,
        RotorCyclicBladeMaxPitchPosRad,
        DiskPitchAngleRad,
        DiskBankAngleRad,
        DiskConingPct,
        RotorLateralTrimPct,
        RotorLongitudinalTrimPct,
        RotorRotationAngleRad,
        ElevTrimPct,
        AilTrimPct,
        RudTrimPct,
        SimOnGround,
        GroundVelocityKts,
    }
}
