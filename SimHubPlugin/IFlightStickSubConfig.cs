/// <summary>
/// Common interface for FlightStickPitchConfig, FlightStickRollConfig, and
/// FlightStickCollectiveConfig.  All three protobuf types share the same four
/// properties but have no generated base type.  This interface lets UI code
/// dispatch once on mode and then work with properties directly.
/// </summary>
internal interface IFlightStickSubConfig
{
    int PosMin { get; set; }
    int PosMax { get; set; }
    float Damping { get; set; }
    float CenteringSpringConst { get; set; }
}

// Partial class extensions — the properties already exist on each generated type,
// so we only need to declare the interface.

partial class FlightStickPitchConfig : IFlightStickSubConfig { }
partial class FlightStickRollConfig : IFlightStickSubConfig { }
partial class FlightStickCollectiveConfig : IFlightStickSubConfig { }
