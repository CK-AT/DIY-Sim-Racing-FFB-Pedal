using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb
{
    /// <summary>
    /// Readable config fields a ConfigIn graph node can source. Mirror of the
    /// write-side OverrideFieldRegistry, but each entry exposes the MERGED
    /// (effective) value read from a live FunctionConfig — i.e. the actual
    /// current value "wherever it is on this axis" — rather than an override delta.
    /// v1: motion range (PosMin/PosMax) on the shared FlightControl config, which
    /// all flight functions (Pitch/Roll/Pedals/Collective) use.
    /// </summary>
    public static class ConfigInFieldCatalog
    {
        public sealed class Field
        {
            public string FieldPath { get; set; } = "";
            public string DisplayName { get; set; } = "";

            /// <summary>Reads the merged/effective value, or null if not present.</summary>
            public Func<global::FunctionConfig, double?> GetMergedValue { get; set; }
        }

        public static readonly IReadOnlyList<Field> Fields = new[]
        {
            new Field
            {
                FieldPath = "FlightControl.PosMin",
                DisplayName = "Motion Pos Min (mm)",
                GetMergedValue = c => c?.FlightControl != null ? (double?)c.FlightControl.PosMin : null
            },
            new Field
            {
                FieldPath = "FlightControl.PosMax",
                DisplayName = "Motion Pos Max (mm)",
                GetMergedValue = c => c?.FlightControl != null ? (double?)c.FlightControl.PosMax : null
            },
        };

        public static Field Get(string fieldPath)
        {
            if (string.IsNullOrEmpty(fieldPath)) return null;
            return Fields.FirstOrDefault(f => string.Equals(f.FieldPath, fieldPath, StringComparison.Ordinal));
        }

        public static IReadOnlyList<string> FieldPaths => Fields.Select(f => f.FieldPath).ToList();
    }
}
