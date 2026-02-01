using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb
{
    /// <summary>
    /// Testable helper class for parameter migration logic.
    /// Extracted from DiyFfbPlugin to enable unit testing.
    /// </summary>
    public static class ParamMigrationHelper
    {
        /// <summary>
        /// Simplified param info for migration logic (avoids dependency on GraphEditor types).
        /// </summary>
        public class ParamInfo
        {
            public string Name { get; set; }
            public double DefaultValue { get; set; }
            public double Min { get; set; }
            public double Max { get; set; }
        }

        /// <summary>
        /// Performs parameter migration when graph hash changes.
        /// Detects changed defaults, clamps out-of-range overrides, and identifies orphaned params.
        /// </summary>
        /// <param name="currentParams">Parameters from the current graph.</param>
        /// <param name="oldSnapshots">Param snapshots from previous review (can be null).</param>
        /// <param name="overrides">User's override values (will be modified if clamping needed).</param>
        /// <param name="newHash">The new graph hash.</param>
        /// <returns>Migration result with changes that may need review.</returns>
        public static ParamMigrationResult Migrate(
            IEnumerable<ParamInfo> currentParams,
            Dictionary<string, DiyFfbPluginSettings.ParamSnapshot> oldSnapshots,
            Dictionary<string, double> overrides,
            string newHash)
        {
            var result = new ParamMigrationResult
            {
                NewHash = newHash
            };

            var paramList = currentParams?.ToList() ?? new List<ParamInfo>();
            var currentParamDict = paramList.ToDictionary(p => p.Name);

            // Build current snapshots
            var currentSnapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>();
            foreach (var param in paramList)
            {
                currentSnapshots[param.Name] = new DiyFfbPluginSettings.ParamSnapshot
                {
                    DefaultValue = param.DefaultValue,
                    Min = param.Min,
                    Max = param.Max
                };
            }
            result.CurrentSnapshots = currentSnapshots;

            var oldSnapshotsDict = oldSnapshots ?? new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>();

            // Check for changed defaults
            foreach (var param in paramList)
            {
                if (oldSnapshotsDict.TryGetValue(param.Name, out var oldSnapshot))
                {
                    if (!NearlyEqual((float)oldSnapshot.DefaultValue, (float)param.DefaultValue))
                    {
                        result.ChangedDefaults.Add(new ParamDefaultChange
                        {
                            ParamName = param.Name,
                            OldDefault = oldSnapshot.DefaultValue,
                            NewDefault = param.DefaultValue
                        });
                    }
                }
            }

            // Check overrides: clamp to new ranges, detect orphans
            if (overrides != null)
            {
                foreach (var kvp in overrides.ToList())
                {
                    string paramName = kvp.Key;
                    double overrideValue = kvp.Value;

                    if (currentParamDict.TryGetValue(paramName, out var param))
                    {
                        // Param still exists - check if value needs clamping
                        if (overrideValue < param.Min)
                        {
                            result.ClampedValues.Add(new ParamClampInfo
                            {
                                ParamName = paramName,
                                OriginalValue = overrideValue,
                                ClampedValue = param.Min,
                                ClampedToMin = true,
                                NewMin = param.Min,
                                NewMax = param.Max
                            });
                            overrides[paramName] = param.Min;
                        }
                        else if (overrideValue > param.Max)
                        {
                            result.ClampedValues.Add(new ParamClampInfo
                            {
                                ParamName = paramName,
                                OriginalValue = overrideValue,
                                ClampedValue = param.Max,
                                ClampedToMax = true,
                                NewMin = param.Min,
                                NewMax = param.Max
                            });
                            overrides[paramName] = param.Max;
                        }
                    }
                    else
                    {
                        // Param no longer in graph - it's an orphan
                        result.AllOrphans.Add(paramName);

                        // Check if it's a NEW orphan (wasn't orphaned before)
                        if (oldSnapshotsDict.ContainsKey(paramName))
                        {
                            result.NewOrphans.Add(paramName);
                        }
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Creates initial param snapshots for a graph (first-time setup).
        /// </summary>
        public static Dictionary<string, DiyFfbPluginSettings.ParamSnapshot> CreateSnapshots(
            IEnumerable<ParamInfo> currentParams)
        {
            var snapshots = new Dictionary<string, DiyFfbPluginSettings.ParamSnapshot>();
            foreach (var param in currentParams ?? Enumerable.Empty<ParamInfo>())
            {
                snapshots[param.Name] = new DiyFfbPluginSettings.ParamSnapshot
                {
                    DefaultValue = param.DefaultValue,
                    Min = param.Min,
                    Max = param.Max
                };
            }
            return snapshots;
        }

        private static bool NearlyEqual(float a, float b)
        {
            return Math.Abs(a - b) < 0.000001f;
        }
    }
}
