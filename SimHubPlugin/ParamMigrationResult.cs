using System.Collections.Generic;

namespace DiyFfb
{
    /// <summary>
    /// Result of parameter migration when graph hash changes.
    /// Contains information about changes that may need user review.
    /// </summary>
    public sealed class ParamMigrationResult
    {
        /// <summary>
        /// True if there are changes that may need user attention.
        /// </summary>
        public bool HasChangesToReview =>
            ChangedDefaults.Count > 0 ||
            ClampedValues.Count > 0 ||
            NewOrphans.Count > 0;

        /// <summary>
        /// Parameters where the default value changed from the stored snapshot.
        /// </summary>
        public List<ParamDefaultChange> ChangedDefaults { get; } = new List<ParamDefaultChange>();

        /// <summary>
        /// Parameters where the user's override was clamped to fit new range.
        /// </summary>
        public List<ParamClampInfo> ClampedValues { get; } = new List<ParamClampInfo>();

        /// <summary>
        /// Overrides that became orphans in this migration (param no longer in graph).
        /// </summary>
        public List<string> NewOrphans { get; } = new List<string>();

        /// <summary>
        /// All current orphaned overrides (existing + newly detected).
        /// </summary>
        public List<string> AllOrphans { get; } = new List<string>();

        /// <summary>
        /// The new hash after migration.
        /// </summary>
        public string NewHash { get; set; }

        /// <summary>
        /// Current param snapshots (for updating profile after review).
        /// </summary>
        public Dictionary<string, DiyFfbPluginSettings.ParamSnapshot> CurrentSnapshots { get; set; }
    }

    /// <summary>
    /// Information about a parameter whose default value changed.
    /// </summary>
    public sealed class ParamDefaultChange
    {
        public string ParamName { get; set; }
        public double OldDefault { get; set; }
        public double NewDefault { get; set; }
    }

    /// <summary>
    /// Information about a parameter value that was clamped to fit new range.
    /// </summary>
    public sealed class ParamClampInfo
    {
        public string ParamName { get; set; }
        public double OriginalValue { get; set; }
        public double ClampedValue { get; set; }
        public bool ClampedToMin { get; set; }
        public bool ClampedToMax { get; set; }
        public double NewMin { get; set; }
        public double NewMax { get; set; }
    }
}
