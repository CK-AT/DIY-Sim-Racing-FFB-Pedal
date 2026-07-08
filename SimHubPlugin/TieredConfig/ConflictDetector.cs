using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Represents a conflict where multiple functions try to override the same axis.
    /// </summary>
    public class AxisConflict
    {
        public int AxisId { get; set; }
        public List<int> ConflictingFunctionIds { get; set; } = new List<int>();

        /// <summary>
        /// Human-readable description of the conflict.
        /// </summary>
        public string Description =>
            $"Axis {AxisId} has conflicting overrides from functions: {string.Join(", ", ConflictingFunctionIds)}";
    }

    /// <summary>
    /// Pure functions for detecting conflicts between active function overrides.
    /// </summary>
    public static class ConflictDetector
    {
        /// <summary>
        /// Find all axis conflicts where multiple active functions override the same axis parameters.
        /// </summary>
        /// <param name="activeFunctions">
        /// Map of function_id → (axis_id → overrides) for currently active functions.
        /// </param>
        /// <returns>List of conflicts, empty if no conflicts.</returns>
        public static List<AxisConflict> FindAxisConflicts(
            Dictionary<int, Dictionary<int, AxisParameterOverrides>> activeFunctions)
        {
            if (activeFunctions == null || activeFunctions.Count == 0)
                return new List<AxisConflict>();

            // Build inverse map: axis_id → list of (function_id, overrides) that affect it
            var axisToFunctions = new Dictionary<int, List<(int functionId, AxisParameterOverrides overrides)>>();

            foreach (var funcEntry in activeFunctions)
            {
                int functionId = funcEntry.Key;
                var axisOverrides = funcEntry.Value;

                if (axisOverrides == null)
                    continue;

                foreach (var axisEntry in axisOverrides)
                {
                    int axisId = axisEntry.Key;
                    var overrides = axisEntry.Value;

                    // Skip empty overrides - they don't conflict
                    if (overrides == null || overrides.IsEmpty)
                        continue;

                    if (!axisToFunctions.ContainsKey(axisId))
                        axisToFunctions[axisId] = new List<(int, AxisParameterOverrides)>();

                    axisToFunctions[axisId].Add((functionId, overrides));
                }
            }

            // Find axes with conflicting overrides
            var conflicts = new List<AxisConflict>();

            foreach (var entry in axisToFunctions)
            {
                int axisId = entry.Key;
                var functions = entry.Value;

                if (functions.Count <= 1)
                    continue;

                // Check if overrides actually conflict (overlap on same fields)
                if (HasFieldOverlap(functions.Select(f => f.overrides).ToList()))
                {
                    conflicts.Add(new AxisConflict
                    {
                        AxisId = axisId,
                        ConflictingFunctionIds = functions.Select(f => f.functionId).ToList()
                    });
                }
            }

            return conflicts;
        }

        /// <summary>
        /// Check if multiple overrides have overlapping fields (actual conflict).
        /// Two functions overriding the same axis but different fields (e.g., one Kinematics, one StaticBalance)
        /// is NOT a conflict.
        /// </summary>
        private static bool HasFieldOverlap(List<AxisParameterOverrides> overrides)
        {
            if (overrides.Count <= 1)
                return false;

            // Count how many overrides touch each field
            int kinematicsCount = overrides.Count(o => o.Kinematics != null);
            int staticBalanceCount = overrides.Count(o => o.StaticBalance != null);

            // Conflict if more than one override touches the same field
            return kinematicsCount > 1 || staticBalanceCount > 1;
        }

        /// <summary>
        /// Simplified conflict check: are any conflicts present?
        /// </summary>
        public static bool HasConflicts(
            Dictionary<int, Dictionary<int, AxisParameterOverrides>> activeFunctions)
        {
            return FindAxisConflicts(activeFunctions).Any();
        }

        /// <summary>
        /// Get axes that are locked due to conflicts.
        /// Locked axes should not have configs sent until conflicts are resolved.
        /// </summary>
        public static HashSet<int> GetLockedAxes(
            Dictionary<int, Dictionary<int, AxisParameterOverrides>> activeFunctions)
        {
            var conflicts = FindAxisConflicts(activeFunctions);
            return new HashSet<int>(conflicts.Select(c => c.AxisId));
        }
    }
}
