using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Event args for axis config changes.
    /// </summary>
    public class AxisConfigChangedEventArgs : EventArgs
    {
        public int AxisId { get; set; }
        public AxisConfig NewConfig { get; set; }
        public bool HasFunctionOverride { get; set; }
        public int? OverridingFunctionId { get; set; }
    }

    /// <summary>
    /// Manages axis configuration lifecycle for function overrides.
    ///
    /// On function activation: merges overrides into base config, sends to ESP32.
    /// On function deactivation: restores base config, sends to ESP32.
    /// </summary>
    public class AxisConfigManager
    {
        // Base configs (before any function override)
        private readonly Dictionary<int, AxisConfig> _baseConfigs = new Dictionary<int, AxisConfig>();

        // Currently active function override per axis (axis_id → function_id)
        private readonly Dictionary<int, int> _activeOverrideFunction = new Dictionary<int, int>();

        // Last-sent config per axis (for diff checking)
        private readonly Dictionary<int, AxisConfig> _lastSentConfigs = new Dictionary<int, AxisConfig>();

        // Current working config per axis (may have override applied)
        private readonly Dictionary<int, AxisConfig> _currentConfigs = new Dictionary<int, AxisConfig>();

        /// <summary>
        /// Event fired when an axis config needs to be sent to ESP32.
        /// </summary>
        public event EventHandler<AxisConfigChangedEventArgs> AxisConfigChanged;

        /// <summary>
        /// Initialize or update base config for an axis.
        /// Call this when reading config from ESP32 or on startup.
        /// </summary>
        public void SetBaseConfig(int axisId, AxisConfig config)
        {
            if (config == null)
                throw new ArgumentNullException(nameof(config));

            _baseConfigs[axisId] = config.Clone();
            _currentConfigs[axisId] = config.Clone();

            // If no active override, this is also the current config
            if (!_activeOverrideFunction.ContainsKey(axisId))
            {
                _lastSentConfigs[axisId] = config.Clone();
            }
        }

        /// <summary>
        /// Get the base (un-overridden) config for an axis.
        /// </summary>
        public AxisConfig GetBaseConfig(int axisId)
        {
            return _baseConfigs.TryGetValue(axisId, out var config) ? config.Clone() : null;
        }

        /// <summary>
        /// Get the current (possibly overridden) config for an axis.
        /// </summary>
        public AxisConfig GetCurrentConfig(int axisId)
        {
            return _currentConfigs.TryGetValue(axisId, out var config) ? config.Clone() : null;
        }

        /// <summary>
        /// Apply a function's axis parameter overrides.
        /// Call this when a function is activated.
        /// </summary>
        /// <param name="functionId">The function being activated.</param>
        /// <param name="axisOverrides">Map of axis_id → overrides for this function.</param>
        /// <param name="diffCheck">If true, only send if config changed. If false, always send.</param>
        public void ApplyFunctionOverrides(
            int functionId,
            Dictionary<int, AxisParameterOverrides> axisOverrides,
            bool diffCheck = true)
        {
            if (axisOverrides == null || axisOverrides.Count == 0)
                return;

            foreach (var entry in axisOverrides)
            {
                int axisId = entry.Key;
                var overrides = entry.Value;

                if (overrides == null || overrides.IsEmpty)
                    continue;

                ApplyFunctionOverride(axisId, functionId, overrides, diffCheck);
            }
        }

        /// <summary>
        /// Apply a function override to a single axis.
        /// </summary>
        public void ApplyFunctionOverride(
            int axisId,
            int functionId,
            AxisParameterOverrides overrides,
            bool diffCheck = true)
        {
            if (overrides == null || overrides.IsEmpty)
                return;

            // Ensure we have a base config
            if (!_baseConfigs.ContainsKey(axisId))
            {
                throw new InvalidOperationException(
                    $"No base config for axis {axisId}. Call SetBaseConfig first.");
            }

            // Store which function is overriding this axis
            _activeOverrideFunction[axisId] = functionId;

            // Merge overrides into base config
            var merged = ConfigMerger.MergeAxisOverrides(_baseConfigs[axisId], overrides);
            _currentConfigs[axisId] = merged;

            // Check if we need to send
            bool shouldSend = !diffCheck ||
                              !_lastSentConfigs.TryGetValue(axisId, out var lastSent) ||
                              ConfigComparer.HasChanges(merged, lastSent);

            if (shouldSend)
            {
                _lastSentConfigs[axisId] = merged.Clone();
                OnAxisConfigChanged(axisId, merged, true, functionId);
            }
        }

        /// <summary>
        /// Clear function override for an axis, restoring base config.
        /// Call this when a function is deactivated.
        /// </summary>
        /// <param name="axisId">The axis to restore.</param>
        /// <param name="diffCheck">If true, only send if config changed.</param>
        public void ClearFunctionOverride(int axisId, bool diffCheck = true)
        {
            if (!_activeOverrideFunction.ContainsKey(axisId))
                return;

            _activeOverrideFunction.Remove(axisId);

            if (!_baseConfigs.TryGetValue(axisId, out var baseConfig))
                return;

            _currentConfigs[axisId] = baseConfig.Clone();

            // Check if we need to send
            bool shouldSend = !diffCheck ||
                              !_lastSentConfigs.TryGetValue(axisId, out var lastSent) ||
                              ConfigComparer.HasChanges(baseConfig, lastSent);

            if (shouldSend)
            {
                _lastSentConfigs[axisId] = baseConfig.Clone();
                OnAxisConfigChanged(axisId, baseConfig, false, null);
            }
        }

        /// <summary>
        /// Clear all function overrides for a specific function.
        /// Call this when the function is deactivated.
        /// </summary>
        public void ClearFunctionOverrides(int functionId, bool diffCheck = true)
        {
            // Find all axes that this function is overriding
            var axesToClear = _activeOverrideFunction
                .Where(kv => kv.Value == functionId)
                .Select(kv => kv.Key)
                .ToList();

            foreach (var axisId in axesToClear)
            {
                ClearFunctionOverride(axisId, diffCheck);
            }
        }

        /// <summary>
        /// Check if an axis has an active function override.
        /// </summary>
        public bool HasFunctionOverride(int axisId)
        {
            return _activeOverrideFunction.ContainsKey(axisId);
        }

        /// <summary>
        /// Get the function ID that is currently overriding an axis, if any.
        /// </summary>
        public int? GetOverridingFunction(int axisId)
        {
            return _activeOverrideFunction.TryGetValue(axisId, out var funcId) ? funcId : (int?)null;
        }

        /// <summary>
        /// Get all axes that have active function overrides.
        /// </summary>
        public Dictionary<int, int> GetActiveOverrides()
        {
            return new Dictionary<int, int>(_activeOverrideFunction);
        }

        /// <summary>
        /// Mark config as sent (update last-sent tracking).
        /// Call this after manually sending a config outside the manager.
        /// </summary>
        public void MarkAsSent(int axisId, AxisConfig config)
        {
            _lastSentConfigs[axisId] = config.Clone();
        }

        /// <summary>
        /// Reset manager state. Call on disconnect or restart.
        /// </summary>
        public void Reset()
        {
            _baseConfigs.Clear();
            _activeOverrideFunction.Clear();
            _lastSentConfigs.Clear();
            _currentConfigs.Clear();
        }

        /// <summary>
        /// Reset tracking for a specific axis.
        /// </summary>
        public void ResetAxis(int axisId)
        {
            _baseConfigs.Remove(axisId);
            _activeOverrideFunction.Remove(axisId);
            _lastSentConfigs.Remove(axisId);
            _currentConfigs.Remove(axisId);
        }

        protected virtual void OnAxisConfigChanged(
            int axisId,
            AxisConfig config,
            bool hasFunctionOverride,
            int? overridingFunctionId)
        {
            AxisConfigChanged?.Invoke(this, new AxisConfigChangedEventArgs
            {
                AxisId = axisId,
                NewConfig = config,
                HasFunctionOverride = hasFunctionOverride,
                OverridingFunctionId = overridingFunctionId
            });
        }
    }
}
