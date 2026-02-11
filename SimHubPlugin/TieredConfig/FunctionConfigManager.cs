using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Event args for function config changes.
    /// </summary>
    public class FunctionConfigChangedEventArgs : EventArgs
    {
        public int FunctionId { get; set; }
        public FunctionConfig NewConfig { get; set; }
        public bool HasProfileOverride { get; set; }
        public bool HasUserOverride { get; set; }
    }

    /// <summary>
    /// Manages function configuration lifecycle for profile/user overrides.
    ///
    /// On profile change: merges profile + user overrides into base config, sends to ESP32.
    /// On profile clear: restores base config, sends to ESP32.
    /// </summary>
    public class FunctionConfigManager
    {
        // Base configs from ESP32 (hardware layer)
        private readonly Dictionary<int, FunctionConfig> _baseConfigs = new Dictionary<int, FunctionConfig>();

        // Last-sent config per function (for diff checking)
        private readonly Dictionary<int, FunctionConfig> _lastSentConfigs = new Dictionary<int, FunctionConfig>();

        // Current working config per function (may have overrides applied)
        private readonly Dictionary<int, FunctionConfig> _currentConfigs = new Dictionary<int, FunctionConfig>();

        // Track which functions have active profile overrides
        private readonly HashSet<int> _functionsWithProfileOverride = new HashSet<int>();

        // Track which functions have active user overrides
        private readonly HashSet<int> _functionsWithUserOverride = new HashSet<int>();

        /// <summary>
        /// Event fired when a function config needs to be sent to ESP32.
        /// </summary>
        public event EventHandler<FunctionConfigChangedEventArgs> FunctionConfigChanged;

        /// <summary>
        /// Initialize or update base config for a function.
        /// Call this when reading config from ESP32 or on startup.
        /// </summary>
        public void SetBaseConfig(int functionId, FunctionConfig config)
        {
            if (config == null)
                throw new ArgumentNullException(nameof(config));

            _baseConfigs[functionId] = config.Clone();
            _currentConfigs[functionId] = config.Clone();

            // If no active overrides, this is also the last-sent config
            if (!_functionsWithProfileOverride.Contains(functionId) &&
                !_functionsWithUserOverride.Contains(functionId))
            {
                _lastSentConfigs[functionId] = config.Clone();
            }
        }

        /// <summary>
        /// Check if we have a base config for a function.
        /// </summary>
        public bool HasBaseConfig(int functionId)
        {
            return _baseConfigs.ContainsKey(functionId);
        }

        /// <summary>
        /// Get the base (un-overridden) config for a function.
        /// </summary>
        public FunctionConfig GetBaseConfig(int functionId)
        {
            return _baseConfigs.TryGetValue(functionId, out var config) ? config.Clone() : null;
        }

        /// <summary>
        /// Get the current (possibly overridden) config for a function.
        /// </summary>
        public FunctionConfig GetCurrentConfig(int functionId)
        {
            return _currentConfigs.TryGetValue(functionId, out var config) ? config.Clone() : null;
        }

        /// <summary>
        /// Apply profile and user overrides to a function.
        /// Call this when a profile is activated or overrides change.
        /// </summary>
        /// <param name="functionId">The function to apply overrides to.</param>
        /// <param name="profileDelta">Profile-level overrides (may be null).</param>
        /// <param name="userDelta">User-level overrides (may be null).</param>
        /// <param name="diffCheck">If true, only send if config changed.</param>
        public void ApplyProfileOverrides(
            int functionId,
            FunctionConfigOverrides profileDelta,
            FunctionConfigOverrides userDelta,
            bool diffCheck = true)
        {
            // Ensure we have a base config
            if (!_baseConfigs.TryGetValue(functionId, out var baseConfig))
            {
                throw new InvalidOperationException(
                    $"No base config for function {functionId}. Call SetBaseConfig first.");
            }

            // Track override state
            bool hasProfile = profileDelta != null && !profileDelta.IsEmpty;
            bool hasUser = userDelta != null && !userDelta.IsEmpty;

            if (hasProfile)
                _functionsWithProfileOverride.Add(functionId);
            else
                _functionsWithProfileOverride.Remove(functionId);

            if (hasUser)
                _functionsWithUserOverride.Add(functionId);
            else
                _functionsWithUserOverride.Remove(functionId);

            // Merge all layers
            var merged = ConfigMerger.MergeAllLayers(baseConfig, profileDelta, userDelta);
            _currentConfigs[functionId] = merged;

            // Check if we need to send
            bool hasLastSent = _lastSentConfigs.TryGetValue(functionId, out var lastSent);
            bool hasChanges = hasLastSent && ConfigComparer.HasChanges(merged, lastSent);
            bool shouldSend = !diffCheck || !hasLastSent || hasChanges;

            if (shouldSend)
            {
                // Note: _lastSentConfigs is NOT updated here. The UI event handler
                // calls MarkAsSent() after actually enqueuing the upload. This prevents
                // the tracking from being poisoned during Init (before UI exists).
                OnFunctionConfigChanged(functionId, merged, hasProfile, hasUser);
            }
        }

        /// <summary>
        /// Clear all profile overrides for a function, restoring base config.
        /// Call this when a profile is deactivated.
        /// </summary>
        /// <param name="functionId">The function to restore.</param>
        /// <param name="diffCheck">If true, only send if config changed.</param>
        /// <param name="fireEvents">If false, update internal state silently (for batched clear+apply).</param>
        public void ClearProfileOverride(int functionId, bool diffCheck = true, bool fireEvents = true)
        {
            _functionsWithProfileOverride.Remove(functionId);
            _functionsWithUserOverride.Remove(functionId);

            if (!_baseConfigs.TryGetValue(functionId, out var baseConfig))
                return;

            _currentConfigs[functionId] = baseConfig.Clone();

            if (!fireEvents)
                return;

            // Check if we need to send
            bool shouldSend = !diffCheck ||
                              !_lastSentConfigs.TryGetValue(functionId, out var lastSent) ||
                              ConfigComparer.HasChanges(baseConfig, lastSent);

            if (shouldSend)
            {
                OnFunctionConfigChanged(functionId, _currentConfigs[functionId], false, false);
            }
        }

        /// <summary>
        /// Clear all profile overrides for all functions.
        /// Call this on profile/vehicle change to restore all functions to base.
        /// </summary>
        /// <param name="diffCheck">If true, only send if config changed.</param>
        /// <param name="fireEvents">If false, update internal state silently (for batched clear+apply).</param>
        public void ClearAllProfileOverrides(bool diffCheck = true, bool fireEvents = true)
        {
            // Get list of functions to clear before modifying collections
            var functionsToClear = _functionsWithProfileOverride
                .Union(_functionsWithUserOverride)
                .ToList();

            foreach (var functionId in functionsToClear)
            {
                ClearProfileOverride(functionId, diffCheck, fireEvents);
            }
        }

        /// <summary>
        /// Check if a function has an active profile override.
        /// </summary>
        public bool HasProfileOverride(int functionId)
        {
            return _functionsWithProfileOverride.Contains(functionId);
        }

        /// <summary>
        /// Check if a function has an active user override.
        /// </summary>
        public bool HasUserOverride(int functionId)
        {
            return _functionsWithUserOverride.Contains(functionId);
        }

        /// <summary>
        /// Check if a function has any active override (profile or user).
        /// </summary>
        public bool HasAnyOverride(int functionId)
        {
            return _functionsWithProfileOverride.Contains(functionId) ||
                   _functionsWithUserOverride.Contains(functionId);
        }

        /// <summary>
        /// Get all function IDs that have active overrides.
        /// </summary>
        public HashSet<int> GetFunctionsWithOverrides()
        {
            var result = new HashSet<int>(_functionsWithProfileOverride);
            result.UnionWith(_functionsWithUserOverride);
            return result;
        }

        /// <summary>
        /// Get all function IDs that have base configs.
        /// </summary>
        public IEnumerable<int> GetKnownFunctionIds()
        {
            return _baseConfigs.Keys;
        }

        /// <summary>
        /// Mark config as sent (update last-sent tracking).
        /// Call this after manually sending a config outside the manager.
        /// </summary>
        public void MarkAsSent(int functionId, FunctionConfig config)
        {
            _lastSentConfigs[functionId] = config.Clone();
        }

        /// <summary>
        /// Clear last-sent tracking for a function.
        /// Call when an upload is suppressed so the next activation re-sends.
        /// </summary>
        public void InvalidateLastSent(int functionId)
        {
            _lastSentConfigs.Remove(functionId);
        }

        /// <summary>
        /// Fire events for any function whose current config differs from last-sent.
        /// Call after a batched clear+apply (fireEvents:false) to emit exactly one
        /// event per changed function with the final merged config.
        /// </summary>
        public void SendAllPendingChanges()
        {
            foreach (var functionId in _currentConfigs.Keys.ToList())
            {
                var current = _currentConfigs[functionId];
                if (!_lastSentConfigs.TryGetValue(functionId, out var lastSent) ||
                    ConfigComparer.HasChanges(current, lastSent))
                {
                    OnFunctionConfigChanged(functionId, current,
                        _functionsWithProfileOverride.Contains(functionId),
                        _functionsWithUserOverride.Contains(functionId));
                }
            }
        }

        /// <summary>
        /// Reset manager state. Call on disconnect or restart.
        /// </summary>
        public void Reset()
        {
            _baseConfigs.Clear();
            _lastSentConfigs.Clear();
            _currentConfigs.Clear();
            _functionsWithProfileOverride.Clear();
            _functionsWithUserOverride.Clear();
        }

        /// <summary>
        /// Reset tracking for a specific function.
        /// </summary>
        public void ResetFunction(int functionId)
        {
            _baseConfigs.Remove(functionId);
            _lastSentConfigs.Remove(functionId);
            _currentConfigs.Remove(functionId);
            _functionsWithProfileOverride.Remove(functionId);
            _functionsWithUserOverride.Remove(functionId);
        }

        protected virtual void OnFunctionConfigChanged(
            int functionId,
            FunctionConfig config,
            bool hasProfileOverride,
            bool hasUserOverride)
        {
            FunctionConfigChanged?.Invoke(this, new FunctionConfigChangedEventArgs
            {
                FunctionId = functionId,
                NewConfig = config,
                HasProfileOverride = hasProfileOverride,
                HasUserOverride = hasUserOverride
            });
        }
    }
}
