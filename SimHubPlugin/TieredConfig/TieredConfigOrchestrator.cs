using System;
using System.Collections.Generic;
using System.Linq;
using ProtbufTest;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Owns the merge-and-apply lifecycle for tiered config (Baseline → Profile → User).
    /// Extracted from DiyFfbPlugin to isolate config orchestration from the plugin god class.
    /// No UI references — pure data orchestration.
    /// </summary>
    public class TieredConfigOrchestrator
    {
        private readonly FunctionConfigManager _functionConfigManager;
        private readonly AxisConfigManager _axisConfigManager;
        private readonly DiyFfbPluginSettings _settings;
        private readonly Action _persistSettings;
        private readonly Func<DiyFfbPluginSettings.AircraftFfbProfile> _getActiveProfile;
        private readonly Func<GraphCategory> _getActiveGraphCategory;
        private readonly Func<string, string, string> _buildProfileKey;

        // Temporary delegate — replaced when GetOrCreateCurrentProfile moves in (Phase 3).
        private readonly Func<DiyFfbPluginSettings.AircraftFfbProfile> _getOrCreateProfile;

        public TieredConfigOrchestrator(
            FunctionConfigManager functionConfigManager,
            AxisConfigManager axisConfigManager,
            DiyFfbPluginSettings settings,
            Action persistSettings,
            Func<DiyFfbPluginSettings.AircraftFfbProfile> getActiveProfile,
            Func<GraphCategory> getActiveGraphCategory,
            Func<string, string, string> buildProfileKey,
            Func<DiyFfbPluginSettings.AircraftFfbProfile> getOrCreateProfile)
        {
            _functionConfigManager = functionConfigManager ?? throw new ArgumentNullException(nameof(functionConfigManager));
            _axisConfigManager = axisConfigManager ?? throw new ArgumentNullException(nameof(axisConfigManager));
            _settings = settings ?? throw new ArgumentNullException(nameof(settings));
            _persistSettings = persistSettings ?? throw new ArgumentNullException(nameof(persistSettings));
            _getActiveProfile = getActiveProfile ?? throw new ArgumentNullException(nameof(getActiveProfile));
            _getActiveGraphCategory = getActiveGraphCategory ?? throw new ArgumentNullException(nameof(getActiveGraphCategory));
            _buildProfileKey = buildProfileKey ?? throw new ArgumentNullException(nameof(buildProfileKey));
            _getOrCreateProfile = getOrCreateProfile ?? throw new ArgumentNullException(nameof(getOrCreateProfile));
        }

        #region Baseline Management — Function

        /// <summary>
        /// Get the function baseline (Baseline layer) for a function.
        /// Returns null if no baseline has been stored.
        /// </summary>
        public FunctionConfig GetFunctionBaseline(int functionId)
        {
            if (_settings.FunctionBaselines == null)
                return null;

            if (!_settings.FunctionBaselines.TryGetValue(functionId, out var json) || string.IsNullOrEmpty(json))
                return null;

            return ProtobufJsonHelper.FromJson<FunctionConfig>(json);
        }

        /// <summary>
        /// Set the function baseline (Baseline layer) for a function.
        /// This stores a complete FunctionConfig snapshot as the baseline default.
        /// </summary>
        public void SetFunctionBaseline(int functionId, FunctionConfig config)
        {
            if (config == null)
                throw new ArgumentNullException(nameof(config));

            if (_settings.FunctionBaselines == null)
                _settings.FunctionBaselines = new Dictionary<int, string>();

            // Convert protobuf to JSON string for storage (protobuf doesn't serialize correctly with JSON.NET)
            string json = ProtobufJsonHelper.ToJson(config);
            _settings.FunctionBaselines[functionId] = json;

            // Persist to disk
            _persistSettings();
        }

        /// <summary>
        /// Check if a function has a stored baseline.
        /// </summary>
        public bool HasFunctionBaseline(int functionId)
        {
            return _settings.FunctionBaselines?.ContainsKey(functionId) == true;
        }

        /// <summary>
        /// Clear stored function baseline. Overrides are preserved and re-apply
        /// when a new baseline arrives (ESP32 upload, import, or manual edit).
        /// </summary>
        public void ClearFunctionBaseline(int functionId)
        {
            _settings.FunctionBaselines?.Remove(functionId);
            _functionConfigManager.ResetFunction(functionId);
            _persistSettings();
        }

        #endregion

        #region Baseline Management — Axis

        /// <summary>
        /// Get the axis baseline (Baseline layer) for an axis.
        /// Returns null if no baseline has been stored.
        /// </summary>
        public AxisConfig GetAxisBaseline(int axisId)
        {
            if (_settings.AxisBaselines == null)
                return null;

            if (!_settings.AxisBaselines.TryGetValue(axisId, out var json) || string.IsNullOrEmpty(json))
                return null;

            return ProtobufJsonHelper.FromJson<AxisConfig>(json);
        }

        /// <summary>
        /// Set the axis baseline (Baseline layer) for an axis.
        /// Stores a complete AxisConfig snapshot as the baseline default.
        /// </summary>
        public void SetAxisBaseline(int axisId, AxisConfig config)
        {
            if (config == null)
                throw new ArgumentNullException(nameof(config));

            if (_settings.AxisBaselines == null)
                _settings.AxisBaselines = new Dictionary<int, string>();

            string json = ProtobufJsonHelper.ToJson(config);
            _settings.AxisBaselines[axisId] = json;

            // Update manager's base config
            _axisConfigManager.SetBaseConfig(axisId, config);

            // Persist to disk
            _persistSettings();
        }

        /// <summary>
        /// Check if an axis has a stored baseline.
        /// </summary>
        public bool HasAxisBaseline(int axisId)
        {
            return _settings.AxisBaselines?.ContainsKey(axisId) == true;
        }

        /// <summary>
        /// Clear stored axis baseline. Overrides are preserved and re-apply
        /// when a new baseline arrives.
        /// </summary>
        public void ClearAxisBaseline(int axisId)
        {
            _settings.AxisBaselines?.Remove(axisId);
            _axisConfigManager.ResetAxis(axisId);
            _persistSettings();
        }

        #endregion

        #region Manager Initialization

        /// <summary>
        /// Initialize FunctionConfigManager with stored baselines and overrides from settings.
        /// Call this during plugin initialization after settings are loaded.
        /// </summary>
        public void InitializeManagerFromSettings()
        {
            if (_settings?.FunctionBaselines == null)
                return;

            // Load all stored baselines into the manager
            foreach (var kvp in _settings.FunctionBaselines)
            {
                int functionId = kvp.Key;
                string json = kvp.Value;

                var baseline = ProtobufJsonHelper.FromJson<FunctionConfig>(json);
                if (baseline == null)
                    continue;

                // Set base config in manager
                _functionConfigManager.SetBaseConfig(functionId, baseline);

                // Apply stored overrides (profile + user)
                ReapplyMergedOverrides(functionId, diffCheck: false);
            }

            // Load axis baselines into the axis config manager
            InitializeAxisManagerFromSettings();
        }

        /// <summary>
        /// Initialize AxisConfigManager with stored axis baselines from settings.
        /// </summary>
        public void InitializeAxisManagerFromSettings()
        {
            if (_settings?.AxisBaselines == null)
                return;

            foreach (var kvp in _settings.AxisBaselines)
            {
                int axisId = kvp.Key;
                var baseline = ProtobufJsonHelper.FromJson<AxisConfig>(kvp.Value);
                if (baseline == null)
                    continue;

                _axisConfigManager.SetBaseConfig(axisId, baseline);
            }
        }

        /// <summary>
        /// Get initial function config from manager (for UI initialization).
        /// Returns merged config (baseline + overrides) if available, otherwise null.
        /// </summary>
        public FunctionConfig GetInitialFunctionConfig(int functionId)
        {
            if (_functionConfigManager.HasBaseConfig(functionId))
            {
                return _functionConfigManager.GetCurrentConfig(functionId);
            }
            return null;
        }

        /// <summary>
        /// Get initial axis config from manager (for UI initialization).
        /// Returns base config if available, otherwise null.
        /// </summary>
        public AxisConfig GetInitialAxisConfig(int axisId)
        {
            if (_axisConfigManager.HasBaseConfig(axisId))
            {
                return _axisConfigManager.GetCurrentConfig(axisId);
            }
            return null;
        }

        #endregion

        #region Override Application

        /// <summary>
        /// Apply function and axis config overrides from a profile.
        /// Called on vehicle/aircraft change.
        /// </summary>
        public void ApplyProfileFunctionOverrides(DiyFfbPluginSettings.AircraftFfbProfile profile)
        {
            SimHub.Logging.Current.Info($"[TieredConfig] ApplyProfileFunctionOverrides: profile={(profile != null ? "exists" : "null")}, " +
                $"activeFuncs={(profile?.ActiveFunctionIds?.Count.ToString() ?? "n/a")}, " +
                $"knownFuncs={string.Join(",", _functionConfigManager.GetKnownFunctionIds())}");

            // Clear existing overrides silently — events are deferred until all overrides
            // are applied, so the ESP32 gets exactly one upload per changed function
            // with the final merged config (no intermediate baseline flash).
            _functionConfigManager.ClearAllProfileOverrides(fireEvents: false);
            _axisConfigManager.Reset(); // Clear function overrides for axes
            InitializeAxisManagerFromSettings(); // Re-populate base configs from stored baselines

            var userOverrides = GetCurrentUserOverrides();

            if (profile == null)
            {
                // No profile, but still apply user overrides to all functions with baselines
                if (userOverrides?.FunctionOverrides != null)
                {
                    foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
                    {
                        userOverrides.FunctionOverrides.TryGetValue(functionId, out var userDelta);
                        if (userDelta != null && !userDelta.IsEmpty)
                        {
                            _functionConfigManager.ApplyProfileOverrides(functionId, null, userDelta);
                        }
                    }
                }
                // Flush: send baseline for functions that lost overrides but weren't re-applied
                _functionConfigManager.SendAllPendingChanges();
                return;
            }

            var activeFunctions = profile.ActiveFunctionIds ?? new HashSet<int>();

            foreach (var functionId in activeFunctions)
            {
                // 1. Apply FunctionConfig overrides
                if (_functionConfigManager.HasBaseConfig(functionId))
                {
                    profile.FunctionOverrides.TryGetValue(functionId, out var profileDelta);
                    FunctionConfigOverrides userDelta = null;
                    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                    _functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta);
                }

                // 2. Apply AxisConfig overrides for this function
                if (_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
                {
                    _axisConfigManager.ApplyFunctionOverrides(functionId, axisOverrides);
                }
            }

            // Apply user overrides for functions NOT in activeFunctions.
            // User-layer overrides (output range, friction, force curve, etc.) follow the
            // user across vehicles and must always be applied regardless of profile settings.
            if (userOverrides?.FunctionOverrides != null)
            {
                foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
                {
                    if (activeFunctions.Contains(functionId))
                        continue; // already handled above

                    userOverrides.FunctionOverrides.TryGetValue(functionId, out var userDelta);
                    if (userDelta != null && !userDelta.IsEmpty)
                    {
                        _functionConfigManager.ApplyProfileOverrides(functionId, null, userDelta);
                    }
                }
            }

            // Flush: send baseline for functions that lost overrides but weren't re-applied
            _functionConfigManager.SendAllPendingChanges();
        }

        /// <summary>
        /// Apply profile overrides to a single function.
        /// Called when a function config is first received from ESP32.
        /// </summary>
        public void ApplyProfileOverridesToFunction(int functionId)
        {
            var profile = _getActiveProfile();
            if (profile?.ActiveFunctionIds?.Contains(functionId) != true)
                return;

            // Only apply if we have a base config from the ESP32
            if (!_functionConfigManager.HasBaseConfig(functionId))
                return;

            ReapplyMergedOverrides(functionId);

            if (_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
            {
                _axisConfigManager.ApplyFunctionOverrides(functionId, axisOverrides);
            }
        }

        /// <summary>
        /// Gather profile + user deltas and re-apply merged overrides for a function.
        /// Centralizes the pattern used by override update/clear methods.
        /// </summary>
        public void ReapplyMergedOverrides(int functionId, bool diffCheck = true)
        {
            var profile = _getActiveProfile();
            FunctionConfigOverrides profileDelta = null;
            profile?.FunctionOverrides?.TryGetValue(functionId, out profileDelta);

            var userOverrides = GetCurrentUserOverrides();
            FunctionConfigOverrides userDelta = null;
            userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);

            _functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta, diffCheck: diffCheck);
        }

        /// <summary>
        /// Re-apply profile/user overrides for the current vehicle profile.
        /// </summary>
        public void ApplyCurrentProfileOverrides()
        {
            var profile = _getActiveProfile();
            ApplyProfileFunctionOverrides(profile);
        }

        #endregion

        #region User Override Retrieval

        /// <summary>
        /// Get user preferences for the current user profile.
        /// </summary>
        public UserPreferences GetCurrentUserOverrides()
        {
            if (_settings?.UserPreferencesProfiles == null)
                return null;

            string userProfile = _settings.CurrentUserProfile ?? System.Environment.UserName;
            if (string.IsNullOrWhiteSpace(userProfile))
                userProfile = System.Environment.UserName;

            _settings.UserPreferencesProfiles.TryGetValue(userProfile, out var prefs);
            return prefs;
        }

        #endregion

        #region Function Activity

        /// <summary>
        /// Check if profile overrides should be applied to a function.
        /// Returns true if the function is in the active profile's function list.
        /// </summary>
        public bool ShouldApplyProfileOverride(int functionId)
        {
            var profile = _getActiveProfile();
            return profile?.ActiveFunctionIds?.Contains(functionId) == true;
        }

        /// <summary>
        /// Check if a function is active for the current vehicle profile.
        /// When a profile exists, uses its ActiveFunctionIds.
        /// When no profile exists, uses graph-category defaults.
        /// </summary>
        public bool IsFunctionActive(int functionId)
        {
            var profile = _getActiveProfile();
            if (profile == null)
                return IsDefaultActiveFunction(functionId);
            return profile.ActiveFunctionIds?.Contains(functionId) == true;
        }

        private bool IsDefaultActiveFunction(int functionId)
        {
            var funcId = (FunctionID)functionId;
            var category = _getActiveGraphCategory();
            switch (category)
            {
                case GraphCategory.Helicopter:
                    return funcId == FunctionID.FlightPedals ||
                           funcId == FunctionID.FlightStickPitch ||
                           funcId == FunctionID.FlightStickRoll ||
                           funcId == FunctionID.FlightStickCollective;
                case GraphCategory.Aircraft:
                    return funcId == FunctionID.FlightPedals ||
                           funcId == FunctionID.FlightStickPitch ||
                           funcId == FunctionID.FlightStickRoll;
                case GraphCategory.Vehicle:
                default:
                    return funcId == FunctionID.BrakePedal ||
                           funcId == FunctionID.AcceleratorPedal ||
                           funcId == FunctionID.ClutchPedal;
            }
        }

        /// <summary>
        /// Populate ActiveFunctionIds on a newly auto-assigned profile with category defaults.
        /// Called after graph load so GetActiveGraphCategory() returns the correct category.
        /// </summary>
        public void SeedDefaultActiveFunctionIds(string gameId, string carId)
        {
            string key = _buildProfileKey(gameId, carId);
            if (string.IsNullOrWhiteSpace(key) ||
                _settings?.AircraftFfbProfiles == null ||
                !_settings.AircraftFfbProfiles.TryGetValue(key, out var profile))
                return;

            var defaults = new HashSet<int>();
            foreach (FunctionID fid in Enum.GetValues(typeof(FunctionID)))
            {
                if (fid == FunctionID.Undefined) continue;
                int id = (int)fid;
                if (IsDefaultActiveFunction(id))
                    defaults.Add(id);
            }
            profile.ActiveFunctionIds = defaults;
        }

        /// <summary>
        /// Set whether a function is active for the current vehicle profile.
        /// When activated, applies profile/user overrides; when deactivated, restores base config.
        /// </summary>
        public void SetFunctionActive(int functionId, bool active)
        {
            var profile = _getOrCreateProfile();
            if (profile == null)
                return;

            if (profile.ActiveFunctionIds == null)
                profile.ActiveFunctionIds = new HashSet<int>();

            bool wasActive = profile.ActiveFunctionIds.Contains(functionId);
            if (active == wasActive)
                return;

            if (active)
            {
                profile.ActiveFunctionIds.Add(functionId);
                // Apply overrides for newly activated function
                ApplyProfileOverridesToFunction(functionId);
            }
            else
            {
                profile.ActiveFunctionIds.Remove(functionId);
                // Clear profile overrides - restore base config
                _functionConfigManager.ClearProfileOverride(functionId);
                // Clear axis overrides for this function
                _axisConfigManager.ClearFunctionOverrides(functionId);

                // Re-apply user overrides (they follow the user, not the profile)
                if (_functionConfigManager.HasBaseConfig(functionId))
                {
                    var userOverrides = GetCurrentUserOverrides();
                    FunctionConfigOverrides userDelta = null;
                    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                    if (userDelta != null && !userDelta.IsEmpty)
                    {
                        _functionConfigManager.ApplyProfileOverrides(functionId, null, userDelta);
                    }
                }
            }
        }

        #endregion
    }
}
