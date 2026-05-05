using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Threading;
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
        private readonly Func<string> _getActiveGameId;
        private readonly Func<string> _getActiveCarId;

        // Per-function throttle for override-driven config merges (~1/sec).
        // Prevents flooding ESP32 during slider dragging.
        private const int OverrideThrottleMs = 1000;
        private readonly Dictionary<int, DispatcherTimer> _throttleTimers = new Dictionary<int, DispatcherTimer>();
        private readonly HashSet<int> _pendingMerges = new HashSet<int>();
        private readonly Dictionary<int, DateTime> _lastMergeUtc = new Dictionary<int, DateTime>();

        // In-memory ConfigOut tier: graph-derived field values that participate in the
        // merge but are never serialized to the profile JSON. Cleared on graph reload
        // so a new graph's ConfigOut wiring fully redefines the contents.
        private readonly Dictionary<int, FunctionConfigOverrides> _configOutOverrides = new Dictionary<int, FunctionConfigOverrides>();

        public TieredConfigOrchestrator(
            FunctionConfigManager functionConfigManager,
            AxisConfigManager axisConfigManager,
            DiyFfbPluginSettings settings,
            Action persistSettings,
            Func<DiyFfbPluginSettings.AircraftFfbProfile> getActiveProfile,
            Func<GraphCategory> getActiveGraphCategory,
            Func<string, string, string> buildProfileKey,
            Func<string> getActiveGameId,
            Func<string> getActiveCarId)
        {
            _functionConfigManager = functionConfigManager ?? throw new ArgumentNullException(nameof(functionConfigManager));
            _axisConfigManager = axisConfigManager ?? throw new ArgumentNullException(nameof(axisConfigManager));
            _settings = settings ?? throw new ArgumentNullException(nameof(settings));
            _persistSettings = persistSettings ?? throw new ArgumentNullException(nameof(persistSettings));
            _getActiveProfile = getActiveProfile ?? throw new ArgumentNullException(nameof(getActiveProfile));
            _getActiveGraphCategory = getActiveGraphCategory ?? throw new ArgumentNullException(nameof(getActiveGraphCategory));
            _buildProfileKey = buildProfileKey ?? throw new ArgumentNullException(nameof(buildProfileKey));
            _getActiveGameId = getActiveGameId ?? throw new ArgumentNullException(nameof(getActiveGameId));
            _getActiveCarId = getActiveCarId ?? throw new ArgumentNullException(nameof(getActiveCarId));
        }

        #region Events

        /// <summary>
        /// Fired when the active context changes (profile/user/vehicle switch, or Upload Changes).
        /// Subscribers should perform full refresh and expect ESP32 config send.
        /// </summary>
        public event EventHandler ContextChanged;

        /// <summary>
        /// Fired when a single override field is edited (NO ESP32 send).
        /// Subscribers should perform targeted badge/UI refresh only.
        /// </summary>
        public event EventHandler<OverrideFieldChangedEventArgs> OverrideFieldChanged;

        /// <summary>
        /// Fires the ContextChanged event to notify subscribers of a full context change.
        /// Call this on profile/vehicle/user switches, or after "Upload Changes" button.
        /// </summary>
        public void OnContextChanged()
        {
            ContextChanged?.Invoke(this, EventArgs.Empty);
        }

        /// <summary>
        /// Fires the OverrideFieldChanged event for a specific field edit (NO ESP32 send).
        /// </summary>
        public void OnOverrideFieldChanged(int functionId, string fieldPath)
        {
            OverrideFieldChanged?.Invoke(this, new OverrideFieldChangedEventArgs(functionId, fieldPath));
        }

        #endregion

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

            // Migrate old flight-function baseline field names before deserialization
            MigrateFlightFunctionBaselines();

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
        /// Migrate stored flight-function baselines through every prior shape
        /// to the current consolidated `flightControl` form. Idempotent.
        /// History:
        ///   1. flightStickPitch/flightStickRoll/flightStickCollective →
        ///      flightStick (plan 02)
        ///   2. flightStick + flightPedals → flightControl (plan 10)
        ///   Plus pos_near_lim / pos_far_lim → pos_min / pos_max for the
        ///   pedal motion range fields.
        /// </summary>
        private void MigrateFlightFunctionBaselines()
        {
            int[] flightFunctionIds = {
                (int)FunctionID.FlightPedals,
                (int)FunctionID.FlightStickPitch,
                (int)FunctionID.FlightStickRoll,
                (int)FunctionID.FlightStickCollective,
            };
            foreach (int id in flightFunctionIds)
            {
                if (!_settings.FunctionBaselines.ContainsKey(id))
                    continue;

                string json = _settings.FunctionBaselines[id];
                string migrated = MigrateFlightControlJson(json);
                if (migrated != json)
                    _settings.FunctionBaselines[id] = migrated;
            }
        }

        /// <summary>
        /// Rewrite legacy flight-function JSON shapes to the unified
        /// flightControl form. Public for tests; idempotent.
        /// </summary>
        public static string MigrateFlightControlJson(string json)
        {
            if (json == null) return null;
            return json
                // Plan 02 collapse (legacy 3-arm form)
                .Replace("\"flightStickPitch\"", "\"flightControl\"")
                .Replace("\"flightStickRoll\"", "\"flightControl\"")
                .Replace("\"flightStickCollective\"", "\"flightControl\"")
                // Plan 10 consolidation (FlightStick + FlightPedals → FlightControl)
                .Replace("\"flightStick\"", "\"flightControl\"")
                .Replace("\"flightPedals\"", "\"flightControl\"")
                // FlightPedalsConfig used pos_near_lim / pos_far_lim; FlightControlConfig uses pos_min / pos_max
                .Replace("\"posNearLim\"", "\"posMin\"")
                .Replace("\"posFarLim\"", "\"posMax\"");
        }

        // Backwards-compatible alias for tests/external callers that referenced
        // the previous migration helper name.
        public static string MigrateFlightStickJson(string json) => MigrateFlightControlJson(json);

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
            // Cancel any pending throttled merges from slider dragging — the old
            // override state is about to be replaced by the new profile.
            CancelAllThrottledMerges();

            // Clear existing overrides silently — events are deferred until all overrides
            // are applied, so the ESP32 gets exactly one upload per changed function
            // with the final merged config (no intermediate baseline flash).
            _functionConfigManager.ClearAllProfileOverrides(fireEvents: false);
            _axisConfigManager.Reset(); // Clear function overrides for axes
            InitializeAxisManagerFromSettings(); // Re-populate base configs from stored baselines

            var userOverrides = GetCurrentUserOverrides();

            if (profile == null)
            {
                // No profile — still apply user overrides AND any ConfigOut tier values
                // to all functions with baselines.
                foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
                {
                    FunctionConfigOverrides userDelta = null;
                    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                    ApplyOverridesIfAnyTier(functionId, profileDelta: null, userDelta: userDelta);
                }
                // Flush: send baseline for functions that lost overrides but weren't re-applied
                _functionConfigManager.SendAllPendingChanges();
                return;
            }

            // Null means never initialized (legacy profile) — seed category defaults
            // so the profile persists them on next save. Empty set means the user
            // explicitly unchecked all functions — respect that.
            if (profile.ActiveFunctionIds == null)
                profile.ActiveFunctionIds = GetDefaultActiveFunctionIds();

            var activeFunctions = profile.ActiveFunctionIds;

            foreach (var functionId in activeFunctions)
            {
                // 1. Apply FunctionConfig overrides
                if (_functionConfigManager.HasBaseConfig(functionId))
                {
                    profile.FunctionOverrides.TryGetValue(functionId, out var profileDelta);
                    FunctionConfigOverrides userDelta = null;
                    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                    _configOutOverrides.TryGetValue(functionId, out var configOutDelta);
                    _functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta, diffCheck: false, configOutDelta: configOutDelta);
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
            // Same applies to graph-derived ConfigOut values for those functions.
            foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
            {
                if (activeFunctions.Contains(functionId))
                    continue; // already handled above

                FunctionConfigOverrides userDelta = null;
                userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                ApplyOverridesIfAnyTier(functionId, profileDelta: null, userDelta: userDelta);
            }

            // Force events for functions transitioning to inactive — even if config
            // hasn't changed, the gateway needs a cleared config to stop joystick output.
            var newActiveIds = profile.ActiveFunctionIds;
            foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
            {
                if (!newActiveIds.Contains(functionId))
                    _functionConfigManager.InvalidateLastSent(functionId);
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
            if (profile == null)
                return;
            bool isActive = profile.ActiveFunctionIds == null
                ? IsDefaultActiveFunction(functionId)
                : profile.ActiveFunctionIds.Contains(functionId);
            if (!isActive)
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

            _configOutOverrides.TryGetValue(functionId, out var configOutDelta);

            _functionConfigManager.ApplyProfileOverrides(functionId, profileDelta, userDelta, diffCheck: diffCheck, configOutDelta: configOutDelta);
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
            if (profile == null)
                return false;
            if (profile.ActiveFunctionIds == null)
                return IsDefaultActiveFunction(functionId);
            return profile.ActiveFunctionIds.Contains(functionId);
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
            if (profile.ActiveFunctionIds == null)
                return IsDefaultActiveFunction(functionId);
            return profile.ActiveFunctionIds.Contains(functionId);
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
        /// Build a HashSet of function IDs that are active by default for the current graph category.
        /// </summary>
        private HashSet<int> GetDefaultActiveFunctionIds()
        {
            var defaults = new HashSet<int>();
            foreach (FunctionID fid in Enum.GetValues(typeof(FunctionID)))
            {
                if (fid == FunctionID.Undefined) continue;
                int id = (int)fid;
                if (IsDefaultActiveFunction(id))
                    defaults.Add(id);
            }
            return defaults;
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

            profile.ActiveFunctionIds = GetDefaultActiveFunctionIds();
        }

        /// <summary>
        /// Set whether a function is active for the current vehicle profile.
        /// When activated, applies profile/user overrides; when deactivated, restores base config.
        /// </summary>
        public void SetFunctionActive(int functionId, bool active)
        {
            var profile = GetOrCreateCurrentProfile();
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

                // Re-apply user overrides (they follow the user, not the profile) and
                // any ConfigOut tier values (graph-derived).
                if (_functionConfigManager.HasBaseConfig(functionId))
                {
                    var userOverrides = GetCurrentUserOverrides();
                    FunctionConfigOverrides userDelta = null;
                    userOverrides?.FunctionOverrides?.TryGetValue(functionId, out userDelta);
                    ApplyOverridesIfAnyTier(functionId, profileDelta: null, userDelta: userDelta);
                }
            }
        }

        #endregion

        #region Profile/Override Accessors

        /// <summary>
        /// Get or create the current vehicle profile.
        /// </summary>
        public DiyFfbPluginSettings.AircraftFfbProfile GetOrCreateCurrentProfile()
        {
            var activeCarId = _getActiveCarId();
            if (_settings == null || string.IsNullOrWhiteSpace(activeCarId))
                return null;

            if (_settings.AircraftFfbProfiles == null)
                _settings.AircraftFfbProfiles = new Dictionary<string, DiyFfbPluginSettings.AircraftFfbProfile>();

            string profileKey = _buildProfileKey(_getActiveGameId(), activeCarId);
            if (string.IsNullOrWhiteSpace(profileKey))
                return null;

            if (!_settings.AircraftFfbProfiles.TryGetValue(profileKey, out var profile))
            {
                profile = new DiyFfbPluginSettings.AircraftFfbProfile();
                _settings.AircraftFfbProfiles[profileKey] = profile;
            }

            return profile;
        }

        /// <summary>
        /// Get the function config overrides for a function in the current vehicle profile.
        /// Returns null if no overrides exist.
        /// </summary>
        public FunctionConfigOverrides GetFunctionOverrides(int functionId)
        {
            var profile = _getActiveProfile();
            if (profile?.FunctionOverrides == null)
                return null;

            profile.FunctionOverrides.TryGetValue(functionId, out var overrides);
            return overrides;
        }

        /// <summary>
        /// Get the user preference overrides for a function.
        /// Returns null if no user overrides exist.
        /// </summary>
        public FunctionConfigOverrides GetUserFunctionOverrides(int functionId)
        {
            var userPrefs = GetCurrentUserOverrides();
            if (userPrefs?.FunctionOverrides == null)
                return null;

            userPrefs.FunctionOverrides.TryGetValue(functionId, out var overrides);
            return overrides;
        }

        /// <summary>
        /// Create a ConfigLayerProvider for UI layer badge display.
        /// </summary>
        public ConfigLayerProvider CreateConfigLayerProvider()
        {
            return new ConfigLayerProvider(
                GetFunctionOverrides,
                GetUserFunctionOverrides,
                GetFunctionBaseline
            );
        }

        #endregion

        #region Override Field Operations

        /// <summary>
        /// Get or create the function config overrides for a function in the current vehicle profile.
        /// </summary>
        public FunctionConfigOverrides GetOrCreateFunctionOverrides(int functionId)
        {
            var profile = GetOrCreateCurrentProfile();
            if (profile == null)
                return null;

            if (profile.FunctionOverrides == null)
                profile.FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>();

            if (!profile.FunctionOverrides.TryGetValue(functionId, out var overrides))
            {
                overrides = new FunctionConfigOverrides();
                profile.FunctionOverrides[functionId] = overrides;
            }

            return overrides;
        }

        /// <summary>
        /// Update a function override value routed to the default layer for the field.
        /// Persists the override immediately but throttles the merge+ESP32 upload to ~1/sec per function.
        /// </summary>
        public void UpdateFunctionOverrideField(int functionId, string fieldName, Action<FunctionConfigOverrides> updateAction)
        {
            // Store override data immediately (no merge/send)
            var targetLayer = GetFunctionOverrideTargetLayer(fieldName);
            if (targetLayer == ConfigLayer.User)
            {
                StoreUserFunctionOverride(functionId, updateAction);
            }
            else
            {
                StoreProfileFunctionOverride(functionId, updateAction);
            }

            // Throttled merge+send to ESP32
            ScheduleThrottledMerge(functionId);

            // Fire OverrideFieldChanged event for badge refresh (NO ESP32 send, NO manager update to avoid loops)
            OnOverrideFieldChanged(functionId, fieldName);
        }

        /// <summary>
        /// Update a graph-derived ConfigOut field for a function. Stores the value in
        /// the in-memory ConfigOut tier (never persisted) and schedules a throttled
        /// merge+send so the change reaches ESP32. Use for fields written by graph
        /// ConfigOut nodes — does NOT touch profile/user overrides.
        ///
        /// Convenience wrapper: equivalent to <see cref="StoreConfigOutField"/> followed
        /// by <see cref="ScheduleConfigOutMerge"/>. Prefer the split form when writing
        /// multiple fields in a batch (e.g. one eval pass producing several ConfigOut
        /// values for the same function) so that the throttled merge sees all writes
        /// at once instead of firing the leading edge after just the first write.
        /// </summary>
        public void UpdateConfigOutField(int functionId, Action<FunctionConfigOverrides> updateAction)
        {
            StoreConfigOutField(functionId, updateAction);
            ScheduleConfigOutMerge(functionId);
        }

        /// <summary>
        /// Write a graph-derived ConfigOut field to the in-memory tier without scheduling
        /// a merge. Pair with <see cref="ScheduleConfigOutMerge"/> after a batch of writes
        /// to ensure the leading-edge merge sees a fully populated tier.
        /// </summary>
        public void StoreConfigOutField(int functionId, Action<FunctionConfigOverrides> updateAction)
        {
            if (updateAction == null) return;

            if (!_configOutOverrides.TryGetValue(functionId, out var overrides))
            {
                overrides = new FunctionConfigOverrides();
                _configOutOverrides[functionId] = overrides;
            }
            updateAction(overrides);
        }

        /// <summary>
        /// Schedule the throttled merge+send for a function whose ConfigOut tier was
        /// just written by <see cref="StoreConfigOutField"/>.
        /// </summary>
        public void ScheduleConfigOutMerge(int functionId)
        {
            ScheduleThrottledMerge(functionId);
        }

        /// <summary>
        /// Get the ConfigOut overlay for a function, or null if none.
        /// </summary>
        public FunctionConfigOverrides GetConfigOutOverrides(int functionId)
        {
            _configOutOverrides.TryGetValue(functionId, out var overrides);
            return overrides;
        }

        /// <summary>
        /// Apply overrides through the manager when any tier (profile, user, or ConfigOut)
        /// has data. Always queries the in-memory ConfigOut tier — graph-derived values
        /// must reach the merged config even when profile/user overrides are empty.
        /// Returns true if a merge was applied, false if all tiers were empty.
        /// </summary>
        private bool ApplyOverridesIfAnyTier(
            int functionId,
            FunctionConfigOverrides profileDelta,
            FunctionConfigOverrides userDelta,
            bool diffCheck = true)
        {
            _configOutOverrides.TryGetValue(functionId, out var configOutDelta);
            bool hasProfile = profileDelta != null && !profileDelta.IsEmpty;
            bool hasUser = userDelta != null && !userDelta.IsEmpty;
            bool hasConfigOut = configOutDelta != null && !configOutDelta.IsEmpty;
            if (!hasProfile && !hasUser && !hasConfigOut)
                return false;
            _functionConfigManager.ApplyProfileOverrides(
                functionId, profileDelta, userDelta, diffCheck: diffCheck, configOutDelta: configOutDelta);
            return true;
        }

        /// <summary>
        /// Discard all ConfigOut overlays (call on graph reload). Triggers a re-merge
        /// for affected functions so the merged config drops the graph-derived values
        /// until the next graph evaluation re-establishes them.
        /// </summary>
        public void ClearConfigOutOverrides()
        {
            if (_configOutOverrides.Count == 0)
                return;

            var affected = _configOutOverrides.Keys.ToList();
            _configOutOverrides.Clear();
            foreach (var functionId in affected)
            {
                if (_functionConfigManager.HasBaseConfig(functionId))
                    ReapplyMergedOverrides(functionId, diffCheck: false);
            }
        }

        /// <summary>
        /// Persist a profile-layer override without triggering merge/send.
        /// </summary>
        private void StoreProfileFunctionOverride(int functionId, Action<FunctionConfigOverrides> updateAction)
        {
            var overrides = GetOrCreateFunctionOverrides(functionId);
            if (overrides == null)
                return;

            updateAction(overrides);
        }

        /// <summary>
        /// Persist a user-layer override without triggering merge/send.
        /// </summary>
        private void StoreUserFunctionOverride(int functionId, Action<FunctionConfigOverrides> updateAction)
        {
            var overrides = GetOrCreateUserFunctionOverrides(functionId);
            if (overrides == null)
                return;

            updateAction(overrides);
        }

        /// <summary>
        /// Schedule a throttled merge+send for a function. Leading edge fires immediately;
        /// subsequent calls within the throttle window are coalesced and fire on the trailing edge.
        /// </summary>
        private void ScheduleThrottledMerge(int functionId)
        {
            if (!_functionConfigManager.HasBaseConfig(functionId))
                return;

            var now = DateTime.UtcNow;

            if (!_lastMergeUtc.TryGetValue(functionId, out var lastMerge) ||
                (now - lastMerge).TotalMilliseconds >= OverrideThrottleMs)
            {
                // Leading edge: send immediately
                ReapplyMergedOverrides(functionId, diffCheck: false);
                _lastMergeUtc[functionId] = now;
                _pendingMerges.Remove(functionId);
                EnsureThrottleTimer(functionId);
            }
            else
            {
                // Within throttle window: mark pending, timer will handle trailing edge
                _pendingMerges.Add(functionId);
                EnsureThrottleTimer(functionId);
            }
        }

        private void EnsureThrottleTimer(int functionId)
        {
            if (_throttleTimers.ContainsKey(functionId))
                return;

            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(OverrideThrottleMs) };
            timer.Tick += (s, e) => OnThrottleTimerTick(functionId);
            _throttleTimers[functionId] = timer;
            timer.Start();
        }

        private void OnThrottleTimerTick(int functionId)
        {
            if (_pendingMerges.Contains(functionId))
            {
                // Trailing edge: send latest state
                _pendingMerges.Remove(functionId);
                _lastMergeUtc[functionId] = DateTime.UtcNow;
                ReapplyMergedOverrides(functionId, diffCheck: false);
            }
            else
            {
                // No more pending changes, stop timer
                CancelThrottledMerge(functionId);
            }
        }

        /// <summary>
        /// Cancel a pending throttled merge for a specific function.
        /// </summary>
        private void CancelThrottledMerge(int functionId)
        {
            if (_throttleTimers.TryGetValue(functionId, out var timer))
            {
                timer.Stop();
                _throttleTimers.Remove(functionId);
            }
            _pendingMerges.Remove(functionId);
            _lastMergeUtc.Remove(functionId);
        }

        /// <summary>
        /// Cancel all pending throttled merges. Call on profile/vehicle change or disconnect.
        /// </summary>
        public void CancelAllThrottledMerges()
        {
            foreach (var timer in _throttleTimers.Values)
                timer.Stop();
            _throttleTimers.Clear();
            _pendingMerges.Clear();
            _lastMergeUtc.Clear();
        }

        /// <summary>
        /// Clear a specific override field for a function.
        /// </summary>
        public void ClearFunctionOverrideField(int functionId, string fieldName, ConfigLayer? layerOverride = null)
        {
            // Cancel any pending throttled merge — the clear methods call
            // ReapplyMergedOverrides immediately, and a stale pending merge
            // could overwrite the cleared state.
            CancelThrottledMerge(functionId);

            var targetLayer = layerOverride ?? GetFunctionOverrideTargetLayer(fieldName);
            if (targetLayer == ConfigLayer.User)
            {
                ClearUserFunctionOverrideField(functionId, fieldName);
            }
            else
            {
                ClearProfileFunctionOverrideField(functionId, fieldName);
            }

            // Fire OverrideFieldChanged event for badge refresh (NO ESP32 send)
            OnOverrideFieldChanged(functionId, fieldName);
        }

        private void ClearProfileFunctionOverrideField(int functionId, string fieldName)
        {
            var profile = _getActiveProfile();
            if (profile?.FunctionOverrides == null)
                return;

            if (!profile.FunctionOverrides.TryGetValue(functionId, out var overrides))
                return;

            ClearOverrideFieldValue(overrides, fieldName);

            // Remove the override entry if it's now empty
            if (overrides.IsEmpty)
            {
                profile.FunctionOverrides.Remove(functionId);
            }

            // Settings auto-save periodically by SimHub

            // Update manager if baseline exists
            if (_functionConfigManager.HasBaseConfig(functionId))
            {
                ReapplyMergedOverrides(functionId, diffCheck: false);
            }
        }

        private void ClearUserFunctionOverrideField(int functionId, string fieldName)
        {
            var prefs = GetCurrentUserOverrides();
            if (prefs?.FunctionOverrides == null)
                return;

            if (!prefs.FunctionOverrides.TryGetValue(functionId, out var overrides))
                return;

            ClearOverrideFieldValue(overrides, fieldName);

            if (overrides.IsEmpty)
            {
                prefs.FunctionOverrides.Remove(functionId);
            }

            // Settings auto-save periodically by SimHub

            // Update manager if baseline exists
            if (_functionConfigManager.HasBaseConfig(functionId))
            {
                ReapplyMergedOverrides(functionId, diffCheck: false);
            }
        }

        /// <summary>
        /// Clear all overrides (profile + user) for a function.
        /// Used when saving baseline to "bake" overrides into the new baseline.
        /// </summary>
        public void ClearAllFunctionOverrides(int functionId)
        {
            // Clear profile overrides
            var profile = _getActiveProfile();
            if (profile?.FunctionOverrides != null)
            {
                profile.FunctionOverrides.Remove(functionId);
            }

            // Clear user overrides
            var userPrefs = GetCurrentUserOverrides();
            if (userPrefs?.FunctionOverrides != null)
            {
                userPrefs.FunctionOverrides.Remove(functionId);
            }

            // Re-apply to manager (clears merged overrides)
            if (IsFunctionActive(functionId))
            {
                ApplyProfileOverridesToFunction(functionId);
            }
        }

        private static void ClearOverrideFieldValue(FunctionConfigOverrides overrides, string fieldName)
        {
            OverrideFieldRegistry.ClearValue(overrides, fieldName);
        }

        #endregion

        #region Override Re-routing and Review

        /// <summary>
        /// Move an override value from one layer to another.
        /// Reads the value from sourceLayer, writes to targetLayer, clears from sourceLayer.
        /// Re-applies merged overrides afterward.
        /// </summary>
        public void RerouteFunctionOverrideField(
            int functionId, string fieldPath,
            ConfigLayer sourceLayer, ConfigLayer targetLayer)
        {
            if (sourceLayer == targetLayer)
                return;

            // 1. Read value from source
            var sourceOverrides = GetOverridesForLayer(functionId, sourceLayer);
            if (sourceOverrides == null)
                return;
            var value = OverrideFieldRegistry.GetValue(sourceOverrides, fieldPath);
            if (value == null)
                return;

            // 2. Write to target
            var targetOverrides = GetOrCreateOverridesForLayer(functionId, targetLayer);
            if (targetOverrides == null)
                return;
            OverrideFieldRegistry.SetValue(targetOverrides, fieldPath, value);

            // 3. Clear from source
            OverrideFieldRegistry.ClearValue(sourceOverrides, fieldPath);
            CleanupEmptyOverrides(functionId, sourceLayer, sourceOverrides);

            // 4. Re-apply merged config
            if (_functionConfigManager.HasBaseConfig(functionId))
                ReapplyMergedOverrides(functionId, diffCheck: false);

            // 5. Notify UI
            OnOverrideFieldChanged(functionId, fieldPath);
        }

        /// <summary>
        /// Bake a field's current effective value into the baseline, clearing the override.
        /// </summary>
        public void BakeFieldToBaseline(int functionId, string fieldPath)
        {
            // 1. Get current effective value from merged FunctionConfig
            var currentConfig = _functionConfigManager.GetCurrentConfig(functionId);
            if (currentConfig == null)
                return;

            // 2. Get baseline, write the field value into it
            var baseline = GetFunctionBaseline(functionId);
            if (baseline == null)
            {
                SimHub.Logging.Current.Warn($"[TieredConfig] BakeFieldToBaseline: no baseline for function {functionId}");
                return;
            }

            var updatedBaseline = baseline.Clone();
            ConfigLayerProvider.WriteFieldToFunctionConfig(updatedBaseline, fieldPath, currentConfig);
            SetFunctionBaseline(functionId, updatedBaseline);

            // 3. Clear override from whichever layer had it
            ClearFunctionOverrideField(functionId, fieldPath, ConfigLayer.User);
            ClearFunctionOverrideField(functionId, fieldPath, ConfigLayer.Profile);
        }

        /// <summary>
        /// Get all active function overrides across all functions and layers.
        /// Used by the Override Review dialog.
        /// </summary>
        public List<OverrideReviewItem> GetAllActiveOverrides()
        {
            var result = new List<OverrideReviewItem>();
            var layerProvider = CreateConfigLayerProvider();

            foreach (var functionId in _functionConfigManager.GetKnownFunctionIds())
            {
                var funcEnum = (FunctionID)functionId;
                var funcName = funcEnum.ToString().CamelCaseToTitleCase();

                foreach (var field in OverrideFieldRegistry.GetAllFields())
                {
                    var sourceLayer = layerProvider.GetFieldSourceLayer(functionId, field.FieldPath);
                    if (sourceLayer == null || sourceLayer == ConfigLayer.Baseline)
                        continue; // No override

                    var value = layerProvider.GetFieldValue(functionId, field.FieldPath, sourceLayer.Value);
                    string valueDisplay;
                    if (field.FieldType == OverrideFieldType.Complex)
                        valueDisplay = "(configured)";
                    else if (field.FormatValue != null)
                        valueDisplay = field.FormatValue(value);
                    else if (value is float f)
                        valueDisplay = f.ToString("F2");
                    else if (value is bool b)
                        valueDisplay = b ? "Enabled" : "Disabled";
                    else
                        valueDisplay = value?.ToString() ?? "(null)";

                    result.Add(new OverrideReviewItem
                    {
                        FunctionId = functionId,
                        FunctionName = funcName,
                        FieldPath = field.FieldPath,
                        FieldDisplayName = field.DisplayName,
                        ValueDisplay = valueDisplay,
                        CurrentLayer = sourceLayer.Value,
                        CanMoveToUser = sourceLayer == ConfigLayer.Profile
                            && !layerProvider.HasFieldValue(functionId, field.FieldPath, ConfigLayer.User),
                        CanMoveToProfile = sourceLayer == ConfigLayer.User
                            && !layerProvider.HasFieldValue(functionId, field.FieldPath, ConfigLayer.Profile)
                    });
                }
            }

            return result;
        }

        private FunctionConfigOverrides GetOverridesForLayer(int functionId, ConfigLayer layer)
        {
            return layer == ConfigLayer.User
                ? GetUserFunctionOverrides(functionId)
                : GetFunctionOverrides(functionId);
        }

        private FunctionConfigOverrides GetOrCreateOverridesForLayer(int functionId, ConfigLayer layer)
        {
            return layer == ConfigLayer.User
                ? GetOrCreateUserFunctionOverrides(functionId)
                : GetOrCreateFunctionOverrides(functionId);
        }

        private void CleanupEmptyOverrides(int functionId, ConfigLayer layer, FunctionConfigOverrides overrides)
        {
            if (!overrides.IsEmpty)
                return;

            if (layer == ConfigLayer.User)
            {
                var prefs = GetCurrentUserOverrides();
                prefs?.FunctionOverrides?.Remove(functionId);
            }
            else
            {
                var profile = _getActiveProfile();
                profile?.FunctionOverrides?.Remove(functionId);
            }
        }

        #endregion

        #region User Preference Management

        private FunctionConfigOverrides GetOrCreateUserFunctionOverrides(int functionId)
        {
            var prefs = GetOrCreateCurrentUserOverrides();
            if (prefs == null)
                return null;

            if (prefs.FunctionOverrides == null)
                prefs.FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>();

            if (!prefs.FunctionOverrides.TryGetValue(functionId, out var overrides))
            {
                overrides = new FunctionConfigOverrides();
                prefs.FunctionOverrides[functionId] = overrides;
            }

            return overrides;
        }

        private UserPreferences GetOrCreateCurrentUserOverrides()
        {
            if (_settings == null)
                return null;

            if (_settings.UserPreferencesProfiles == null)
                _settings.UserPreferencesProfiles = new Dictionary<string, UserPreferences>();

            string userProfile = _settings.CurrentUserProfile ?? System.Environment.UserName;
            if (string.IsNullOrWhiteSpace(userProfile))
                userProfile = System.Environment.UserName;

            if (!_settings.UserPreferencesProfiles.TryGetValue(userProfile, out var prefs))
            {
                prefs = new UserPreferences();
                _settings.UserPreferencesProfiles[userProfile] = prefs;
            }

            return prefs;
        }

        /// <summary>
        /// Set the current user profile name and ensure its preferences entry exists.
        /// </summary>
        public void SetCurrentUserProfile(string userProfile)
        {
            if (_settings == null)
                return;

            var normalized = string.IsNullOrWhiteSpace(userProfile)
                ? System.Environment.UserName
                : userProfile.Trim();

            if (string.IsNullOrWhiteSpace(normalized))
                normalized = System.Environment.UserName;

            _settings.CurrentUserProfile = normalized;

            if (_settings.UserPreferencesProfiles == null)
                _settings.UserPreferencesProfiles = new Dictionary<string, UserPreferences>();

            if (!_settings.UserPreferencesProfiles.ContainsKey(normalized))
                _settings.UserPreferencesProfiles[normalized] = new UserPreferences();

            ApplyCurrentProfileOverrides();

            // Notify UI to refresh badges and labels
            OnContextChanged();
        }

        #endregion

        #region Utility

        private static string NormalizeFunctionOverrideFieldPath(string fieldName)
        {
            return OverrideFieldRegistry.NormalizeFieldPath(fieldName);
        }

        private static ConfigLayer GetFunctionOverrideTargetLayer(string fieldName)
        {
            // Use the registry's per-field DefaultLayer first (FlightControlDamping etc.
            // are explicitly registered as User-layer); fall back to FieldRouter's
            // prefix heuristic only for unregistered fields. Plan 10 broke routing for
            // flight_control.* paths because FieldRouter only knew flight_stick. /
            // flight_pedals. prefixes — overrides silently routed to Profile and were
            // dropped when no active profile existed.
            var normalized = NormalizeFunctionOverrideFieldPath(fieldName);
            return OverrideFieldRegistry.GetTargetLayer(normalized);
        }

        #endregion

        #region Axis Parameter Override API

        /// <summary>
        /// Data structure for functions that link to an axis.
        /// </summary>
        public class FunctionAxisLink
        {
            public int FunctionId { get; set; }
            public string FunctionName { get; set; }
            public bool HasOverride { get; set; }
        }

        /// <summary>
        /// Get all functions that link to a specific axis.
        /// </summary>
        public List<FunctionAxisLink> GetFunctionsLinkingToAxis(int axisId)
        {
            var result = new List<FunctionAxisLink>();
            var targetAxisId = (AxisID)axisId & AxisID.Mask;

            foreach (var funcId in _functionConfigManager.GetKnownFunctionIds())
            {
                var config = _functionConfigManager.GetCurrentConfig(funcId);
                if (config?.Base?.LinkedAxes == null)
                    continue;

                foreach (var linkedAxis in config.Base.LinkedAxes)
                {
                    var maskedAxis = linkedAxis & AxisID.Mask;
                    if (maskedAxis == AxisID.AxisUndefined)
                        break;

                    if (maskedAxis == targetAxisId)
                    {
                        var funcEnum = (FunctionID)funcId;
                        result.Add(new FunctionAxisLink
                        {
                            FunctionId = funcId,
                            FunctionName = funcEnum.ToString().CamelCaseToTitleCase(),
                            HasOverride = HasAxisParameterOverride(funcId, axisId)
                        });
                        break;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Check if an axis parameter override exists for a function.
        /// </summary>
        public bool HasAxisParameterOverride(int functionId, int axisId)
        {
            if (_settings?.FunctionAxisOverrides == null)
                return false;

            if (!_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
                return false;

            if (!axisOverrides.TryGetValue(axisId, out var overrides))
                return false;

            return overrides != null && !overrides.IsEmpty;
        }

        /// <summary>
        /// Get axis parameter overrides for a function.
        /// Returns null if no overrides exist.
        /// </summary>
        public AxisParameterOverrides GetAxisParameterOverride(int functionId, int axisId)
        {
            if (_settings?.FunctionAxisOverrides == null)
                return null;

            if (!_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
                return null;

            axisOverrides.TryGetValue(axisId, out var overrides);
            return overrides;
        }

        /// <summary>
        /// Get or create axis parameter overrides for a function.
        /// </summary>
        public AxisParameterOverrides GetOrCreateAxisParameterOverride(int functionId, int axisId)
        {
            if (_settings == null)
                return null;

            if (_settings.FunctionAxisOverrides == null)
                _settings.FunctionAxisOverrides = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>();

            if (!_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
            {
                axisOverrides = new Dictionary<int, AxisParameterOverrides>();
                _settings.FunctionAxisOverrides[functionId] = axisOverrides;
            }

            if (!axisOverrides.TryGetValue(axisId, out var overrides))
            {
                overrides = new AxisParameterOverrides();
                axisOverrides[axisId] = overrides;
            }

            return overrides;
        }

        /// <summary>
        /// Set axis parameter overrides for a function.
        /// If the function is active, the overrides are applied immediately.
        /// </summary>
        public void SetAxisParameterOverride(int functionId, int axisId, AxisParameterOverrides overrides)
        {
            if (_settings == null || overrides == null)
                return;

            if (_settings.FunctionAxisOverrides == null)
                _settings.FunctionAxisOverrides = new Dictionary<int, Dictionary<int, AxisParameterOverrides>>();

            if (!_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
            {
                axisOverrides = new Dictionary<int, AxisParameterOverrides>();
                _settings.FunctionAxisOverrides[functionId] = axisOverrides;
            }

            axisOverrides[axisId] = overrides;

            // If function is active, re-apply overrides
            if (IsFunctionActive(functionId))
            {
                _axisConfigManager.ApplyFunctionOverrides(functionId, axisOverrides);
            }
        }

        /// <summary>
        /// Update axis parameter override and re-apply if function is active.
        /// </summary>
        public void UpdateAxisParameterOverride(int functionId, int axisId, Action<AxisParameterOverrides> updateAction)
        {
            var overrides = GetOrCreateAxisParameterOverride(functionId, axisId);
            if (overrides == null)
                return;

            updateAction(overrides);

            // If function is active, re-apply overrides
            if (IsFunctionActive(functionId) && _settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
            {
                _axisConfigManager.ApplyFunctionOverrides(functionId, axisOverrides);
            }
        }

        /// <summary>
        /// Clear axis parameter overrides for a function on a specific axis.
        /// </summary>
        public void ClearAxisParameterOverride(int functionId, int axisId)
        {
            if (_settings?.FunctionAxisOverrides == null)
                return;

            if (!_settings.FunctionAxisOverrides.TryGetValue(functionId, out var axisOverrides))
                return;

            axisOverrides.Remove(axisId);

            // Clean up empty dictionaries
            if (axisOverrides.Count == 0)
            {
                _settings.FunctionAxisOverrides.Remove(functionId);
            }

            // If function is active, clear the override from the axis config manager
            if (IsFunctionActive(functionId))
            {
                if (axisOverrides.Count > 0)
                {
                    _axisConfigManager.ApplyFunctionOverrides(functionId, axisOverrides);
                }
                else
                {
                    _axisConfigManager.ClearFunctionOverrides(functionId);
                }
            }
        }

        /// <summary>
        /// Clear all axis parameter overrides for a function.
        /// </summary>
        public void ClearAllAxisParameterOverrides(int functionId)
        {
            if (_settings?.FunctionAxisOverrides == null)
                return;

            _settings.FunctionAxisOverrides.Remove(functionId);

            // If function is active, clear the overrides
            if (IsFunctionActive(functionId))
            {
                _axisConfigManager.ClearFunctionOverrides(functionId);
            }
        }

        #endregion

        #region ESP32 Authority

        /// <summary>
        /// Handle an incoming function config from ESP32 or file import.
        /// When fromEsp32 is true and a stored baseline exists, the plugin is authoritative:
        /// the incoming config is ignored and the current merged config is returned for push-back.
        /// Otherwise, the incoming config becomes the new base and overrides are applied.
        /// </summary>
        /// <returns>
        /// authorityOverride=true when the plugin overrode ESP32; resultConfig is the merged config to push back.
        /// authorityOverride=false when the incoming config was accepted; resultConfig is the new merged config.
        /// </returns>
        public (bool authorityOverride, FunctionConfig resultConfig)
            HandleIncomingFunctionConfig(int functionId, FunctionConfig incoming, bool fromEsp32)
        {
            if (fromEsp32 && HasFunctionBaseline(functionId))
            {
                var mergedConfig = _functionConfigManager.GetCurrentConfig(functionId);
                if (mergedConfig != null)
                {
                    // Only push back if merged config actually differs from what ESP32 reported
                    bool needsPushBack = !ConfigComparer.AreEqual(mergedConfig, incoming);
                    return (needsPushBack, mergedConfig);
                }
                // Fall through if manager state was unexpectedly lost
            }

            // Accept incoming config as the new base
            _functionConfigManager.SetBaseConfig(functionId, incoming);

            // Apply overrides through manager for consistent state tracking
            if (ShouldApplyProfileOverride(functionId))
            {
                ApplyProfileOverridesToFunction(functionId);
            }
            else
            {
                // No profile override, but apply user overrides AND ConfigOut tier through
                // the manager so graph-derived config (e.g. vibration ratios) reaches the
                // merged config even when profile/user are empty.
                var userOverrides = GetUserFunctionOverrides(functionId);
                ApplyOverridesIfAnyTier(functionId, profileDelta: null, userDelta: userOverrides);
            }

            var currentConfig = _functionConfigManager.GetCurrentConfig(functionId);
            return (false, currentConfig ?? incoming);
        }

        /// <summary>
        /// Handle an incoming axis config from ESP32 or file import.
        /// When fromEsp32 is true and a stored baseline exists, the plugin is authoritative:
        /// the incoming config is ignored and the current merged config is returned for push-back.
        /// Otherwise, the incoming config becomes the new base and function overrides are re-applied.
        /// </summary>
        public (bool authorityOverride, AxisConfig resultConfig)
            HandleIncomingAxisConfig(int axisId, AxisConfig incoming, bool fromEsp32)
        {
            if (fromEsp32 && HasAxisBaseline(axisId))
            {
                // Re-init from settings if manager state was lost (axis manager
                // is Reset() on vehicle change, which clears base configs)
                if (!_axisConfigManager.HasBaseConfig(axisId))
                {
                    var baseline = GetAxisBaseline(axisId);
                    if (baseline != null)
                    {
                        baseline.AxisId = incoming.AxisId;
                        _axisConfigManager.SetBaseConfig(axisId, baseline);
                    }
                }

                var mergedConfig = _axisConfigManager.GetCurrentConfig(axisId);
                if (mergedConfig != null)
                {
                    // Only push back if merged config actually differs from what ESP32 reported
                    bool needsPushBack = !ConfigComparer.AreEqual(mergedConfig, incoming);
                    return (needsPushBack, mergedConfig);
                }
                // Fall through if manager state was unexpectedly lost
            }

            // Accept incoming config as the new base
            _axisConfigManager.SetBaseConfig(axisId, incoming);

            // Re-apply active function override if one exists
            int? overridingFunc = _axisConfigManager.GetOverridingFunction(axisId);
            if (overridingFunc.HasValue)
            {
                var overrides = GetAxisParameterOverride(overridingFunc.Value, axisId);
                if (overrides != null && !overrides.IsEmpty)
                {
                    _axisConfigManager.ApplyFunctionOverride(
                        axisId, overridingFunc.Value, overrides, diffCheck: false);
                }
            }

            var currentConfig = _axisConfigManager.GetCurrentConfig(axisId);
            return (false, currentConfig ?? incoming);
        }

        #endregion
    }
}
