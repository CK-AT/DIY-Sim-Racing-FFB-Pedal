using System;
using System.Collections.Generic;
using System.Linq;
using DiyFfb.TieredConfig;
using ProtbufTest;
using SimHubPlugin.TestCommon;

namespace DiyFfb.TieredConfigTests
{
    /// <summary>
    /// Tests for TieredConfigOrchestrator reroute, bake, and review operations (Plan 32 Phase 5).
    /// </summary>
    public static class OrchestratorRerouteTests
    {
        // Shared state for test orchestrator construction
        private static DiyFfbPluginSettings.AircraftFfbProfile _activeProfile;

        private static TieredConfigOrchestrator CreateOrchestrator(DiyFfbPluginSettings settings = null)
        {
            settings = settings ?? new DiyFfbPluginSettings();
            _activeProfile = new DiyFfbPluginSettings.AircraftFfbProfile
            {
                FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>(),
                ActiveFunctionIds = new HashSet<int> { 1 }
            };
            settings.AircraftFfbProfiles["test|car"] = _activeProfile;
            settings.CurrentUserProfile = "TestUser";
            settings.UserPreferencesProfiles["TestUser"] = new UserPreferences
            {
                FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>()
            };

            var fcm = new FunctionConfigManager();
            var acm = new AxisConfigManager();

            var orchestrator = new TieredConfigOrchestrator(
                fcm, acm, settings,
                () => { }, // persistSettings no-op
                () => _activeProfile,
                () => GraphCategory.Vehicle,
                (game, car) => $"{game}|{car}",
                () => "test",
                () => "car");

            return orchestrator;
        }

        private static FunctionConfig CreateBaseline()
        {
            var config = new FunctionConfig
            {
                SimulatedMass = 10.0f,
                Friction = 1.0f,
                Base = new FunctionBase
                {
                    FunctionId = FunctionID.BrakePedal,
                    OutputMin = 0.0f,
                    OutputMax = 1.0f
                }
            };
            return config;
        }

        public static List<TestResult> RunAll()
        {
            return new List<TestResult>
            {
                // RerouteFunctionOverrideField tests
                TestRunner.RunTest("Reroute_UserToProfile_MovesValue", Reroute_UserToProfile_MovesValue),
                TestRunner.RunTest("Reroute_ProfileToUser_MovesValue", Reroute_ProfileToUser_MovesValue),
                TestRunner.RunTest("Reroute_SameLayer_NoOp", Reroute_SameLayer_NoOp),
                TestRunner.RunTest("Reroute_NoSourceValue_NoOp", Reroute_NoSourceValue_NoOp),

                // BakeFieldToBaseline tests
                TestRunner.RunTest("Bake_WritesToBaselineAndClearsOverride", Bake_WritesToBaselineAndClearsOverride),
                TestRunner.RunTest("Bake_NoBaseline_NoOp", Bake_NoBaseline_NoOp),
                TestRunner.RunTest("Bake_ClearsBothLayers", Bake_ClearsBothLayers),

                // GetAllActiveOverrides tests
                TestRunner.RunTest("GetAllOverrides_ReturnsUserAndProfileItems", GetAllOverrides_ReturnsUserAndProfileItems),
                TestRunner.RunTest("GetAllOverrides_EmptyWhenNoOverrides", GetAllOverrides_EmptyWhenNoOverrides),
                TestRunner.RunTest("GetAllOverrides_SetsCanMoveFlags", GetAllOverrides_SetsCanMoveFlags),

                // ConfigOut tier tests
                TestRunner.RunTest("ConfigOut_DoesNotTouchPersistentOverrides", ConfigOut_DoesNotTouchPersistentOverrides),
                TestRunner.RunTest("ConfigOut_AffectsMergedConfig", ConfigOut_AffectsMergedConfig),
                TestRunner.RunTest("ConfigOut_Clear_RemovesValueFromMergedConfig", ConfigOut_Clear_RemovesValueFromMergedConfig),
                TestRunner.RunTest("ConfigOut_ReapplyMergedOverrides_IncludesConfigOut", ConfigOut_ReapplyMergedOverrides_IncludesConfigOut),
                TestRunner.RunTest("ConfigOut_AppliedOnIncomingBaseline_WithEmptyOverrides", ConfigOut_AppliedOnIncomingBaseline_WithEmptyOverrides),
                TestRunner.RunTest("ConfigOut_StoreThenSchedule_BatchAvoidsPartialMerge", ConfigOut_StoreThenSchedule_BatchAvoidsPartialMerge),
            };
        }

        #region RerouteFunctionOverrideField

        private static bool Reroute_UserToProfile_MovesValue()
        {
            var orchestrator = CreateOrchestrator();

            // Set up baseline
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);

            // Initialize manager from settings so it has a base config
            orchestrator.InitializeManagerFromSettings();

            // Create user override
            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.SimulatedMass = 15.0f;

            // Re-apply so manager knows about the override
            orchestrator.ReapplyMergedOverrides(1, diffCheck: false);

            // Reroute from User to Profile
            orchestrator.RerouteFunctionOverrideField(1, "simulated_mass", ConfigLayer.User, ConfigLayer.Profile);

            // Verify: user should be cleared, profile should have value
            var userAfter = orchestrator.GetUserFunctionOverrides(1);
            var profileAfter = orchestrator.GetFunctionOverrides(1);

            bool userCleared = userAfter == null || userAfter.SimulatedMass == null;
            bool profileHasValue = profileAfter != null && profileAfter.SimulatedMass == 15.0f;

            if (!userCleared) throw new Exception("User override not cleared after reroute");
            if (!profileHasValue) throw new Exception($"Profile override not set after reroute. Profile={profileAfter?.SimulatedMass}");

            return true;
        }

        private static bool Reroute_ProfileToUser_MovesValue()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // Create profile override
            var profileOverrides = orchestrator.GetOrCreateFunctionOverrides(1);
            profileOverrides.Friction = 5.0f;

            orchestrator.ReapplyMergedOverrides(1, diffCheck: false);

            // Reroute from Profile to User
            orchestrator.RerouteFunctionOverrideField(1, "friction", ConfigLayer.Profile, ConfigLayer.User);

            var profileAfter = orchestrator.GetFunctionOverrides(1);
            var userAfter = orchestrator.GetUserFunctionOverrides(1);

            bool profileCleared = profileAfter == null || profileAfter.Friction == null;
            bool userHasValue = userAfter != null && userAfter.Friction == 5.0f;

            if (!profileCleared) throw new Exception("Profile override not cleared after reroute");
            if (!userHasValue) throw new Exception($"User override not set after reroute. User={userAfter?.Friction}");

            return true;
        }

        private static bool Reroute_SameLayer_NoOp()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.SimulatedMass = 20.0f;

            // Reroute User → User should be a no-op
            orchestrator.RerouteFunctionOverrideField(1, "simulated_mass", ConfigLayer.User, ConfigLayer.User);

            var userAfter = orchestrator.GetUserFunctionOverrides(1);
            if (userAfter?.SimulatedMass != 20.0f)
                throw new Exception("Same-layer reroute should not modify value");

            return true;
        }

        private static bool Reroute_NoSourceValue_NoOp()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // No overrides exist — reroute should be a no-op
            orchestrator.RerouteFunctionOverrideField(1, "simulated_mass", ConfigLayer.User, ConfigLayer.Profile);

            var profileAfter = orchestrator.GetFunctionOverrides(1);
            bool noProfileValue = profileAfter == null || profileAfter.SimulatedMass == null;

            if (!noProfileValue)
                throw new Exception("Reroute from empty source should not create target value");

            return true;
        }

        #endregion

        #region BakeFieldToBaseline

        private static bool Bake_WritesToBaselineAndClearsOverride()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // Create user override for simulated_mass
            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.SimulatedMass = 25.0f;
            orchestrator.ReapplyMergedOverrides(1, diffCheck: false);

            // Bake to baseline
            orchestrator.BakeFieldToBaseline(1, "simulated_mass");

            // Verify: baseline should now have 25.0, user override should be cleared
            var updatedBaseline = orchestrator.GetFunctionBaseline(1);
            var userAfter = orchestrator.GetUserFunctionOverrides(1);

            if (updatedBaseline.SimulatedMass != 25.0f)
                throw new Exception($"Baseline should be 25.0 after bake, got {updatedBaseline.SimulatedMass}");
            if (userAfter != null && userAfter.SimulatedMass != null)
                throw new Exception("User override should be cleared after bake");

            return true;
        }

        private static bool Bake_NoBaseline_NoOp()
        {
            var orchestrator = CreateOrchestrator();

            // No baseline set — bake should be a no-op (no crash)
            orchestrator.BakeFieldToBaseline(1, "simulated_mass");

            // Should not crash, and no baseline should exist
            if (orchestrator.HasFunctionBaseline(1))
                throw new Exception("No baseline should exist after bake with no initial baseline");

            return true;
        }

        private static bool Bake_ClearsBothLayers()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // Create overrides in both layers for the same field
            var profileOverrides = orchestrator.GetOrCreateFunctionOverrides(1);
            profileOverrides.Friction = 3.0f;

            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.Friction = 7.0f;

            orchestrator.ReapplyMergedOverrides(1, diffCheck: false);

            // Bake — should use effective value (User wins = 7.0) and clear both layers
            orchestrator.BakeFieldToBaseline(1, "friction");

            var updatedBaseline = orchestrator.GetFunctionBaseline(1);
            var profileAfter = orchestrator.GetFunctionOverrides(1);
            var userAfter = orchestrator.GetUserFunctionOverrides(1);

            if (updatedBaseline.Friction != 7.0f)
                throw new Exception($"Baseline should be 7.0 after bake, got {updatedBaseline.Friction}");
            if (profileAfter != null && profileAfter.Friction != null)
                throw new Exception("Profile override should be cleared after bake");
            if (userAfter != null && userAfter.Friction != null)
                throw new Exception("User override should be cleared after bake");

            return true;
        }

        #endregion

        #region GetAllActiveOverrides

        private static bool GetAllOverrides_ReturnsUserAndProfileItems()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // Set user override
            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.SimulatedMass = 12.0f;

            // Set profile override
            var profileOverrides = orchestrator.GetOrCreateFunctionOverrides(1);
            profileOverrides.Friction = 3.5f;

            var items = orchestrator.GetAllActiveOverrides();

            var massItem = items.FirstOrDefault(i => i.FieldPath == "SimulatedMass");
            var frictionItem = items.FirstOrDefault(i => i.FieldPath == "Friction");

            if (massItem == null)
                throw new Exception("Should find simulated_mass override");
            if (massItem.CurrentLayer != ConfigLayer.User)
                throw new Exception($"simulated_mass should be User layer, got {massItem.CurrentLayer}");

            if (frictionItem == null)
                throw new Exception("Should find friction override");
            if (frictionItem.CurrentLayer != ConfigLayer.Profile)
                throw new Exception($"friction should be Profile layer, got {frictionItem.CurrentLayer}");

            return true;
        }

        private static bool GetAllOverrides_EmptyWhenNoOverrides()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            var items = orchestrator.GetAllActiveOverrides();

            if (items.Count != 0)
                throw new Exception($"Should return 0 items with no overrides, got {items.Count}");

            return true;
        }

        private static bool GetAllOverrides_SetsCanMoveFlags()
        {
            var orchestrator = CreateOrchestrator();
            var baseline = CreateBaseline();
            orchestrator.SetFunctionBaseline(1, baseline);
            orchestrator.InitializeManagerFromSettings();

            // User override only
            var userOverrides = orchestrator.GetOrCreateUserFunctionOverrides(1);
            userOverrides.SimulatedMass = 12.0f;

            var items = orchestrator.GetAllActiveOverrides();
            var massItem = items.FirstOrDefault(i => i.FieldPath == "SimulatedMass");

            if (massItem == null)
                throw new Exception("Should find simulated_mass override");
            if (!massItem.CanMoveToProfile)
                throw new Exception("CanMoveToProfile should be true when only User has value");
            if (massItem.CanMoveToUser)
                throw new Exception("CanMoveToUser should be false when already in User");

            return true;
        }

        #endregion

        #region ConfigOut tier

        // ConfigOut writes go to an in-memory tier in the orchestrator. They must NOT
        // appear in the persisted Profile or User override stores, must show up in the
        // merged config the manager hands to the UI/ESP32, and must disappear from the
        // merged config when the tier is cleared (graph reload).

        private static bool ConfigOut_DoesNotTouchPersistentOverrides()
        {
            var orchestrator = CreateOrchestrator();
            orchestrator.SetFunctionBaseline(1, CreateBaseline());
            orchestrator.InitializeManagerFromSettings();

            orchestrator.UpdateConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));

            var profileAfter = orchestrator.GetFunctionOverrides(1);
            var userAfter = orchestrator.GetUserFunctionOverrides(1);

            bool profileClean = profileAfter == null || profileAfter.IsEmpty;
            bool userClean = userAfter == null || userAfter.IsEmpty;

            if (!profileClean) throw new Exception($"ConfigOut leaked into profile overrides (SimulatedMass={profileAfter?.SimulatedMass})");
            if (!userClean) throw new Exception($"ConfigOut leaked into user overrides (SimulatedMass={userAfter?.SimulatedMass})");

            // Round-trip through the orchestrator's own accessor
            var configOut = orchestrator.GetConfigOutOverrides(1);
            if (configOut?.SimulatedMass != 7.0f)
                throw new Exception($"ConfigOut tier should hold SimulatedMass=7.0, got {configOut?.SimulatedMass}");

            return true;
        }

        private static bool ConfigOut_AffectsMergedConfig()
        {
            var orchestrator = CreateOrchestrator();
            orchestrator.SetFunctionBaseline(1, CreateBaseline());  // SimulatedMass=10 in baseline
            orchestrator.InitializeManagerFromSettings();

            orchestrator.UpdateConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));

            var merged = orchestrator.GetInitialFunctionConfig(1);
            if (merged == null) throw new Exception("Merged config should be available");
            if (Math.Abs(merged.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception($"Merged config should reflect ConfigOut value 7.0, got {merged.SimulatedMass}");

            return true;
        }

        private static bool ConfigOut_Clear_RemovesValueFromMergedConfig()
        {
            var orchestrator = CreateOrchestrator();
            orchestrator.SetFunctionBaseline(1, CreateBaseline());  // baseline SimulatedMass=10
            orchestrator.InitializeManagerFromSettings();

            orchestrator.UpdateConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));

            // Sanity: ConfigOut applied
            var afterApply = orchestrator.GetInitialFunctionConfig(1);
            if (Math.Abs(afterApply.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception("Pre-clear: ConfigOut should drive merged value to 7.0");

            // Clear (graph reload analogue)
            orchestrator.ClearConfigOutOverrides();

            // The merged config must revert to baseline now that ConfigOut is gone.
            var afterClear = orchestrator.GetInitialFunctionConfig(1);
            if (Math.Abs(afterClear.SimulatedMass - 10.0f) > 1e-6f)
                throw new Exception($"After clear: merged config should revert to baseline 10.0, got {afterClear.SimulatedMass}");

            // Tier itself should be empty
            var configOut = orchestrator.GetConfigOutOverrides(1);
            if (configOut != null && !configOut.IsEmpty)
                throw new Exception("ConfigOut tier should be empty after Clear");

            return true;
        }

        private static bool ConfigOut_ReapplyMergedOverrides_IncludesConfigOut()
        {
            var orchestrator = CreateOrchestrator();
            orchestrator.SetFunctionBaseline(1, CreateBaseline());
            orchestrator.InitializeManagerFromSettings();

            orchestrator.UpdateConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));

            // An explicit re-apply (e.g. from a profile-tier change elsewhere) must
            // still include the in-memory ConfigOut layer in the merge.
            orchestrator.ReapplyMergedOverrides(1, diffCheck: false);

            var merged = orchestrator.GetInitialFunctionConfig(1);
            if (Math.Abs(merged.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception($"ReapplyMergedOverrides dropped ConfigOut: SimulatedMass={merged.SimulatedMass}");

            return true;
        }

        // Regression: when ESP32 sends a fresh function config and the plugin has
        // ConfigOut tier values but empty profile/user overrides, the merge path
        // used to silently drop the ConfigOut tier (gated only on user being
        // non-empty). Symptom: graph-derived config (vibration ratios, phase, etc.)
        // never reached the firmware on baseline arrival.
        private static bool ConfigOut_AppliedOnIncomingBaseline_WithEmptyOverrides()
        {
            var orchestrator = CreateOrchestrator();

            // Populate the ConfigOut tier BEFORE any baseline exists — mirrors the
            // real flow where graph eval starts producing values before firmware
            // reports its function config.
            orchestrator.UpdateConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));

            // Now firmware sends its baseline. No profile, no user overrides.
            var incoming = CreateBaseline();   // SimulatedMass = 10 in baseline
            var (authorityOverride, resultConfig) = orchestrator.HandleIncomingFunctionConfig(1, incoming, fromEsp32: true);

            // The merged config returned to the caller (and stored in the manager)
            // must reflect the ConfigOut tier, not the bare baseline.
            if (resultConfig == null)
                throw new Exception("HandleIncomingFunctionConfig returned null config");
            if (Math.Abs(resultConfig.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception($"Merged config dropped ConfigOut on baseline arrival: SimulatedMass={resultConfig.SimulatedMass} (expected 7.0)");

            // The manager's current config should also reflect the ConfigOut value.
            var stored = orchestrator.GetInitialFunctionConfig(1);
            if (Math.Abs(stored.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception($"Stored merged config wrong: SimulatedMass={stored.SimulatedMass} (expected 7.0)");

            return true;
        }

        // Regression: when a single eval pass writes multiple ConfigOut fields for the
        // same function, calling UpdateConfigOutField per-field made the throttled
        // leading-edge merge fire after only the first write — sending a partially-
        // populated tier (other slots = baseline zeros) to ESP32. The fix splits
        // store from schedule: callers do all stores first, then schedule once per
        // affected function so the leading edge sees the full tier.
        private static bool ConfigOut_StoreThenSchedule_BatchAvoidsPartialMerge()
        {
            var orchestrator = CreateOrchestrator();
            orchestrator.SetFunctionBaseline(1, CreateBaseline());
            orchestrator.InitializeManagerFromSettings();

            // Batch: write two fields, schedule once.
            orchestrator.StoreConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "simulated_mass", 7.0f));
            orchestrator.StoreConfigOutField(1,
                ovr => OverrideFieldRegistry.SetValue(ovr, "friction", 3.0f));
            orchestrator.ScheduleConfigOutMerge(1);

            var merged = orchestrator.GetInitialFunctionConfig(1);
            if (merged == null)
                throw new Exception("Merged config missing after batch schedule");
            if (Math.Abs(merged.SimulatedMass - 7.0f) > 1e-6f)
                throw new Exception($"SimulatedMass dropped from batched store: {merged.SimulatedMass}");
            if (Math.Abs(merged.Friction - 3.0f) > 1e-6f)
                throw new Exception($"Friction dropped from batched store: {merged.Friction}");

            return true;
        }

        #endregion

        /// <summary>
        /// Helper to access the private GetOrCreateUserFunctionOverrides via the public API path.
        /// </summary>
        private static FunctionConfigOverrides GetOrCreateUserFunctionOverrides(
            this TieredConfigOrchestrator orchestrator, int functionId)
        {
            // Use the public UpdateFunctionOverrideField path to ensure user overrides exist,
            // then retrieve them. But for test setup, we'll create directly via settings.
            // Actually, we need a simpler path. Let's use the GetCurrentUserOverrides.
            var prefs = orchestrator.GetCurrentUserOverrides();
            if (prefs == null) return null;
            if (prefs.FunctionOverrides == null)
                prefs.FunctionOverrides = new Dictionary<int, FunctionConfigOverrides>();
            if (!prefs.FunctionOverrides.TryGetValue(functionId, out var overrides))
            {
                overrides = new FunctionConfigOverrides();
                prefs.FunctionOverrides[functionId] = overrides;
            }
            return overrides;
        }
    }
}
