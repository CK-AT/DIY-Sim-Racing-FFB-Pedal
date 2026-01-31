using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using User.PluginSdkDemo.Controls;
using User.PluginSdkDemo.Helpers;

namespace User.PluginSdkDemo.GraphEditor
{
    public partial class ParamReviewWindow : Window
    {
        private readonly DiyFfbPlugin _plugin;
        private readonly ParamMigrationResult _migrationResult;
        private List<ActiveParamEntry> _activeParams;
        private List<OrphanEntry> _orphans;

        public ParamReviewWindow(DiyFfbPlugin plugin, ParamMigrationResult migrationResult)
        {
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);
            _plugin = plugin;
            _migrationResult = migrationResult;

            LoadData();
        }

        private void LoadData()
        {
            TextGraphPath.Text = $"Graph: {_plugin.GetActiveGraphPath()}";

            var profile = GetCurrentProfile();
            var currentParams = _plugin.GetAllGraphParams();
            var changedDefaultNames = new HashSet<string>(_migrationResult.ChangedDefaults.Select(c => c.ParamName));
            var clampedNames = _migrationResult.ClampedValues.ToDictionary(c => c.ParamName, c => c);

            // Build active params list
            _activeParams = new List<ActiveParamEntry>();
            foreach (var param in currentParams)
            {
                bool hasOverride = profile?.GraphParamValues?.ContainsKey(param.Name) == true;
                double currentValue = hasOverride
                    ? profile.GraphParamValues[param.Name]
                    : param.DefaultValue;

                var entry = new ActiveParamEntry
                {
                    ParamName = param.Name,
                    CurrentValue = currentValue,
                    DefaultValue = param.DefaultValue,
                    Min = param.Min,
                    Max = param.Max,
                    HasOverride = hasOverride,
                    DefaultChanged = changedDefaultNames.Contains(param.Name)
                };

                if (clampedNames.TryGetValue(param.Name, out var clampInfo))
                {
                    entry.ValueClamped = true;
                    entry.ClampNote = clampInfo.ClampedToMin
                        ? $"{clampInfo.OriginalValue:F2} -> {clampInfo.ClampedValue:F2} (min)"
                        : $"{clampInfo.OriginalValue:F2} -> {clampInfo.ClampedValue:F2} (max)";
                }

                _activeParams.Add(entry);
            }

            GridActiveParams.ItemsSource = _activeParams;

            // Build orphans list
            _orphans = new List<OrphanEntry>();
            if (_migrationResult.AllOrphans != null && profile?.GraphParamValues != null)
            {
                foreach (var orphanName in _migrationResult.AllOrphans)
                {
                    if (profile.GraphParamValues.TryGetValue(orphanName, out var value))
                    {
                        _orphans.Add(new OrphanEntry
                        {
                            ParamName = orphanName,
                            StoredValue = value,
                            IsNew = _migrationResult.NewOrphans.Contains(orphanName)
                        });
                    }
                }
            }

            GridOrphans.ItemsSource = _orphans;
            ExpanderOrphans.Visibility = _orphans.Count > 0 ? Visibility.Visible : Visibility.Collapsed;
            ExpanderOrphans.Header = $"Orphaned Overrides ({_orphans.Count})";

            UpdateStatus();
        }

        private void UpdateStatus()
        {
            var parts = new List<string>();
            if (_migrationResult.ChangedDefaults.Count > 0)
                parts.Add($"{_migrationResult.ChangedDefaults.Count} default(s) changed (yellow)");
            if (_migrationResult.ClampedValues.Count > 0)
                parts.Add($"{_migrationResult.ClampedValues.Count} value(s) clamped (orange)");
            if (_migrationResult.NewOrphans.Count > 0)
                parts.Add($"{_migrationResult.NewOrphans.Count} new orphan(s)");

            TextStatus.Text = parts.Count > 0
                ? string.Join("; ", parts)
                : "No changes to review.";
        }

        private DiyFfbPluginSettings.AircraftFfbProfile GetCurrentProfile()
        {
            return _plugin.GetType()
                .GetMethod("GetCurrentAircraftProfile", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                ?.Invoke(_plugin, null) as DiyFfbPluginSettings.AircraftFfbProfile;
        }

        private void OnResetParamClick(object sender, RoutedEventArgs e)
        {
            if (sender is System.Windows.Controls.Button btn && btn.Tag is string paramName)
            {
                var profile = GetCurrentProfile();
                if (profile?.GraphParamValues != null && profile.GraphParamValues.ContainsKey(paramName))
                {
                    _plugin.ResetGraphParamValue(paramName);
                    LoadData();
                }
            }
        }

        private void OnResetAllClick(object sender, RoutedEventArgs e)
        {
            var result = ThemedMessageBox.Show(
                "Reset all parameters to their default values?\n\nThis will remove all your customizations for this vehicle.",
                "Reset All Parameters",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);

            if (result == MessageBoxResult.Yes)
            {
                var profile = GetCurrentProfile();
                if (profile?.GraphParamValues != null)
                {
                    // Reset each param individually to trigger UI updates
                    var paramNames = profile.GraphParamValues.Keys.ToList();
                    foreach (var paramName in paramNames)
                    {
                        _plugin.ResetGraphParamValue(paramName);
                    }
                    LoadData();
                }
            }
        }

        private void OnDeleteOrphanClick(object sender, RoutedEventArgs e)
        {
            if (sender is System.Windows.Controls.Button btn && btn.Tag is string paramName)
            {
                var profile = GetCurrentProfile();
                if (profile?.GraphParamValues != null && profile.GraphParamValues.ContainsKey(paramName))
                {
                    profile.GraphParamValues.Remove(paramName);
                    _orphans.RemoveAll(o => o.ParamName == paramName);
                    GridOrphans.ItemsSource = null;
                    GridOrphans.ItemsSource = _orphans;
                    ExpanderOrphans.Header = $"Orphaned Overrides ({_orphans.Count})";
                    if (_orphans.Count == 0)
                        ExpanderOrphans.Visibility = Visibility.Collapsed;
                }
            }
        }

        private void OnDeleteAllOrphansClick(object sender, RoutedEventArgs e)
        {
            var result = ThemedMessageBox.Show(
                $"Delete all {_orphans.Count} orphaned override(s)?\n\nThese are parameter values for parameters that no longer exist in the current graph.",
                "Delete All Orphans",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);

            if (result == MessageBoxResult.Yes)
            {
                var profile = GetCurrentProfile();
                if (profile?.GraphParamValues != null)
                {
                    foreach (var orphan in _orphans.ToList())
                    {
                        profile.GraphParamValues.Remove(orphan.ParamName);
                    }
                    _orphans.Clear();
                    GridOrphans.ItemsSource = null;
                    GridOrphans.ItemsSource = _orphans;
                    ExpanderOrphans.Header = "Orphaned Overrides (0)";
                    ExpanderOrphans.Visibility = Visibility.Collapsed;
                }
            }
        }

        private void OnMarkReviewedClick(object sender, RoutedEventArgs e)
        {
            var profile = GetCurrentProfile();
            if (profile != null && _migrationResult != null)
            {
                profile.LastReviewedGraphHash = _migrationResult.NewHash;
                profile.LastReviewedParamSnapshots = _migrationResult.CurrentSnapshots;
            }

            DialogResult = true;
            Close();
        }

        private void OnCloseClick(object sender, RoutedEventArgs e)
        {
            Close();
        }
    }

    public class ActiveParamEntry : INotifyPropertyChanged
    {
        public string ParamName { get; set; }
        public double CurrentValue { get; set; }
        public double DefaultValue { get; set; }
        public double Min { get; set; }
        public double Max { get; set; }
        public bool HasOverride { get; set; }
        public bool DefaultChanged { get; set; }
        public bool ValueClamped { get; set; }
        public string ClampNote { get; set; }

        public string CurrentValueDisplay => CurrentValue.ToString("F2");
        public string DefaultDisplay => DefaultChanged
            ? $"{DefaultValue:F2} *"
            : DefaultValue.ToString("F2");
        public string RangeDisplay => $"[{Min:F2}, {Max:F2}]";

        public event PropertyChangedEventHandler PropertyChanged;
    }

    public class OrphanEntry : INotifyPropertyChanged
    {
        public string ParamName { get; set; }
        public double StoredValue { get; set; }
        public bool IsNew { get; set; }

        public string StoredValueDisplay => StoredValue.ToString("F2");

        public event PropertyChangedEventHandler PropertyChanged;
    }
}
