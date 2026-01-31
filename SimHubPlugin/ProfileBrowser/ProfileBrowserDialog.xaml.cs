using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Windows;
using System.Windows.Input;
using Newtonsoft.Json;
using User.PluginSdkDemo.Controls;
using User.PluginSdkDemo.GraphEditor;
using User.PluginSdkDemo.Helpers;

namespace User.PluginSdkDemo.ProfileBrowser
{
    public partial class ProfileBrowserDialog : Window
    {
        // Public result properties
        public ProfileBrowserEntry SelectedEntry { get; private set; }
        public bool UseGraphOnly { get; private set; }
        public bool UseTuning { get; private set; }

        // Data binding
        public ObservableCollection<ProfileBrowserEntry> Items { get; } = new ObservableCollection<ProfileBrowserEntry>();

        // Constructor params
        private readonly ProfileBrowserMode _mode;
        private readonly DiyFfbPlugin _plugin;
        private readonly string _currentGameId;
        private readonly string _currentCarId;
        private bool _isInitialized;

        public ProfileBrowserDialog(
            DiyFfbPlugin plugin,
            ProfileBrowserMode mode,
            string gameId = null,
            string carId = null)
        {
            _plugin = plugin;
            _mode = mode;
            _currentGameId = gameId;
            _currentCarId = carId;

            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);

            ListItems.ItemsSource = Items;
            ConfigureForMode();
            _isInitialized = true;
            LoadInitialTab();
        }

        private void ConfigureForMode()
        {
            switch (_mode)
            {
                case ProfileBrowserMode.NewVehicle:
                    TextTitle.Text = "Select Starting Point";
                    RadioTemplates.IsChecked = true;
                    ButtonExport.Visibility = Visibility.Collapsed;
                    break;
                case ProfileBrowserMode.ManageProfiles:
                    TextTitle.Text = "Profile Browser";
                    RadioMyVehicles.IsChecked = true;
                    break;
                case ProfileBrowserMode.CopyFromVehicle:
                    TextTitle.Text = "Copy From Vehicle";
                    RadioMyVehicles.IsChecked = true;
                    ButtonExport.Visibility = Visibility.Collapsed;
                    break;
            }

            UpdateButtonStates();
        }

        private void LoadInitialTab()
        {
            if (RadioTemplates.IsChecked == true)
                LoadTemplates();
            else if (RadioMyVehicles.IsChecked == true)
                LoadStoredProfiles();
        }

        private void OnTabChanged(object sender, RoutedEventArgs e)
        {
            if (!_isInitialized)
                return;

            if (RadioTemplates.IsChecked == true)
                LoadTemplates();
            else if (RadioMyVehicles.IsChecked == true)
                LoadStoredProfiles();
        }

        private void LoadTemplates()
        {
            Items.Clear();
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            var templates = GraphTemplateRegistry.GetTemplates(_currentGameId, baseDir);

            foreach (var t in templates)
            {
                Items.Add(ProfileBrowserEntry.FromTemplate(t));
            }

            if (Items.Count > 0)
                ListItems.SelectedIndex = 0;

            UpdateDetailsPanel();
            UpdateButtonStates();
        }

        private void LoadStoredProfiles()
        {
            Items.Clear();
            var profiles = _plugin?.Settings?.AircraftFfbProfiles;
            if (profiles == null)
            {
                UpdateDetailsPanel();
                UpdateButtonStates();
                return;
            }

            foreach (var kvp in profiles)
            {
                // GraphPath is now stored directly in the profile
                string graphPath = kvp.Value?.GraphPath ?? "";
                Items.Add(ProfileBrowserEntry.FromProfile(kvp.Key, kvp.Value, graphPath));
            }

            if (Items.Count > 0)
                ListItems.SelectedIndex = 0;

            UpdateDetailsPanel();
            UpdateButtonStates();
        }

        private void OnImportClick(object sender, RoutedEventArgs e)
        {
            var dialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
                DefaultExt = "json",
                Title = "Import Profile"
            };

            if (dialog.ShowDialog() != true)
                return;

            try
            {
                string json = File.ReadAllText(dialog.FileName);

                // Try new ExportedProfile format first
                var exported = JsonConvert.DeserializeObject<DiyFfbPluginSettings.ExportedProfile>(json);
                if (exported?.Profile != null)
                {
                    var entry = ProfileBrowserEntry.FromImportedFile(exported);
                    Items.Clear();
                    Items.Add(entry);
                    ListItems.SelectedItem = entry;
                    RadioTemplates.IsChecked = false;
                    RadioMyVehicles.IsChecked = false;
                    UpdateDetailsPanel();
                    UpdateButtonStates();
                    return;
                }

                // Try legacy AircraftFfbProfile format
                var legacyProfile = JsonConvert.DeserializeObject<DiyFfbPluginSettings.AircraftFfbProfile>(json);
                if (legacyProfile != null)
                {
                    var legacyExported = new DiyFfbPluginSettings.ExportedProfile
                    {
                        Version = 1,
                        ProfileKey = Path.GetFileNameWithoutExtension(dialog.FileName),
                        Profile = legacyProfile
                    };
                    var entry = ProfileBrowserEntry.FromImportedFile(legacyExported);
                    Items.Clear();
                    Items.Add(entry);
                    ListItems.SelectedItem = entry;
                    RadioTemplates.IsChecked = false;
                    RadioMyVehicles.IsChecked = false;
                    UpdateDetailsPanel();
                    UpdateButtonStates();
                    return;
                }

                ThemedMessageBox.Show(this, "Could not read profile from file.", "Import Failed",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
            }
            catch (Exception ex)
            {
                ThemedMessageBox.Show(this, $"Error reading file: {ex.Message}", "Import Failed",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void OnSelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            UpdateDetailsPanel();
            UpdateButtonStates();
        }

        private void UpdateDetailsPanel()
        {
            if (ListItems.SelectedItem is ProfileBrowserEntry entry)
            {
                TextSelectedName.Text = entry.Name;
                TextSelectedGraph.Text = string.IsNullOrEmpty(entry.GraphPath) ? "(none)" : entry.GraphPath;

                if (entry.Source == ProfileEntrySource.Template)
                {
                    TextSelectedTuning.Text = "Template (no tuning)";
                }
                else if (entry.HasTuning)
                {
                    TextSelectedTuning.Text = $"{entry.TunedParamCount} parameter(s) customized";
                }
                else
                {
                    TextSelectedTuning.Text = "No customizations";
                }
            }
            else
            {
                TextSelectedName.Text = "-";
                TextSelectedGraph.Text = "-";
                TextSelectedTuning.Text = "-";
            }
        }

        private void UpdateButtonStates()
        {
            var entry = ListItems.SelectedItem as ProfileBrowserEntry;
            bool hasSelection = entry != null;

            ButtonUseGraphOnly.IsEnabled = hasSelection;
            ButtonUseGraphTuning.IsEnabled = hasSelection && (entry.HasTuning || entry.Source != ProfileEntrySource.Template);
            ButtonExport.IsEnabled = hasSelection && entry.CanExport;

            // Hide "Use Graph + Tuning" for templates with no tuning
            if (entry != null && entry.Source == ProfileEntrySource.Template)
            {
                ButtonUseGraphTuning.Visibility = Visibility.Collapsed;
            }
            else
            {
                ButtonUseGraphTuning.Visibility = Visibility.Visible;
            }
        }

        private void OnItemDoubleClick(object sender, MouseButtonEventArgs e)
        {
            if (ListItems.SelectedItem is ProfileBrowserEntry entry)
            {
                SelectedEntry = entry;
                UseGraphOnly = entry.Source == ProfileEntrySource.Template;
                UseTuning = !UseGraphOnly;
                DialogResult = true;
            }
        }

        private void OnUseGraphOnlyClick(object sender, RoutedEventArgs e)
        {
            if (ListItems.SelectedItem is ProfileBrowserEntry entry)
            {
                SelectedEntry = entry;
                UseGraphOnly = true;
                UseTuning = false;
                DialogResult = true;
            }
        }

        private void OnUseGraphTuningClick(object sender, RoutedEventArgs e)
        {
            if (ListItems.SelectedItem is ProfileBrowserEntry entry)
            {
                SelectedEntry = entry;
                UseGraphOnly = false;
                UseTuning = true;
                DialogResult = true;
            }
        }

        private void OnDeleteClick(object sender, RoutedEventArgs e)
        {
            if (((System.Windows.Controls.Button)sender).DataContext is ProfileBrowserEntry entry &&
                entry.Source == ProfileEntrySource.StoredProfile)
            {
                var result = ThemedMessageBox.Show(this,
                    $"Delete profile for '{entry.Name}'?\n\nThis will remove all saved tuning for this vehicle.",
                    "Confirm Delete",
                    MessageBoxButton.YesNo,
                    MessageBoxImage.Warning);

                if (result == MessageBoxResult.Yes)
                {
                    _plugin?.Settings?.AircraftFfbProfiles?.Remove(entry.ProfileKey);
                    Items.Remove(entry);
                    UpdateDetailsPanel();
                    UpdateButtonStates();
                }
            }
        }

        private void OnExportClick(object sender, RoutedEventArgs e)
        {
            if (!(ListItems.SelectedItem is ProfileBrowserEntry entry) || !entry.CanExport)
                return;

            string suggestedName = (entry.Name ?? "profile").Replace(' ', '_').Replace(':', '_');
            var saveDialog = new Microsoft.Win32.SaveFileDialog
            {
                Filter = "JSON files (*.json)|*.json",
                DefaultExt = "json",
                FileName = $"{suggestedName}_ffb.json",
                Title = "Export Profile"
            };

            if (saveDialog.ShowDialog() != true)
                return;

            try
            {
                var exported = new DiyFfbPluginSettings.ExportedProfile
                {
                    Version = 1,
                    ProfileKey = entry.ProfileKey,
                    GraphPath = entry.GraphPath,
                    ExportedAt = DateTime.UtcNow.ToString("o"),
                    Profile = entry.Profile
                };

                string json = JsonConvert.SerializeObject(exported, Formatting.Indented);
                File.WriteAllText(saveDialog.FileName, json);

                ThemedMessageBox.Show(this, $"Profile exported to:\n{saveDialog.FileName}", "Export Complete",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
            catch (Exception ex)
            {
                ThemedMessageBox.Show(this, $"Error exporting profile: {ex.Message}", "Export Failed",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void OnCancelClick(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
        }
    }
}
