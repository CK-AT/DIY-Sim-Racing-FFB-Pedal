using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using Newtonsoft.Json;
using DiyFfb.Controls;
using DiyFfb.GraphEditor;
using DiyFfb.Helpers;

namespace DiyFfb.ProfileBrowser
{
    public partial class ProfileBrowserDialog : Window
    {
        private const string AllGamesFilter = "<All Games>";
        private const string ImportsFilter = "<Imports>";

        // Public result properties
        public ProfileBrowserEntry SelectedEntry { get; private set; }
        public bool UseGraphOnly { get; private set; }
        public bool UseTuning { get; private set; }
        // Plan 17: when false, caller keeps the current vehicle's graph and
        // applies only the source profile's tuning. Set by "Use Tuning Only".
        public bool UseSourceGraph { get; private set; } = true;

        // Data binding
        public ObservableCollection<ProfileBrowserEntry> Items { get; } = new ObservableCollection<ProfileBrowserEntry>();

        // All stored profiles (unfiltered)
        private List<ProfileBrowserEntry> _allStoredProfiles = new List<ProfileBrowserEntry>();

        // Staged imports (not yet saved to library)
        private List<ProfileBrowserEntry> _stagedImports = new List<ProfileBrowserEntry>();

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
            {
                PanelGameFilter.Visibility = Visibility.Collapsed;
                LoadTemplates();
            }
            else if (RadioMyVehicles.IsChecked == true)
            {
                PanelGameFilter.Visibility = Visibility.Visible;
                LoadStoredProfiles();
            }
        }

        private void LoadTemplates()
        {
            PanelGameFilter.Visibility = Visibility.Collapsed;
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
            PanelGameFilter.Visibility = Visibility.Visible;
            Items.Clear();
            _allStoredProfiles.Clear();

            var profiles = _plugin?.Settings?.AircraftFfbProfiles;
            if (profiles == null)
            {
                PopulateGameFilter();
                UpdateDetailsPanel();
                UpdateButtonStates();
                return;
            }

            // Identify the currently active vehicle so its entry can be flagged.
            string currentKey = _plugin?.GetActiveProfileKey();

            // Build full list of profiles with game ID extracted
            foreach (var kvp in profiles)
            {
                string graphPath = kvp.Value?.GraphPath ?? "";
                var entry = ProfileBrowserEntry.FromProfile(kvp.Key, kvp.Value, graphPath);
                entry.GameId = ExtractGameId(kvp.Key);
                entry.IsCurrent = !string.IsNullOrEmpty(currentKey)
                    && string.Equals(kvp.Key, currentKey, StringComparison.Ordinal);
                _allStoredProfiles.Add(entry);
            }

            PopulateGameFilter();
            ApplyGameFilter();

            // Bring the active vehicle's profile into view (if it survived filtering).
            var current = Items.FirstOrDefault(e => e.IsCurrent);
            if (current != null)
            {
                ListItems.ScrollIntoView(current);
            }
        }

        private string ExtractGameId(string profileKey)
        {
            if (string.IsNullOrEmpty(profileKey))
                return null;

            int sep = profileKey.IndexOf("::");
            return sep > 0 ? profileKey.Substring(0, sep) : null;
        }

        private void PopulateGameFilter()
        {
            var previousSelection = ComboGameFilter.SelectedItem as string;
            ComboGameFilter.Items.Clear();
            ComboGameFilter.Items.Add(AllGamesFilter);

            // Add <Imports> option if there are staged imports
            if (_stagedImports.Count > 0)
            {
                ComboGameFilter.Items.Add(ImportsFilter);
            }

            // Get distinct game IDs, sorted
            var gameIds = _allStoredProfiles
                .Where(p => !string.IsNullOrEmpty(p.GameId))
                .Select(p => p.GameId)
                .Distinct()
                .OrderBy(g => g)
                .ToList();

            foreach (var gameId in gameIds)
            {
                ComboGameFilter.Items.Add(gameId);
            }

            // Select appropriate default
            if (previousSelection != null && ComboGameFilter.Items.Contains(previousSelection))
            {
                ComboGameFilter.SelectedItem = previousSelection;
            }
            else if (!string.IsNullOrEmpty(_currentGameId) && ComboGameFilter.Items.Contains(_currentGameId))
            {
                ComboGameFilter.SelectedItem = _currentGameId;
            }
            else
            {
                ComboGameFilter.SelectedItem = AllGamesFilter;
            }
        }

        private void ApplyGameFilter()
        {
            Items.Clear();
            string selectedGame = ComboGameFilter.SelectedItem as string;

            IEnumerable<ProfileBrowserEntry> filtered;
            if (selectedGame == ImportsFilter)
            {
                filtered = _stagedImports;
            }
            else if (!string.IsNullOrEmpty(selectedGame) && selectedGame != AllGamesFilter)
            {
                filtered = _allStoredProfiles.Where(p => p.GameId == selectedGame);
            }
            else
            {
                filtered = _allStoredProfiles;
            }

            foreach (var entry in filtered)
            {
                Items.Add(entry);
            }

            if (Items.Count > 0)
                ListItems.SelectedIndex = 0;

            UpdateDetailsPanel();
            UpdateButtonStates();
        }

        private void OnGameFilterChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (!_isInitialized)
                return;

            ApplyGameFilter();
        }

        private void OnImportClick(object sender, RoutedEventArgs e)
        {
            var dialog = new Microsoft.Win32.OpenFileDialog
            {
                Filter = "JSON files (*.json)|*.json|All files (*.*)|*.*",
                DefaultExt = "json",
                Title = "Import Profiles",
                Multiselect = true
            };

            if (dialog.ShowDialog() != true || dialog.FileNames.Length == 0)
                return;

            var importedEntries = new List<ProfileBrowserEntry>();
            var failedFiles = new List<string>();

            foreach (string filePath in dialog.FileNames)
            {
                try
                {
                    var entry = ImportSingleFile(filePath);
                    if (entry != null)
                    {
                        importedEntries.Add(entry);
                    }
                    else
                    {
                        failedFiles.Add(Path.GetFileName(filePath));
                    }
                }
                catch (Exception ex)
                {
                    failedFiles.Add($"{Path.GetFileName(filePath)}: {ex.Message}");
                }
            }

            if (importedEntries.Count > 0)
            {
                // Add to staged imports (don't replace existing staged imports)
                _stagedImports.AddRange(importedEntries);

                // Ensure My Vehicles tab is active
                RadioMyVehicles.IsChecked = true;

                // Rebuild filter to include <Imports>
                PopulateGameFilter();

                // Switch to Imports filter
                ComboGameFilter.SelectedItem = ImportsFilter;
            }

            if (failedFiles.Count > 0)
            {
                string message = failedFiles.Count == 1
                    ? $"Failed to import: {failedFiles[0]}"
                    : $"Failed to import {failedFiles.Count} file(s):\n\n{string.Join("\n", failedFiles.Take(5))}" +
                      (failedFiles.Count > 5 ? $"\n...and {failedFiles.Count - 5} more" : "");

                ThemedMessageBox.Show(this, message, "Import Warning",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }

        private ProfileBrowserEntry ImportSingleFile(string filePath)
        {
            string json = File.ReadAllText(filePath);

            // Try new ExportedProfile format first
            var exported = JsonConvert.DeserializeObject<DiyFfbPluginSettings.ExportedProfile>(json);
            if (exported?.Profile != null)
            {
                return ProfileBrowserEntry.FromImportedFile(exported);
            }

            // Try legacy AircraftFfbProfile format
            var legacyProfile = JsonConvert.DeserializeObject<DiyFfbPluginSettings.AircraftFfbProfile>(json);
            if (legacyProfile != null)
            {
                var legacyExported = new DiyFfbPluginSettings.ExportedProfile
                {
                    Version = 1,
                    ProfileKey = Path.GetFileNameWithoutExtension(filePath),
                    Profile = legacyProfile
                };
                return ProfileBrowserEntry.FromImportedFile(legacyExported);
            }

            return null;
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
            var selectedItems = ListItems.SelectedItems.Cast<ProfileBrowserEntry>().ToList();
            int count = selectedItems.Count;
            bool hasSelection = count > 0;
            bool singleSelection = count == 1;
            var entry = singleSelection ? selectedItems[0] : null;

            bool hasAnyStaged = selectedItems.Any(e => e.Source == ProfileEntrySource.ImportedFile);
            bool hasAnyDeletable = selectedItems.Any(e => e.CanDelete);

            // Save to Library: visible when viewing imports, enabled when staged items selected
            bool showingImports = (ComboGameFilter.SelectedItem as string) == ImportsFilter;
            ButtonSaveToLibrary.Visibility = showingImports ? Visibility.Visible : Visibility.Collapsed;
            ButtonSaveToLibrary.IsEnabled = hasAnyStaged;

            // Delete: visible on My Vehicles tab (not Templates), enabled when deletable items selected
            bool showingMyVehicles = RadioMyVehicles.IsChecked == true;
            ButtonDeleteSelected.Visibility = showingMyVehicles ? Visibility.Visible : Visibility.Collapsed;
            ButtonDeleteSelected.IsEnabled = hasAnyDeletable;

            // Graph Only / Graph+Tuning: only for single selection
            ButtonUseGraphOnly.IsEnabled = singleSelection;
            ButtonUseGraphTuning.IsEnabled = singleSelection && entry != null &&
                (entry.HasTuning || entry.Source != ProfileEntrySource.Template);
            // Use Tuning Only: source must have tuning, and current vehicle
            // must already have a graph assigned (otherwise "tuning only" has
            // no graph to apply against).
            bool currentVehicleHasGraph = !string.IsNullOrWhiteSpace(_plugin?.GetActiveGraphPath());
            ButtonUseTuningOnly.IsEnabled = singleSelection && entry != null &&
                entry.HasTuning && currentVehicleHasGraph;

            // Export: only for single selection of exportable item
            ButtonExport.IsEnabled = singleSelection && entry?.CanExport == true;

            // Hide "Use Graph + Tuning" and "Use Tuning Only" for templates
            // (templates have no stored tuning to copy).
            if (entry != null && entry.Source == ProfileEntrySource.Template)
            {
                ButtonUseGraphTuning.Visibility = Visibility.Collapsed;
                ButtonUseTuningOnly.Visibility = Visibility.Collapsed;
            }
            else
            {
                ButtonUseGraphTuning.Visibility = Visibility.Visible;
                ButtonUseTuningOnly.Visibility = Visibility.Visible;
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
                UseSourceGraph = true;
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
                UseSourceGraph = true;
                UseGraphOnly = false;
                UseTuning = true;
                DialogResult = true;
            }
        }

        private void OnUseTuningOnlyClick(object sender, RoutedEventArgs e)
        {
            if (ListItems.SelectedItem is ProfileBrowserEntry entry)
            {
                SelectedEntry = entry;
                UseSourceGraph = false;
                UseGraphOnly = false;
                UseTuning = true;
                DialogResult = true;
            }
        }

        private void OnDeleteClick(object sender, RoutedEventArgs e)
        {
            if (!(((System.Windows.Controls.Button)sender).DataContext is ProfileBrowserEntry entry))
                return;

            if (entry.Source == ProfileEntrySource.StoredProfile)
            {
                var result = ThemedMessageBox.Show(this,
                    $"Delete profile for '{entry.Name}'?\n\nThis will remove all saved tuning for this vehicle.",
                    "Confirm Delete",
                    MessageBoxButton.YesNo,
                    MessageBoxImage.Warning);

                if (result == MessageBoxResult.Yes)
                {
                    _plugin?.Settings?.AircraftFfbProfiles?.Remove(entry.ProfileKey);
                    _allStoredProfiles.Remove(entry);
                    Items.Remove(entry);
                    UpdateDetailsPanel();
                    UpdateButtonStates();
                }
            }
            else if (entry.Source == ProfileEntrySource.ImportedFile)
            {
                // Remove staged import without confirmation
                _stagedImports.Remove(entry);
                Items.Remove(entry);

                // If no more staged imports, rebuild filter to remove <Imports> option
                if (_stagedImports.Count == 0)
                {
                    PopulateGameFilter();
                    // Switch away from Imports filter since it's now empty
                    if ((ComboGameFilter.SelectedItem as string) == ImportsFilter)
                    {
                        ComboGameFilter.SelectedItem = AllGamesFilter;
                    }
                }

                UpdateDetailsPanel();
                UpdateButtonStates();
            }
        }

        private void OnDeleteSelectedClick(object sender, RoutedEventArgs e)
        {
            var selectedItems = ListItems.SelectedItems
                .Cast<ProfileBrowserEntry>()
                .Where(entry => entry.CanDelete)
                .ToList();

            if (selectedItems.Count == 0)
                return;

            // Check if any are stored profiles (require confirmation)
            var storedProfiles = selectedItems.Where(x => x.Source == ProfileEntrySource.StoredProfile).ToList();
            if (storedProfiles.Count > 0)
            {
                string message = storedProfiles.Count == 1
                    ? $"Delete profile for '{storedProfiles[0].Name}'?\n\nThis will remove all saved tuning for this vehicle."
                    : $"Delete {storedProfiles.Count} profiles?\n\nThis will remove all saved tuning for these vehicles.";

                var result = ThemedMessageBox.Show(this, message, "Confirm Delete",
                    MessageBoxButton.YesNo, MessageBoxImage.Warning);

                if (result != MessageBoxResult.Yes)
                    return;
            }

            // Delete all selected items
            foreach (var entry in selectedItems)
            {
                if (entry.Source == ProfileEntrySource.StoredProfile)
                {
                    _plugin?.Settings?.AircraftFfbProfiles?.Remove(entry.ProfileKey);
                    _allStoredProfiles.Remove(entry);
                }
                else if (entry.Source == ProfileEntrySource.ImportedFile)
                {
                    _stagedImports.Remove(entry);
                }
                Items.Remove(entry);
            }

            // Refresh filter if needed
            if (_stagedImports.Count == 0 && (ComboGameFilter.SelectedItem as string) == ImportsFilter)
            {
                PopulateGameFilter();
                ComboGameFilter.SelectedItem = AllGamesFilter;
            }

            UpdateDetailsPanel();
            UpdateButtonStates();
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

        private void OnSaveToLibraryClick(object sender, RoutedEventArgs e)
        {
            var selectedItems = ListItems.SelectedItems
                .Cast<ProfileBrowserEntry>()
                .Where(entry => entry.Source == ProfileEntrySource.ImportedFile)
                .ToList();

            if (selectedItems.Count == 0)
                return;

            // Check for duplicates
            var existingKeys = _plugin?.Settings?.AircraftFfbProfiles?.Keys.ToHashSet() ?? new HashSet<string>();
            var duplicates = selectedItems.Where(x => !string.IsNullOrEmpty(x.ProfileKey) && existingKeys.Contains(x.ProfileKey)).ToList();

            if (duplicates.Count > 0)
            {
                string message = duplicates.Count == 1
                    ? $"Profile '{duplicates[0].Name}' already exists in your library.\n\nOverwrite it?"
                    : $"{duplicates.Count} profiles already exist in your library.\n\nOverwrite them?";

                var result = ThemedMessageBox.Show(this, message, "Duplicate Profiles",
                    MessageBoxButton.YesNo, MessageBoxImage.Question);

                if (result != MessageBoxResult.Yes)
                {
                    // Remove duplicates from the list to save
                    selectedItems = selectedItems.Except(duplicates).ToList();
                    if (selectedItems.Count == 0)
                        return;
                }
            }

            foreach (var entry in selectedItems)
            {
                // Save to settings
                if (_plugin?.Settings?.AircraftFfbProfiles != null && !string.IsNullOrEmpty(entry.ProfileKey))
                {
                    // Remove existing entry from _allStoredProfiles if overwriting
                    var existing = _allStoredProfiles.FirstOrDefault(x => x.ProfileKey == entry.ProfileKey);
                    if (existing != null)
                        _allStoredProfiles.Remove(existing);

                    _plugin.Settings.AircraftFfbProfiles[entry.ProfileKey] = entry.Profile;
                }

                // Move from staged to stored
                _stagedImports.Remove(entry);
                entry.Source = ProfileEntrySource.StoredProfile;
                _allStoredProfiles.Add(entry);
            }

            // Refresh filter and view
            PopulateGameFilter();

            // If no more staged imports, switch to appropriate game filter
            if (_stagedImports.Count == 0)
            {
                var lastSaved = selectedItems.LastOrDefault();
                if (lastSaved != null && !string.IsNullOrEmpty(lastSaved.GameId))
                    ComboGameFilter.SelectedItem = lastSaved.GameId;
                else
                    ComboGameFilter.SelectedItem = AllGamesFilter;
            }
            else
            {
                ApplyGameFilter(); // Stay on Imports
            }
        }

        private void OnCancelClick(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
        }
    }
}
