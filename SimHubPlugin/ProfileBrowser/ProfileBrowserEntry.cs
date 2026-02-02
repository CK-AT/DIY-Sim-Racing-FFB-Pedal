using System.ComponentModel;
using System.IO;
using DiyFfb.GraphEditor;

namespace DiyFfb.ProfileBrowser
{
    /// <summary>
    /// Identifies the source of a profile browser entry.
    /// </summary>
    public enum ProfileEntrySource
    {
        /// <summary>From GraphTemplateRegistry - a template with no tuning.</summary>
        Template,

        /// <summary>From AircraftFfbProfiles - a stored profile with tuning.</summary>
        StoredProfile,

        /// <summary>Loaded from an external file via Import.</summary>
        ImportedFile
    }

    /// <summary>
    /// View model for items displayed in the Profile Browser dialog.
    /// </summary>
    public class ProfileBrowserEntry : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        // Identity
        public string ProfileKey { get; set; }
        public ProfileEntrySource Source { get; set; }
        public string GameId { get; set; }

        // Display properties
        public string Name { get; set; }
        public string GraphPath { get; set; }
        public string GraphName => string.IsNullOrEmpty(GraphPath) ? "" : Path.GetFileName(GraphPath);
        public string Category { get; set; }
        public string Description { get; set; }

        // Tuning info (for stored profiles)
        public bool HasTuning { get; set; }
        public int TunedParamCount { get; set; }

        // Profile data (for stored profiles and imports)
        public DiyFfbPluginSettings.AircraftFfbProfile Profile { get; set; }

        // Template data (for templates)
        public GraphTemplateEntry TemplateEntry { get; set; }

        // UI state
        private bool _isSelected;
        public bool IsSelected
        {
            get => _isSelected;
            set
            {
                if (_isSelected != value)
                {
                    _isSelected = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsSelected)));
                }
            }
        }

        /// <summary>
        /// Whether this entry can be deleted (stored profiles and staged imports).
        /// </summary>
        public bool CanDelete => Source == ProfileEntrySource.StoredProfile || Source == ProfileEntrySource.ImportedFile;

        /// <summary>
        /// Whether this entry can be exported (only stored profiles and imports).
        /// </summary>
        public bool CanExport => Source == ProfileEntrySource.StoredProfile || Source == ProfileEntrySource.ImportedFile;

        /// <summary>
        /// Creates an entry from a graph template.
        /// </summary>
        public static ProfileBrowserEntry FromTemplate(GraphTemplateEntry template)
        {
            return new ProfileBrowserEntry
            {
                Source = ProfileEntrySource.Template,
                Name = template.Name,
                Description = template.Description,
                Category = template.Category,
                GraphPath = template.TemplatePath,
                TemplateEntry = template,
                HasTuning = false,
                TunedParamCount = 0
            };
        }

        /// <summary>
        /// Creates an entry from a stored aircraft profile.
        /// </summary>
        public static ProfileBrowserEntry FromProfile(string profileKey, DiyFfbPluginSettings.AircraftFfbProfile profile, string graphPath)
        {
            int paramCount = profile?.GraphParamValues?.Count ?? 0;
            string displayName = FormatProfileKeyAsName(profileKey);

            return new ProfileBrowserEntry
            {
                Source = ProfileEntrySource.StoredProfile,
                ProfileKey = profileKey,
                Name = displayName,
                GraphPath = graphPath,
                Profile = profile,
                HasTuning = paramCount > 0,
                TunedParamCount = paramCount
            };
        }

        /// <summary>
        /// Creates an entry from an imported file.
        /// </summary>
        public static ProfileBrowserEntry FromImportedFile(DiyFfbPluginSettings.ExportedProfile exported)
        {
            int paramCount = exported?.Profile?.GraphParamValues?.Count ?? 0;
            string displayName = FormatProfileKeyAsName(exported?.ProfileKey ?? "Imported Profile");

            // Extract game ID from profile key
            string gameId = null;
            if (!string.IsNullOrEmpty(exported?.ProfileKey))
            {
                int sep = exported.ProfileKey.IndexOf("::");
                if (sep > 0)
                    gameId = exported.ProfileKey.Substring(0, sep);
            }

            return new ProfileBrowserEntry
            {
                Source = ProfileEntrySource.ImportedFile,
                ProfileKey = exported?.ProfileKey,
                Name = displayName,
                GameId = gameId,
                GraphPath = exported?.GraphPath,
                Profile = exported?.Profile,
                HasTuning = paramCount > 0,
                TunedParamCount = paramCount,
                Description = $"Exported: {exported?.ExportedAt ?? "unknown"}"
            };
        }

        /// <summary>
        /// Formats a profile key (e.g., "XPlane11::Cessna_172") as a friendly name.
        /// </summary>
        private static string FormatProfileKeyAsName(string profileKey)
        {
            if (string.IsNullOrWhiteSpace(profileKey))
                return "(Unknown)";

            // Split on "::" to separate game and vehicle
            int sep = profileKey.IndexOf("::");
            if (sep > 0 && sep < profileKey.Length - 2)
            {
                string vehicle = profileKey.Substring(sep + 2);
                // Replace underscores with spaces for readability
                return vehicle.Replace('_', ' ');
            }

            return profileKey.Replace('_', ' ');
        }
    }
}
