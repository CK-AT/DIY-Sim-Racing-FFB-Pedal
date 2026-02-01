using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using DiyFfb.Helpers;

namespace DiyFfb.GraphEditor
{
    /// <summary>
    /// Result of the SharedGraphSaveDialog.
    /// </summary>
    public enum SharedGraphSaveResult
    {
        Cancel,
        SaveAnyway,
        SaveAsCopy
    }

    /// <summary>
    /// View model for include usage display.
    /// </summary>
    public sealed class IncludeUsageViewModel
    {
        public string GraphName { get; set; }
        public List<string> Vehicles { get; set; } = new List<string>();
    }

    /// <summary>
    /// Dialog shown when saving a graph that is shared by multiple vehicles.
    /// </summary>
    public partial class SharedGraphSaveDialog : Window
    {
        public SharedGraphSaveResult Result { get; private set; } = SharedGraphSaveResult.Cancel;

        private readonly GraphUsageReport _report;

        public SharedGraphSaveDialog(GraphUsageReport report)
        {
            _report = report;
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);
            PopulateUsageDetails();
        }

        private void PopulateUsageDetails()
        {
            // Direct users
            if (_report.DirectUsers.Count > 0)
            {
                var directUserLabels = _report.DirectUsers
                    .Select(key => FormatVehicleKey(key, key == _report.CurrentVehicleKey))
                    .ToList();
                ListDirectUsers.ItemsSource = directUserLabels;
                PanelDirectUsage.Visibility = Visibility.Visible;
            }
            else
            {
                PanelDirectUsage.Visibility = Visibility.Collapsed;
            }

            // Included by
            if (_report.IncludedBy.Count > 0)
            {
                var includeViewModels = _report.IncludedBy
                    .Where(i => i.VehicleKeys.Count > 0)
                    .Select(i => new IncludeUsageViewModel
                    {
                        GraphName = Path.GetFileName(i.IncludingGraphPath),
                        Vehicles = i.VehicleKeys.Select(k => FormatVehicleKey(k, false)).ToList()
                    })
                    .ToList();

                if (includeViewModels.Count > 0)
                {
                    ListIncludedBy.ItemsSource = includeViewModels;
                    PanelIncludedBy.Visibility = Visibility.Visible;
                }
                else
                {
                    PanelIncludedBy.Visibility = Visibility.Collapsed;
                }
            }
            else
            {
                PanelIncludedBy.Visibility = Visibility.Collapsed;
            }
        }

        /// <summary>
        /// Formats a vehicle key for display.
        /// Example: "XPLANE11::Cessna_172" -> "XPlane: Cessna_172"
        /// </summary>
        private string FormatVehicleKey(string key, bool isCurrent)
        {
            if (string.IsNullOrEmpty(key))
                return key;

            // Parse "gameId::carId" format
            int sep = key.IndexOf("::");
            string display;
            if (sep > 0)
            {
                string gameId = key.Substring(0, sep);
                string carId = key.Substring(sep + 2);

                // Simplify game ID
                string gameName = gameId.Replace("_", " ");
                if (gameName.Length > 10)
                    gameName = gameName.Substring(0, 10);

                display = $"{gameName}: {carId}";
            }
            else
            {
                display = key;
            }

            if (isCurrent)
                display += " (current)";

            return display;
        }

        private void OnCancelClick(object sender, RoutedEventArgs e)
        {
            Result = SharedGraphSaveResult.Cancel;
            DialogResult = false;
            Close();
        }

        private void OnSaveAnywayClick(object sender, RoutedEventArgs e)
        {
            Result = SharedGraphSaveResult.SaveAnyway;
            DialogResult = true;
            Close();
        }

        private void OnSaveAsCopyClick(object sender, RoutedEventArgs e)
        {
            Result = SharedGraphSaveResult.SaveAsCopy;
            DialogResult = true;
            Close();
        }
    }
}
