using System;
using System.Collections.Generic;
using System.Windows;

namespace User.PluginSdkDemo.GraphEditor
{
    public partial class GraphTemplateSelectorDialog : Window
    {
        public GraphTemplateEntry SelectedTemplate { get; private set; }

        public GraphTemplateSelectorDialog(string gameId, string carId, IEnumerable<GraphTemplateEntry> templates)
        {
            InitializeComponent();

            TextGame.Text = string.IsNullOrWhiteSpace(gameId) ? "(unknown)" : gameId;
            TextVehicle.Text = string.IsNullOrWhiteSpace(carId) ? "(unknown)" : carId;

            ListTemplates.ItemsSource = templates ?? Array.Empty<GraphTemplateEntry>();

            if (ListTemplates.Items.Count > 0)
            {
                ListTemplates.SelectedIndex = 0;
            }
        }

        private void ButtonSkip_Click(object sender, RoutedEventArgs e)
        {
            SelectedTemplate = null;
            DialogResult = true;
        }

        private void ButtonSelect_Click(object sender, RoutedEventArgs e)
        {
            if (ListTemplates.SelectedItem is GraphTemplateEntry entry)
            {
                SelectedTemplate = entry;
                DialogResult = true;
            }
            else
            {
                MessageBox.Show(this, "Please select a template from the list.", "No Selection",
                    MessageBoxButton.OK, MessageBoxImage.Information);
            }
        }

        private void ListTemplates_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (ListTemplates.SelectedItem is GraphTemplateEntry entry)
            {
                SelectedTemplate = entry;
                DialogResult = true;
            }
        }
    }

    public sealed class GraphTemplateEntry
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string Category { get; set; }
        public string TemplatePath { get; set; }
        public string[] GameIds { get; set; }
    }
}
