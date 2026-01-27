using System;
using System.Collections;
using System.Windows;
using System.Windows.Controls;

namespace User.PluginSdkDemo.GraphEditor
{
    public partial class PreviewWindow : Window
    {
        private bool _suppressLiveInputsNotification;

        public PreviewWindow()
        {
            InitializeComponent();
        }

        public event EventHandler<bool> LiveInputsToggled;

        public void SetSources(IEnumerable previewInputs, IEnumerable previewParams)
        {
            PreviewInputsList.ItemsSource = previewInputs;
            PreviewParamsList.ItemsSource = previewParams;
        }

        public void SetLiveInputsState(bool enabled)
        {
            _suppressLiveInputsNotification = true;
            CheckLiveInputs.IsChecked = enabled;
            _suppressLiveInputsNotification = false;
        }

        public void SetInputsEnabled(bool enabled)
        {
            PreviewInputsList.IsEnabled = enabled;
            PreviewParamsList.IsEnabled = enabled;
        }

        public void SetStatusText(string text)
        {
            TextPreviewStatus.Text = text ?? string.Empty;
        }

        private void CheckLiveInputs_Changed(object sender, RoutedEventArgs e)
        {
            if (_suppressLiveInputsNotification)
            {
                return;
            }

            LiveInputsToggled?.Invoke(this, CheckLiveInputs.IsChecked == true);
        }
    }
}
