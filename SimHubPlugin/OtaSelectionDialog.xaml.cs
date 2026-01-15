using System;
using System.Collections.ObjectModel;
using System.Windows;
using System.Windows.Input;

namespace User.PluginSdkDemo
{
    public partial class OtaSelectionDialog : Window
    {
        public ObservableCollection<OtaTargetEntry> Targets { get; }

        public event EventHandler StartRequested;

        public OtaSelectionDialog(ObservableCollection<OtaTargetEntry> targets)
        {
            Targets = targets ?? new ObservableCollection<OtaTargetEntry>();
            DataContext = this;
            InitializeComponent();
        }

        private void btn_close_Click(object sender, RoutedEventArgs e)
        {
            Close();
        }

        private void btn_start_Click(object sender, RoutedEventArgs e)
        {
            StartRequested?.Invoke(this, EventArgs.Empty);
        }

        public void SetBindingUrl(string url)
        {
            if (TextBlock_BindingUrl != null)
            {
                TextBlock_BindingUrl.Text = string.IsNullOrWhiteSpace(url) ? "Binding URL: -" : $"Binding URL: {url}";
                TextBlock_BindingUrl.ToolTip = url;
            }
        }

        public void AppendLog(string line)
        {
            if (TextBox_Log != null)
            {
                TextBox_Log.AppendText(line + Environment.NewLine);
                TextBox_Log.ScrollToEnd();
            }
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                DragMove();
            }
        }
    }
}
