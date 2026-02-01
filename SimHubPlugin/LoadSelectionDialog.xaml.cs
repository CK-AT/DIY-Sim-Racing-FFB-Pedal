using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace DiyFfb
{
    /// <summary>
    /// Interaction logic for LoadSelectionDialog.xaml
    /// </summary>
    public partial class LoadSelectionDialog : Window
    {
        public bool LoadRequested { get; set; } = false;
        public bool UploadRequested { get; set; } = true;
        public Dictionary<AxisID, AxisConfig> axis_configs = new Dictionary<AxisID, AxisConfig>();
        public Dictionary<FunctionID, FunctionConfig> function_configs = new Dictionary<FunctionID, FunctionConfig>();

        public LoadSelectionDialog(DiyFfbPluginUI ui, Dictionary<AxisID, AxisConfig> axis_configs, Dictionary<FunctionID, FunctionConfig> function_configs)
        {
            DataContext = ui;
            this.axis_configs = axis_configs;
            this.function_configs = function_configs;
            InitializeComponent();
        }

        private void btn_abort_Click(object sender, RoutedEventArgs e)
        {
            LoadRequested = false;
            this.Close();
        }

        private void btn_load_Click(object sender, RoutedEventArgs e)
        {
            LoadRequested = true;
            this.Close();
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
                this.DragMove();
        }
        private void OnPreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Escape)
            {
                LoadRequested = false;
                this.Close();
            }
            else if (e.Key == Key.Enter)
            {
                LoadRequested = true;
                this.Close();
            }
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                btn_load.Content = "Load";
                UploadRequested = false;
            }
        }
        private void OnPreviewKeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.LeftCtrl || e.Key == Key.RightCtrl)
            {
                btn_load.Content = "Load and Upload";
                UploadRequested = true;
            }
        }
    }
}
