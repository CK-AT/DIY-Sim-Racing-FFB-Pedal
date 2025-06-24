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

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for SaveSelectionDialog.xaml
    /// </summary>
    public partial class SaveSelectionDialog : Window
    {
        public bool SaveRequested { get; set; } = false;
        public SaveSelectionDialog(DiyFfbPluginUI ui)
        {
            DataContext = ui;
            InitializeComponent();
        }

        private void btn_abort_Click(object sender, RoutedEventArgs e)
        {
            SaveRequested = false;
            this.Close();
        }

        private void btn_store_Click(object sender, RoutedEventArgs e)
        {
            SaveRequested = true;
            this.Close();
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
                this.DragMove();
        }
    }
}
