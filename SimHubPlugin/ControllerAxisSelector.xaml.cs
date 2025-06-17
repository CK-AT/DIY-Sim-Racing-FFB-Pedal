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
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for AxisSelector.xaml
    /// </summary>
    public partial class ControllerAxisSelector : UserControl
    {
        public class ControllerAxisChangedEventArgs : EventArgs
        {
            ControllerAxis _new_value;
            public ControllerAxis Value { get { return _new_value; } }
            public ControllerAxisChangedEventArgs(ControllerAxis new_value)
            {
                _new_value = new_value;
            }
        }
        public delegate void ControllerAxisChangedEventHandler(object sender, ControllerAxisChangedEventArgs e);
        public event ControllerAxisChangedEventHandler ControllerAxisChanged; 
        public ControllerAxisSelector()
        {
            InitializeComponent();
        }

        public ControllerAxis Value { get { return (ControllerAxis)(cb_axis_id.SelectedIndex); } set { cb_axis_id.SelectedIndex = (int)value; } }

        private void cb_axis_id_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ControllerAxisChanged?.Invoke(this, new ControllerAxisChangedEventArgs(Value));
        }
    }
}
