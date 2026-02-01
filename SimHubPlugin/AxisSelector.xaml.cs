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

namespace DiyFfb
{
    /// <summary>
    /// Interaction logic for AxisSelector.xaml
    /// </summary>
    public partial class AxisSelector : UserControl
    {
        public class AxisIDChangedEventArgs : EventArgs
        {
            AxisID _new_value;
            public AxisID Value { get { return _new_value; } }
            public AxisIDChangedEventArgs(AxisID new_value)
            {
                _new_value = new_value;
            }
        }
        public delegate void AxisIDChangedEventHandler(object sender, AxisIDChangedEventArgs e);
        public event AxisIDChangedEventHandler AxisIDChanged; 
        public AxisSelector()
        {
            InitializeComponent();
        }

        public AxisID Value { get { return (AxisID)(cb_axis_id.SelectedIndex); } set { cb_axis_id.SelectedIndex = (int)value; } }

        private void cb_axis_id_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            AxisIDChanged?.Invoke(this, new AxisIDChangedEventArgs(Value));
        }
    }
}
