using System.Windows;
using DiyFfb.Helpers;

namespace DiyFfb.ProfileBrowser
{
    /// <summary>
    /// How a picked custom graph should be stored for the vehicle.
    /// </summary>
    public enum GraphStorageMode
    {
        Cancel,
        Reference,
        Copy
    }

    /// <summary>
    /// Small choice dialog shown after the user picks a custom graph file in the
    /// Profile Browser: reference the file in place, or copy it into the managed
    /// library. Mirrors the SharedGraphSaveDialog pattern.
    /// </summary>
    public partial class GraphStorageModeDialog : Window
    {
        public GraphStorageMode Result { get; private set; } = GraphStorageMode.Cancel;

        public GraphStorageModeDialog()
        {
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);
        }

        private void OnCancelClick(object sender, RoutedEventArgs e)
        {
            Result = GraphStorageMode.Cancel;
            DialogResult = false;
            Close();
        }

        private void OnReferenceClick(object sender, RoutedEventArgs e)
        {
            Result = GraphStorageMode.Reference;
            DialogResult = true;
            Close();
        }

        private void OnCopyClick(object sender, RoutedEventArgs e)
        {
            Result = GraphStorageMode.Copy;
            DialogResult = true;
            Close();
        }
    }
}
