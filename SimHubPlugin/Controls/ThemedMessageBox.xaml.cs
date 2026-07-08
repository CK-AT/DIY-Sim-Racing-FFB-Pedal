using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;

namespace DiyFfb.Controls
{
    /// <summary>
    /// A dark-themed message box that matches the plugin's visual style.
    /// </summary>
    public partial class ThemedMessageBox : Window
    {
        private MessageBoxResult _result = MessageBoxResult.None;

        private ThemedMessageBox()
        {
            InitializeComponent();
            KeyDown += OnKeyDown;
        }

        /// <summary>
        /// Shows a themed message box.
        /// </summary>
        public static MessageBoxResult Show(
            string message,
            string title,
            MessageBoxButton buttons = MessageBoxButton.OK,
            MessageBoxImage icon = MessageBoxImage.None)
        {
            return Show(null, message, title, buttons, icon);
        }

        /// <summary>
        /// Shows a themed message box with an owner window.
        /// </summary>
        public static MessageBoxResult Show(
            Window owner,
            string message,
            string title,
            MessageBoxButton buttons = MessageBoxButton.OK,
            MessageBoxImage icon = MessageBoxImage.None)
        {
            var dialog = new ThemedMessageBox();
            dialog.TitleText.Text = title;
            dialog.Title = title;
            dialog.MessageText.Text = message;
            dialog.SetIcon(icon);
            dialog.SetButtons(buttons);

            if (owner != null)
            {
                dialog.Owner = owner;
                dialog.WindowStartupLocation = WindowStartupLocation.CenterOwner;
            }
            else
            {
                dialog.Topmost = true;
                dialog.WindowStartupLocation = WindowStartupLocation.CenterScreen;
            }

            dialog.ShowDialog();
            return dialog._result;
        }

        private void SetIcon(MessageBoxImage icon)
        {
            if (icon == MessageBoxImage.None)
            {
                IconBorder.Visibility = Visibility.Collapsed;
                return;
            }

            IconBorder.Visibility = Visibility.Visible;

            // Note: Some MessageBoxImage values are aliases (e.g. Error/Stop/Hand are all 16)
            // Use if/else instead of switch to avoid duplicate case errors
            if (icon == MessageBoxImage.Information)
            {
                IconBorder.Background = new SolidColorBrush(Color.FromRgb(0x4A, 0x6A, 0x8A)); // Blue
                IconText.Text = "i";
                IconText.Foreground = Brushes.White;
            }
            else if (icon == MessageBoxImage.Warning) // Also matches Exclamation (both are 48)
            {
                IconBorder.Background = new SolidColorBrush(Color.FromRgb(0x8A, 0x7A, 0x4A)); // Yellow/amber
                IconText.Text = "!";
                IconText.Foreground = Brushes.White;
            }
            else if (icon == MessageBoxImage.Error) // Also matches Stop and Hand (all are 16)
            {
                IconBorder.Background = new SolidColorBrush(Color.FromRgb(0x8A, 0x4A, 0x4A)); // Red
                IconText.Text = "X";
                IconText.Foreground = Brushes.White;
            }
            else if (icon == MessageBoxImage.Question)
            {
                IconBorder.Background = new SolidColorBrush(Color.FromRgb(0x4A, 0x6A, 0x8A)); // Blue
                IconText.Text = "?";
                IconText.Foreground = Brushes.White;
            }
            else
            {
                IconBorder.Visibility = Visibility.Collapsed;
            }
        }

        private void SetButtons(MessageBoxButton buttons)
        {
            ButtonPanel.Children.Clear();

            switch (buttons)
            {
                case MessageBoxButton.OK:
                    AddButton("OK", MessageBoxResult.OK, true, false);
                    break;

                case MessageBoxButton.OKCancel:
                    AddButton("Cancel", MessageBoxResult.Cancel, false, true);
                    AddButton("OK", MessageBoxResult.OK, true, false);
                    break;

                case MessageBoxButton.YesNo:
                    AddButton("No", MessageBoxResult.No, false, false);
                    AddButton("Yes", MessageBoxResult.Yes, true, false);
                    break;

                case MessageBoxButton.YesNoCancel:
                    AddButton("Cancel", MessageBoxResult.Cancel, false, true);
                    AddButton("No", MessageBoxResult.No, false, false);
                    AddButton("Yes", MessageBoxResult.Yes, true, false);
                    break;
            }
        }

        private void AddButton(string text, MessageBoxResult result, bool isPrimary, bool isCancel)
        {
            var button = new Button
            {
                Content = text,
                MinWidth = 80,
                Height = 28,
                Margin = new Thickness(8, 0, 0, 0),
                Foreground = Brushes.White,
                BorderThickness = new Thickness(1),
                Cursor = Cursors.Hand,
                IsCancel = isCancel
            };

            if (isPrimary)
            {
                button.Background = new SolidColorBrush(Color.FromRgb(0x4A, 0x6A, 0x4A)); // Green
                button.BorderBrush = new SolidColorBrush(Color.FromRgb(0x5A, 0x8A, 0x5A));
                button.IsDefault = true;
            }
            else
            {
                button.Background = new SolidColorBrush(Color.FromRgb(0x3A, 0x3A, 0x3A));
                button.BorderBrush = new SolidColorBrush(Color.FromRgb(0x5A, 0x5A, 0x5A));
            }

            button.Click += (s, e) =>
            {
                _result = result;
                DialogResult = result != MessageBoxResult.Cancel && result != MessageBoxResult.No;
                Close();
            };

            ButtonPanel.Children.Add(button);
        }

        private void OnTitleBarMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                DragMove();
            }
        }

        private void OnCloseClick(object sender, RoutedEventArgs e)
        {
            _result = MessageBoxResult.Cancel;
            DialogResult = false;
            Close();
        }

        private void OnKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Escape)
            {
                _result = MessageBoxResult.Cancel;
                DialogResult = false;
                Close();
            }
        }
    }
}
