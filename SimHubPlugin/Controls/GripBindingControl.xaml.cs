using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;

namespace DiyFfb.Controls
{
    public partial class GripBindingControl : UserControl
    {
        private DiyFfbPluginSettings _settings;
        private ButtonInputReader _reader;
        private string _activeBindSignal;
        private DispatcherTimer _bindTimer;
        private int _bindTimeoutTicks;

        public GripBindingControl()
        {
            InitializeComponent();
        }

        public void Initialize(DiyFfbPluginSettings settings, ButtonInputReader reader)
        {
            _settings = settings;
            _reader = reader;
            if (_settings.GripButtonBindings == null)
                _settings.GripButtonBindings = new Dictionary<string, ButtonBinding>();
            RefreshBindingList();
        }

        private void RefreshBindingList()
        {
            if (_settings == null) return;

            var items = GraphSignalCatalog.GripSignalNames.Select(signal =>
            {
                _settings.GripButtonBindings.TryGetValue(signal, out var binding);
                bool isBound = binding != null && binding.Type != BindingType.None;
                bool isListening = _activeBindSignal == signal;

                return new BindingRow
                {
                    SignalName = signal,
                    Label = SignalToLabel(signal),
                    DisplayName = isListening ? "Press a button or key..."
                        : isBound ? binding.DisplayName : "(not bound)",
                    DisplayColor = isBound ? Brushes.White : Brushes.Gray,
                    BindButtonText = isListening ? "Cancel" : "Bind...",
                    ClearVisibility = isBound && !isListening ? Visibility.Visible : Visibility.Collapsed
                };
            }).ToList();

            BindingList.ItemsSource = items;
        }

        private static string SignalToLabel(string signal)
        {
            // "Grip.TrimHat.Up" → "Trim Hat Up"
            var parts = signal.Split('.');
            if (parts.Length < 2) return signal;
            var name = string.Join(".", parts.Skip(1));
            // Insert spaces before capitals
            var result = new System.Text.StringBuilder();
            foreach (char c in name)
            {
                if (c == '.') { result.Append(' '); continue; }
                if (char.IsUpper(c) && result.Length > 0 && result[result.Length - 1] != ' ')
                    result.Append(' ');
                result.Append(c);
            }
            return result.ToString();
        }

        private void OnBindClick(object sender, RoutedEventArgs e)
        {
            var signal = (sender as Button)?.Tag as string;
            if (string.IsNullOrEmpty(signal)) return;

            if (_activeBindSignal == signal)
            {
                // Cancel
                StopListening();
                return;
            }

            _activeBindSignal = signal;
            _bindTimeoutTicks = 0;
            RefreshBindingList();

            // Start polling for input
            _bindTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(50) };
            _bindTimer.Tick += BindTimer_Tick;
            _bindTimer.Start();
        }

        private void BindTimer_Tick(object sender, EventArgs e)
        {
            _bindTimeoutTicks++;
            if (_bindTimeoutTicks > 100) // 5 seconds
            {
                StopListening();
                return;
            }

            if (_reader == null) return;

            _reader.Poll();
            var binding = _reader.ScanForPress();
            if (binding != null)
            {
                _settings.GripButtonBindings[_activeBindSignal] = binding;
                StopListening();
            }
        }

        private void StopListening()
        {
            _bindTimer?.Stop();
            _bindTimer = null;
            _activeBindSignal = null;
            RefreshBindingList();
        }

        private void OnClearClick(object sender, RoutedEventArgs e)
        {
            var signal = (sender as Button)?.Tag as string;
            if (string.IsNullOrEmpty(signal)) return;
            _settings.GripButtonBindings.Remove(signal);
            RefreshBindingList();
        }

        private void OnClearAllClick(object sender, RoutedEventArgs e)
        {
            _settings.GripButtonBindings.Clear();
            RefreshBindingList();
        }

        private class BindingRow
        {
            public string SignalName { get; set; }
            public string Label { get; set; }
            public string DisplayName { get; set; }
            public Brush DisplayColor { get; set; }
            public string BindButtonText { get; set; }
            public Visibility ClearVisibility { get; set; }
        }
    }
}
