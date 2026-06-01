using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using DiyFfb.GraphEditor;

namespace DiyFfb
{
    /// <summary>
    /// Utility class for building WPF controls from graph parameter definitions.
    /// Used by both the graph editor and function configuration panels.
    /// </summary>
    public static class GraphParamControlBuilder
    {
        private const double DefaultParamControlHeight = 20.0;

        /// <summary>
        /// Builds an optional mute checkbox for params whose GraphParamUi has
        /// MuteValue set. Returns null when the param has no MuteValue (no UI).
        /// The checkbox reflects <paramref name="initialMuted"/> and invokes
        /// <paramref name="onMuteChanged"/> when toggled.
        /// </summary>
        public static CheckBox BuildMuteCheckbox(
            GraphParam param,
            bool initialMuted,
            Action<bool> onMuteChanged)
        {
            if (param?.Ui?.MuteValue.HasValue != true) return null;
            double muteValue = param.Ui.MuteValue.Value;
            var check = new CheckBox
            {
                Foreground = Brushes.LightGray,
                VerticalAlignment = VerticalAlignment.Center,
                Margin = new Thickness(6, 0, 0, 0),
                IsChecked = initialMuted,
                ToolTip = $"Mute (substitute {muteValue.ToString("0.###", CultureInfo.InvariantCulture)} during evaluation)"
            };
            check.Checked += (_, __) => onMuteChanged?.Invoke(true);
            check.Unchecked += (_, __) => onMuteChanged?.Invoke(false);
            return check;
        }

        public static FrameworkElement BuildControl(
            GraphParam param,
            Action<double> onValueChanged,
            double width = 120,
            double? initialValue = null)
        {
            if (param == null)
            {
                throw new ArgumentNullException(nameof(param));
            }

            double currentValue = initialValue ?? param.DefaultValue;
            GraphParamUi ui = param.Ui;
            string widget = (ui?.Widget ?? "").Trim().ToLowerInvariant();
            if (string.IsNullOrWhiteSpace(widget))
            {
                widget = "slider";
            }

            if (widget == "checkbox")
            {
                return BuildCheckbox(currentValue, onValueChanged);
            }

            if (widget == "enum")
            {
                return BuildEnum(ui, currentValue, onValueChanged, width);
            }

            if (widget == "slider" || widget == "knob")
            {
                return BuildSlider(param, ui, currentValue, onValueChanged, width, widget == "knob");
            }

            // Default: text box
            return BuildTextBox(currentValue, onValueChanged, width);
        }

        private static CheckBox BuildCheckbox(double currentValue, Action<double> onValueChanged)
        {
            var check = new CheckBox
            {
                Foreground = Brushes.LightGray,
                VerticalAlignment = VerticalAlignment.Center,
                IsChecked = currentValue > 0.5
            };
            check.Checked += (_, __) => onValueChanged?.Invoke(1.0);
            check.Unchecked += (_, __) => onValueChanged?.Invoke(0.0);
            return check;
        }

        private static ComboBox BuildEnum(GraphParamUi ui, double currentValue, Action<double> onValueChanged, double width)
        {
            var combo = new ComboBox
            {
                Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                Foreground = Brushes.White,
                BorderBrush = new SolidColorBrush(Color.FromRgb(74, 74, 74)),
                BorderThickness = new Thickness(1),
                ItemsSource = ui?.Options,
                DisplayMemberPath = "Label",
                SelectedValuePath = "Value",
                Width = width
            };

            combo.SelectedItem = FindOptionForValue(ui?.Options, currentValue);
            combo.SelectionChanged += (_, __) =>
            {
                if (combo.SelectedItem is GraphParamOption option)
                {
                    double value = ParseOptionValue(option.Value, currentValue);
                    onValueChanged?.Invoke(value);
                }
            };
            return combo;
        }

        private static Slider BuildSlider(GraphParam param, GraphParamUi ui, double currentValue,
            Action<double> onValueChanged, double width, bool isKnob)
        {
            GetParamRange(param, out double min, out double max);

            var slider = new Slider
            {
                Minimum = min,
                Maximum = max,
                Value = currentValue,
                Width = width,
                Height = 10,
                HorizontalAlignment = HorizontalAlignment.Left
            };

            // Apply SimHub's slider style
            try
            {
                var style = Application.Current.TryFindResource("SliderStyle_single_H") as Style;
                if (style != null)
                {
                    slider.Style = style;
                }
            }
            catch
            {
                // Fallback if style not found
                slider.Background = new SolidColorBrush(Color.FromRgb(30, 30, 30));
                slider.Foreground = Brushes.White;
            }

            // Set step behavior
            if (ui?.Step.HasValue == true && ui.Step.Value > 0)
            {
                slider.SmallChange = ui.Step.Value;
                slider.TickFrequency = ui.Step.Value;
                slider.IsSnapToTickEnabled = true;
            }
            else
            {
                // Default snap behavior for sliders
                double range = max - min;
                slider.SmallChange = range / 1000.0;
                slider.TickFrequency = range / 1000.0;
                slider.IsSnapToTickEnabled = true;
            }

            if (isKnob)
            {
                slider.IsSnapToTickEnabled = ui?.Step.HasValue == true && ui.Step.Value > 0;
                slider.TickFrequency = ui?.Step.HasValue == true && ui.Step.Value > 0
                    ? ui.Step.Value
                    : (max - min) / 10.0;
                slider.BorderBrush = new SolidColorBrush(Color.FromRgb(90, 90, 120));
                slider.BorderThickness = new Thickness(1);
            }

            slider.ValueChanged += (_, __) => onValueChanged?.Invoke(slider.Value);
            return slider;
        }

        private static TextBox BuildTextBox(double currentValue, Action<double> onValueChanged, double width)
        {
            var text = new TextBox
            {
                Background = new SolidColorBrush(Color.FromRgb(30, 30, 30)),
                Foreground = Brushes.White,
                BorderBrush = new SolidColorBrush(Color.FromRgb(74, 74, 74)),
                BorderThickness = new Thickness(1),
                Padding = new Thickness(2, 0, 2, 0),
                Width = width,
                Height = DefaultParamControlHeight,
                Text = currentValue.ToString("F3", CultureInfo.InvariantCulture)
            };

            text.TextChanged += (_, __) =>
            {
                if (TryParseDouble(text.Text, out var value))
                {
                    onValueChanged?.Invoke(value);
                }
            };

            return text;
        }

        private static void GetParamRange(GraphParam param, out double min, out double max)
        {
            min = param.Min;
            max = param.Max;
            if (Math.Abs(max - min) < 1e-9)
            {
                max = min + 1.0;
            }
        }

        private static GraphParamOption FindOptionForValue(System.Collections.Generic.List<GraphParamOption> options, double value)
        {
            if (options == null || options.Count == 0)
            {
                return null;
            }

            foreach (var option in options)
            {
                double parsed = ParseOptionValue(option.Value, double.NaN);
                if (!double.IsNaN(parsed) && Math.Abs(parsed - value) < 1e-6)
                {
                    return option;
                }
            }

            return options[0];
        }

        private static double ParseOptionValue(string text, double fallback)
        {
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }
            return fallback;
        }

        private static bool TryParseDouble(string text, out double value)
        {
            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) ||
                   double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        }
    }
}
