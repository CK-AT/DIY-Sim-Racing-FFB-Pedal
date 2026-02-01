using System;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Windows;
using DiyFfb.Helpers;

namespace DiyFfb.GraphEditor
{
    public partial class GraphParamUiDialog : Window
    {
        private readonly GraphParam _param;
        private readonly ObservableCollection<GraphParamOptionItem> _options = new ObservableCollection<GraphParamOptionItem>();

        public GraphParamUiDialog(GraphParam param, string[] widgetOptions)
        {
            _param = param ?? throw new ArgumentNullException(nameof(param));
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);
            ComboWidget.ItemsSource = widgetOptions ?? Array.Empty<string>();

            var ui = param.Ui ?? new GraphParamUi();
            TextHeader.Text = $"Param UI: {param.Name}";
            ComboWidget.SelectedItem = string.IsNullOrWhiteSpace(ui.Widget) ? (widgetOptions?.Length > 0 ? widgetOptions[0] : "") : ui.Widget;
            EditLabel.Text = ui.Label ?? "";
            EditGroup.Text = ui.Group ?? "";
            EditUnits.Text = ui.Units ?? "";
            EditStep.Text = ui.Step.HasValue ? ui.Step.Value.ToString("F3", CultureInfo.InvariantCulture) : "";
            EditPrecision.Text = ui.Precision.HasValue ? ui.Precision.Value.ToString(CultureInfo.InvariantCulture) : "";
            CheckLogScale.IsChecked = ui.LogScale;
            EditDefault.Text = param.DefaultValue.ToString("F3", CultureInfo.InvariantCulture);
            EditMin.Text = param.Min.ToString("F3", CultureInfo.InvariantCulture);
            EditMax.Text = param.Max.ToString("F3", CultureInfo.InvariantCulture);

            foreach (var option in ui.Options)
            {
                _options.Add(new GraphParamOptionItem { Value = option.Value, Label = option.Label });
            }
            GridOptions.ItemsSource = _options;
        }

        private void ButtonAddOption_Click(object sender, RoutedEventArgs e)
        {
            _options.Add(new GraphParamOptionItem());
        }

        private void ButtonRemoveOption_Click(object sender, RoutedEventArgs e)
        {
            if (GridOptions.SelectedItem is GraphParamOptionItem selected)
            {
                _options.Remove(selected);
            }
        }

        private void ButtonCancel_Click(object sender, RoutedEventArgs e)
        {
            DialogResult = false;
        }

        private void ButtonSave_Click(object sender, RoutedEventArgs e)
        {
            var ui = _param.Ui ?? new GraphParamUi();
            ui.Widget = (ComboWidget.SelectedItem as string) ?? "";
            ui.Label = EditLabel.Text?.Trim() ?? "";
            ui.Group = EditGroup.Text?.Trim() ?? "";
            ui.Units = EditUnits.Text?.Trim() ?? "";
            ui.Step = ParseNullableDouble(EditStep.Text);
            ui.Precision = ParseNullableInt(EditPrecision.Text);
            ui.LogScale = CheckLogScale.IsChecked == true;

            ui.Options.Clear();
            foreach (var option in _options)
            {
                if (string.IsNullOrWhiteSpace(option.Value) && string.IsNullOrWhiteSpace(option.Label))
                {
                    continue;
                }
                ui.Options.Add(new GraphParamOption
                {
                    Value = option.Value ?? "",
                    Label = option.Label ?? ""
                });
            }
            _param.Ui = ui;
            ApplyParamRange();
            DialogResult = true;
        }

        private void ApplyParamRange()
        {
            double? defaultValue = ParseNullableDouble(EditDefault.Text);
            double? minValue = ParseNullableDouble(EditMin.Text);
            double? maxValue = ParseNullableDouble(EditMax.Text);

            if (minValue.HasValue)
            {
                _param.Min = minValue.Value;
            }
            if (maxValue.HasValue)
            {
                _param.Max = maxValue.Value;
            }
            if (defaultValue.HasValue)
            {
                _param.DefaultValue = defaultValue.Value;
            }
        }

        private static double? ParseNullableDouble(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
            {
                return null;
            }
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }
            return null;
        }

        private static int? ParseNullableInt(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
            {
                return null;
            }
            if (int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var value))
            {
                return value;
            }
            return null;
        }

        private sealed class GraphParamOptionItem
        {
            public string Value { get; set; } = "";
            public string Label { get; set; } = "";
        }
    }
}
