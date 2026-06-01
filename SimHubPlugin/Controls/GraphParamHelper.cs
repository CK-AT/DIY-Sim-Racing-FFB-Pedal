using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using DiyFfb.GraphEditor;

namespace DiyFfb.Controls
{
    /// <summary>
    /// Shared graph parameter UI infrastructure for WPF controls that host a
    /// GraphParamsPanel. Eliminates duplicated slider/label generation, event
    /// handling, and formatting code across FlightPedalsConfigControl and
    /// FlightStickConfigControl.
    /// </summary>
    internal sealed class GraphParamHelper
    {
        private readonly StackPanel _panel;
        private readonly Func<DiyFfbPlugin> _getPlugin;
        private readonly Func<string> _getGroupFilter;
        private readonly string _logPrefix;
        private readonly Dispatcher _dispatcher;

        private readonly Dictionary<string, FrameworkElement> _controls = new Dictionary<string, FrameworkElement>();
        private readonly Dictionary<string, Label> _labels = new Dictionary<string, Label>();
        private bool _isUpdating;

        public GraphParamHelper(
            StackPanel panel,
            Func<DiyFfbPlugin> getPlugin,
            Func<string> getGroupFilter,
            string logPrefix,
            Dispatcher dispatcher)
        {
            _panel = panel;
            _getPlugin = getPlugin;
            _getGroupFilter = getGroupFilter;
            _logPrefix = logPrefix;
            _dispatcher = dispatcher;
        }

        /// <summary>
        /// Subscribe to plugin graph parameter events.
        /// </summary>
        public void Subscribe()
        {
            var plugin = _getPlugin();
            if (plugin != null)
            {
                plugin.ActiveGraphChanged += OnActiveGraphChanged;
                plugin.GraphParamChanged += OnGraphParamChanged;
                plugin.ParamMutesCleared += OnParamMutesCleared;
            }
        }

        /// <summary>
        /// Unsubscribe from plugin graph parameter events.
        /// </summary>
        public void Unsubscribe()
        {
            var plugin = _getPlugin();
            if (plugin != null)
            {
                plugin.ActiveGraphChanged -= OnActiveGraphChanged;
                plugin.GraphParamChanged -= OnGraphParamChanged;
                plugin.ParamMutesCleared -= OnParamMutesCleared;
            }
        }

        private void OnParamMutesCleared(object sender, EventArgs e)
        {
            // Bulk-cleared by profile/vehicle/user switch. Rebuild so checkboxes
            // reflect the new state. Per-click toggles use ParamMuteChanged
            // (not subscribed here — the click already updates its own UI).
            _dispatcher.Invoke(new Action(Refresh));
        }

        /// <summary>
        /// Rebuild all graph parameter controls in the panel.
        /// Call from SetGui and SwitchFunction.
        /// </summary>
        public void Refresh()
        {
            try
            {
                _panel.Children.Clear();
                _controls.Clear();
                _labels.Clear();

                var plugin = _getPlugin();
                if (plugin == null)
                {
                    return;
                }

                var allParams = plugin.GetActiveGraphParams();
                if (allParams == null || allParams.Count == 0)
                {
                    return;
                }

                string groupFilter = _getGroupFilter();
                var orderedNames = plugin.GetActiveGraphParamOrder();
                var filteredParams = allParams.Values
                    .Where(p => MatchesGroup(p.Ui?.Group, groupFilter))
                    .ToList();
                var orderedParams = OrderParamsByGraph(orderedNames, filteredParams);

                foreach (var param in orderedParams)
                {
                    double currentValue = plugin.GetGraphParamValue(param.Name);

                    var panel = new StackPanel
                    {
                        Width = 400,
                        Height = 40,
                        Orientation = Orientation.Vertical,
                        Background = null
                    };

                    var label = new Label
                    {
                        Foreground = Brushes.White,
                        FontSize = 10,
                        FontFamily = new FontFamily("Arial"),
                        HorizontalAlignment = HorizontalAlignment.Left,
                        VerticalAlignment = VerticalAlignment.Top,
                        Content = FormatParamLabel(param, currentValue),
                        Padding = new Thickness(0, 0, 0, 8)
                    };

                    var muteCheckbox = GraphParamControlBuilder.BuildMuteCheckbox(
                        param,
                        plugin.IsParamMuted(param.Name),
                        muted => plugin.SetParamMuted(param.Name, muted));

                    double controlWidth = muteCheckbox != null ? 370.0 : 400.0;
                    var control = GraphParamControlBuilder.BuildControl(
                        param,
                        value =>
                        {
                            if (!_isUpdating)
                            {
                                try
                                {
                                    _isUpdating = true;
                                    plugin.SetGraphParamValue(param.Name, value);
                                    if (_labels.TryGetValue(param.Name, out var lbl))
                                    {
                                        lbl.Content = FormatParamLabel(param, value);
                                    }
                                }
                                finally
                                {
                                    _isUpdating = false;
                                }
                            }
                        },
                        width: controlWidth,
                        initialValue: currentValue
                    );

                    panel.Children.Add(label);
                    if (muteCheckbox != null)
                    {
                        var row = new StackPanel
                        {
                            Orientation = Orientation.Horizontal,
                            HorizontalAlignment = HorizontalAlignment.Left
                        };
                        row.Children.Add(control);
                        row.Children.Add(muteCheckbox);
                        panel.Children.Add(row);
                    }
                    else
                    {
                        panel.Children.Add(control);
                    }
                    _panel.Children.Add(panel);
                    _controls[param.Name] = control;
                    _labels[param.Name] = label;
                }
            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Error($"[{_logPrefix}] RefreshGraphParams failed: {ex.Message}", ex);
            }
        }

        private void OnActiveGraphChanged(object sender, EventArgs e)
        {
            _dispatcher.Invoke(new Action(Refresh));
        }

        private void OnGraphParamChanged(object sender, GraphParamChangedEventArgs e)
        {
            if (_isUpdating)
            {
                return;
            }

            _dispatcher.Invoke(() =>
            {
                _isUpdating = true;
                try
                {
                    if (_labels.TryGetValue(e.ParamName, out var label))
                    {
                        var allParams = _getPlugin()?.GetActiveGraphParams();
                        if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                        {
                            label.Content = FormatParamLabel(param, e.Value);
                        }
                    }

                    if (_controls.TryGetValue(e.ParamName, out var control))
                    {
                        if (control is Slider slider)
                        {
                            slider.Value = e.Value;
                        }
                        else if (control is TextBox textBox)
                        {
                            int precision = 3;
                            var allParams = _getPlugin()?.GetActiveGraphParams();
                            if (allParams != null && allParams.TryGetValue(e.ParamName, out var param))
                            {
                                precision = param.Ui?.Precision ?? 3;
                            }
                            textBox.Text = e.Value.ToString($"F{precision}");
                        }
                    }
                }
                finally
                {
                    _isUpdating = false;
                }
            });
        }

        internal static string FormatParamLabel(GraphParam param, double currentValue)
        {
            string label = param.Ui?.Label ?? param.Name;
            int precision = param.Ui?.Precision ?? 3;
            string valueStr = currentValue.ToString($"F{precision}");

            if (!string.IsNullOrWhiteSpace(param.Ui?.Units))
            {
                return $"{label}: {valueStr}{param.Ui.Units}";
            }
            else
            {
                return $"{label}: {valueStr}";
            }
        }

        internal static List<GraphParam> OrderParamsByGraph(IReadOnlyList<string> orderedNames, IEnumerable<GraphParam> parameters)
        {
            var map = new Dictionary<string, GraphParam>(StringComparer.OrdinalIgnoreCase);
            foreach (var param in parameters)
            {
                if (!string.IsNullOrWhiteSpace(param?.Name))
                {
                    map[param.Name] = param;
                }
            }

            var ordered = new List<GraphParam>();
            if (orderedNames != null)
            {
                foreach (var name in orderedNames)
                {
                    if (map.TryGetValue(name, out var param))
                    {
                        ordered.Add(param);
                        map.Remove(name);
                    }
                }
            }

            ordered.AddRange(map.Values.OrderBy(p => p.Ui?.Label ?? p.Name));
            return ordered;
        }

        internal static bool MatchesGroup(string paramGroup, string filter)
        {
            if (string.IsNullOrWhiteSpace(filter))
                return false;
            if (string.IsNullOrWhiteSpace(paramGroup))
                return false;
            return paramGroup.StartsWith(filter, StringComparison.OrdinalIgnoreCase);
        }
    }
}
