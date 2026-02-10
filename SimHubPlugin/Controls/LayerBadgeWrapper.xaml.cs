using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using DiyFfb.TieredConfig;

namespace DiyFfb.Controls
{
    /// <summary>
    /// A wrapper control that displays layer badges ([U]/[P]) for override fields.
    /// Wraps any content and overlays a badge in the upper-right corner showing
    /// which layer (User/Profile/Baseline) the current value comes from.
    /// </summary>
    public partial class LayerBadgeWrapper : ContentControl
    {
        /// <summary>
        /// Fired when an override is cleared via context menu.
        /// Parent control should update the UI control to show the new value from the baseline.
        /// </summary>
        public event EventHandler<OverrideClearedEventArgs> OverrideCleared;

        /// <summary>
        /// Fired when an override is re-routed to a different layer via context menu.
        /// Parent control should update the UI to reflect the new layer state.
        /// </summary>
        public event EventHandler<OverrideReroutedEventArgs> OverrideRerouted;

        public class OverrideClearedEventArgs : EventArgs
        {
            public string FieldPath { get; set; }
            public ConfigLayer ClearedLayer { get; set; }
        }

        public class OverrideReroutedEventArgs : EventArgs
        {
            public string FieldPath { get; set; }
            public ConfigLayer FromLayer { get; set; }
            public ConfigLayer ToLayer { get; set; }
        }

        static LayerBadgeWrapper()
        {
            // Set default style key to enable template lookup
            DefaultStyleKeyProperty.OverrideMetadata(
                typeof(LayerBadgeWrapper),
                new FrameworkPropertyMetadata(typeof(LayerBadgeWrapper)));
        }

        // Dependency properties for configuration
        public static readonly DependencyProperty FieldPathProperty =
            DependencyProperty.Register(
                nameof(FieldPath),
                typeof(string),
                typeof(LayerBadgeWrapper),
                new PropertyMetadata(null, OnConfigChanged));

        public static readonly DependencyProperty FunctionIdProperty =
            DependencyProperty.Register(
                nameof(FunctionId),
                typeof(int),
                typeof(LayerBadgeWrapper),
                new PropertyMetadata(-1, OnConfigChanged));

        public static readonly DependencyProperty PluginProperty =
            DependencyProperty.Register(
                nameof(Plugin),
                typeof(DiyFfbPlugin),
                typeof(LayerBadgeWrapper),
                new PropertyMetadata(null, OnConfigChanged));

        /// <summary>
        /// Field path (e.g., "output_min" or "static_balance_tuning.enabled")
        /// </summary>
        public string FieldPath
        {
            get => (string)GetValue(FieldPathProperty);
            set => SetValue(FieldPathProperty, value);
        }

        /// <summary>
        /// Function ID for this field
        /// </summary>
        public int FunctionId
        {
            get => (int)GetValue(FunctionIdProperty);
            set => SetValue(FunctionIdProperty, value);
        }

        /// <summary>
        /// Reference to the plugin instance for querying layer information
        /// </summary>
        public DiyFfbPlugin Plugin
        {
            get => (DiyFfbPlugin)GetValue(PluginProperty);
            set => SetValue(PluginProperty, value);
        }

        // UI elements (set by template)
        private Border _badge;
        private TextBlock _badgeText;

        public LayerBadgeWrapper()
        {
            // Load the control template from the resource dictionary
            var resourceDict = new ResourceDictionary
            {
                Source = new Uri("/DiyFfbPlugin;component/Controls/LayerBadgeWrapper.xaml", UriKind.Relative)
            };

            // Apply the style explicitly
            if (resourceDict.Contains(typeof(LayerBadgeWrapper)))
            {
                this.Style = (Style)resourceDict[typeof(LayerBadgeWrapper)];
            }
        }

        public override void OnApplyTemplate()
        {
            base.OnApplyTemplate();

            // Get template parts
            _badge = GetTemplateChild("PART_Badge") as Border;
            _badgeText = GetTemplateChild("PART_BadgeText") as TextBlock;

            // Set up tooltip and context menu
            if (_badge != null)
            {
                _badge.MouseEnter += OnBadgeMouseEnter;
                _badge.MouseRightButtonDown += OnBadgeRightClick;
            }

            UpdateBadge();
        }

        private static void OnConfigChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is LayerBadgeWrapper wrapper)
            {
                wrapper.UpdateBadge();
            }
        }

        /// <summary>
        /// Updates the badge display based on current field and layer state.
        /// Call this when the effective value changes due to profile/user context switch.
        /// </summary>
        public void UpdateBadge()
        {
            if (_badge == null || _badgeText == null || Plugin == null)
                return;

            if (string.IsNullOrEmpty(FieldPath) || FunctionId < 0)
            {
                _badge.Visibility = Visibility.Hidden;
                return;
            }

            // Get field definition from registry
            var field = OverrideFieldRegistry.GetField(FieldPath);
            if (field == null)
            {
                _badge.Visibility = Visibility.Hidden;
                return;
            }

            // Get source layer from plugin's ConfigLayerProvider
            var sourceLayer = GetSourceLayer();

            if (sourceLayer == null || sourceLayer == ConfigLayer.Baseline)
            {
                // No override - hide badge
                _badge.Visibility = Visibility.Hidden;
                return;
            }

            // Show badge
            _badge.Visibility = Visibility.Visible;
            _badgeText.Text = ConfigLayerProvider.GetLayerBadgeText(sourceLayer);

            // Set badge color
            _badge.Background = GetLayerColor(sourceLayer.Value);
        }

        private ConfigLayer? GetSourceLayer()
        {
            if (Plugin == null)
                return null;

            // Create ConfigLayerProvider on demand using plugin methods
            var layerProvider = Plugin.ConfigOrchestrator.CreateConfigLayerProvider();
            return layerProvider.GetFieldSourceLayer(FunctionId, FieldPath);
        }

        private Brush GetLayerColor(ConfigLayer layer)
        {
            switch (layer)
            {
                case ConfigLayer.User:
                    return new SolidColorBrush(Color.FromRgb(0x64, 0xB5, 0xF6)); // Blue
                case ConfigLayer.Profile:
                    return new SolidColorBrush(Color.FromRgb(0x4C, 0xAF, 0x50)); // Green
                case ConfigLayer.Baseline:
                    return new SolidColorBrush(Color.FromRgb(0x75, 0x75, 0x75)); // Gray
                default:
                    return Brushes.Transparent;
            }
        }

        private void OnBadgeMouseEnter(object sender, System.Windows.Input.MouseEventArgs e)
        {
            UpdateTooltip();
        }

        private void UpdateTooltip()
        {
            if (_badge == null)
                return;

            var field = OverrideFieldRegistry.GetField(FieldPath);
            if (field == null || Plugin == null || FunctionId < 0)
            {
                _badge.ToolTip = null;
                return;
            }

            var layerProvider = Plugin.ConfigOrchestrator.CreateConfigLayerProvider();
            var sourceLayer = layerProvider.GetFieldSourceLayer(FunctionId, FieldPath);

            // Build enhanced tooltip showing all layer values
            var tooltip = new System.Text.StringBuilder();
            tooltip.AppendLine(field.DisplayName);
            tooltip.AppendLine("─────────────────────");

            // Get values from all layers
            var userValue = GetLayerValueString(layerProvider, ConfigLayer.User, field);
            var profileValue = GetLayerValueString(layerProvider, ConfigLayer.Profile, field);
            var baselineValue = GetLayerValueString(layerProvider, ConfigLayer.Baseline, field);

            // Display with active indicator
            tooltip.AppendLine($"User:     {userValue}{(sourceLayer == ConfigLayer.User ? " ◄ active" : "")}");
            tooltip.AppendLine($"Profile:  {profileValue}{(sourceLayer == ConfigLayer.Profile ? " ◄ active" : "")}");
            tooltip.AppendLine($"Baseline: {baselineValue}{(sourceLayer == ConfigLayer.Baseline ? " ◄ active" : "")}");

            _badge.ToolTip = tooltip.ToString().TrimEnd();
        }

        private string GetLayerValueString(ConfigLayerProvider layerProvider, ConfigLayer layer, OverrideFieldDefinition field)
        {
            var hasValue = layerProvider.HasFieldValue(FunctionId, FieldPath, layer);
            if (!hasValue)
            {
                return "(not set)";
            }

            // For complex fields, just show "(configured)"
            if (field.FieldType == OverrideFieldType.Complex)
            {
                return "(configured)";
            }

            // For scalar fields, get and format the value
            var value = layerProvider.GetFieldValue(FunctionId, FieldPath, layer);
            if (value == null)
            {
                return "(not set)";
            }

            // Use field's FormatValue if available
            if (field.FormatValue != null)
            {
                return field.FormatValue(value);
            }

            // Default formatting
            if (value is float f)
                return f.ToString("F2");
            if (value is bool b)
                return b ? "Enabled" : "Disabled";

            return value.ToString();
        }

        private void OnBadgeRightClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
                return;

            var field = OverrideFieldRegistry.GetField(FieldPath);
            if (field == null)
                return;

            // Build context menu
            var contextMenu = new ContextMenu();
            var layerProvider = Plugin.ConfigOrchestrator.CreateConfigLayerProvider();

            // Section 1: Layer values (header items, not clickable)
            var userValue = GetLayerValueString(layerProvider, ConfigLayer.User, field);
            var profileValue = GetLayerValueString(layerProvider, ConfigLayer.Profile, field);
            var baselineValue = GetLayerValueString(layerProvider, ConfigLayer.Baseline, field);
            var sourceLayer = layerProvider.GetFieldSourceLayer(FunctionId, FieldPath);

            var userHeader = new MenuItem
            {
                Header = $"{(sourceLayer == ConfigLayer.User ? "✓ " : "  ")}User: {userValue}",
                IsEnabled = false
            };
            var profileHeader = new MenuItem
            {
                Header = $"{(sourceLayer == ConfigLayer.Profile ? "✓ " : "  ")}Profile: {profileValue}",
                IsEnabled = false
            };
            var baselineHeader = new MenuItem
            {
                Header = $"{(sourceLayer == ConfigLayer.Baseline || sourceLayer == null ? "✓ " : "  ")}Baseline: {baselineValue}",
                IsEnabled = false
            };

            contextMenu.Items.Add(userHeader);
            contextMenu.Items.Add(profileHeader);
            contextMenu.Items.Add(baselineHeader);
            contextMenu.Items.Add(new Separator());

            // Section 2: Clear operations
            if (layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.User))
            {
                var clearUser = new MenuItem { Header = "Clear User override" };
                clearUser.Click += (s, args) => OnClearOverride(ConfigLayer.User);
                contextMenu.Items.Add(clearUser);
            }

            if (layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.Profile))
            {
                var clearProfile = new MenuItem { Header = "Clear Profile override" };
                clearProfile.Click += (s, args) => OnClearOverride(ConfigLayer.Profile);
                contextMenu.Items.Add(clearProfile);
            }

            if (contextMenu.Items.Count > 4) // Has clear items
            {
                contextMenu.Items.Add(new Separator());
            }

            // Section 3: Re-route and bake operations
            bool hasUserValue = layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.User);
            bool hasProfileValue = layerProvider.HasFieldValue(FunctionId, FieldPath, ConfigLayer.Profile);

            if (sourceLayer == ConfigLayer.User && !hasProfileValue)
            {
                var moveToProfile = new MenuItem { Header = "Move to Profile" };
                moveToProfile.Click += (s, args) => OnRerouteOverride(ConfigLayer.User, ConfigLayer.Profile);
                contextMenu.Items.Add(moveToProfile);
            }
            else if (sourceLayer == ConfigLayer.Profile && !hasUserValue)
            {
                var moveToUser = new MenuItem { Header = "Move to User" };
                moveToUser.Click += (s, args) => OnRerouteOverride(ConfigLayer.Profile, ConfigLayer.User);
                contextMenu.Items.Add(moveToUser);
            }

            if (sourceLayer == ConfigLayer.User || sourceLayer == ConfigLayer.Profile)
            {
                var saveToBaseline = new MenuItem { Header = "Save to Baseline" };
                saveToBaseline.Click += (s, args) => OnBakeToBaseline();
                contextMenu.Items.Add(saveToBaseline);
            }

            _badge.ContextMenu = contextMenu;
            contextMenu.IsOpen = true;
            e.Handled = true;
        }

        private void OnClearOverride(ConfigLayer layer)
        {
            if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
                return;

            Plugin.ConfigOrchestrator.ClearFunctionOverrideField(FunctionId, FieldPath, layer);
            UpdateBadge();
            UpdateTooltip();

            OverrideCleared?.Invoke(this, new OverrideClearedEventArgs
            {
                FieldPath = FieldPath,
                ClearedLayer = layer
            });
        }

        private void OnRerouteOverride(ConfigLayer fromLayer, ConfigLayer toLayer)
        {
            if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
                return;

            Plugin.ConfigOrchestrator.RerouteFunctionOverrideField(FunctionId, FieldPath, fromLayer, toLayer);
            UpdateBadge();
            UpdateTooltip();

            OverrideRerouted?.Invoke(this, new OverrideReroutedEventArgs
            {
                FieldPath = FieldPath,
                FromLayer = fromLayer,
                ToLayer = toLayer
            });
        }

        private void OnBakeToBaseline()
        {
            if (Plugin == null || FunctionId < 0 || string.IsNullOrEmpty(FieldPath))
                return;

            Plugin.ConfigOrchestrator.BakeFieldToBaseline(FunctionId, FieldPath);
            UpdateBadge();
            UpdateTooltip();

            OverrideCleared?.Invoke(this, new OverrideClearedEventArgs
            {
                FieldPath = FieldPath,
                ClearedLayer = ConfigLayer.Baseline // signals "baked to baseline"
            });
        }
    }
}
