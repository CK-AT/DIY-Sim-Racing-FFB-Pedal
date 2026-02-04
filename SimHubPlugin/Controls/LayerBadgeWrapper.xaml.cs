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
    /// which layer (User/Profile/Hardware) the current value comes from.
    /// </summary>
    public partial class LayerBadgeWrapper : ContentControl
    {
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
            // Template will be loaded from XAML
        }

        public override void OnApplyTemplate()
        {
            base.OnApplyTemplate();

            // Get template parts
            _badge = GetTemplateChild("PART_Badge") as Border;
            _badgeText = GetTemplateChild("PART_BadgeText") as TextBlock;

            // Set up tooltip
            if (_badge != null)
            {
                _badge.MouseEnter += OnBadgeMouseEnter;
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
                _badge.Visibility = Visibility.Collapsed;
                return;
            }

            // Get field definition from registry
            var field = OverrideFieldRegistry.GetField(FieldPath);
            if (field == null)
            {
                _badge.Visibility = Visibility.Collapsed;
                return;
            }

            // Get source layer from plugin's ConfigLayerProvider
            var sourceLayer = GetSourceLayer();

            if (sourceLayer == null || sourceLayer == ConfigLayer.Hardware)
            {
                // No override - hide badge
                _badge.Visibility = Visibility.Collapsed;
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
            var layerProvider = Plugin.CreateConfigLayerProvider();
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
                case ConfigLayer.Hardware:
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
            var sourceLayer = GetSourceLayer();
            if (sourceLayer == null)
            {
                ToolTip = null;
                return;
            }

            var field = OverrideFieldRegistry.GetField(FieldPath);
            if (field == null)
            {
                ToolTip = ConfigLayerProvider.GetLayerTooltip(sourceLayer);
                return;
            }

            // Simple tooltip showing field name and layer source
            ToolTip = $"{field.DisplayName}\n{ConfigLayerProvider.GetLayerTooltip(sourceLayer)}";
        }
    }
}
