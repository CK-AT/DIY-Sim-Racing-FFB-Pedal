using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;

namespace DiyFfb.Controls
{
    /// <summary>
    /// Shared badge infrastructure for WPF controls that host LayerBadgeWrapper elements.
    /// Eliminates duplicated badge wiring, refresh, and initialization code across
    /// AutomotivePedalConfigControl, FlightPedalsConfigControl, FlightStickConfigControl,
    /// ShifterConfigControl, and FunctionConfigControl.
    /// </summary>
    internal sealed class BadgeHelper
    {
        private readonly FrameworkElement _root;
        private readonly Func<DiyFfbPlugin> _getPlugin;
        private readonly Func<Function> _getFunction;
        private readonly EventHandler<LayerBadgeWrapper.OverrideClearedEventArgs> _onOverrideCleared;

        public BadgeHelper(
            FrameworkElement root,
            Func<DiyFfbPlugin> getPlugin,
            Func<Function> getFunction,
            EventHandler<LayerBadgeWrapper.OverrideClearedEventArgs> onOverrideCleared)
        {
            _root = root;
            _getPlugin = getPlugin;
            _getFunction = getFunction;
            _onOverrideCleared = onOverrideCleared;
        }

        /// <summary>
        /// Subscribe to plugin events and badge clear events. Call from OnLoaded.
        /// </summary>
        public void Subscribe()
        {
            var plugin = _getPlugin();
            if (plugin != null)
            {
                plugin.ContextChanged += OnContextChanged;
                plugin.OverrideFieldChanged += OnOverrideFieldChanged;
            }

            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(_root))
            {
                wrapper.OverrideCleared += _onOverrideCleared;
            }
        }

        /// <summary>
        /// Unsubscribe from plugin events and badge clear events. Call from OnUnloaded.
        /// </summary>
        public void Unsubscribe()
        {
            var plugin = _getPlugin();
            if (plugin != null)
            {
                plugin.ContextChanged -= OnContextChanged;
                plugin.OverrideFieldChanged -= OnOverrideFieldChanged;
            }

            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(_root))
            {
                wrapper.OverrideCleared -= _onOverrideCleared;
            }
        }

        /// <summary>
        /// Set Plugin and FunctionId on all badge wrappers in the visual tree.
        /// Call from SwitchFunction or after the control is fully rendered.
        /// </summary>
        public void InitializeBadges()
        {
            var plugin = _getPlugin();
            var function = _getFunction();
            if (plugin == null || function == null) return;

            int functionId = (int)function.ID;
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(_root))
            {
                wrapper.Plugin = plugin;
                wrapper.FunctionId = functionId;
            }
        }

        /// <summary>
        /// Refresh all badge wrappers in the visual tree.
        /// </summary>
        public void RefreshAllBadges()
        {
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(_root))
            {
                wrapper.UpdateBadge();
            }
        }

        /// <summary>
        /// Refresh only the badge wrapper matching the given field path.
        /// </summary>
        public void RefreshBadgeForField(string fieldPath)
        {
            foreach (var wrapper in FindVisualChildren<LayerBadgeWrapper>(_root))
            {
                if (wrapper.FieldPath == fieldPath)
                {
                    wrapper.UpdateBadge();
                }
            }
        }

        private void OnContextChanged(object sender, EventArgs e)
        {
            RefreshAllBadges();
        }

        private void OnOverrideFieldChanged(object sender, OverrideFieldChangedEventArgs e)
        {
            var function = _getFunction();
            if (function != null && e.FunctionId == (int)function.ID)
            {
                RefreshBadgeForField(e.FieldPath);
            }
        }

        /// <summary>
        /// Recursively find all children of a given type in the visual tree.
        /// </summary>
        public static IEnumerable<T> FindVisualChildren<T>(DependencyObject parent) where T : DependencyObject
        {
            if (parent == null) yield break;
            for (int i = 0; i < VisualTreeHelper.GetChildrenCount(parent); i++)
            {
                var child = VisualTreeHelper.GetChild(parent, i);
                if (child is T t) yield return t;
                foreach (var descendant in FindVisualChildren<T>(child)) yield return descendant;
            }
        }
    }
}
