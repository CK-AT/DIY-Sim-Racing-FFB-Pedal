using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using DiyFfb.Helpers;
using DiyFfb.TieredConfig;

namespace DiyFfb.Controls
{
    public partial class OverrideReviewDialog : Window
    {
        private readonly TieredConfigOrchestrator _orchestrator;
        private List<OverrideReviewItem> _items;

        public OverrideReviewDialog(TieredConfigOrchestrator orchestrator)
        {
            InitializeComponent();
            SourceInitialized += (s, e) => DarkTitleBar.Enable(this);
            _orchestrator = orchestrator;
            RefreshItems();
        }

        private void RefreshItems()
        {
            _items = _orchestrator.GetAllActiveOverrides();
            GridOverrides.ItemsSource = _items;

            if (_items.Count == 0)
            {
                TextNoOverrides.Visibility = Visibility.Visible;
                GridOverrides.Visibility = Visibility.Collapsed;
                ButtonClearAllUser.IsEnabled = false;
                ButtonClearAllProfile.IsEnabled = false;
                TextStatus.Text = "No overrides are active.";
            }
            else
            {
                TextNoOverrides.Visibility = Visibility.Collapsed;
                GridOverrides.Visibility = Visibility.Visible;

                int userCount = _items.Count(i => i.CurrentLayer == ConfigLayer.User);
                int profileCount = _items.Count(i => i.CurrentLayer == ConfigLayer.Profile);
                int funcCount = _items.Select(i => i.FunctionId).Distinct().Count();

                ButtonClearAllUser.IsEnabled = userCount > 0;
                ButtonClearAllProfile.IsEnabled = profileCount > 0;

                TextStatus.Text = $"{_items.Count} override(s) across {funcCount} function(s)";
            }
        }

        private void OnMoveClick(object sender, RoutedEventArgs e)
        {
            var item = (OverrideReviewItem)((Button)sender).DataContext;
            if (item.CanMoveToUser)
            {
                _orchestrator.RerouteFunctionOverrideField(
                    item.FunctionId, item.FieldPath,
                    ConfigLayer.Profile, ConfigLayer.User);
            }
            else if (item.CanMoveToProfile)
            {
                _orchestrator.RerouteFunctionOverrideField(
                    item.FunctionId, item.FieldPath,
                    ConfigLayer.User, ConfigLayer.Profile);
            }
            RefreshItems();
        }

        private void OnBakeClick(object sender, RoutedEventArgs e)
        {
            var item = (OverrideReviewItem)((Button)sender).DataContext;
            _orchestrator.BakeFieldToBaseline(item.FunctionId, item.FieldPath);
            RefreshItems();
        }

        private void OnClearClick(object sender, RoutedEventArgs e)
        {
            var item = (OverrideReviewItem)((Button)sender).DataContext;
            _orchestrator.ClearFunctionOverrideField(item.FunctionId, item.FieldPath, item.CurrentLayer);
            RefreshItems();
        }

        private void OnClearAllUserClick(object sender, RoutedEventArgs e)
        {
            var userItems = _items.Where(i => i.CurrentLayer == ConfigLayer.User).ToList();
            if (userItems.Count == 0) return;

            var result = ThemedMessageBox.Show(
                $"Clear all {userItems.Count} user override(s)?",
                "Clear All User Overrides",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);

            if (result == MessageBoxResult.Yes)
            {
                foreach (var item in userItems)
                    _orchestrator.ClearFunctionOverrideField(item.FunctionId, item.FieldPath, ConfigLayer.User);
                RefreshItems();
            }
        }

        private void OnClearAllProfileClick(object sender, RoutedEventArgs e)
        {
            var profileItems = _items.Where(i => i.CurrentLayer == ConfigLayer.Profile).ToList();
            if (profileItems.Count == 0) return;

            var result = ThemedMessageBox.Show(
                $"Clear all {profileItems.Count} profile override(s)?",
                "Clear All Profile Overrides",
                MessageBoxButton.YesNo,
                MessageBoxImage.Warning);

            if (result == MessageBoxResult.Yes)
            {
                foreach (var item in profileItems)
                    _orchestrator.ClearFunctionOverrideField(item.FunctionId, item.FieldPath, ConfigLayer.Profile);
                RefreshItems();
            }
        }

        private void OnCloseClick(object sender, RoutedEventArgs e)
        {
            Close();
        }
    }
}
