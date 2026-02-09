using System;
using System.Windows;
using System.Windows.Controls;
using MahApps.Metro.Controls;

namespace DiyFfb.Controls
{
    /// <summary>
    /// Shared axis state tracking and travel marker display for range slider controls.
    /// Eliminates duplicated OnAxisStateUpdate, UpdateTrimCenter, and UpdateTravelMarkers
    /// code from FlightPedalsConfigControl and FlightStickConfigControl.
    /// </summary>
    internal sealed class TravelDisplayHelper
    {
        private readonly Canvas _canvas;
        private readonly FrameworkElement _axisPositionMarker;
        private readonly FrameworkElement _trimCenterMarker;
        private readonly RangeSlider _slider;
        private readonly Func<double> _getPosMin;
        private readonly Func<double> _getPosMax;

        // Axis state
        private double _latestAxisPosition;
        private bool _hasAxisPosition;
        private double _latestTrimCenter;
        private bool _hasTrimCenter;

        public double LatestAxisForce { get; private set; }

        public TravelDisplayHelper(
            Canvas canvas,
            FrameworkElement axisPositionMarker,
            FrameworkElement trimCenterMarker,
            RangeSlider slider,
            Func<double> getPosMin,
            Func<double> getPosMax)
        {
            _canvas = canvas;
            _axisPositionMarker = axisPositionMarker;
            _trimCenterMarker = trimCenterMarker;
            _slider = slider;
            _getPosMin = getPosMin;
            _getPosMax = getPosMax;
        }

        /// <summary>
        /// Filters the axis state update and stores position/force if it matches
        /// the function's primary linked axis. Returns true if state was updated.
        /// </summary>
        public bool TryUpdateAxisState(FunctionConfig functionConfig, AxisState axisState)
        {
            if (functionConfig?.Base == null || functionConfig.Base.LinkedAxes.Count == 0)
                return false;

            AxisID primaryAxis = functionConfig.Base.LinkedAxes[0];
            if (primaryAxis == AxisID.AxisUndefined || (primaryAxis & AxisID.Mask) != axisState.AxisId)
                return false;

            _latestAxisPosition = axisState.Position;
            _hasAxisPosition = true;
            LatestAxisForce = axisState.Force;
            return true;
        }

        /// <summary>
        /// Queries the plugin for the current graph trim offset.
        /// </summary>
        public void UpdateTrimCenter(DiyFfbPlugin plugin, FunctionID functionId)
        {
            float trimMm = 0.0f;
            _hasTrimCenter = plugin != null && plugin.TryGetGraphTrimOffset(functionId, out trimMm);
            if (_hasTrimCenter)
                _latestTrimCenter = trimMm;
        }

        /// <summary>
        /// Positions the axis position and trim center markers on the travel canvas.
        /// </summary>
        public void UpdateTravelMarkers()
        {
            if (_canvas == null || _axisPositionMarker == null || _trimCenterMarker == null)
                return;

            double width = _canvas.ActualWidth;
            if (width <= 0.0)
                return;

            double posMin = _slider?.LowerValue ?? _getPosMin();
            double posMax = _slider?.UpperValue ?? _getPosMax();
            double rangeMin = _slider?.Minimum ?? _getPosMin();
            double rangeMax = _slider?.Maximum ?? _getPosMax();

            if (_hasAxisPosition)
            {
                if (Tools.TryComputeMarkerX(_latestAxisPosition, posMin, posMax, rangeMin, rangeMax, width, out double posX))
                    Canvas.SetLeft(_axisPositionMarker, posX - _axisPositionMarker.Width / 2.0);
            }

            if (_hasTrimCenter)
            {
                double center = (posMin + posMax) / 2.0;
                double trimPos = center + _latestTrimCenter;
                if (Tools.TryComputeMarkerX(trimPos, posMin, posMax, rangeMin, rangeMax, width, out double trimX))
                    Canvas.SetLeft(_trimCenterMarker, trimX - _trimCenterMarker.Width / 2.0);
            }
        }
    }
}
