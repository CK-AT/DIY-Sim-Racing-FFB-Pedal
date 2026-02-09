using System;
using MahApps.Metro.Controls;

namespace DiyFfb.Controls
{
    /// <summary>
    /// Shared kinematic bounds logic for range sliders. Extracts the common
    /// pattern from FlightPedalsConfigControl, FlightStickConfigControl,
    /// and SplineForceCurve.
    /// </summary>
    internal static class KinematicBoundsHelper
    {
        /// <summary>
        /// Computes travel range bounds from ESP32 kinematic parameters.
        /// Returns false if the range is degenerate (e.g., uninitialized ESP32 data).
        /// </summary>
        public static bool TryGetTravelBounds(
            KinematicParameters parameters,
            out double boundsMin,
            out double boundsMax)
        {
            double min = parameters.ContactPointPosMinAbs / 10.0;
            double max = parameters.ContactPointPosMaxAbs / 10.0;
            boundsMin = Math.Min(min, max);
            boundsMax = Math.Max(min, max);
            return boundsMin < boundsMax;
        }

        /// <summary>
        /// Sets slider bounds and restores config values to counteract WPF clamping.
        /// Caller must wrap this in an is_updating guard with ContextIdle deferred clear.
        /// </summary>
        public static void ApplyBoundsToSlider(
            RangeSlider slider,
            double boundsMin,
            double boundsMax,
            double configLower,
            double configUpper)
        {
            slider.Minimum = boundsMin;
            slider.Maximum = boundsMax;
            slider.LowerValue = configLower;
            slider.UpperValue = configUpper;
        }
    }
}
