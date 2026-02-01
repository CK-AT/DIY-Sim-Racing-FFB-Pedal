
using System;
using System.Globalization;
using System.Text.RegularExpressions;
using System.Windows.Controls;

namespace DiyFfb
{
    internal class Tools
    {
        public static double Normalize(double value, double min, double max)
        {
            double range = (max - min);
            if (range < 0.0000001)
            {
                return 0;
            }
            if (value <= min)
            {
                return 0;
            }
            if (value >= max)
            {
                return 1;
            }
            return (value - min) / range;
        }

        public static bool TryComputeMarkerX(double value, double lower, double upper, double min, double max, double width, out double x)
        {
            x = 0.0;
            if (width <= 0.0)
            {
                return false;
            }

            if (min > max)
            {
                double swap = min;
                min = max;
                max = swap;
            }
            if (lower > upper)
            {
                double swap = lower;
                lower = upper;
                upper = swap;
            }
            if (max - min <= 0.0)
            {
                return false;
            }

            double lowerNorm = Normalize(lower, min, max);
            double upperNorm = Normalize(upper, min, max);
            if (upperNorm < lowerNorm)
            {
                double swap = lowerNorm;
                lowerNorm = upperNorm;
                upperNorm = swap;
            }

            double selectedWidth = (upperNorm - lowerNorm) * width;
            double valueNorm = Normalize(value, lower, upper);
            x = (lowerNorm * width) + (valueNorm * selectedWidth);
            return true;
        }

        public static bool TryAutoTuneLoadGain(double axisForceAbs, double loadForceAbs, double currentGain, double minGainAbs, double maxGainAbs,
                                               double ratioLow, double ratioHigh, double gainStep, double minForceAbs,
                                               out double updatedGain)
        {
            updatedGain = currentGain;
            if (axisForceAbs < minForceAbs)
            {
                return false;
            }

            double ratio = loadForceAbs / axisForceAbs;
            double sign = Math.Sign(currentGain);
            if (sign == 0.0)
            {
                sign = 1.0;
            }
            double gainAbs = Math.Abs(currentGain);
            if (ratio > ratioHigh)
            {
                gainAbs = Math.Max(minGainAbs, gainAbs - gainStep);
            }
            else if (ratio < ratioLow)
            {
                gainAbs = Math.Min(maxGainAbs, gainAbs + gainStep);
            }
            else
            {
                return false;
            }

            updatedGain = sign * gainAbs;
            return true;
        }
    }

    internal static class TextBoxExtension
    {
        public static void SetTextWithoutEvent(this TextBox textbox, string text)
        {
            textbox.GetType().GetProperty("Text").SetValue(textbox, text, null);
        }
    }

    internal static class StringExtensions
    {
        public static string ConstCaseToTitleCaseSentence(this string title)
        {
            return new CultureInfo("en").TextInfo.ToTitleCase(title.ToLower().Replace("_", " "));
        }
        public static string CamelCaseToTitleCase(this string str)
        {
            return Regex.Replace(str, "[a-z][A-Z]", m => $"{m.Value[0]} {m.Value[1]}");
        }
    }
}
