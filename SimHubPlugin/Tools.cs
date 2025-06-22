
using System.Globalization;
using System.Text.RegularExpressions;
using System.Windows.Controls;

namespace User.PluginSdkDemo
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
