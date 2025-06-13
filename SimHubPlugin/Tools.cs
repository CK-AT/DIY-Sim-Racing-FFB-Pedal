
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
}
