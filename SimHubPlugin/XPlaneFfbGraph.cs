using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace User.PluginSdkDemo
{
    internal static class XPlaneFfbGraph
    {
        public const int DefaultSteps = 60;

        public static float GetMaxIasKts(float vrefKts)
        {
            return Math.Max(40.0f, vrefKts * 2.0f);
        }

        public static void BuildGainCurves(float vrefKts, float springRef, float damperRef, double width, double height,
                                           out PointCollection springPoints, out PointCollection damperPoints,
                                           out float maxIasKts, out float maxGain)
        {
            maxIasKts = GetMaxIasKts(vrefKts);
            maxGain = Math.Max(0.1f, Math.Max(springRef, damperRef)) * XPlaneFfbMath.MaxQScale * 1.1f;

            int steps = DefaultSteps;
            springPoints = new PointCollection(steps + 1);
            damperPoints = new PointCollection(steps + 1);

            for (int i = 0; i <= steps; i++)
            {
                float ias = maxIasKts * i / steps;
                float scale = XPlaneFfbMath.ComputeQScaleFromIasKts(ias, vrefKts);
                float spring = springRef * scale;
                float damper = damperRef * scale;

                double x = (ias / maxIasKts) * width;
                double springY = height - (spring / maxGain) * height;
                double damperY = height - (damper / maxGain) * height;
                springPoints.Add(new Point(x, springY));
                damperPoints.Add(new Point(x, damperY));
            }
        }

        public static void UpdateGainGrid(Canvas canvas, float vrefKts, float maxIasKts, float maxGain)
        {
            if (canvas == null)
            {
                return;
            }

            double width = canvas.Width;
            double height = canvas.Height;
            if (width <= 0.0 || height <= 0.0)
            {
                return;
            }

            for (int i = canvas.Children.Count - 1; i >= 0; i--)
            {
                FrameworkElement element = canvas.Children[i] as FrameworkElement;
                if (element?.Tag as string == "GainGrid")
                {
                    canvas.Children.RemoveAt(i);
                }
            }

            AddGridLine(0, 0, 0, height, 0.6, true);
            AddGridLine(0, height, width, height, 0.6, true);

            AddVerticalTick(0.0, "0");
            AddVerticalTick(vrefKts / maxIasKts, String.Format("{0:F0}", vrefKts));
            AddVerticalTick(1.0, String.Format("{0:F0}", maxIasKts));

            AddHorizontalTick(0.0, "0");
            AddHorizontalTick(0.5, String.Format("{0:F2}", maxGain * 0.5f));
            AddHorizontalTick(1.0, String.Format("{0:F2}", maxGain));

            void AddGridLine(double x1, double y1, double x2, double y2, double opacity, bool axis)
            {
                Line line = new Line
                {
                    X1 = x1,
                    X2 = x2,
                    Y1 = y1,
                    Y2 = y2,
                    Stroke = new SolidColorBrush(Color.FromArgb(axis ? (byte)140 : (byte)80, 255, 255, 255)),
                    StrokeThickness = axis ? 1.0 : 0.5,
                    Opacity = opacity,
                    Tag = "GainGrid"
                };
                Panel.SetZIndex(line, -1);
                canvas.Children.Add(line);
            }

            void AddVerticalTick(double xNorm, string label)
            {
                double x = Math.Max(0.0, Math.Min(1.0, xNorm)) * width;
                AddGridLine(x, 0, x, height, 0.35, false);
                Line tick = new Line
                {
                    X1 = x,
                    X2 = x,
                    Y1 = height,
                    Y2 = height - 4,
                    Stroke = Brushes.White,
                    StrokeThickness = 1,
                    Tag = "GainGrid"
                };
                Panel.SetZIndex(tick, 0);
                canvas.Children.Add(tick);

                TextBlock text = new TextBlock
                {
                    Text = label,
                    Foreground = Brushes.White,
                    FontSize = 9,
                    Tag = "GainGrid"
                };
                Canvas.SetLeft(text, Math.Min(width - 18, Math.Max(0, x - 6)));
                Canvas.SetTop(text, height + 2);
                Panel.SetZIndex(text, 1);
                canvas.Children.Add(text);
            }

            void AddHorizontalTick(double yNorm, string label)
            {
                double y = height - Math.Max(0.0, Math.Min(1.0, yNorm)) * height;
                AddGridLine(0, y, width, y, 0.35, false);
                Line tick = new Line
                {
                    X1 = 0,
                    X2 = 4,
                    Y1 = y,
                    Y2 = y,
                    Stroke = Brushes.White,
                    StrokeThickness = 1,
                    Tag = "GainGrid"
                };
                Panel.SetZIndex(tick, 0);
                canvas.Children.Add(tick);

                TextBlock text = new TextBlock
                {
                    Text = label,
                    Foreground = Brushes.White,
                    FontSize = 9,
                    Tag = "GainGrid"
                };
                Canvas.SetLeft(text, -24);
                Canvas.SetTop(text, Math.Max(0, Math.Min(height - 12, y - 6)));
                Panel.SetZIndex(text, 1);
                canvas.Children.Add(text);
            }
        }
    }
}
