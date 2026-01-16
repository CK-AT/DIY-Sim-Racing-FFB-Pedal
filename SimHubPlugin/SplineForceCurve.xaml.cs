using System;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Effects;
using System.Windows.Shapes;
using MahApps.Metro.Controls;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for SplineForceCurve.xaml
    /// </summary>
    public partial class SplineForceCurve : UserControl
    {
        private DiyFfbPluginUI ui;
        private DiyFfbPlugin plugin;
        private SplineForceCurveConfig config;
        bool is_dragging;
        private Point offset;
        public delegate void ThresoldChangedEventHandler(double new_threshold);
        public event ThresoldChangedEventHandler ABSThresoldChanged;
        public event ThresoldChangedEventHandler BitePointThresoldChanged;
        public delegate void RangeSettingsChangedEventHandler(SplineForceCurve spline_force_curve);
        public event RangeSettingsChangedEventHandler RangeSettingsChanged;
        bool is_updating = false;
        private bool hasAxisRange;

        public SplineForceCurve()
        {
            config = GetDefaultConfig();
            InitializeComponent();
        }

        public void SetGui(DiyFfbPluginUI ui, DiyFfbPlugin plugin)
        {
            this.ui = ui;
            this.plugin = plugin;
            DrawGridLines();
        }

        public void ResetAxisRange()
        {
            hasAxisRange = false;
        }

        public static SplineForceCurveConfig GetDefaultConfig()
        {
            SplineForceCurveConfig new_config = new SplineForceCurveConfig();
            new_config.FRelPoints.AddRange(new uint[] { 0, 20, 40, 60, 80, 100 });
            new_config.CubicSplineParamsA.AddRange(new float[] { 0, 0, 0, 0, 0 });
            new_config.CubicSplineParamsB.AddRange(new float[] { 0, 0, 0, 0, 0 });
            new_config.PosMin = 10;
            new_config.PosMax = 70;
            new_config.FMin = 7.0f * 9.81f;
            new_config.FMax = 50.0f * 9.81f;
            new_config.ForceDirection = ForceDirection.Subtract;
            return new_config;
        }
        public void OnKinematicParametersChanged(KinematicParameters parameters)
        {
            hasAxisRange = true;
            double min = parameters.ContactPointPosMinAbs / 10.0;
            double max = parameters.ContactPointPosMaxAbs / 10.0;
            Rangeslider_travel_range.Minimum = Math.Min(min, max);
            Rangeslider_travel_range.Maximum = Math.Max(min, max);
        }

        public void UpdateConfig(SplineForceCurveConfig new_config)
        {
            config = new_config;
            is_updating = true;
            if (!hasAxisRange)
            {
                ApplyFallbackTravelRange();
            }
            Rangeslider_travel_range.LowerValue = config.PosMin;
            Rangeslider_travel_range.UpperValue = config.PosMax;
            Rangeslider_force_range.LowerValue = config.FMin / 9.81;
            Rangeslider_force_range.UpperValue = config.FMax / 9.81;
            is_updating = false;

            text_point_pos.Visibility = Visibility.Hidden;

            rect_State.Visibility = Visibility.Visible;
            text_state.Visibility = Visibility.Visible;

            Canvas.SetTop(rect_State, canvas.Height - rect_State.Height / 2);
            Canvas.SetLeft(rect_State, -rect_State.Width / 2);
            Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) /*+ rect_State.Width*/);
            Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - rect_State.Height);
            text_state.Text = "0%";

            rect_SABS.Visibility = Visibility.Hidden;
            rect_SABS_Control.Visibility = Visibility.Hidden;
            text_SABS.Visibility = Visibility.Hidden;

            text_BP.Visibility = Visibility.Hidden;
            rect_BP_Control.Visibility = Visibility.Hidden;

            UpdateSpline();
        }

        private void ApplyFallbackTravelRange()
        {
            if (Rangeslider_travel_range == null)
            {
                return;
            }

            double min = Math.Min(config.PosMin, config.PosMax);
            double max = Math.Max(config.PosMin, config.PosMax);
            Rangeslider_travel_range.Minimum = min;
            Rangeslider_travel_range.Maximum = max;
        }

        public void OnABSConfigUpdate(ABSEffectConfig abs_config)
        {
            //set for ABS slider
            if (abs_config.SimLevel > 0)
            {
                double control_rect_value_max = 100;
                double dyy = canvas.Height / control_rect_value_max;
                Canvas.SetTop(rect_SABS_Control, (control_rect_value_max - abs_config.SimLevel) * dyy - rect_SABS_Control.Height / 2);
                Canvas.SetLeft(rect_SABS_Control, 0);
                Canvas.SetTop(rect_SABS, 0);
                Canvas.SetLeft(rect_SABS, 0);
                rect_SABS.Height = canvas.Height - abs_config.SimLevel * dyy;
                Canvas.SetTop(text_SABS, Canvas.GetTop(rect_SABS_Control) - text_SABS.Height - rect_SABS_Control.Height);
                Canvas.SetLeft(text_SABS, canvas.Width - text_SABS.Width);
                text_SABS.Text = "ABS trigger value: " + abs_config.SimLevel + "%";
                rect_SABS.Visibility = Visibility.Visible;
                rect_SABS_Control.Visibility = Visibility.Visible;
                text_SABS.Visibility = Visibility.Visible;
            }
            else
            {
                rect_SABS.Visibility = Visibility.Hidden;
                rect_SABS_Control.Visibility = Visibility.Hidden;
                text_SABS.Visibility = Visibility.Hidden;
            }
        }

        public void OnBitePointConfigUpdate(BitePointEffectConfig bp_config)
        {
            if (bp_config.TriggerValue > 0)
            {
                //Bite point control
                double BP_max = 100;
                double dx = (double)canvas.Width / BP_max;
                text_BP.Text = "Bite Point:\n" + ((float)bp_config.TriggerValue) + "%";
                Canvas.SetLeft(rect_BP_Control, bp_config.TriggerValue * dx - rect_BP_Control.Width / 2);
                Canvas.SetLeft(text_BP, Canvas.GetLeft(rect_BP_Control) + rect_BP_Control.Width + 3);
                Canvas.SetTop(text_BP, canvas.Height - text_BP.Height - 15);
                text_BP.Visibility = Visibility.Visible;
                rect_BP_Control.Visibility = Visibility.Visible;
            }
            else
            {
                text_BP.Visibility = Visibility.Hidden;
                rect_BP_Control.Visibility = Visibility.Hidden;
            }
        }

        private void DrawGridLines()
        {
            // Specify the number of rows and columns for the grid
            int rowCount = 5;
            int columnCount = 5;

            // Calculate the width and height of each cell
            double cellWidth = canvas.Width / columnCount;
            double cellHeight = canvas.Height / rowCount;



            // Draw horizontal gridlines
            for (int i = 1; i < rowCount; i++)
            {
                Line line = new Line
                {
                    X1 = 0,
                    Y1 = i * cellHeight,
                    X2 = canvas.Width,
                    Y2 = i * cellHeight,
                    //Stroke = Brush.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1

                };
                Line line2 = new Line
                {
                    X1 = 0,
                    Y1 = i * cellHeight,
                    X2 = canvas.Width,
                    Y2 = i * cellHeight,
                    //Stroke = Brush.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1

                };
                canvas.Children.Add(line);
            }

            // Draw vertical gridlines
            for (int i = 1; i < columnCount; i++)
            {
                Line line = new Line
                {
                    X1 = i * cellWidth,
                    Y1 = 0,
                    X2 = i * cellWidth,
                    Y2 = canvas.Height,
                    //Stroke = Brushes.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                Line line2 = new Line
                {
                    X1 = i * cellWidth,
                    Y1 = 0,
                    X2 = i * cellWidth,
                    Y2 = canvas.Height,
                    //Stroke = Brushes.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                canvas.Children.Add(line);
            }
        }


        private void UpdateSpline()
        {
            double[] x = new double[6] {0, 20, 40, 60, 80, 100};
            double[] y = Array.ConvertAll<uint, double>(config.FRelPoints.ToArray(), val => val);
            double x_quantity = 100;
            double y_max = 100;
            double dx = canvas.Width / x_quantity;
            double dy = canvas.Height / y_max;
            //draw pedal force-travel curve

            // Use cubic interpolation to smooth the original data
            (double[] xs2, double[] ys2, double[] a, double[] b) = Cubic.Interpolate1D(x, y, 100);


            config.CubicSplineParamsA.Clear();
            config.CubicSplineParamsA.AddRange(Array.ConvertAll<double, float>(a.ToArray(), val => (float)val));
            config.CubicSplineParamsB.Clear();
            config.CubicSplineParamsB.AddRange(Array.ConvertAll<double, float>(b.ToArray(), val => (float)val));

            System.Windows.Media.PointCollection myPointCollection2 = new System.Windows.Media.PointCollection();


            for (int pointIdx = 0; pointIdx < 100; pointIdx++)
            {
                System.Windows.Point Pointlcl = new System.Windows.Point(dx * xs2[pointIdx], dy * ys2[pointIdx]);
                myPointCollection2.Add(Pointlcl);
            }

            Polyline_BrakeForceCurve.Points = myPointCollection2;
            double dyy = canvas.Height / 100;
            Canvas.SetTop(rect0, canvas.Height - dyy * config.FRelPoints[0] - rect0.Height / 2);
            Canvas.SetLeft(rect0, 0 * canvas.Width / 5 - rect0.Width / 2);
            Canvas.SetTop(rect1, canvas.Height - dyy * config.FRelPoints[1] - rect1.Height / 2);
            Canvas.SetLeft(rect1, 1 * canvas.Width / 5 - rect1.Width / 2);
            Canvas.SetTop(rect2, canvas.Height - dyy * config.FRelPoints[2] - rect2.Height / 2);
            Canvas.SetLeft(rect2, 2 * canvas.Width / 5 - rect2.Width / 2);
            Canvas.SetTop(rect3, canvas.Height - dyy * config.FRelPoints[3] - rect3.Height / 2);
            Canvas.SetLeft(rect3, 3 * canvas.Width / 5 - rect3.Width / 2);
            Canvas.SetTop(rect4, canvas.Height - dyy * config.FRelPoints[4] - rect4.Height / 2);
            Canvas.SetLeft(rect4, 4 * canvas.Width / 5 - rect4.Width / 2);
            Canvas.SetTop(rect5, canvas.Height - dyy * config.FRelPoints[5] - rect5.Height / 2);
            Canvas.SetLeft(rect5, 5 * canvas.Width / 5 - rect5.Width / 2);
        }

        /// <summary>
        /// Updates the marker on the force curve
        ///
        /// </summary>
        /// <param name="axis_state">received AxisState structure</param>
        /// <returns>Normalized axis position in percent</returns>
        /// 
        public double OnAxisStateUpdate(global::AxisState axis_state)
        {
            text_point_pos.Visibility = Visibility.Hidden;
            double pos_norm = Tools.Normalize(axis_state.Position, config.PosMin, config.PosMax);
            double f_norm = Tools.Normalize(axis_state.Force, config.FMin, config.FMax);
            text_state.Text = String.Format("{0:F1}kg\n{1:F1}mm", axis_state.Force / 9.81, axis_state.Position);

            Canvas.SetLeft(rect_State, canvas.Width * pos_norm - rect_State.Width / 2);
            Canvas.SetTop(rect_State, canvas.Height - canvas.Height * f_norm - rect_State.Height / 2);
            double phi_text = (Math.PI * pos_norm) + (Math.PI / 4.0);
            double offset_x = Math.Cos(phi_text) * rect_State.Width * 2.0;
            double offset_y = Math.Sin(phi_text) * rect_State.Width * 2.0;
            Canvas.SetLeft(text_state, Canvas.GetLeft(rect_State) + offset_x);
            Canvas.SetTop(text_state, Canvas.GetTop(rect_State) - offset_y);
            return pos_norm * 100.0;
        }

        private void btn_scurve_Click(object sender, RoutedEventArgs e)
        {
            config.FRelPoints[0] = 0;
            config.FRelPoints[1] = 7;
            config.FRelPoints[2] = 28;
            config.FRelPoints[3] = 70;
            config.FRelPoints[4] = 93;
            config.FRelPoints[5] = 100;
            UpdateSpline();
        }
        private void btn_10xcurve_Click(object sender, RoutedEventArgs e)
        {
            config.FRelPoints[0] = 0;
            config.FRelPoints[1] = 43;
            config.FRelPoints[2] = 69;
            config.FRelPoints[3] = 85;
            config.FRelPoints[4] = 95;
            config.FRelPoints[5] = 100;
            UpdateSpline();
        }
        private void btn_logcurve_Click(object sender, RoutedEventArgs e)
        {
            config.FRelPoints[0] = 0;
            config.FRelPoints[1] = 6;
            config.FRelPoints[2] = 17;
            config.FRelPoints[3] = 33;
            config.FRelPoints[4] = 59;
            config.FRelPoints[5] = 100;
            UpdateSpline();
        }
        private void btn_linearcurve_Click(object sender, RoutedEventArgs e)
        {
            config.FRelPoints[0] = 0;
            config.FRelPoints[1] = 20;
            config.FRelPoints[2] = 40;
            config.FRelPoints[3] = 60;
            config.FRelPoints[4] = 80;
            config.FRelPoints[5] = 100;
            UpdateSpline();
        }
        private void Rectangle_MouseMove_ABS(object sender, MouseEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas
                double min_positon = 0.05 * canvas.Height; // 95% (5% from top)
                double max_position = 0.5 * canvas.Height; // 50%
                y = Math.Max(min_positon, Math.Min(y, max_position));
                //Canvas.SetTop(rect_SABS, y);
                rect_SABS.Height = y;
                double threshold = (canvas.Height - y) / canvas.Height;
                text_SABS.Text = String.Format("ABS trigger value: {0}%", threshold * 100.0);
                Canvas.SetTop(text_SABS, y - rect_SABS_Control.Height - text_SABS.Height);
                Canvas.SetTop(rectangle, y - rect_SABS_Control.Height / 2);
                ABSThresoldChanged?.Invoke(threshold * 100.0);
            }
        }

        private void Rectangle_MouseMove_BP(object sender, MouseEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                //Bite point control
                if (rectangle.Name == "rect_BP_Control")
                {
                    // Ensure the rectangle stays within the canvas
                    double x = e.GetPosition(canvas).X - offset.X;
                    double min_position = 0.1 * canvas.Width - rect_BP_Control.Width / 2;
                    double max_position = 0.9 * canvas.Width - rect_BP_Control.Width / 2;

                    x = Math.Max(min_position, Math.Min(x, max_position));
                    double threshold = (x + rect_BP_Control.Width / 2) / canvas.Width;
                    text_SABS.Text = String.Format("Bite Point:\n{0}%", threshold * 100.0);
                    Canvas.SetLeft(rectangle, x);
                    Canvas.SetLeft(text_BP, Canvas.GetLeft(rect_BP_Control) + rect_BP_Control.Width + 3);
                    Canvas.SetTop(text_BP, canvas.Height - text_BP.Height - 15);
                    BitePointThresoldChanged?.Invoke(threshold);
                }
            }
        }

        private void Rectangle_MouseMove(object sender, MouseEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                //double x = e.GetPosition(canvas).X - offset.X;
                double y = e.GetPosition(canvas).Y - offset.Y;

                // Ensure the rectangle stays within the canvas
                //x = Math.Max(0, Math.Min(x, canvas.ActualWidth - rectangle.ActualWidth));
                y = Math.Max(-1 * rectangle.Height / 2, Math.Min(y, canvas.Height - rectangle.Height / 2));

                //Canvas.SetLeft(rectangle, x);
                Canvas.SetTop(rectangle, y);
                double y_actual = (canvas.Height - y - rectangle.Height / 2) / canvas.Height;
                if (rectangle.Name == "rect0")
                {
                    config.FRelPoints[0] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:0%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }
                if (rectangle.Name == "rect1")
                {
                    config.FRelPoints[1] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:20%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }
                if (rectangle.Name == "rect2")
                {
                    config.FRelPoints[2] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:40%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }
                if (rectangle.Name == "rect3")
                {
                    config.FRelPoints[3] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:60%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }
                if (rectangle.Name == "rect4")
                {
                    config.FRelPoints[4] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:80%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }
                if (rectangle.Name == "rect5")
                {
                    config.FRelPoints[5] = Convert.ToUInt16(y_actual * 100);
                    text_point_pos.Text = String.Format("Travel:100%\nForce: {0}%", Math.Round(y_actual * 100.0));
                }

                text_point_pos.Visibility = Visibility.Visible;
                UpdateSpline();
            }
        }

        private void btn_plus_maxforce_Click(object sender, RoutedEventArgs e)
        {
            Rangeslider_force_range.UpperValue = Rangeslider_force_range.UpperValue + 0.1;
        }

        private void btn_minus_maxforce_Click(object sender, RoutedEventArgs e)
        {
            Rangeslider_force_range.UpperValue = Rangeslider_force_range.UpperValue - 0.1;
        }

        private void btn_plus_preload_Click(object sender, RoutedEventArgs e)
        {
            Rangeslider_force_range.LowerValue = Rangeslider_force_range.LowerValue + 0.1;
        }

        private void btn_minus_preload_Click(object sender, RoutedEventArgs e)
        {
            Rangeslider_force_range.LowerValue = Rangeslider_force_range.LowerValue - 0.1;
        }

        private void Rangeslider_travel_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.PosMin = Convert.ToInt16(e.NewValue);
            }
            if (Label_min_pos != null)
            {
                Label_min_pos.Content = String.Format("Min\n{0}mm", config.PosMin);
            }
            RangeSettingsChanged?.Invoke(this);
        }

        private void Rangeslider_travel_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.PosMax = Convert.ToInt16(e.NewValue);
            }
            if (Label_max_pos != null)
            {
                Label_max_pos.Content = String.Format("Max\n{0}mm", config.PosMax);
            }
            RangeSettingsChanged?.Invoke(this);
        }

        private void Rangeslider_force_range_UpperValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.FMax = (float)(e.NewValue * 9.81);
            }
            if (Label_max_force != null)
            {
                Label_max_force.Content = String.Format("Max force\n{0:F1}kg", e.NewValue);
            }
            RangeSettingsChanged?.Invoke(this);
        }

        private void Rangeslider_force_range_LowerValueChanged(object sender, RangeParameterChangedEventArgs e)
        {
            if (!is_updating)
            {
                config.FMin = (float)(e.NewValue * 9.81);
            }
            if (Label_min_force != null)
            {
                Label_min_force.Content = String.Format("Preload\n{0:F1}kg", e.NewValue);
            }
            RangeSettingsChanged?.Invoke(this);
        }

        private void Rectangle_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            is_dragging = true;
            var rectangle = sender as Rectangle;
            offset = e.GetPosition(rectangle);
            rectangle.CaptureMouse();
            if (rectangle.Name != "rect_SABS_Control" & rectangle.Name != "rect_BP_Control")
            {
                var dropShadowEffect = new DropShadowEffect
                {
                    ShadowDepth = 0,
                    BlurRadius = 15,
                    Color = Colors.White,
                    Opacity = 1
                };
                rectangle.Fill = ui.MouseDownColor;
                rectangle.Effect = dropShadowEffect;
            }
        }
        private void Rectangle_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                is_dragging = false;
                rectangle.ReleaseMouseCapture();
                text_point_pos.Visibility = Visibility.Hidden;
                if (rectangle.Name != "rect_SABS_Control" & rectangle.Name != "rect_BP_Control")
                {
                    var dropShadowEffect = new DropShadowEffect
                    {
                        ShadowDepth = 0,
                        BlurRadius = 20,
                        Color = Colors.White,
                        Opacity = 0
                    };
                    rectangle.Fill = ui.MouseUpColor;
                    rectangle.Effect = dropShadowEffect;
                }
            }
        }
    }
}
