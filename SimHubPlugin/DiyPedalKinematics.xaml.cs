using System;
using System.Linq;
using System.Text.RegularExpressions;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Effects;
using System.Windows.Shapes;
using MathNet.Numerics;

namespace User.PluginSdkDemo
{
    /// <summary>
    /// Interaction logic for UserControl1.xaml
    /// </summary>
    public partial class DiyPedalKinematics : UserControl
    {
        public struct KinematicParameterResults
        {
            public double[] l_sled;
            public double[] x_contact_point;
            public double[] r_force_link_foot;
            public double[] angles;

            public KinematicParameters KinematicParameters { get
                {
                    double[] coeffs_sled_pos_over_contact_point_pos = Fit.Polynomial(x_contact_point, l_sled, 4);
                    double[] coeffs_force_factor_over_contact_point_pos = Fit.Polynomial(x_contact_point, r_force_link_foot, 4);

                    KinematicParameters new_parameters = new KinematicParameters();
                    new_parameters.CoeffsSledPosOverContactPointPos.AddRange(coeffs_sled_pos_over_contact_point_pos);
                    new_parameters.CoeffsForceFactorOverContactPointPos.AddRange(coeffs_force_factor_over_contact_point_pos);
                    new_parameters.ContactPointPosMinAbs = (int)(x_contact_point.First() * 10.0);
                    new_parameters.ContactPointPosMaxAbs = (int)(x_contact_point.Last() * 10.0);
                    return new_parameters;
                }
            }

            public double GetAngleFromContactPointPosition(double position)
            {
                return Interpolate.Linear(x_contact_point, angles).Interpolate(position);
            }

            public double GetSledPositionFromContactPointPosition(double position)
            {
                return Interpolate.Linear(x_contact_point, l_sled).Interpolate(position);
            }

            public double MinAngle
            { 
                get
                {
                    return angles.First();
                }
            }
            public double MaxAngle
            {
                get
                {
                    return angles.Last();
                }
            }
        }

        private DIYPedalKinematicConfig config;
        private bool is_dragging = false;
        private Point offset;
        private SettingsControlDemo gui;
        private DIY_FFB plugin;
        private int gridline_kinematic_count_original = 0;
        public delegate void DebugMessageEventHandler(string message);
        public event DebugMessageEventHandler DebugMessage;
        public delegate void KinematicParametersChangedEventHandler(KinematicParameters parameters);
        public event KinematicParametersChangedEventHandler KinematicParametersChanged;
        KinematicParameterResults _kinematic_results;
        double last_axis_position = 0.0;

        public DiyPedalKinematics()
        {
            config = GetDefaultConfig();
            InitializeComponent();
        }

        public void SetGui(SettingsControlDemo gui, DIY_FFB plugin)
        {
            this.gui = gui;
            this.plugin = plugin;
            DrawGridLines();
        }

        public void OnAxisStateUpdate(global::AxisState axis_state)
        {
            last_axis_position = axis_state.Position;
            UpdateJointDrawing(false);
        }

        public static DIYPedalKinematicConfig GetDefaultConfig()
        {
            DIYPedalKinematicConfig new_config = new DIYPedalKinematicConfig();
            new_config.LPivotFoot = 180;
            new_config.LPivotLink = 100;
            new_config.LLink = 153;
            new_config.LPivotSledY = 32;
            new_config.LPivotSledXMin = 82;
            new_config.LSledStroke = 114;
            return new_config;
        }

        public static KinematicParameters CalcKinematicParameters(DIYPedalKinematicConfig config)
        {
            var result = CalcParameters(config);
            return result.KinematicParameters;
        }

        public void UpdateConfig(DIYPedalKinematicConfig new_config)
        {
            config = new_config;
            Label_kinematic_b_canvas.SetTextWithoutEvent(config.LPivotLink.ToString());
            Label_kinematic_a_canvas.SetTextWithoutEvent(config.LLink.ToString());
            Label_kinematic_c_hort_canvas.SetTextWithoutEvent(config.LPivotSledXMin.ToString());
            Label_kinematic_c_vert_canvas.SetTextWithoutEvent(config.LPivotSledY.ToString());
            Label_kinematic_d_canvas.SetTextWithoutEvent((config.LPivotFoot - config.LPivotLink).ToString());
            Label_travel_canvas.SetTextWithoutEvent(config.LSledStroke.ToString());
            UpdateJointDrawing(true);
        }

        private void rect_joint_MouseMove(object sender, MouseEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                double y = e.GetPosition(canvas_kinematic).Y - offset.Y;
                double x = e.GetPosition(canvas_kinematic).X - offset.X;
                //double y = e.GetPosition(canvas).Y - offset.Y;


                /*
                if (rectangle.Name == "rect8")
                {
                    // Ensure the rectangle stays within the canvas
                    double dy = 250 / (canvas_vert_slider.Height);
                    double min_position = Canvas.GetTop(rect8) - rectangle.Height / 2;
                    double max_position = Canvas.GetTop(rect9) + rectangle.Height / 2;
                    double min_limit = canvas_vert_slider.Height - 0 / dy;
                    double max_limit = canvas_vert_slider.Height - 250 / dy;
                }
                */
            }
        }

        private void Rectangle_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            is_dragging = true;
            var rectangle = sender as Rectangle;
            offset = e.GetPosition(rectangle);
            rectangle.CaptureMouse();
            var dropShadowEffect = new DropShadowEffect
            {
                ShadowDepth = 0,
                BlurRadius = 15,
                Color = Colors.White,
                Opacity = 1
            };
            rectangle.Fill = gui.MouseDownColor;
            rectangle.Effect = dropShadowEffect;
        }
        private void Rectangle_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (is_dragging)
            {
                var rectangle = sender as Rectangle;
                is_dragging = false;
                rectangle.ReleaseMouseCapture();
                var dropShadowEffect = new DropShadowEffect
                {
                    ShadowDepth = 0,
                    BlurRadius = 20,
                    Color = Colors.White,
                    Opacity = 0
                };
                rectangle.Fill = gui.MouseUpColor;
                rectangle.Effect = dropShadowEffect;
            }
        }

        private static KinematicParameterResults CalcParameters(DIYPedalKinematicConfig config)
        {
            //A= kinematic joint C
            //B= Kinematic joint A
            //C= Kinematic joint B
            //O=O
            //D=D

            //parameter calculation
            double l_pivot_link = config.LPivotLink;
            double l_pivot_sled_x_min = config.LPivotSledXMin;
            double l_pivot_sled_y = config.LPivotSledY;
            double Travel_length = config.LSledStroke;
            double l_link = config.LLink;
            double l_pivot_foot = config.LPivotFoot;

            KinematicParameterResults results = new KinematicParameterResults();

            results.l_sled = Generate.LinearSpaced(1000, 0.0, Travel_length);
            results.x_contact_point = new double[1000];
            results.r_force_link_foot = new double[1000];
            results.angles = new double[1000];

            for (int i = 0; i < results.l_sled.Length; i++)
            {
                double l_pivot_sled_x = l_pivot_sled_x_min + results.l_sled[i]; // horizontal position of the sled connection point relative to the pivot/origin
                double phi_pivot_sled = Math.Atan2(l_pivot_sled_y, l_pivot_sled_x); // angle of the connecting line between pivot and sled connection point (from horizontal axis)
                double l_pivot_sled = Math.Sqrt(l_pivot_sled_y * l_pivot_sled_y + l_pivot_sled_x * l_pivot_sled_x); // length of the connecting line between pivot and sled connection point
                double phi_link_pivot_sled = Math.Acos((l_link * l_link - l_pivot_link * l_pivot_link - l_pivot_sled * l_pivot_sled) / (l_pivot_link * l_pivot_sled * -2)); // angle between the connecting line between pivot and link and the connecting line between pivot and sled connection point
                double phi_link_sled_pivot = Math.Acos((l_pivot_link * l_pivot_link - l_link * l_link - l_pivot_sled * l_pivot_sled) / (l_link * l_pivot_sled * -2)); // angle between the link and the connecting line between pivot and sled connection point
                double phi_pivot_link_sled = Math.PI - phi_link_pivot_sled - phi_link_sled_pivot; // third angle of the triangle between pivot, link connection point and sled connection point
                double phi_ped_vert = (Math.PI / 2.0) - (phi_link_pivot_sled + phi_pivot_sled); // pedal angle (from vertical axis)
                double phi_foot = -phi_ped_vert; // foot angle (pedal normal, from horizontal axis)
                double phi_link = -(Math.PI / 2.0) - phi_ped_vert + phi_pivot_link_sled; // link angle (from horizontal axis)
                double phi_foot_link = phi_foot - phi_link; // angle between foot force direction and link
                double r_force_foot_link = (l_pivot_foot / l_pivot_link) * Math.Cos(phi_foot_link); // ratio between foot force and load cell measurement
                results.r_force_link_foot[i] = 1.0 / r_force_foot_link; // ratio between load cell measurement and foot force
                double l_foot = phi_ped_vert * l_pivot_foot; // foot position as an arc length relative to vertical axis
                results.x_contact_point[i] = l_foot;
                results.angles[i] = phi_ped_vert;
            }
            return results;
        }

        private void UpdateKinematicCalcs() {
            _kinematic_results = CalcParameters(config);
            KinematicParametersChanged?.Invoke(_kinematic_results.KinematicParameters);
        }

        private void UpdateJointDrawing(bool update_calculation)
        {
            if (update_calculation)
            { 
                UpdateKinematicCalcs();
            }

            double l_pivot_link = config.LPivotLink;
            double l_pivot_sled_x_min = config.LPivotSledXMin;
            double l_pivot_sled_y = config.LPivotSledY;
            double Travel_length = config.LSledStroke;
            double l_link = config.LLink;
            double l_pivot_foot = config.LPivotFoot;

            Label_kinematic_b_canvas.Text = "" + config.LPivotLink;
            Label_kinematic_c_hort_canvas.Text = "" + config.LPivotSledXMin;
            Label_kinematic_c_vert_canvas.Text = "" + config.LPivotSledY;
            Label_kinematic_a_canvas.Text = "" + config.LLink;
            Label_kinematic_d_canvas.Text = "" + (config.LPivotFoot - config.LPivotLink);
            Label_travel_canvas.Text = "" + config.LSledStroke;

            double pedal_angle = _kinematic_results.GetAngleFromContactPointPosition(last_axis_position);
            double Current_travel_position = _kinematic_results.GetSledPositionFromContactPointPosition(last_axis_position);
            double min_angle = _kinematic_results.MinAngle;
            double max_angle = _kinematic_results.MaxAngle;

            Label_kinematic_pedal_angle.Content = "Current Pedal Angle: " + Math.Round(pedal_angle / Math.PI * 180) + "°,";
            Label_kinematic_pedal_angle.Content = Label_kinematic_pedal_angle.Content + " Max Pedal Angle:" + Math.Round(max_angle / Math.PI * 180) + "°,";
            Label_kinematic_pedal_angle.Content = Label_kinematic_pedal_angle.Content + " Min Pedal Angle:" + Math.Round(min_angle / Math.PI * 180) + "°,";
            Label_kinematic_pedal_angle.Content = Label_kinematic_pedal_angle.Content + " Angle Travel:" + Math.Round((max_angle - min_angle) / Math.PI * 180) + "°";

            if (plugin == null) return;

            Label_kinematic_scale.Content = Math.Round(plugin.Settings.kinematicDiagram_zeroPos_scale, 1);

            double pedal_angle_from_horizontal = (Math.PI / 2.0) - pedal_angle;
            double A_X = l_pivot_link * Math.Cos(pedal_angle_from_horizontal);
            double A_Y = l_pivot_link * Math.Sin(pedal_angle_from_horizontal);
            double D_X = l_pivot_foot * Math.Cos(pedal_angle_from_horizontal);
            double D_Y = l_pivot_foot * Math.Sin(pedal_angle_from_horizontal);
            double scale_factor = plugin.Settings.kinematicDiagram_zeroPos_scale;
            double shifting_OX = plugin.Settings.kinematicDiagram_zeroPos_OX;
            double shifting_OY = plugin.Settings.kinematicDiagram_zeroPos_OY;
            //set rect position
            Canvas.SetLeft(rect_joint_O, shifting_OX - rect_joint_O.Width / 2);
            Canvas.SetTop(rect_joint_O, canvas_kinematic.Height - shifting_OY - rect_joint_O.Height / 2);
            Canvas.SetLeft(rect_joint_C, A_X / scale_factor - rect_joint_C.Width / 2 + shifting_OX);
            Canvas.SetTop(rect_joint_C, canvas_kinematic.Height - A_Y / scale_factor - rect_joint_C.Height / 2 - shifting_OY);
            Canvas.SetLeft(rect_joint_A, l_pivot_sled_x_min / scale_factor - rect_joint_A.Width / 2 + shifting_OX);
            Canvas.SetTop(rect_joint_A, canvas_kinematic.Height - 0 / scale_factor - rect_joint_A.Height / 2 - shifting_OY);
            Canvas.SetLeft(rect_joint_B, l_pivot_sled_x_min / scale_factor - rect_joint_B.Width / 2 + Current_travel_position / scale_factor + shifting_OX);
            Canvas.SetTop(rect_joint_B, canvas_kinematic.Height - l_pivot_sled_y / scale_factor - rect_joint_B.Height / 2 - shifting_OY);
            Canvas.SetLeft(rect_joint_D, D_X / scale_factor - rect_joint_A.Width / 2 + shifting_OX);
            Canvas.SetTop(rect_joint_D, canvas_kinematic.Height - D_Y / scale_factor - rect_joint_A.Height / 2 - shifting_OY);

            Canvas.SetLeft(Label_joint_C, Canvas.GetLeft(rect_joint_C) - Label_joint_C.Width);
            Canvas.SetTop(Label_joint_C, Canvas.GetTop(rect_joint_C));
            Canvas.SetLeft(Label_joint_A, Canvas.GetLeft(rect_joint_A) + rect_joint_A.Width / 2 - Label_joint_A.Width / 2);
            Canvas.SetTop(Label_joint_A, Canvas.GetTop(rect_joint_A) - Label_joint_A.Height);
            Canvas.SetLeft(Label_joint_B, Canvas.GetLeft(rect_joint_B) + Label_joint_B.Width);
            Canvas.SetTop(Label_joint_B, Canvas.GetTop(rect_joint_B));
            Canvas.SetLeft(Label_joint_D, Canvas.GetLeft(rect_joint_D) - Label_joint_D.Width);
            Canvas.SetTop(Label_joint_D, Canvas.GetTop(rect_joint_D));
            Canvas.SetLeft(Label_joint_O, Canvas.GetLeft(rect_joint_O) - Label_joint_O.Width);
            Canvas.SetTop(Label_joint_O, Canvas.GetTop(rect_joint_O));

            Canvas.SetLeft(SP_kinematic_b_canvas, (Canvas.GetLeft(rect_joint_C) + shifting_OX) / 2 - SP_kinematic_b_canvas.Width / 2 - Label_kinematic_b_canvas.Width / 2);
            Canvas.SetTop(SP_kinematic_b_canvas, (Canvas.GetTop(rect_joint_C) + canvas_kinematic.Height - shifting_OY) / 2 - Label_kinematic_b_canvas.Height / 2);
            Canvas.SetLeft(SP_kinematic_c_hort_canvas, (Canvas.GetLeft(rect_joint_A) + shifting_OX) / 2 - SP_kinematic_c_hort_canvas.Width / 2 - 5);
            Canvas.SetTop(SP_kinematic_c_hort_canvas, (Canvas.GetTop(rect_joint_A) + canvas_kinematic.Height - shifting_OY) / 2 + Label_kinematic_c_hort_canvas.Height / 2 - 5);
            Canvas.SetLeft(SP_kinematic_c_vert_canvas, Canvas.GetLeft(rect_joint_B) - rect_joint_B.Width - SP_kinematic_c_vert_canvas.Width / 2 + Label_kinematic_c_vert_canvas.Width);
            Canvas.SetTop(SP_kinematic_c_vert_canvas, (Canvas.GetTop(rect_joint_A) + Canvas.GetTop(rect_joint_B)) / 2 - Label_kinematic_c_vert_canvas.Height / 2 + 5);
            Canvas.SetLeft(SP_kinematic_a_canvas, (Canvas.GetLeft(rect_joint_A) + Canvas.GetLeft(rect_joint_C)) / 2 - SP_kinematic_a_canvas.Width / 2 + Label_kinematic_a_canvas.Width / 2);
            Canvas.SetTop(SP_kinematic_a_canvas, (Canvas.GetTop(rect_joint_A) + Canvas.GetTop(rect_joint_C)) / 2 - Label_kinematic_a_canvas.Height);
            Canvas.SetLeft(SP_kinematic_d_canvas, (Canvas.GetLeft(rect_joint_C) + Canvas.GetLeft(rect_joint_D)) / 2 - SP_kinematic_d_canvas.Width / 2 - Label_kinematic_d_canvas.Width / 2);
            Canvas.SetTop(SP_kinematic_d_canvas, (Canvas.GetTop(rect_joint_C) + +Canvas.GetTop(rect_joint_D)) / 2 - Label_kinematic_d_canvas.Height / 2);
            Canvas.SetLeft(SP_travel_canvas, (Canvas.GetLeft(rect_joint_A) + (l_pivot_sled_x_min + Travel_length) / scale_factor + shifting_OX) / 2 - SP_travel_canvas.Width / 2);
            Canvas.SetTop(SP_travel_canvas, (Canvas.GetTop(rect_joint_A) + canvas_kinematic.Height - shifting_OY) / 2 + Label_travel_canvas.Height / 2 - 5);

            this.Line_kinematic_b.X1 = shifting_OX;
            this.Line_kinematic_b.Y1 = canvas_kinematic.Height - shifting_OY;
            this.Line_kinematic_b.X2 = A_X / scale_factor + shifting_OX;
            this.Line_kinematic_b.Y2 = canvas_kinematic.Height - A_Y / scale_factor - shifting_OY;

            this.Line_kinematic_c_hort.X1 = shifting_OX;
            this.Line_kinematic_c_hort.Y1 = canvas_kinematic.Height - shifting_OY;
            this.Line_kinematic_c_hort.X2 = l_pivot_sled_x_min / scale_factor + shifting_OX;
            this.Line_kinematic_c_hort.Y2 = canvas_kinematic.Height - shifting_OY;

            this.Line_kinematic_c_vert.X1 = (l_pivot_sled_x_min + Current_travel_position) / scale_factor + shifting_OX;
            this.Line_kinematic_c_vert.Y1 = canvas_kinematic.Height - shifting_OY;
            this.Line_kinematic_c_vert.X2 = (l_pivot_sled_x_min + Current_travel_position) / scale_factor + shifting_OX;
            this.Line_kinematic_c_vert.Y2 = canvas_kinematic.Height - l_pivot_sled_y / scale_factor - shifting_OY;

            this.Line_kinematic_a.X1 = (l_pivot_sled_x_min + Current_travel_position) / scale_factor + shifting_OX;
            this.Line_kinematic_a.Y1 = canvas_kinematic.Height - l_pivot_sled_y / scale_factor - shifting_OY;
            this.Line_kinematic_a.X2 = A_X / scale_factor + shifting_OX;
            this.Line_kinematic_a.Y2 = canvas_kinematic.Height - A_Y / scale_factor - shifting_OY;

            this.Line_kinematic_d.X1 = A_X / scale_factor + shifting_OX;
            this.Line_kinematic_d.Y1 = canvas_kinematic.Height - A_Y / scale_factor - shifting_OY;
            this.Line_kinematic_d.X2 = D_X / scale_factor + shifting_OX;
            this.Line_kinematic_d.Y2 = canvas_kinematic.Height - D_Y / scale_factor - shifting_OY;

            this.Line_Pedal_Travel.X1 = l_pivot_sled_x_min / scale_factor + shifting_OX;
            this.Line_Pedal_Travel.Y1 = canvas_kinematic.Height - shifting_OY;
            this.Line_Pedal_Travel.X2 = (l_pivot_sled_x_min + Travel_length) / scale_factor + shifting_OX;
            this.Line_Pedal_Travel.Y2 = canvas_kinematic.Height - shifting_OY;

        }

        private void DrawGridLines()
        {
            double OX = plugin.Settings.kinematicDiagram_zeroPos_OX;
            double OY = plugin.Settings.kinematicDiagram_zeroPos_OY;
            double scale_i = plugin.Settings.kinematicDiagram_zeroPos_scale;

            if (gridline_kinematic_count_original > 0)
            {
                for (int i = 0; i < gridline_kinematic_count_original; i++)
                {
                    if (canvas_kinematic.Children.Count != 0)
                    {
                        canvas_kinematic.Children.RemoveAt(canvas_kinematic.Children.Count - 1);
                    }
                }
            }
            double scale = scale_i;
            double gridlineSpacing = 50 / scale;

            double cellWidth = gridlineSpacing;
            double cellHeight = gridlineSpacing;

            // we want the gridlines to be centered at pedal position O
            // --> calculate an offset
            double xOffset = OX % gridlineSpacing;
            double yOffset = OY % gridlineSpacing;


            int rowCount = (int)Math.Floor((canvas_kinematic.Height - 0 * yOffset) / gridlineSpacing);
            int columnCount = (int)Math.Floor((canvas_kinematic.Width - 0 * xOffset) / gridlineSpacing);


            // Draw horizontal gridlines
            for (int i = 0; i < rowCount; i++)
            {

                Line line2 = new Line
                {
                    X1 = 0,
                    Y1 = canvas_kinematic.Height - (yOffset + i * cellHeight),
                    X2 = 400,
                    Y2 = canvas_kinematic.Height - (yOffset + i * cellHeight),
                    //Stroke = Brush.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1

                };
                canvas_kinematic.Children.Add(line2);
            }

            // Draw vertical gridlines
            for (int i = 0; i < columnCount; i++)
            {

                Line line2 = new Line
                {
                    X1 = xOffset + i * cellWidth,
                    Y1 = 0,
                    X2 = xOffset + i * cellWidth,
                    Y2 = canvas_kinematic.Height,
                    //Stroke = Brushes.Black,
                    Stroke = System.Windows.Media.Brushes.LightSteelBlue,
                    StrokeThickness = 1,
                    Opacity = 0.1
                };
                canvas_kinematic.Children.Add(line2);

            }
            gridline_kinematic_count_original = columnCount + rowCount;
        }

        private bool Kinematic_check(double OA, double OB, double BC, double CA, double travel)
        {

            double OC = Math.Sqrt((OB + travel) * (OB + travel) + BC * BC);
            double pedal_angle_1 = Math.Acos((OA * OA + OC * OC - CA * CA) / (2 * OA * OC));
            double pedal_angle_2 = Math.Atan2(BC, (OB + travel));


            double pedal_angle = pedal_angle_1 + pedal_angle_2;
            if (pedal_angle_1 != double.NaN && pedal_angle_2 != double.NaN)
            {
                if (pedal_angle <= Math.PI * 0.6)
                {
                    if ((OA + CA) > OC)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }



        }

        private void btn_plus_OA_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA + 1, OB, BC, CA, config.LSledStroke))
            {

                config.LPivotLink += 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_minus_OA_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA - 1, OB, BC, CA, config.LSledStroke))
            {
                config.LPivotLink -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_plus_OB_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB + 1, BC, CA, config.LSledStroke))
            {
                config.LPivotSledXMin += 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_minus_OB_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB - 1, BC, CA, config.LSledStroke))
            {
                config.LPivotSledXMin -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_plus_BC_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB, BC + 1, CA, config.LSledStroke))
            {
                config.LPivotSledY += 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_minus_BC_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB, BC - 1, CA, config.LSledStroke))
            {
                config.LPivotSledY -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_plus_CA_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB, BC, CA + 1, config.LSledStroke))
            {
                config.LLink += 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_minus_CA_Click(object sender, RoutedEventArgs e)
        {
            double OA = config.LPivotLink;
            double OB = config.LPivotSledXMin;
            double BC = config.LPivotSledY;
            double CA = config.LLink;
            if (Kinematic_check(OA, OB, BC, CA - 1, config.LSledStroke))
            {
                config.LLink -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_plus_AD_Click(object sender, RoutedEventArgs e)
        {
            config.LPivotFoot += 1;
            UpdateJointDrawing(true);
        }

        private void btn_minus_AD_Click(object sender, RoutedEventArgs e)
        {
            if ((config.LPivotFoot - config.LPivotLink) > 2)
            {
                config.LPivotFoot -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                DebugMessage?.Invoke("Pedal Kinematic calculation error");
            }
        }

        private void btn_plus_travel_Click(object sender, RoutedEventArgs e)
        {
            if (config.LSledStroke <= 200)
            {
                config.LSledStroke += 1;
                UpdateJointDrawing(true);
            }
            else
            {
                config.LSledStroke = 200;
            }

        }

        private void btn_minus_travel_Click(object sender, RoutedEventArgs e)
        {
            if (config.LSledStroke >= 30)
            {
                config.LSledStroke -= 1;
                UpdateJointDrawing(true);
            }
            else
            {
                config.LSledStroke = 30;
            }

        }


        private void SP_canvas_MouseEnter(object sender, MouseEventArgs e)
        {
            btn_plus_kinematic_b_canvas.Visibility = Visibility.Visible;
            btn_minus_kinematic_b_canvas.Visibility = Visibility.Visible;
            btn_plus_kinematic_c_hort_canvas.Visibility = Visibility.Visible;
            btn_minus_kinematic_c_hort_canvas.Visibility = Visibility.Visible;
            btn_plus_kinematic_c_vert_canvas.Visibility = Visibility.Visible;
            btn_minus_kinematic_c_vert_canvas.Visibility = Visibility.Visible;
            btn_plus_kinematic_a_canvas.Visibility = Visibility.Visible;
            btn_minus_kinematic_a_canvas.Visibility = Visibility.Visible;
            btn_plus_kinematic_d_canvas.Visibility = Visibility.Visible;
            btn_minus_kinematic_d_canvas.Visibility = Visibility.Visible;
            btn_plus_travel_canvas.Visibility = Visibility.Visible;
            btn_minus_travel_canvas.Visibility = Visibility.Visible;
        }
        private void SP_canvas_MouseLeave(object sender, MouseEventArgs e)
        {
            btn_plus_kinematic_b_canvas.Visibility = Visibility.Hidden;
            btn_minus_kinematic_b_canvas.Visibility = Visibility.Hidden;
            btn_plus_kinematic_c_hort_canvas.Visibility = Visibility.Hidden;
            btn_minus_kinematic_c_hort_canvas.Visibility = Visibility.Hidden;
            btn_plus_kinematic_c_vert_canvas.Visibility = Visibility.Hidden;
            btn_minus_kinematic_c_vert_canvas.Visibility = Visibility.Hidden;
            btn_plus_kinematic_a_canvas.Visibility = Visibility.Hidden;
            btn_minus_kinematic_a_canvas.Visibility = Visibility.Hidden;
            btn_plus_kinematic_d_canvas.Visibility = Visibility.Hidden;
            btn_minus_kinematic_d_canvas.Visibility = Visibility.Hidden;
            btn_plus_travel_canvas.Visibility = Visibility.Hidden;
            btn_minus_travel_canvas.Visibility = Visibility.Hidden;
        }

        private void btn_minus_kinematic_scale_Click(object sender, RoutedEventArgs e)
        {
            if (plugin.Settings.kinematicDiagram_zeroPos_scale > 0.7)
            {
                plugin.Settings.kinematicDiagram_zeroPos_scale = plugin.Settings.kinematicDiagram_zeroPos_scale - 0.1;
                DrawGridLines();
                UpdateJointDrawing(false);
                //Label_kinematic_scale.Content = Plugin.Settings.kinematicDiagram_zeroPos_scale;
            }
        }

        private void btn_plus_kinematic_scale_Click(object sender, RoutedEventArgs e)
        {
            if (plugin.Settings.kinematicDiagram_zeroPos_scale < 2)
            {
                plugin.Settings.kinematicDiagram_zeroPos_scale = plugin.Settings.kinematicDiagram_zeroPos_scale + 0.1;
                DrawGridLines();
                UpdateJointDrawing(false);
                //Label_kinematic_scale.Content = Plugin.Settings.kinematicDiagram_zeroPos_scale;
            }
        }

        private void Kinematic_TextBox_TextChanged(object sender, TextChangedEventArgs e)
        {
            var textbox = sender as System.Windows.Controls.TextBox;
            if (textbox.Name == "Label_kinematic_b_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    double OA = result;
                    double OB = config.LPivotSledXMin;
                    double BC = config.LPivotSledY;
                    double CA = config.LLink;
                    if (Kinematic_check(OA, OB, BC, CA, config.LSledStroke))
                    {
                        config.LPivotLink = (uint)(result);
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
            if (textbox.Name == "Label_kinematic_c_hort_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    double OA = config.LPivotLink;
                    double OB = result;
                    double BC = config.LPivotSledY;
                    double CA = config.LLink;
                    if (Kinematic_check(OA, OB, BC, CA, config.LSledStroke))
                    {
                        config.LPivotSledXMin = (uint)(result);
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
            if (textbox.Name == "Label_kinematic_c_vert_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    double OA = config.LPivotLink;
                    double OB = config.LPivotSledXMin;
                    double BC = result;
                    double CA = config.LLink;
                    if (Kinematic_check(OA, OB, BC, CA, config.LSledStroke))
                    {
                        config.LPivotSledY = (uint)(result);
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
            if (textbox.Name == "Label_kinematic_a_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    double OA = config.LPivotLink;
                    double OB = config.LPivotSledXMin;
                    double BC = config.LPivotSledY;
                    double CA = result;
                    if (Kinematic_check(OA, OB, BC, CA, config.LSledStroke))
                    {
                        config.LLink = (uint)(result);
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
            if (textbox.Name == "Label_kinematic_d_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    if (result >= 0 && result <= 100)
                    {
                        config.LPivotFoot = (uint)result + config.LPivotLink;
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
            if (textbox.Name == "Label_travel_canvas")
            {
                if (int.TryParse(textbox.Text, out int result))
                {
                    if (result >= 10 && result <= 200)
                    {
                        config.LSledStroke = (uint)result;
                        UpdateJointDrawing(true);
                    }
                    else
                    {
                        DebugMessage?.Invoke("Pedal Kinematic calculation error");
                    }
                }
            }
        }
        private void NumericTextBox_PreviewTextInput(object sender, TextCompositionEventArgs e)
        {
            Regex regex = new Regex("^[.][0-9]+$|^[0-9]*[.]{0,4}[0-9]*$");

            System.Windows.Controls.TextBox textBox = (System.Windows.Controls.TextBox)sender;

            e.Handled = !regex.IsMatch(textBox.Text + e.Text);
        }

    }
}
