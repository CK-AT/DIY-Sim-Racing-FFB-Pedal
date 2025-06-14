// See https://aka.ms/new-console-template for more information
using Google.Protobuf;
using COBS.NET;
using pb = global::Google.Protobuf;
using ProtbufTest;

ProtobufSerial serial = new ProtobufSerial("COM3", 3000000);
while (serial.Open(true) == false)
{
    Console.WriteLine("Failed to open port, retrying...");
    await Task.Delay(1000);
}

int num_read_errors = 0;
int num_send_errors = 0;
int num_mutation_errors = 0;
int num_inner_loops = 100;
int num_outer_loops = 10;
Message test = new Message();
FFBAction action = new FFBAction();
action.FunctionId = FunctionID.Brake;
action.AutomotivePedal = new AutomotivePedalFFBAction();
action.AutomotivePedal.TriggerAbs = true;
//test.FfbAction = action;

var axis_cfg = new AxisConfig();
axis_cfg.Store = false;
axis_cfg.AxisId = AxisID._1;
axis_cfg.KinematicParameters = new KinematicParameters();
axis_cfg.KinematicParameters.CoeffsForceFactorOverContactPointPos.Clear();
axis_cfg.KinematicParameters.CoeffsForceFactorOverContactPointPos.AddRange([0.0, 1.1, 2.2, 3.3, 4.4]);
axis_cfg.KinematicParameters.CoeffsSledPosOverContactPointPos.Clear();
axis_cfg.KinematicParameters.CoeffsSledPosOverContactPointPos.AddRange([5.5, 6.6, 7.7, 8.8, 9.9]);
axis_cfg.KfConstVel = new KFConstVelConfig();
axis_cfg.KfConstVel.NoiseScaling = 128;
//test.AxisConfig = axis_cfg;

AxisAction axis_action = new AxisAction();
axis_action.Restart = true;
//test.AxisAction = axis_action;

FunctionConfig function_config = new FunctionConfig();
function_config.Base = new FunctionBase();
function_config.Base.OutputMin = -2.0f;
function_config.Base.OutputMax = 2.0f;
function_config.Base.ControllerOutputAxis = ControllerAxis.RZ;
function_config.Base.FunctionId = FunctionID.Brake;
function_config.Base.LinkedAxes.AddRange([AxisID._1, AxisID.AxisUndefined, AxisID.AxisUndefined, AxisID.AxisUndefined]);
function_config.Base.OutputMode = OutputMode.Force;
function_config.Base.Store = false;
function_config.AutomotivePedal = new AutomotivePedalConfig();
function_config.AutomotivePedal.DamperConfig = new DamperConfig();
function_config.AutomotivePedal.DamperConfig.PositiveFactor = 0.1f;
function_config.AutomotivePedal.DamperConfig.NegativeFactor = 0.1f;
function_config.AutomotivePedal.PosEnd = 100;
function_config.AutomotivePedal.PosIdle = 150;
function_config.AutomotivePedal.ForceCurveConfig = new SplineForceCurveConfig();
function_config.AutomotivePedal.ForceCurveConfig.PosMin = 100;
function_config.AutomotivePedal.ForceCurveConfig.PosMax = 150;
function_config.AutomotivePedal.ForceCurveConfig.FMin = 70;
function_config.AutomotivePedal.ForceCurveConfig.FMax = 500;
function_config.AutomotivePedal.ForceCurveConfig.ForceDirection = ForceDirection.Subtract;
function_config.AutomotivePedal.ForceCurveConfig.FRelPoints.AddRange([0, 20, 40, 60, 80, 100]);
function_config.AutomotivePedal.ForceCurveConfig.CubicSplineParamsA.AddRange([0.0f, 0.0f, 0.0f, 0.0f, 0.0f]);
function_config.AutomotivePedal.ForceCurveConfig.CubicSplineParamsB.AddRange([0.0f, 0.0f, 0.0f, 0.0f, 0.0f]);
test.FunctionConfig = function_config;


var watch = System.Diagnostics.Stopwatch.StartNew();
int cnt = 0;
JsonFormatter json_fromatter = new JsonFormatter(JsonFormatter.Settings.Default);
Console.WriteLine(json_fromatter.Format(test));
//serial.WriteMessage(test);
while (true)
{
    if (cnt == 0)
    {
        //serial.WriteMessage(test);
    }
    Message rx_msg = await serial.ReceiveMessage<Message>();
    if (rx_msg != null)
    {
        if (rx_msg.PayloadCase == Message.PayloadOneofCase.AxisLogMessage)
        {
            Console.WriteLine("{0} : {1}", rx_msg.AxisLogMessage.AxisId, rx_msg.AxisLogMessage.Msg.TrimEnd());
        }
        else if (rx_msg.PayloadCase == Message.PayloadOneofCase.AxisState)
        {
            //Console.WriteLine("{0} : f = {1}N, x = {2}mm", rx_msg.AxisState.AxisId, rx_msg.AxisState.Force, rx_msg.AxisState.Position);
        }
        else if (rx_msg.PayloadCase == Message.PayloadOneofCase.GatewayLogMessage)
        {
            Console.WriteLine("{0} : {1}", rx_msg.GatewayLogMessage.GatewayId, rx_msg.GatewayLogMessage.Msg.TrimEnd());
        }
        else if (rx_msg.PayloadCase == Message.PayloadOneofCase.GatewayState)
        {
            //Console.WriteLine("axes_present: 0x{0:X2}", rx_msg.GatewayState.AxesPresent);
        }
        else
        {
            Console.WriteLine("{0}", rx_msg);
        }

    }
    cnt++;
    if (cnt > 1000) cnt = 0;
}
for (int outer_loop = 0; outer_loop < num_outer_loops;  outer_loop++)
{
    for (int i = 0; i < num_inner_loops; i++)
    {
        //var memStream = new MemoryStream(test.CalculateSize());
        //test.WriteTo(memStream);
        //byte[] protobuf_data = memStream.ToArray();
        //Console.WriteLine("Demo protobuf encoded, size = {0} ( {1} )", test.CalculateSize(), BitConverter.ToString(protobuf_data).Replace("-", " "));
        //byte[] encodedData = COBS.NET.COBS.Encode(protobuf_data);
        //Console.WriteLine("COBS encoded, size = {0} ( {1} )", encodedData.Length, BitConverter.ToString(encodedData).Replace("-", " "));
        //serial.WriteRawData(encodedData);
        if (serial.WriteMessage(test))
        {
            Message rx_msg = await serial.ReceiveMessage<Message>();
            if (rx_msg != null)
            {
                //test.ClearPayload();
                //test.MergeFrom(decodedData, 0, decodedData.Length);
                //if (test.AxisCfg.AxisId != 1)
                //{
                //    //Console.WriteLine("Mutation error!");
                //    num_mutation_errors++;
                //}
                //else
                //{
                //    //Console.WriteLine("Echo OK.");
                //}
                //test.AxisCfg.AxisId = 0;
                ////Console.WriteLine("COBS decoded, size = {0} ( {1} )", decodedData.Length, BitConverter.ToString(decodedData).Replace("-", " "));
            }
            else
            {
                num_read_errors++;
                //await Task.Delay(500);
                //Console.WriteLine("Nothing received!");
            }
        }
        else
        {
            num_send_errors++;
            await Task.Delay(500);
            Console.WriteLine("Failed to write...");
            //while (serial.Open() == false)
            //{
            //    Console.WriteLine("Failed to open port, retrying...");
            //    await Task.Delay(1000);
            //}
        }
    }
    serial.Close();
    serial.Open(true);
}

watch.Stop();
var elapsedMs = watch.ElapsedMilliseconds;

Console.WriteLine("loops: {0} @ {1}ms/loop; send_errors: {2}, read_errors: {3}, mutation_errors: {4}", num_inner_loops * num_outer_loops, elapsedMs / (num_inner_loops * num_outer_loops), num_send_errors, num_read_errors, num_mutation_errors);

serial.Close();

