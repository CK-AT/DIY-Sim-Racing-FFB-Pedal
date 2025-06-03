// See https://aka.ms/new-console-template for more information
using Google.Protobuf;
using COBS.NET;
using pb = global::Google.Protobuf;
using ProtbufTest;

ProtobufSerial serial = new ProtobufSerial("COM4", 921600);
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
action.TriggerAbs = true;
//test.FfbAction = action;

var axis_cfg = new AxisConfig();
axis_cfg.Store = false;
axis_cfg.AxisId = AxisID.Axis1;
axis_cfg.CoeffsForceFactorOverContactPointPos.Clear();
axis_cfg.CoeffsForceFactorOverContactPointPos.AddRange([0.0, 1.1, 2.2, 3.3, 4.4]);
axis_cfg.CoeffsSledPosOverContactPointPos.Clear();
axis_cfg.CoeffsSledPosOverContactPointPos.AddRange([5.5, 6.6, 7.7, 8.8, 9.9]);
axis_cfg.KfConstVel = new KFConstVelConfig();
axis_cfg.KfConstVel.NoiseScaling = 128;
//test.AxisConfig = axis_cfg;

AxisAction axis_action = new AxisAction();
axis_action.Restart = true;
//test.AxisAction = axis_action;

var watch = System.Diagnostics.Stopwatch.StartNew();
int cnt = 0;
JsonFormatter json_fromatter = new JsonFormatter(JsonFormatter.Settings.Default);
Console.WriteLine(json_fromatter.Format(test));
while (true)
{
    if (cnt == 0)
    {
        serial.WriteMessage(test);
    }
    Message rx_msg = await serial.ReceiveMessage<Message>();
    if (rx_msg != null)
    {
        if (rx_msg.PayloadCase == Message.PayloadOneofCase.LogMessage)
        {
            Console.WriteLine("{0} : {1}", rx_msg.LogMessage.AxisId, rx_msg.LogMessage.Msg.TrimEnd());
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

