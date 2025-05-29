// See https://aka.ms/new-console-template for more information
using Google.Protobuf;
using COBS.NET;
using pb = global::Google.Protobuf;
using ProtbufTest;

AsyncSerial serial = new AsyncSerial("COM4", 921600);
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
TestMessageWithOptions test = new TestMessageWithOptions();
test.Num = 22;
test.Mac = pb.ByteString.CopyFrom(new byte[] { 0x10, 0x20, 0x30, 0x40 });
test.Str = "Demo";
var watch = System.Diagnostics.Stopwatch.StartNew();
for (int outer_loop = 0; outer_loop < num_outer_loops;  outer_loop++)
{
    for (int i = 0; i < num_inner_loops; i++)
    {
        var memStream = new MemoryStream(test.CalculateSize());
        test.WriteTo(memStream);
        byte[] protobuf_data = memStream.ToArray();
        //Console.WriteLine("Demo protobuf encoded, size = {0} ( {1} )", test.CalculateSize(), BitConverter.ToString(protobuf_data).Replace("-", " "));
        //byte[] encodedData = COBS.NET.COBS.Encode(protobuf_data);
        //Console.WriteLine("COBS encoded, size = {0} ( {1} )", encodedData.Length, BitConverter.ToString(encodedData).Replace("-", " "));
        //serial.WriteRawData(encodedData);
        if (serial.WriteFrame(protobuf_data))
        {
            byte[] decodedData = await serial.ReceiveFrame(500, 10);
            if (decodedData.Length > 0)
            {
                test.MergeFrom(decodedData, 0, decodedData.Length);
                if (test.Num != 23)
                {
                    //Console.WriteLine("Mutation error!");
                    num_mutation_errors++;
                }
                else
                {
                    //Console.WriteLine("Echo OK.");
                }
                test.Num = 22;
                //Console.WriteLine("COBS decoded, size = {0} ( {1} )", decodedData.Length, BitConverter.ToString(decodedData).Replace("-", " "));
            }
            else
            {
                num_read_errors++;
                await Task.Delay(500);
                Console.WriteLine("Nothing received!");
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

