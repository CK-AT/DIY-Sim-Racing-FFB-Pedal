// See https://aka.ms/new-console-template for more information
using Google.Protobuf;
using COBS.NET;
using pb = global::Google.Protobuf;
using ProtbufTest;

AsyncSerial serial = new AsyncSerial("COM15", 921600);
if (serial.Open() == true)
{
    int num_read_errors = 0;
    int num_decode_errors = 0;
    int num_mutation_errors = 0;
    int num_loops = 1000;
    var watch = System.Diagnostics.Stopwatch.StartNew();
    TestMessageWithOptions test = new TestMessageWithOptions();
    test.Num = 22;
    test.Mac = pb.ByteString.CopyFrom(new byte[] { 0x10, 0x20, 0x30, 0x40 });
    test.Str = "Demo";
    for (int i = 0; i < num_loops; i++)
    {
        var memStream = new MemoryStream(test.CalculateSize());
        test.WriteTo(memStream);
        byte[] protobuf_data = memStream.ToArray();
        //Console.WriteLine("Demo protobuf encoded, size = {0} ( {1} )", test.CalculateSize(), BitConverter.ToString(protobuf_data).Replace("-", " "));
        byte[] encodedData = COBS.NET.COBS.Encode(protobuf_data);
        //Console.WriteLine("COBS encoded, size = {0} ( {1} )", encodedData.Length, BitConverter.ToString(encodedData).Replace("-", " "));
        serial.WriteData(encodedData);
        byte[] buffer = await serial.ReceiveData(500, 10);
        if (buffer.Length > 0)
        {
            //Console.WriteLine("Data received, size = {0} ( {1} )", buffer.Length, BitConverter.ToString(buffer).Replace("-", " "));
            try
            {
                byte[] decodedData = COBS.NET.COBS.Decode(buffer);
                test.MergeFrom(decodedData, 0, decodedData.Length);
                if (test.Num != 23) num_mutation_errors++;
                test.Num = 22;
                //Console.WriteLine("COBS decoded, size = {0} ( {1} )", decodedData.Length, BitConverter.ToString(decodedData).Replace("-", " "));
            } catch (System.ArgumentException)
            {
                num_decode_errors++;
            }
        } else
        {
            num_read_errors++;
            //Console.WriteLine("Nothing received");
        }
    }
    watch.Stop();
    var elapsedMs = watch.ElapsedMilliseconds;

    Console.WriteLine("loops: {0} @ {1}ms/loop;  read_errors: {2};  decode_errors: {3}, mutation_errors: {4}", num_loops, elapsedMs / num_loops, num_read_errors, num_decode_errors, num_mutation_errors);

    serial.Close();
}

