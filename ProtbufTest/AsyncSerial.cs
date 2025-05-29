using System.IO.Ports;
using System.Threading.Tasks.Dataflow;
using COBS.NET;

namespace ProtbufTest
{
    public class AsyncSerial
    {
        SerialPort port;
        BufferBlock<Memory<byte>> rx_fifo = new BufferBlock<Memory<byte>>();
        CancellationTokenSource cts = new CancellationTokenSource();
        bool _auto_reconnect = false;

        public AsyncSerial(string port_name, Int32 baud_rate)
        {
            port = new SerialPort(port_name, baud_rate);
        }
        
        public bool Open(bool auto_reconnect=false)
        {
            _auto_reconnect=auto_reconnect;
            cts.Cancel();
            cts = new CancellationTokenSource();
            try
            {
                port.Open();
            }
            catch (FileNotFoundException)
            {
                return false;
            }
            catch (UnauthorizedAccessException)
            {
                return false;
            }
            Task.Run(async () => await RxTask());
            return true;
        }
        private async Task RxTask()
        {
            try
            {
                while (true)
                {
                    var chunk = new Memory<byte>(new byte[32]);
                    var num_bytes = await port.BaseStream.ReadAsync(chunk, cts.Token);
                    rx_fifo.Post(chunk.Slice(0, num_bytes));
                    if (cts.IsCancellationRequested) break;
                }
            }
            catch (UnauthorizedAccessException) { }
        }

        public async Task<byte[]> ReceiveRawData(int max_size=500, int timeout=30)
        {
            if (!port.IsOpen && _auto_reconnect)
            {
                Open(true);
            }
            var ms = new MemoryStream();
            int num_bytes = 0;
            // use timeout for the first byte but reduce to 2ms for successive bytes
            TimeSpan curr_timeout = TimeSpan.FromMilliseconds(timeout);
            try
            {
                while (num_bytes < max_size)
                {
                    var chunk = await rx_fifo.ReceiveAsync(curr_timeout);
                    ms.Write(chunk.Span);
                    curr_timeout = TimeSpan.FromMilliseconds(2);
                }
            } catch (TimeoutException)
            {

            }
            return ms.ToArray();
        }
        public async Task<byte[]> ReceiveFrame(int max_size = 500, int timeout = 30)
        {
            try
            {
                return COBS.NET.COBS.Decode(await ReceiveRawData(max_size, timeout));
            }
            catch (ArgumentException) { }
            return [];
        }

        public bool WriteRawData(byte[] data)
        {
            if (!port.IsOpen && _auto_reconnect)
            {
                if (!Open(true)) return false;
            }
            if (port.IsOpen)
            {
                port.Write(data, 0, data.Length);
                return true;
            }
            return false;
        }
        public bool WriteFrame(byte[] data)
        {
            return WriteRawData(COBS.NET.COBS.Encode(data));
        }
        public void Close()
        {
            _auto_reconnect = false;
            cts.Cancel();
            port.Close();
        }
    }
}
