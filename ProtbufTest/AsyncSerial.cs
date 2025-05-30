using System.IO.Ports;
using System.Threading.Tasks.Dataflow;

namespace ProtbufTest
{
    public class AsyncSerial
    {
        SerialPort port;
        BufferBlock<byte> rx_fifo = new BufferBlock<byte>();
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
                    foreach (var item in chunk.ToArray().Take(num_bytes))
                    {
                        rx_fifo.Post(item);
                    }
                    if (cts.IsCancellationRequested) break;
                }
            }
            catch (UnauthorizedAccessException) { }
        }

        public async Task<byte[]> ReceiveData(int max_size=500, int timeout=30)
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
                    var item = await rx_fifo.ReceiveAsync(curr_timeout);
                    ms.WriteByte(item);
                    num_bytes++;
                    curr_timeout = TimeSpan.FromMilliseconds(3);
                }
            } catch (TimeoutException)
            {

            }
            return ms.ToArray();
        }

        public async Task<byte[]> ReceiveDataTill(byte break_char = 0x00, int max_size = 500, int timeout = 30)
        {
            if (!port.IsOpen && _auto_reconnect)
            {
                Open(true);
            }
            var ms = new MemoryStream();
            int num_bytes = 0;
            bool break_char_received = false;
            // use timeout for the first byte but reduce to 2ms for successive bytes
            TimeSpan curr_timeout = TimeSpan.FromMilliseconds(timeout);
            try
            {
                while (num_bytes < max_size)
                {
                    var item = await rx_fifo.ReceiveAsync(curr_timeout);
                    if (item != 0)
                    {
                        ms.WriteByte(item);
                        num_bytes++;
                        break_char_received = true;
                    }
                    else if (break_char_received)
                    {
                        ms.WriteByte(item);
                        break;
                    }
                    curr_timeout = TimeSpan.FromMilliseconds(3);
                }
            }
            catch (TimeoutException) { }
            return ms.ToArray();
        }

        public bool WriteData(byte[] data)
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

        public void Close()
        {
            _auto_reconnect = false;
            cts.Cancel();
            port.Close();
        }
    }
}
