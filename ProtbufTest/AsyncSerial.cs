using Duplicati.StreamUtil;
using System;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;

namespace ProtbufTest
{
    public class AsyncSerial
    {
        SerialPort port;
        byte[] rx_buffer = new byte[1024];
        BufferBlock<byte> rx_fifo = new BufferBlock<byte>();
        CancellationTokenSource cts = new CancellationTokenSource();
        Task rx_task;

        public AsyncSerial(string port_name, Int32 baud_rate)
        {
            port = new SerialPort(port_name, baud_rate);
        }
        
        public bool Open()
        {
            try
            {
                port.Open();
            }
            catch (System.IO.FileNotFoundException)
            {
                return false;
            }
            rx_task = Task.Run(async () => await RxTask());
            return true;
        }
        private async Task RxTask()
        {
            while (true)
            {
                var num_bytes = await port.BaseStream.ReadAsync(rx_buffer, 0, rx_buffer.Length, cts.Token);
                for (int i = 0; i < num_bytes; i++)
                {
                    rx_fifo.Post(rx_buffer[i]);
                }
                if (cts.IsCancellationRequested) break;
            }
        }

        public async Task<byte[]> ReceiveData(int max_size=500, int timeout=30)
        {
            var ms = new MemoryStream();
            byte[] val = new byte[1];
            int num_bytes = 0;
            TimeSpan curr_timeout = TimeSpan.FromMilliseconds(timeout);
            try
            {
                while (num_bytes < max_size)
                {
                    val[0] = await rx_fifo.ReceiveAsync(curr_timeout);
                    ms.Write(val);
                    curr_timeout = TimeSpan.FromMilliseconds(2);
                }
            } catch (TimeoutException)
            {

            }
            return ms.ToArray();
        }

        public void WriteData(byte[] data)
        {
            port.Write(data, 0, data.Length);
        }
        public void Close()
        {
            cts.Cancel();
            port.Close();
        }
    }
}
