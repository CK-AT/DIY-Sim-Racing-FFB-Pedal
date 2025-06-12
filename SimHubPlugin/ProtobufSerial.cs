using System;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Google.Protobuf;
using SimHub.Plugins.OutputPlugins.GraphicalDash;

namespace ProtbufTest
{
    public class ProtobufSerial<T> : FrameSerial where T : IMessage<T>, new()
    {
        public ProtobufSerial(string port_name, Int32 baud_rate) : base(port_name, baud_rate) {
            Task.Run(async () => await RxTask());
        }
        public delegate void MessageHandler(object sender, object message);
        public event MessageHandler OnMessage;
        private CancellationTokenSource cts = new CancellationTokenSource();

 
        public new void Close()
        {
            base.Close();
        }

        public async Task<T> ReceiveMessage(int max_size = 500, int timeout = 30)
        {
            var buffer = await ReceiveFrame(max_size, timeout);
            if (buffer.Length == 0) return default(T);
            T msg = new T();
            msg.MergeFrom(buffer);
            return msg;
        }

        public bool WriteMessage(IMessage msg)
        {
            var size = msg.CalculateSize();
            if (size == 0) return false;
            var ms = new MemoryStream(size);
            msg.WriteTo(ms);
            return WriteFrame(ms.ToArray());
        }
        private async Task RxTask()
        {
            while (true)
            {
                try
                {
                    var msg = await ReceiveMessage();
                    if (msg != null)
                    {
                        OnMessage(this, msg);
                    }
                    if (cts.IsCancellationRequested) break;
                }
                catch (Exception) { }
            }
        }

    }
}
