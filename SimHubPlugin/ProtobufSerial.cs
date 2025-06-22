using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Google.Protobuf;

namespace ProtbufTest
{
    public class ProtobufSerial<T> : FrameSerial where T : IMessage<T>, new()
    {
        public ProtobufSerial(string port_name, Int32 baud_rate) : base(port_name, baud_rate) {
        }
        public delegate void MessageHandler(object sender, object message);
        public event MessageHandler OnMessage;
        private CancellationTokenSource cts = new CancellationTokenSource();

        
        public new bool Open(bool auto_reconnect = false)
        {
            cts.Cancel();
            cts = new CancellationTokenSource();
            Task.Run(async () => await RxTask(cts.Token));
            return base.Open(auto_reconnect);
        }

        public new void Close()
        {
            cts.Cancel();
            base.Close();
        }

        public async Task<T> ReceiveMessage(int max_size = 500, CancellationToken token = new CancellationToken())
        {
            var buffer = await ReceiveFrame(max_size, token);
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
        private async Task RxTask(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    var msg = await ReceiveMessage(token: token);
                    if (msg != null)
                    {
                        OnMessage?.Invoke(this, msg);
                    }
                }
                catch (Exception) { }
            }
        }

    }
}
