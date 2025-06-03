using Google.Protobuf;

namespace ProtbufTest
{
    public class ProtobufSerial(string port_name, Int32 baud_rate) : FrameSerial(port_name, baud_rate)
    {
        public async Task<T> ReceiveMessage<T>(int max_size = 500, int timeout = 30) where T : IMessage<T>, new()
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
    }
}
