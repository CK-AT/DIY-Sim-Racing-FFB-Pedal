using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using NullFX.CRC;

namespace ProtbufTest
{
    public class FrameSerial : AsyncSerial
    {
        private byte _break_char;

        public FrameSerial(string port_name, Int32 baud_rate, byte break_char = 0x00) : base(port_name, baud_rate) {
            _break_char = break_char;
        }
        
        public async Task<byte[]> ReceiveFrame(int max_size = 500, CancellationToken token = new CancellationToken())
        {
            try
            {
                var decoded = COBS.NET.COBS.Decode(await ReceiveDataTill(_break_char, max_size, token));
                var crc = Crc16.ComputeChecksum(Crc16Algorithm.Modbus, decoded, 0, decoded.Length - 2);
                if (crc == BitConverter.ToUInt16(decoded, decoded.Length - 2))
                {
                    return decoded.AsSpan(0, decoded.Length - 2).ToArray();
                }
                else
                {
                    return new byte[0];
                }
            }
            catch (ArgumentException) { }
            return new byte[0];
        }

        public bool WriteFrame(byte[] data)
        {
            MemoryStream ms = new MemoryStream();
            ms.Write(data, 0, data.Length);
            ms.Write(BitConverter.GetBytes(Crc16.ComputeChecksum(Crc16Algorithm.Modbus, data)), 0, 2);
            return WriteData(COBS.NET.COBS.Encode(ms.ToArray()));
        }
    }
}
