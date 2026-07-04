// Plan 19 - Pure-C# SimConnect wire format. Knows nothing about MSFS
// or SimHub; just encodes and decodes bytes. All MSFS-specific knowledge
// (SimVar list, sample-rate policy, reconnect) lives in MsfsSimConnectClient.
//
// Protocol details derived from publicly-known SimConnect SDK header
// (Microsoft) and cross-checked against EvenAR/node-simconnect (LGPL-3.0)
// for the integer constants. No source copied; constants reproduced from
// the published values per the clean-room rule in plan 19 section 9.
//
// Wire format (little-endian throughout):
//   Outbound 16-byte header: size(u32) protocol(u32) (0xF0000000|type)(u32) sendId(u32)
//   Inbound  12-byte header: size(u32) protocol(u32) recvId(u32)
// Bodies follow each header, message-specific.

using System;
using System.Text;

namespace DiyFfb.Msfs
{
    internal static class SimConnectProtocol
    {
        // ---------- header / framing constants ----------
        public const int OutboundHeaderSize = 16;
        public const int InboundHeaderSize  = 12;
        public const uint OutboundTypeMask  = 0xF0000000;

        // ---------- generic SimConnect constants ----------
        public const uint ObjectIdUser = 0;
        public const uint Unused       = 0xFFFFFFFF;

        // ---------- protocol versions (header offset 4) ----------
        // Reproduced from node-simconnect/src/enums/Protocol.ts:
        //   FSX_RTM = 0x2, FSX_SP1 = 0x3, FSX_SP2 = 0x4,
        //   KittyHawk = 0x5, SunRise = 0x6 (MSFS 2024).
        public const uint ProtocolFsxSp2    = 0x4;
        public const uint ProtocolKittyHawk = 0x5;
        public const uint ProtocolSunRise   = 0x6;

        // SunRise (MSFS 2024) Open packet body values. From
        // node-simconnect/src/SimConnectConnection.ts openPacketData table.
        public const string SunRiseAlias       = "RS";
        public const uint   SunRiseMajor       = 12;
        public const uint   SunRiseMinor       = 2;
        public const uint   SunRiseBuildMajor  = 282174;
        public const uint   SunRiseBuildMinor  = 999;

        // ---------- outbound function IDs (offset 8 with OutboundTypeMask) ----------
        public const uint SendIdOpen                   = 0x01;
        public const uint SendIdAddToDataDefinition    = 0x0c;
        public const uint SendIdClearDataDefinition    = 0x0d;
        public const uint SendIdRequestDataOnSimObject = 0x0e;

        // ---------- inbound recv IDs (offset 8) ----------
        // Sequential from 0 per the canonical SimConnect SDK header
        // (SIMCONNECT_RECV_ID enum). Cross-checked against node-simconnect.
        public const uint RecvIdNull           = 0;
        public const uint RecvIdException      = 1;
        public const uint RecvIdOpen           = 2;
        public const uint RecvIdQuit           = 3;
        public const uint RecvIdEvent          = 4;
        public const uint RecvIdSimObjectData  = 8;

        // ---------- data types (passed to AddToDataDefinition) ----------
        public const uint DataTypeFloat64 = 4;

        // ---------- period values (passed to RequestDataOnSimObject) ----------
        // SIM_FRAME is the heli-rate path; matches the EXE bridge.
        public const uint PeriodNever       = 0;
        public const uint PeriodOnce        = 1;
        public const uint PeriodVisualFrame = 2;
        public const uint PeriodSimFrame    = 3;
        public const uint PeriodSecond      = 4;

        // ---------- exception codes (reported in RecvException body) ----------
        public const uint ExceptionVersionMismatch = 5;

        // -----------------------------------------------------------------
        //  Outbound builders. Each returns the total byte count written.
        //  Caller pre-allocates a sufficiently large byte[] (caller knows
        //  the upper bound — see WorstCaseAddDataDefSize).
        // -----------------------------------------------------------------

        public static int WriteOpen(byte[] buf, uint protocol, string appName, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdOpen);
            o += WriteFixedAsciiString(buf, o, appName, 256);
            o += WriteUInt32(buf, o, 0);              // flags? always zero
            buf[o++] = 0x00;                          // single padding byte
            o += WriteFixedAsciiString(buf, o, SunRiseAlias, 3);
            o += WriteUInt32(buf, o, SunRiseMajor);
            o += WriteUInt32(buf, o, SunRiseMinor);
            o += WriteUInt32(buf, o, SunRiseBuildMajor);
            o += WriteUInt32(buf, o, SunRiseBuildMinor);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        public static int WriteAddToDataDefinition(byte[] buf, uint protocol,
            uint defineId, string datumName, string unitsName,
            uint dataType, float epsilon, uint datumId, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdAddToDataDefinition);
            o += WriteUInt32(buf, o, defineId);
            o += WriteFixedAsciiString(buf, o, datumName, 256);
            o += WriteFixedAsciiString(buf, o, unitsName ?? string.Empty, 256);
            o += WriteUInt32(buf, o, dataType);
            o += WriteFloat32(buf, o, epsilon);
            o += WriteUInt32(buf, o, datumId);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // ClearDataDefinition body is just the define ID. Used to reset a
        // data definition before re-registering (plan 23 re-registration) and
        // to recycle the scratch definition between per-var probes.
        public static int WriteClearDataDefinition(byte[] buf, uint protocol,
            uint defineId, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdClearDataDefinition);
            o += WriteUInt32(buf, o, defineId);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        public static int WriteRequestDataOnSimObject(byte[] buf, uint protocol,
            uint requestId, uint defineId, uint objectId, uint period,
            uint flags, uint origin, uint interval, uint limit, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdRequestDataOnSimObject);
            o += WriteUInt32(buf, o, requestId);
            o += WriteUInt32(buf, o, defineId);
            o += WriteUInt32(buf, o, objectId);
            o += WriteUInt32(buf, o, period);
            o += WriteUInt32(buf, o, flags);
            o += WriteUInt32(buf, o, origin);
            o += WriteUInt32(buf, o, interval);
            o += WriteUInt32(buf, o, limit);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Upper bound for any packet we build: Open (296), AddDataDef (544),
        // RequestData (48). Pre-allocate a single 1024-byte send buffer in
        // the client and reuse it.
        public const int MaxOutboundPacketBytes = 1024;

        // -----------------------------------------------------------------
        //  Inbound parsers. Caller has already framed one whole packet
        //  (read size from offset 0, read remainder into the same buffer).
        //  These parse the body in place.
        // -----------------------------------------------------------------

        public static bool TryReadHeader(byte[] buf, int len,
            out int totalSize, out uint protocol, out uint recvId)
        {
            totalSize = 0; protocol = 0; recvId = 0;
            if (len < InboundHeaderSize) return false;
            totalSize = (int)ReadUInt32(buf, 0);
            protocol  = ReadUInt32(buf, 4);
            recvId    = ReadUInt32(buf, 8);
            return totalSize >= InboundHeaderSize && totalSize <= len;
        }

        // Returns the payload offset within `buf` and the count of defines.
        // Payload bytes are then `buf[payloadOffset .. payloadOffset + defineCount * 8]`
        // for an all-FLOAT64 data definition with `defineCount` entries.
        public static bool TryReadSimObjectData(byte[] buf, int totalSize,
            out uint requestId, out uint objectId, out uint defineId,
            out uint flags, out uint defineCount, out int payloadOffset)
        {
            requestId = objectId = defineId = flags = defineCount = 0;
            payloadOffset = 0;

            // Body layout after the 12-byte header:
            //   reqID(4) objID(4) defID(4) flags(4) entryNumber(4) outOf(4) defineCount(4) payload...
            const int BodyControlBytes = 28;
            if (totalSize < InboundHeaderSize + BodyControlBytes) return false;

            int o = InboundHeaderSize;
            requestId   = ReadUInt32(buf, o); o += 4;
            objectId    = ReadUInt32(buf, o); o += 4;
            defineId    = ReadUInt32(buf, o); o += 4;
            flags       = ReadUInt32(buf, o); o += 4;
            o += 4; // entryNumber (unused for non-object-type requests)
            o += 4; // outOf
            defineCount = ReadUInt32(buf, o); o += 4;
            payloadOffset = o;
            return true;
        }

        public static bool TryReadException(byte[] buf, int totalSize,
            out uint exceptionCode, out uint sendId, out uint index)
        {
            exceptionCode = sendId = index = 0;
            const int BodyBytes = 12;
            if (totalSize < InboundHeaderSize + BodyBytes) return false;
            int o = InboundHeaderSize;
            exceptionCode = ReadUInt32(buf, o); o += 4;
            sendId        = ReadUInt32(buf, o); o += 4;
            index         = ReadUInt32(buf, o);
            return true;
        }

        // -----------------------------------------------------------------
        //  Byte-level primitives. Plain LE writes/reads against a byte[].
        // -----------------------------------------------------------------

        private static int WriteHeaderPlaceholder(byte[] buf, uint protocol, uint sendType)
        {
            // Offset 0: size — filled in by FinaliseHeader once total is known.
            WriteUInt32(buf, 0, 0);
            WriteUInt32(buf, 4, protocol);
            WriteUInt32(buf, 8, OutboundTypeMask | sendType);
            WriteUInt32(buf, 12, 0); // sendId placeholder
            return OutboundHeaderSize;
        }

        private static void FinaliseHeader(byte[] buf, int totalSize, uint sendId)
        {
            WriteUInt32(buf, 0, (uint)totalSize);
            WriteUInt32(buf, 12, sendId);
        }

        private static int WriteUInt32(byte[] buf, int offset, uint value)
        {
            buf[offset]     = (byte)(value        & 0xFF);
            buf[offset + 1] = (byte)((value >>  8) & 0xFF);
            buf[offset + 2] = (byte)((value >> 16) & 0xFF);
            buf[offset + 3] = (byte)((value >> 24) & 0xFF);
            return 4;
        }

        private static int WriteFloat32(byte[] buf, int offset, float value)
        {
            byte[] tmp = BitConverter.GetBytes(value);
            // BitConverter is LE on x86/x64 Windows; SimHub host is x86 (PE32).
            Buffer.BlockCopy(tmp, 0, buf, offset, 4);
            return 4;
        }

        private static int WriteFixedAsciiString(byte[] buf, int offset, string value, int fieldSize)
        {
            // Null-pad/truncate to exactly fieldSize bytes. ASCII because
            // SimConnect SimVar / unit names are ASCII identifiers.
            int copy = 0;
            if (!string.IsNullOrEmpty(value))
            {
                byte[] bytes = Encoding.ASCII.GetBytes(value);
                copy = Math.Min(bytes.Length, fieldSize);
                Buffer.BlockCopy(bytes, 0, buf, offset, copy);
            }
            for (int i = copy; i < fieldSize; i++) buf[offset + i] = 0;
            return fieldSize;
        }

        private static uint ReadUInt32(byte[] buf, int offset)
        {
            return (uint)(buf[offset]
                | (buf[offset + 1] <<  8)
                | (buf[offset + 2] << 16)
                | (buf[offset + 3] << 24));
        }

        public static double ReadFloat64(byte[] buf, int offset)
        {
            return BitConverter.ToDouble(buf, offset);
        }
    }
}
