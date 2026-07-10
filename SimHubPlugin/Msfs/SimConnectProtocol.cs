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
        // Plan 24: K: key/sim events. MapClientEventToSimEvent (0x04) +
        // TransmitClientEvent (0x05) are FSX-era, stable across MSFS 2024 —
        // reproduced from node-simconnect and matching the SDK function order.
        public const uint SendIdMapClientEventToSimEvent = 0x04;
        public const uint SendIdTransmitClientEvent      = 0x05;
        public const uint SendIdAddToDataDefinition    = 0x0c;
        public const uint SendIdClearDataDefinition    = 0x0d;
        public const uint SendIdRequestDataOnSimObject = 0x0e;
        // Plan 24: write path. 0x10 is sequential in the same SDK enum
        // (SetDataOnSimObject). UNVALIDATED against live MSFS — cross-check the
        // id + body layout against node-simconnect (setDataOnSimObject) and
        // read-back before trusting. See plan 24 §4.2 clean-room caution.
        public const uint SendIdSetDataOnSimObject     = 0x10;
        // Plan 24 Phase 2: Input Event API. Function ids reproduced from
        // node-simconnect (SimConnectConnection: enumerateInputEvents=0x4f,
        // setInputEvent=0x51) per the clean-room rule. UNVALIDATED against live
        // MSFS — gate on a live enumerate + SetInputEvent round-trip (plan §4.5).
        public const uint SendIdEnumerateInputEvents   = 0x4f;
        public const uint SendIdSetInputEvent          = 0x51;

        // ---------- SetDataOnSimObject flags (SIMCONNECT_DATA_SET_FLAG) ----------
        // 0 = untagged: a single contiguous block of `ArrayCount` datums.
        public const uint DataSetFlagDefault = 0;

        // ---------- TransmitClientEvent group / flags (from MSFS 2024 SDK) ----------
        public const uint GroupPriorityHighest        = 1;          // SIMCONNECT_GROUP_PRIORITY_HIGHEST
        public const uint EventFlagGroupIdIsPriority  = 0x00000010; // interpret GroupID as priority

        // ---------- inbound recv IDs (offset 8) ----------
        // Sequential from 0 per the canonical SimConnect SDK header
        // (SIMCONNECT_RECV_ID enum). Cross-checked against node-simconnect.
        public const uint RecvIdNull           = 0;
        public const uint RecvIdException      = 1;
        public const uint RecvIdOpen           = 2;
        public const uint RecvIdQuit           = 3;
        public const uint RecvIdEvent          = 4;
        public const uint RecvIdSimObjectData  = 8;
        // Plan 24 Phase 2: RECV_ID_ENUMERATE_INPUT_EVENTS = 34, reproduced from
        // node-simconnect (SimConnectSocket RecvID enum). UNVALIDATED.
        public const uint RecvIdEnumerateInputEvents = 34;

        // ---------- INPUT_EVENT_TYPE (SIMCONNECT_INPUT_EVENT_TYPE) ----------
        public const uint InputEventTypeDouble = 0;
        public const uint InputEventTypeString = 1;

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

        // Plan 24: write a single FLOAT64 to one datum of a data definition on
        // the user aircraft. Body after the 16-byte header:
        //   DefineID(u32) ObjectID(u32) Flags(u32) ArrayCount(u32) cbUnitSize(u32) value(f64)
        // ArrayCount = 1 (one element), cbUnitSize = 8 (one FLOAT64). UNVALIDATED
        // wire format — see the caution on SendIdSetDataOnSimObject.
        public static int WriteSetDataOnSimObjectFloat64(byte[] buf, uint protocol,
            uint defineId, uint objectId, double value, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdSetDataOnSimObject);
            o += WriteUInt32(buf, o, defineId);
            o += WriteUInt32(buf, o, objectId);
            o += WriteUInt32(buf, o, DataSetFlagDefault);
            o += WriteUInt32(buf, o, 1);   // ArrayCount: one element
            o += WriteUInt32(buf, o, 8);   // cbUnitSize: one FLOAT64
            o += WriteFloat64(buf, o, value);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Plan 24: map a client-event id to a named sim/key event (K:). Body:
        //   ClientEventID(u32) EventName(256 ASCII). Fire-and-forget registration;
        //   a bad name yields an async exception, never closes the pipe.
        public static int WriteMapClientEventToSimEvent(byte[] buf, uint protocol,
            uint clientEventId, string eventName, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdMapClientEventToSimEvent);
            o += WriteUInt32(buf, o, clientEventId);
            o += WriteFixedAsciiString(buf, o, eventName ?? string.Empty, 256);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Plan 24: fire a mapped client event at an object with a DWORD value.
        //   ObjectID(u32) EventID(u32) dwData(u32) GroupID(u32) Flags(u32).
        public static int WriteTransmitClientEvent(byte[] buf, uint protocol,
            uint objectId, uint clientEventId, uint data, uint groupId, uint flags, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdTransmitClientEvent);
            o += WriteUInt32(buf, o, objectId);
            o += WriteUInt32(buf, o, clientEventId);
            o += WriteUInt32(buf, o, data);
            o += WriteUInt32(buf, o, groupId);
            o += WriteUInt32(buf, o, flags);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Plan 24 Phase 2: request an enumeration of the current aircraft's input
        // events. Body: RequestID(u32). Reply is one or more paged
        // RECV_ENUMERATE_INPUT_EVENTS (list template). UNVALIDATED wire format.
        public static int WriteEnumerateInputEvents(byte[] buf, uint protocol,
            uint requestId, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdEnumerateInputEvents);
            o += WriteUInt32(buf, o, requestId);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Plan 24 Phase 2: actuate an input event by its 64-bit hash with a
        // FLOAT64 value. Body: Hash(u64) cbUnitSize(u32)=8 value(f64). Mirrors
        // node-simconnect setInputEvent numeric path. UNVALIDATED wire format.
        public static int WriteSetInputEventFloat64(byte[] buf, uint protocol,
            ulong hash, double value, uint sendId)
        {
            int o = WriteHeaderPlaceholder(buf, protocol, SendIdSetInputEvent);
            o += WriteUInt64(buf, o, hash);
            o += WriteUInt32(buf, o, 8);   // cbUnitSize: one FLOAT64
            o += WriteFloat64(buf, o, value);
            FinaliseHeader(buf, o, sendId);
            return o;
        }

        // Upper bound for any packet we build: Open (296), AddDataDef (544),
        // RequestData (48), SetData (44), SetInputEvent (36). Pre-allocate a
        // single 1024-byte send buffer in the client and reuse it.
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

        // Plan 24 Phase 2: one entry of a RECV_ENUMERATE_INPUT_EVENTS page.
        public struct InputEventRecord
        {
            public string Name;
            public ulong  Hash;
            public uint   Type;
        }

        // Plan 24 Phase 2: parse one page of RECV_ENUMERATE_INPUT_EVENTS. It is a
        // list template — after the 12-byte header:
        //   requestID(u32) arraySize(u32) entryNumber(u32) outOf(u32)
        // then `arraySize` entries of { name[64] ASCII, hash(u64), type(u32) }.
        // DEFENSIVE (plan §4.5): a short/garbled packet yields only the entries
        // that actually fit — never reads past totalSize, never throws. Returns
        // false only if the control header itself doesn't fit. UNVALIDATED layout.
        public static bool TryReadEnumerateInputEvents(byte[] buf, int totalSize,
            out uint requestId, out uint arraySize, out uint entryNumber,
            out uint outOf, System.Collections.Generic.List<InputEventRecord> records)
        {
            requestId = arraySize = entryNumber = outOf = 0;
            const int ControlBytes = 16;
            const int EntryBytes = 64 + 8 + 4; // name + hash + type = 76
            if (totalSize < InboundHeaderSize + ControlBytes) return false;

            int o = InboundHeaderSize;
            requestId   = ReadUInt32(buf, o); o += 4;
            arraySize   = ReadUInt32(buf, o); o += 4;
            entryNumber = ReadUInt32(buf, o); o += 4;
            outOf       = ReadUInt32(buf, o); o += 4;

            if (records == null) return true;

            // Cap the declared count to what actually fits — a wrong/garbled
            // arraySize must not walk off the buffer.
            long avail = (long)totalSize - o;
            uint fit = avail > 0 ? (uint)(avail / EntryBytes) : 0;
            uint count = arraySize < fit ? arraySize : fit;

            for (uint i = 0; i < count; i++)
            {
                var rec = new InputEventRecord
                {
                    Name = ReadFixedAsciiString(buf, o, 64),
                    Hash = ReadUInt64(buf, o + 64),
                    Type = ReadUInt32(buf, o + 72)
                };
                records.Add(rec);
                o += EntryBytes;
            }
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

        private static int WriteFloat64(byte[] buf, int offset, double value)
        {
            byte[] tmp = BitConverter.GetBytes(value);
            Buffer.BlockCopy(tmp, 0, buf, offset, 8);
            return 8;
        }

        private static int WriteUInt64(byte[] buf, int offset, ulong value)
        {
            for (int i = 0; i < 8; i++) buf[offset + i] = (byte)((value >> (8 * i)) & 0xFF);
            return 8;
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

        private static ulong ReadUInt64(byte[] buf, int offset)
        {
            ulong v = 0;
            for (int i = 0; i < 8; i++) v |= (ulong)buf[offset + i] << (8 * i);
            return v;
        }

        // Read a null-terminated ASCII string from a fixed-size field.
        private static string ReadFixedAsciiString(byte[] buf, int offset, int fieldSize)
        {
            int end = offset;
            int limit = offset + fieldSize;
            while (end < limit && buf[end] != 0) end++;
            return end > offset ? Encoding.ASCII.GetString(buf, offset, end - offset) : string.Empty;
        }
    }
}
