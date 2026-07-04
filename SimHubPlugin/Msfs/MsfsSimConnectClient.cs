// Plan 19 - In-process SimConnect client. Talks to MSFS 2024 directly over
// the named pipe \\.\pipe\Microsoft Flight Simulator\SimConnect, registers a
// SimVar data definition, subscribes at SIM_FRAME, and hands each sample to a
// callback for the plugin to publish into latestMsfsPacket.
//
// Plan 23 - The data definition is now DYNAMIC and FAILURE-TOLERANT. It is the
// 36 fixed defaults (MsfsSimVarTable, always valid) followed by zero or more
// graph-declared custom vars (SimVars / LVARs). Custom vars are validated in
// isolation before joining the streaming definition: each is added to a scratch
// definition and probed with PERIOD_ONCE. A bad A: name raises an exception on
// its probe and is dropped (recorded in FailedVars); everything else keeps
// streaming. Registration is decoupled from streaming so a typo can never take
// the whole telemetry stream down.
//
// Lifecycle mirrors the EXE bridge's outer loop: connect -> handshake ->
// register -> subscribe -> read; on QUIT or pipe break, tear down and retry
// every 2 seconds. Safe to Start() before MSFS is running. Custom-var changes
// (SetCustomVars) are picked up by the worker thread and trigger an in-session
// re-registration; the caller never writes to the pipe.

using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.Win32.SafeHandles;

namespace DiyFfb.Msfs
{
    // A graph-declared custom variable to register on top of the 36 defaults.
    internal sealed class MsfsCustomVar
    {
        public readonly string Alias;  // graph-facing key (MSFS.<Alias>)
        public readonly string Name;   // raw SimConnect datum, e.g. "L:FOO" / "GENERAL ENG RPM:1"
        public readonly string Units;  // SimConnect unit string

        public MsfsCustomVar(string alias, string name, string units)
        {
            Alias = alias ?? "";
            Name  = name ?? "";
            Units = units ?? "";
        }
    }

    // A custom var whose PERIOD_ONCE probe raised — dropped from the stream.
    internal sealed class MsfsFailedVar
    {
        public readonly string Alias;
        public readonly string Name;
        public readonly uint   ExceptionCode;

        public MsfsFailedVar(string alias, string name, uint code)
        {
            Alias = alias; Name = name; ExceptionCode = code;
        }
    }

    internal sealed class MsfsSimConnectClient : IDisposable
    {
        private const string PipeName = "Microsoft Flight Simulator\\SimConnect";
        private const uint Protocol = SimConnectProtocol.ProtocolSunRise;
        private const uint DefineId  = 1;
        private const uint RequestId = 1;
        // Scratch ids for per-var probing — a separate range so they can never
        // collide with the streaming definition/request.
        private const uint ProbeDefineId  = 1000;
        private const uint ProbeRequestId = 1000;
        private const int  ConnectTimeoutMs   = 1000;
        private const int  HandshakeTimeoutMs = 5000;
        private const int  ReconnectDelayMs   = 2000;
        // A bad name's exception is prompt on a local pipe; a good var yields no
        // PERIOD_ONCE reply while the sim is paused. So the drain rule is
        // invalid <=> exception, and "no exception within this window" = valid.
        private const int  ProbeTimeoutMs = 100;
        // Guardrail on custom-var count (buffer + per-frame cost).
        private const int  MaxCustomVars = 64;

        private readonly int DefaultCount = MsfsSimVarTable.SampleCount;

        private readonly Action<double[], IReadOnlyDictionary<string, double>> _onSample;
        private readonly Action<string> _log;
        private readonly object _stateLock = new object();

        private Thread _worker;
        private CancellationTokenSource _cts;
        private NamedPipeClientStream _pipe;
        private uint _sendIdSeq;
        private volatile bool _connected;
        private DateTime _lastSampleUtc = DateTime.MinValue;

        // Desired custom-var list (set by SetCustomVars, consumed by the worker).
        private List<MsfsCustomVar> _pendingCustoms = new List<MsfsCustomVar>();
        private volatile bool _customsDirty;

        // Survivors currently in the streaming definition, in payload order:
        // DefaultCount defaults first, then surviving customs. Owned by the
        // worker; a snapshot is used per frame. Failed customs recorded aside.
        private List<MsfsCustomVar> _survivingCustoms = new List<MsfsCustomVar>();
        private List<MsfsFailedVar> _failed = new List<MsfsFailedVar>();

        // Reused buffers. _rxBuffer holds one inbound packet; ~500 FLOAT64 slots
        // incl. header, ample for 36 defaults + up to MaxCustomVars customs.
        private readonly byte[] _rxBuffer = new byte[4096];
        private readonly byte[] _txBuffer = new byte[SimConnectProtocol.MaxOutboundPacketBytes];
        private double[] _sampleBuffer = new double[MsfsSimVarTable.SampleCount];

        public MsfsSimConnectClient(
            Action<double[], IReadOnlyDictionary<string, double>> onSample,
            Action<string> log)
        {
            _onSample = onSample ?? throw new ArgumentNullException(nameof(onSample));
            _log      = log ?? (_ => { });
        }

        public bool IsConnected => _connected;
        public DateTime LastSampleUtc { get { lock (_stateLock) return _lastSampleUtc; } }

        // Snapshot of custom vars whose probe failed on the current aircraft.
        public IReadOnlyList<MsfsFailedVar> FailedVars
        {
            get { lock (_stateLock) return _failed.ToArray(); }
        }

        // Replace the desired custom-var list. Worker-thread ownership: this only
        // stores the pending list and flags it; the worker re-registers at a safe
        // point. Never writes to the pipe from the caller's thread. Excess vars
        // beyond MaxCustomVars are dropped (logged by the worker on apply).
        public void SetCustomVars(IReadOnlyList<MsfsCustomVar> customs)
        {
            var copy = new List<MsfsCustomVar>();
            if (customs != null)
            {
                foreach (var c in customs)
                {
                    if (c == null || string.IsNullOrEmpty(c.Name) || string.IsNullOrEmpty(c.Alias))
                        continue;
                    copy.Add(c);
                }
            }
            lock (_stateLock) { _pendingCustoms = copy; }
            _customsDirty = true;
        }

        public void Start()
        {
            lock (_stateLock)
            {
                if (_worker != null) return;
                _cts = new CancellationTokenSource();
                _worker = new Thread(WorkerLoop)
                {
                    IsBackground = true,
                    Name = "MsfsSimConnectClient",
                };
                _worker.Start();
            }
        }

        public void Stop()
        {
            CancellationTokenSource cts;
            Thread worker;
            NamedPipeClientStream pipe;
            lock (_stateLock)
            {
                cts = _cts; worker = _worker; pipe = _pipe;
                _cts = null; _worker = null;
            }
            if (cts == null) return;

            try { cts.Cancel(); } catch { }
            try { pipe?.Dispose(); } catch { }
            try { worker?.Join(2000); } catch { }
            try { cts.Dispose(); } catch { }
        }

        public void Dispose() => Stop();

        // -----------------------------------------------------------------
        //  Worker thread
        // -----------------------------------------------------------------

        private enum LoopResult { Disconnected, Reconfigure }

        private void WorkerLoop()
        {
            CancellationToken ct;
            lock (_stateLock) { ct = _cts.Token; }

            while (!ct.IsCancellationRequested)
            {
                try
                {
                    using (var pipe = new NamedPipeClientStream(
                        ".", PipeName, PipeDirection.InOut, PipeOptions.None))
                    {
                        lock (_stateLock) { _pipe = pipe; }

                        try
                        {
                            pipe.Connect(ConnectTimeoutMs);
                        }
                        catch (TimeoutException)
                        {
                            DelayOrCancel(ReconnectDelayMs, ct);
                            continue;
                        }

                        if (ct.IsCancellationRequested) break;

                        if (!Handshake(pipe, ct))
                        {
                            DelayOrCancel(ReconnectDelayMs, ct);
                            continue;
                        }

                        // (Re)configure the definition in-session until the pipe
                        // breaks or MSFS quits. A custom-var change comes back as
                        // Reconfigure and rebuilds without reconnecting. The first
                        // configure of a fresh session needs no clear (the define
                        // doesn't exist yet); re-registrations do.
                        LoopResult result = LoopResult.Reconfigure;
                        bool firstConfigure = true;
                        while (result == LoopResult.Reconfigure && !ct.IsCancellationRequested)
                        {
                            Configure(pipe, clearFirst: !firstConfigure);
                            firstConfigure = false;
                            _connected = true;
                            result = ReadLoop(pipe, ct);
                        }
                    }
                }
                catch (Exception ex) when (
                    ex is IOException || ex is ObjectDisposedException ||
                    ex is EndOfStreamException || ex is UnauthorizedAccessException)
                {
                    // Expected: pipe broke, MSFS quit, or we got Stop()'d.
                }
                catch (Exception ex)
                {
                    _log("[MsfsSimConnect] unexpected: " + ex.Message);
                }
                finally
                {
                    _connected = false;
                    lock (_stateLock) { _pipe = null; }
                }

                DelayOrCancel(ReconnectDelayMs, ct);
            }

            _log("[MsfsSimConnect] worker exiting.");
        }

        // -----------------------------------------------------------------
        //  Handshake
        // -----------------------------------------------------------------

        private bool Handshake(NamedPipeClientStream pipe, CancellationToken ct)
        {
            uint sendId = NextSendId();
            int n = SimConnectProtocol.WriteOpen(_txBuffer, Protocol, "DiyFfbPlugin", sendId);
            pipe.Write(_txBuffer, 0, n);
            pipe.Flush();

            DateTime deadline = DateTime.UtcNow.AddMilliseconds(HandshakeTimeoutMs);
            while (!ct.IsCancellationRequested && DateTime.UtcNow < deadline)
            {
                if (!ReadOnePacket(pipe, out int totalSize)) return false;
                if (!SimConnectProtocol.TryReadHeader(_rxBuffer, totalSize,
                        out _, out _, out uint recvId)) continue;

                if (recvId == SimConnectProtocol.RecvIdOpen)
                {
                    return true;
                }
                if (recvId == SimConnectProtocol.RecvIdException)
                {
                    if (SimConnectProtocol.TryReadException(_rxBuffer, totalSize,
                            out uint code, out uint badSendId, out _))
                    {
                        _log($"[MsfsSimConnect] Open rejected: exception={code} sendId={badSendId}");
                    }
                    return false;
                }
            }
            _log("[MsfsSimConnect] handshake timeout.");
            return false;
        }

        // -----------------------------------------------------------------
        //  Registration (defaults + probed customs) + subscription
        // -----------------------------------------------------------------

        // Rebuild the streaming data definition, then subscribe at SIM_FRAME.
        // Defaults are trusted (added unprobed); customs are validated one at a
        // time and only survivors join the definition.
        private void Configure(NamedPipeClientStream pipe, bool clearFirst)
        {
            _customsDirty = false;

            List<MsfsCustomVar> customs;
            lock (_stateLock) { customs = _pendingCustoms; }

            var survivors = new List<MsfsCustomVar>();
            var failed = new List<MsfsFailedVar>();

            // Reset the streaming definition before rebuilding it. Skipped on a
            // fresh session where the define doesn't exist yet (avoids a stray
            // "unknown define" exception).
            if (clearFirst)
            {
                WriteAndFlush(pipe, SimConnectProtocol.WriteClearDataDefinition(
                    _txBuffer, Protocol, DefineId, NextSendId()));
            }

            // Defaults: datumId = index 0..35, in table order.
            for (int i = 0; i < MsfsSimVarTable.Entries.Length; i++)
            {
                MsfsSimVarTable.Entry e = MsfsSimVarTable.Entries[i];
                AddToDefinition(pipe, DefineId, e.Name, e.Units, (uint)i);
            }
            WriteFlush(pipe);

            // Customs: probe each in isolation, append survivors after the
            // defaults (datumId continues from DefaultCount).
            int added = 0;
            if (customs != null)
            {
                foreach (var c in customs)
                {
                    if (added >= MaxCustomVars)
                    {
                        _log($"[MsfsSimConnect] custom var cap ({MaxCustomVars}) reached; '{c.Name}' and any after it were skipped.");
                        break;
                    }

                    if (ProbeCustom(pipe, c, out uint code))
                    {
                        AddToDefinition(pipe, DefineId, c.Name, c.Units,
                            (uint)(DefaultCount + survivors.Count));
                        WriteFlush(pipe);
                        survivors.Add(c);
                        added++;
                    }
                    else
                    {
                        failed.Add(new MsfsFailedVar(c.Alias, c.Name, code));
                        _log($"[MsfsSimConnect] custom var rejected: '{c.Name}' (alias '{c.Alias}') exception={code}.");
                    }
                }
            }

            int total = DefaultCount + survivors.Count;
            if (_sampleBuffer.Length < total) _sampleBuffer = new double[total];

            lock (_stateLock)
            {
                _survivingCustoms = survivors;
                _failed = failed;
            }

            Subscribe(pipe);
            _log($"[MsfsSimConnect] connected, streaming SIM_FRAME: {DefaultCount} defaults + {survivors.Count} custom ({failed.Count} rejected).");
        }

        // Validate one custom var in a scratch definition. Returns true if the
        // var is usable. Drain rule: an exception matching the add or request
        // sendId => invalid; a PERIOD_ONCE data reply or timeout => valid.
        private bool ProbeCustom(NamedPipeClientStream pipe, MsfsCustomVar c, out uint exceptionCode)
        {
            exceptionCode = 0;

            WriteAndFlush(pipe, SimConnectProtocol.WriteClearDataDefinition(
                _txBuffer, Protocol, ProbeDefineId, NextSendId()));

            uint addSend = NextSendId();
            WriteAndFlush(pipe, SimConnectProtocol.WriteAddToDataDefinition(
                _txBuffer, Protocol, ProbeDefineId, c.Name, c.Units,
                SimConnectProtocol.DataTypeFloat64, epsilon: 0.0f,
                datumId: SimConnectProtocol.Unused, sendId: addSend));

            uint reqSend = NextSendId();
            WriteAndFlush(pipe, SimConnectProtocol.WriteRequestDataOnSimObject(
                _txBuffer, Protocol,
                requestId: ProbeRequestId, defineId: ProbeDefineId,
                objectId: SimConnectProtocol.ObjectIdUser,
                period: SimConnectProtocol.PeriodOnce,
                flags: 0, origin: 0, interval: 0, limit: 0, sendId: reqSend));

            bool valid = true;
            DateTime deadline = DateTime.UtcNow.AddMilliseconds(ProbeTimeoutMs);
            while (DateTime.UtcNow < deadline)
            {
                if (!TryReadPacketBeforeDeadline(pipe, deadline, out int totalSize))
                    break; // no more data before deadline -> no exception -> valid
                if (!SimConnectProtocol.TryReadHeader(_rxBuffer, totalSize,
                        out _, out _, out uint recvId))
                    continue;

                if (recvId == SimConnectProtocol.RecvIdException)
                {
                    if (SimConnectProtocol.TryReadException(_rxBuffer, totalSize,
                            out uint code, out uint sendId, out _)
                        && (sendId == addSend || sendId == reqSend))
                    {
                        exceptionCode = code;
                        valid = false;
                        break;
                    }
                    // Unrelated exception (e.g. the earlier clear on an unknown
                    // define) — ignore and keep draining.
                }
                else if (recvId == SimConnectProtocol.RecvIdSimObjectData)
                {
                    if (SimConnectProtocol.TryReadSimObjectData(_rxBuffer, totalSize,
                            out uint rid, out _, out _, out _, out _, out _)
                        && rid == ProbeRequestId)
                    {
                        valid = true;
                        break;
                    }
                }
                // Ignore other messages (QUIT is handled by the read failing).
            }

            // Recycle the scratch definition for the next probe.
            WriteAndFlush(pipe, SimConnectProtocol.WriteClearDataDefinition(
                _txBuffer, Protocol, ProbeDefineId, NextSendId()));
            return valid;
        }

        private void AddToDefinition(NamedPipeClientStream pipe, uint defineId,
            string name, string units, uint datumId)
        {
            int n = SimConnectProtocol.WriteAddToDataDefinition(
                _txBuffer, Protocol, defineId, name, units,
                SimConnectProtocol.DataTypeFloat64, epsilon: 0.0f,
                datumId: datumId, sendId: NextSendId());
            pipe.Write(_txBuffer, 0, n);
        }

        private void Subscribe(NamedPipeClientStream pipe)
        {
            int n = SimConnectProtocol.WriteRequestDataOnSimObject(
                _txBuffer, Protocol,
                requestId: RequestId, defineId: DefineId,
                objectId: SimConnectProtocol.ObjectIdUser,
                period: SimConnectProtocol.PeriodSimFrame,
                flags: 0, origin: 0, interval: 0, limit: 0, sendId: NextSendId());
            pipe.Write(_txBuffer, 0, n);
            pipe.Flush();
        }

        // Stop the SIM_FRAME stream before clearing/rebuilding the definition.
        private void StopStream(NamedPipeClientStream pipe)
        {
            WriteAndFlush(pipe, SimConnectProtocol.WriteRequestDataOnSimObject(
                _txBuffer, Protocol,
                requestId: RequestId, defineId: DefineId,
                objectId: SimConnectProtocol.ObjectIdUser,
                period: SimConnectProtocol.PeriodNever,
                flags: 0, origin: 0, interval: 0, limit: 0, sendId: NextSendId()));
        }

        // -----------------------------------------------------------------
        //  Main read loop
        // -----------------------------------------------------------------

        private LoopResult ReadLoop(NamedPipeClientStream pipe, CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                // Custom vars changed — stop the stream and re-register.
                if (_customsDirty)
                {
                    StopStream(pipe);
                    return LoopResult.Reconfigure;
                }

                if (!ReadOnePacket(pipe, out int totalSize)) return LoopResult.Disconnected;
                if (!SimConnectProtocol.TryReadHeader(_rxBuffer, totalSize,
                        out _, out _, out uint recvId)) continue;

                if (recvId == SimConnectProtocol.RecvIdSimObjectData)
                {
                    HandleSimObjectData(totalSize);
                }
                else if (recvId == SimConnectProtocol.RecvIdQuit)
                {
                    _log("[MsfsSimConnect] MSFS quit, reconnecting.");
                    return LoopResult.Disconnected;
                }
                else if (recvId == SimConnectProtocol.RecvIdException)
                {
                    if (SimConnectProtocol.TryReadException(_rxBuffer, totalSize,
                            out uint code, out uint sendId, out _))
                    {
                        _log($"[MsfsSimConnect] exception={code} sendId={sendId}");
                    }
                }
            }
            return LoopResult.Disconnected;
        }

        private void HandleSimObjectData(int totalSize)
        {
            if (!SimConnectProtocol.TryReadSimObjectData(_rxBuffer, totalSize,
                    out uint requestId, out _, out _, out _, out uint defineCount,
                    out int payloadOffset))
                return;
            if (requestId != RequestId) return;

            List<MsfsCustomVar> customs;
            lock (_stateLock) { customs = _survivingCustoms; }
            int expected = DefaultCount + customs.Count;

            // A default going missing is the only fatal case (should not happen).
            // A short/long count vs the survivor set means the definition and our
            // view diverged — skip the frame; re-registration will resync.
            if (defineCount != (uint)expected) return;
            if ((int)defineCount < DefaultCount) return;

            int bytesNeeded = (int)defineCount * 8;
            if (payloadOffset + bytesNeeded > totalSize) return;

            for (int i = 0; i < (int)defineCount; i++)
            {
                _sampleBuffer[i] = SimConnectProtocol.ReadFloat64(_rxBuffer, payloadOffset + i * 8);
            }

            IReadOnlyDictionary<string, double> customValues;
            if (customs.Count == 0)
            {
                customValues = EmptyCustoms;
            }
            else
            {
                var map = new Dictionary<string, double>(customs.Count, StringComparer.Ordinal);
                for (int i = 0; i < customs.Count; i++)
                {
                    map[customs[i].Alias] = _sampleBuffer[DefaultCount + i];
                }
                customValues = map;
            }

            lock (_stateLock) { _lastSampleUtc = DateTime.UtcNow; }

            try { _onSample(_sampleBuffer, customValues); }
            catch (Exception ex) { _log("[MsfsSimConnect] callback threw: " + ex.Message); }
        }

        private static readonly IReadOnlyDictionary<string, double> EmptyCustoms =
            new Dictionary<string, double>(0);

        // -----------------------------------------------------------------
        //  Pipe I/O
        // -----------------------------------------------------------------

        private void WriteAndFlush(NamedPipeClientStream pipe, int n)
        {
            pipe.Write(_txBuffer, 0, n);
            pipe.Flush();
        }

        private void WriteFlush(NamedPipeClientStream pipe)
        {
            pipe.Flush();
        }

        private bool ReadOnePacket(NamedPipeClientStream pipe, out int totalSize)
        {
            totalSize = 0;
            if (!ReadFully(pipe, _rxBuffer, 0, 4)) return false;
            int size = (int)((uint)_rxBuffer[0]
                | ((uint)_rxBuffer[1] <<  8)
                | ((uint)_rxBuffer[2] << 16)
                | ((uint)_rxBuffer[3] << 24));
            if (size < SimConnectProtocol.InboundHeaderSize || size > _rxBuffer.Length)
            {
                return false;
            }
            if (!ReadFully(pipe, _rxBuffer, 4, size - 4)) return false;
            totalSize = size;
            return true;
        }

        // .NET pipe streams don't support read timeouts, and a blocking read
        // would hang a good-var probe forever while the sim is paused (no
        // PERIOD_ONCE reply). So we poll with PeekNamedPipe and only read once a
        // whole packet is buffered; "nothing before the deadline" => silent =>
        // valid (per the drain rule). Returns false on deadline or pipe error.
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool PeekNamedPipe(
            SafePipeHandle hNamedPipe, byte[] lpBuffer, uint nBufferSize,
            IntPtr lpBytesRead, out uint lpTotalBytesAvail, IntPtr lpBytesLeftThisMessage);

        private readonly byte[] _peek4 = new byte[4];

        private bool TryReadPacketBeforeDeadline(NamedPipeClientStream pipe, DateTime deadline, out int totalSize)
        {
            totalSize = 0;
            SafePipeHandle handle = pipe.SafePipeHandle;
            while (DateTime.UtcNow < deadline)
            {
                if (!PeekNamedPipe(handle, _peek4, 4, IntPtr.Zero, out uint avail, IntPtr.Zero))
                    return false; // pipe broken/closed
                if (avail >= 4)
                {
                    int size = (int)((uint)_peek4[0]
                        | ((uint)_peek4[1] <<  8)
                        | ((uint)_peek4[2] << 16)
                        | ((uint)_peek4[3] << 24));
                    if (size < SimConnectProtocol.InboundHeaderSize || size > _rxBuffer.Length)
                        return false; // garbage frame
                    if (avail >= (uint)size)
                        return ReadOnePacket(pipe, out totalSize); // whole packet buffered
                }
                Thread.Sleep(2);
            }
            return false;
        }

        private static bool ReadFully(Stream stream, byte[] buf, int offset, int count)
        {
            int got = 0;
            while (got < count)
            {
                int n = stream.Read(buf, offset + got, count - got);
                if (n <= 0) return false;
                got += n;
            }
            return true;
        }

        private uint NextSendId()
        {
            unchecked { return ++_sendIdSeq; }
        }

        private static void DelayOrCancel(int ms, CancellationToken ct)
        {
            try { Thread.Sleep(ms); } catch { }
            if (ct.IsCancellationRequested) return;
        }
    }
}
