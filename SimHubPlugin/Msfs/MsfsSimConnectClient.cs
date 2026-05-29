// Plan 19 - In-process SimConnect client. Replaces MsfsFfbDataProvider.exe
// + UDP loopback. Talks to MSFS 2024 directly over the named pipe
// \\.\pipe\Microsoft Flight Simulator\SimConnect, registers the 35-entry
// SimVar definition from MsfsSimVarTable, subscribes at SIM_FRAME, and
// hands each sample (35 doubles) to a callback for the plugin to publish
// into latestMsfsPacket.
//
// Lifecycle mirrors the EXE bridge's outer loop: connect → handshake →
// register → subscribe → read; on QUIT or pipe break, tear down and
// retry every 2 seconds. Safe to Start() before MSFS is running.

using System;
using System.IO;
using System.IO.Pipes;
using System.Threading;

namespace DiyFfb.Msfs
{
    internal sealed class MsfsSimConnectClient : IDisposable
    {
        private const string PipeName = "Microsoft Flight Simulator\\SimConnect";
        private const uint   Protocol = SimConnectProtocol.ProtocolSunRise;
        private const uint   DefineId  = 1;
        private const uint   RequestId = 1;
        private const int    ConnectTimeoutMs = 1000;
        private const int    HandshakeTimeoutMs = 5000;
        private const int    ReconnectDelayMs = 2000;

        private readonly Action<double[]> _onSample;
        private readonly Action<string>   _log;
        private readonly object           _stateLock = new object();

        private Thread                  _worker;
        private CancellationTokenSource _cts;
        private NamedPipeClientStream   _pipe;
        private uint                    _sendIdSeq;
        private volatile bool           _connected;
        private DateTime                _lastSampleUtc = DateTime.MinValue;

        // Reused per-sample buffer to avoid GC churn. Sized for an
        // all-FLOAT64 35-entry data definition (~320 bytes) plus headroom.
        private readonly byte[]   _rxBuffer    = new byte[4096];
        private readonly byte[]   _txBuffer    = new byte[SimConnectProtocol.MaxOutboundPacketBytes];
        private readonly double[] _sampleBuffer = new double[MsfsSimVarTable.SampleCount];

        public MsfsSimConnectClient(Action<double[]> onSample, Action<string> log)
        {
            _onSample = onSample ?? throw new ArgumentNullException(nameof(onSample));
            _log      = log ?? (_ => { });
        }

        public bool IsConnected => _connected;
        public DateTime LastSampleUtc { get { lock (_stateLock) return _lastSampleUtc; } }

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
            // Dispose the pipe so any in-flight Read/Connect unblocks.
            try { pipe?.Dispose(); } catch { }
            try { worker?.Join(2000); } catch { }
            try { cts.Dispose(); } catch { }
        }

        public void Dispose() => Stop();

        // -----------------------------------------------------------------
        //  Worker thread
        // -----------------------------------------------------------------

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
                            // MSFS not running yet — quiet retry.
                            DelayOrCancel(ReconnectDelayMs, ct);
                            continue;
                        }

                        if (ct.IsCancellationRequested) break;

                        if (!Handshake(pipe, ct))
                        {
                            DelayOrCancel(ReconnectDelayMs, ct);
                            continue;
                        }

                        RegisterSimVars(pipe);
                        SubscribeSimObjectData(pipe);

                        _log("[MsfsSimConnect] connected, streaming SIM_FRAME at 35 doubles/sample.");
                        _connected = true;
                        ReadLoop(pipe, ct);
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
        //  Handshake / registration / subscription
        // -----------------------------------------------------------------

        private bool Handshake(NamedPipeClientStream pipe, CancellationToken ct)
        {
            uint sendId = NextSendId();
            int n = SimConnectProtocol.WriteOpen(_txBuffer, Protocol, "DiyFfbPlugin", sendId);
            pipe.Write(_txBuffer, 0, n);
            pipe.Flush();

            // Drain until we see RECV_OPEN or RECV_EXCEPTION, or time out.
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
                // Ignore any other message types during handshake.
            }
            _log("[MsfsSimConnect] handshake timeout.");
            return false;
        }

        private void RegisterSimVars(NamedPipeClientStream pipe)
        {
            for (int i = 0; i < MsfsSimVarTable.Entries.Length; i++)
            {
                MsfsSimVarTable.Entry e = MsfsSimVarTable.Entries[i];
                int n = SimConnectProtocol.WriteAddToDataDefinition(
                    _txBuffer, Protocol,
                    DefineId, e.Name, e.Units,
                    SimConnectProtocol.DataTypeFloat64,
                    epsilon: 0.0f,
                    datumId: SimConnectProtocol.Unused,
                    sendId: NextSendId());
                pipe.Write(_txBuffer, 0, n);
            }
            pipe.Flush();
        }

        private void SubscribeSimObjectData(NamedPipeClientStream pipe)
        {
            int n = SimConnectProtocol.WriteRequestDataOnSimObject(
                _txBuffer, Protocol,
                requestId: RequestId,
                defineId:  DefineId,
                objectId:  SimConnectProtocol.ObjectIdUser,
                period:    SimConnectProtocol.PeriodSimFrame,
                flags:     0,
                origin:    0,
                interval:  0,
                limit:     0,
                sendId:    NextSendId());
            pipe.Write(_txBuffer, 0, n);
            pipe.Flush();
        }

        // -----------------------------------------------------------------
        //  Main read loop
        // -----------------------------------------------------------------

        private void ReadLoop(NamedPipeClientStream pipe, CancellationToken ct)
        {
            while (!ct.IsCancellationRequested)
            {
                if (!ReadOnePacket(pipe, out int totalSize)) return;
                if (!SimConnectProtocol.TryReadHeader(_rxBuffer, totalSize,
                        out _, out _, out uint recvId)) continue;

                if (recvId == SimConnectProtocol.RecvIdSimObjectData)
                {
                    HandleSimObjectData(totalSize);
                }
                else if (recvId == SimConnectProtocol.RecvIdQuit)
                {
                    _log("[MsfsSimConnect] MSFS quit, reconnecting.");
                    return;
                }
                else if (recvId == SimConnectProtocol.RecvIdException)
                {
                    if (SimConnectProtocol.TryReadException(_rxBuffer, totalSize,
                            out uint code, out uint sendId, out _))
                    {
                        _log($"[MsfsSimConnect] exception={code} sendId={sendId}");
                    }
                }
                // Silently ignore unknown recv IDs (events, lists, etc.).
            }
        }

        private void HandleSimObjectData(int totalSize)
        {
            if (!SimConnectProtocol.TryReadSimObjectData(_rxBuffer, totalSize,
                    out uint requestId, out _, out _, out _, out uint defineCount,
                    out int payloadOffset))
                return;
            if (requestId != RequestId) return;
            if (defineCount != MsfsSimVarTable.SampleCount) return;

            int bytesNeeded = (int)defineCount * 8;
            if (payloadOffset + bytesNeeded > totalSize) return;

            for (int i = 0; i < MsfsSimVarTable.SampleCount; i++)
            {
                _sampleBuffer[i] = SimConnectProtocol.ReadFloat64(_rxBuffer, payloadOffset + i * 8);
            }

            lock (_stateLock) { _lastSampleUtc = DateTime.UtcNow; }

            try { _onSample(_sampleBuffer); }
            catch (Exception ex) { _log("[MsfsSimConnect] callback threw: " + ex.Message); }
        }

        // -----------------------------------------------------------------
        //  Pipe I/O
        // -----------------------------------------------------------------

        // Reads one packet into _rxBuffer starting at offset 0. Returns
        // false on EOF / partial-read failure.
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
                // Garbage frame — bail and reconnect upstream.
                return false;
            }
            if (!ReadFully(pipe, _rxBuffer, 4, size - 4)) return false;
            totalSize = size;
            return true;
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
