using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using ProtbufTest;

namespace DiyFfb
{
    /// <summary>
    /// Represents a discovered gateway device.
    /// </summary>
    public class DiscoveredGateway
    {
        public string PortName { get; set; }
        public GatewayID GatewayId { get; set; }
        public string FirmwareVersion { get; set; }
        public string Board { get; set; }
        public string DeviceUid { get; set; }

        public override string ToString()
        {
            return $"Gateway {GatewayId} on {PortName} (FW: {FirmwareVersion}, Board: {Board})";
        }
    }

    /// <summary>
    /// Scans all available COM ports to auto-detect connected gateways.
    /// </summary>
    public class GatewayScanner
    {
        private const int ProbeTimeoutMs = 1000;
        private const int BaudRate = 3000000;

        public delegate void ScanProgressHandler(string portName, int current, int total);
        public event ScanProgressHandler OnScanProgress;

        /// <summary>
        /// Scans all available COM ports and returns a list of discovered gateways.
        /// </summary>
        /// <param name="cancellationToken">Token to cancel the scan operation</param>
        /// <returns>List of discovered gateways</returns>
        public async Task<List<DiscoveredGateway>> ScanForGateways(CancellationToken cancellationToken = default)
        {
            var gateways = new List<DiscoveredGateway>();
            string[] portNames = SerialPort.GetPortNames();

            for (int i = 0; i < portNames.Length; i++)
            {
                if (cancellationToken.IsCancellationRequested)
                {
                    break;
                }

                string portName = portNames[i];
                OnScanProgress?.Invoke(portName, i + 1, portNames.Length);

                try
                {
                    var gateway = await ProbePortForGateway(portName, cancellationToken);
                    if (gateway != null)
                    {
                        gateways.Add(gateway);
                    }
                }
                catch (Exception ex)
                {
                    // Port probe failed - not a gateway or port in use
                    SimHub.Logging.Current.Debug($"[GatewayScanner] Failed to probe {portName}: {ex.Message}");
                }
            }

            return gateways;
        }

        /// <summary>
        /// Probes a single port to determine if it has a gateway connected.
        /// </summary>
        private async Task<DiscoveredGateway> ProbePortForGateway(string portName, CancellationToken cancellationToken)
        {
            ProtobufSerial<Message> serial = null;
            var responseReceived = new TaskCompletionSource<DeviceInfo>();

            try
            {
                serial = new ProtobufSerial<Message>(portName, BaudRate);

                // Set up message handler to catch DeviceInfo response
                void MessageHandler(object sender, object message)
                {
                    if (message is Message msg && msg.PayloadCase == Message.PayloadOneofCase.DeviceInfo)
                    {
                        if (msg.DeviceInfo.SourceCase == DeviceInfo.SourceOneofCase.GatewayId)
                        {
                            responseReceived.TrySetResult(msg.DeviceInfo);
                        }
                    }
                }

                serial.OnMessage += MessageHandler;

                // Try to open the port
                if (!serial.Open())
                {
                    return null;
                }

                // Give the ESP32 time to boot if it just powered up
                await Task.Delay(100, cancellationToken);

                // Request device info - try gateway IDs 1-4
                for (int gatewayId = 1; gatewayId <= 4; gatewayId++)
                {
                    var request = new Message
                    {
                        DeviceInfoRequest = new DeviceInfoRequest
                        {
                            GatewayId = (GatewayID)gatewayId
                        }
                    };

                    serial.WriteMessage(request);

                    // Wait for response with timeout
                    using (var timeoutCts = new CancellationTokenSource(ProbeTimeoutMs / 4))
                    using (var linkedCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeoutCts.Token))
                    {
                        try
                        {
                            linkedCts.Token.Register(() => responseReceived.TrySetCanceled());
                            var deviceInfo = await responseReceived.Task;

                            // Found a gateway!
                            return new DiscoveredGateway
                            {
                                PortName = portName,
                                GatewayId = deviceInfo.GatewayId,
                                FirmwareVersion = deviceInfo.FwVersion ?? "Unknown",
                                Board = deviceInfo.Board ?? "Unknown",
                                DeviceUid = deviceInfo.DeviceUid ?? ""
                            };
                        }
                        catch (TaskCanceledException)
                        {
                            // Timeout or cancellation - try next gateway ID
                            responseReceived = new TaskCompletionSource<DeviceInfo>();
                        }
                    }
                }

                // No gateway responded
                return null;
            }
            catch (Exception ex)
            {
                SimHub.Logging.Current.Debug($"[GatewayScanner] Exception probing {portName}: {ex.Message}");
                return null;
            }
            finally
            {
                serial?.Close();
            }
        }

        /// <summary>
        /// Quick check if a specific port has a gateway (non-async version for backwards compatibility).
        /// </summary>
        public bool IsPortGateway(string portName, int timeoutMs = 500)
        {
            try
            {
                var task = ProbePortForGateway(portName, CancellationToken.None);
                task.Wait(timeoutMs);
                return task.IsCompleted && task.Result != null;
            }
            catch
            {
                return false;
            }
        }
    }
}
