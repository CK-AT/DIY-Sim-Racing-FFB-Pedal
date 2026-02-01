using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace DiyFfb
{
    public class OtaUpdateCoordinator
    {
        private readonly Func<AxisID, string> axisVersionProvider;
        private readonly Func<GatewayID, string> gatewayVersionProvider;
        private readonly Action<AxisID> axisUpdateSender;
        private readonly Action<GatewayID> gatewayUpdateSender;
        private readonly Action<string> log;

        public int RetryCount { get; set; } = 1;
        public TimeSpan RetryTimeout { get; set; } = TimeSpan.FromSeconds(30);
        public TimeSpan PollInterval { get; set; } = TimeSpan.FromSeconds(1);

        public OtaUpdateCoordinator(
            Func<AxisID, string> axisVersionProvider,
            Func<GatewayID, string> gatewayVersionProvider,
            Action<AxisID> axisUpdateSender,
            Action<GatewayID> gatewayUpdateSender,
            Action<string> log)
        {
            this.axisVersionProvider = axisVersionProvider;
            this.gatewayVersionProvider = gatewayVersionProvider;
            this.axisUpdateSender = axisUpdateSender;
            this.gatewayUpdateSender = gatewayUpdateSender;
            this.log = log;
        }

        public async Task RunAsync(
            IReadOnlyList<AxisID> axes,
            IReadOnlyList<GatewayID> gateways,
            string expectedVersion,
            CancellationToken token)
        {
            if (axes.Count > 0)
            {
                bool axesOk = await UpdateAxesAsync(axes, expectedVersion, token).ConfigureAwait(true);
                if (!axesOk)
                {
                    log?.Invoke("OTA: axis update did not reach expected version.");
                }
            }

            if (gateways.Count > 0)
            {
                bool gatewaysOk = await UpdateGatewaysAsync(gateways, expectedVersion, token).ConfigureAwait(true);
                if (!gatewaysOk)
                {
                    log?.Invoke("OTA: gateway update did not reach expected version.");
                }
            }
        }

        private async Task<bool> UpdateAxesAsync(IReadOnlyList<AxisID> axes, string expectedVersion, CancellationToken token)
        {
            return await UpdateTargetsAsync(
                axes,
                expectedVersion,
                axisUpdateSender,
                axisVersionProvider,
                "axis",
                token).ConfigureAwait(true);
        }

        private async Task<bool> UpdateGatewaysAsync(IReadOnlyList<GatewayID> gateways, string expectedVersion, CancellationToken token)
        {
            return await UpdateTargetsAsync(
                gateways,
                expectedVersion,
                gatewayUpdateSender,
                gatewayVersionProvider,
                "gateway",
                token).ConfigureAwait(true);
        }

        private async Task<bool> UpdateTargetsAsync<TId>(
            IReadOnlyList<TId> targets,
            string expectedVersion,
            Action<TId> sender,
            Func<TId, string> versionProvider,
            string label,
            CancellationToken token)
        {
            if (targets.Count == 0)
            {
                return true;
            }

            if (string.IsNullOrWhiteSpace(expectedVersion) || expectedVersion == "-")
            {
                log?.Invoke($"OTA: skipping {label} version checks (no expected version).");
                foreach (var target in targets)
                {
                    sender(target);
                }
                return true;
            }

            for (int attempt = 0; attempt <= RetryCount; attempt++)
            {
                log?.Invoke($"OTA: sending {label} update (attempt {attempt + 1}/{RetryCount + 1}).");
                foreach (var target in targets)
                {
                    sender(target);
                }

                bool ok = await WaitForTargetsAsync(targets, expectedVersion, versionProvider, token).ConfigureAwait(true);
                if (ok)
                {
                    return true;
                }
            }

            return false;
        }

        private async Task<bool> WaitForTargetsAsync<TId>(
            IReadOnlyList<TId> targets,
            string expectedVersion,
            Func<TId, string> versionProvider,
            CancellationToken token)
        {
            DateTime deadline = DateTime.UtcNow.Add(RetryTimeout);
            while (DateTime.UtcNow < deadline)
            {
                token.ThrowIfCancellationRequested();
                bool allMatch = targets.All(t =>
                {
                    string version = versionProvider(t);
                    return string.Equals(version, expectedVersion, StringComparison.OrdinalIgnoreCase);
                });
                if (allMatch)
                {
                    return true;
                }
                await Task.Delay(PollInterval, token).ConfigureAwait(true);
            }
            return false;
        }
    }
}
