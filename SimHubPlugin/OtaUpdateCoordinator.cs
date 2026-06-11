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
        public TimeSpan RetryTimeout { get; set; } = TimeSpan.FromSeconds(45);
        public TimeSpan PollInterval { get; set; } = TimeSpan.FromSeconds(1);

        // Delay between sends when no expected version is known (no completion signal
        // to wait on). Best-effort spacing so targets don't all start downloading at once.
        public TimeSpan StaggerDelay { get; set; } = TimeSpan.FromSeconds(2);

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
                    log?.Invoke("OTA: one or more axes did not reach expected version.");
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

        // Updates targets one at a time: send to a single target, wait for it to report the
        // expected version, then move on. This avoids the WiFi/HTTP congestion that caused
        // some targets to fail when all of them downloaded firmware simultaneously. Retries
        // are per-target, so a single stuck unit is re-flashed in place rather than restarting
        // the whole batch.
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
                for (int i = 0; i < targets.Count; i++)
                {
                    sender(targets[i]);
                    if (i < targets.Count - 1)
                    {
                        await Task.Delay(StaggerDelay, token).ConfigureAwait(true);
                    }
                }
                return true;
            }

            bool allOk = true;
            for (int i = 0; i < targets.Count; i++)
            {
                TId target = targets[i];

                // Already on the expected version (e.g. a retry of a partially-completed run).
                if (TargetMatches(target, expectedVersion, versionProvider))
                {
                    log?.Invoke($"OTA: {label} {i + 1}/{targets.Count} already on {expectedVersion}, skipping.");
                    continue;
                }

                bool ok = false;
                for (int attempt = 0; attempt <= RetryCount; attempt++)
                {
                    log?.Invoke($"OTA: updating {label} {i + 1}/{targets.Count} (attempt {attempt + 1}/{RetryCount + 1}).");
                    sender(target);

                    ok = await WaitForTargetAsync(target, expectedVersion, versionProvider, token).ConfigureAwait(true);
                    if (ok)
                    {
                        log?.Invoke($"OTA: {label} {i + 1}/{targets.Count} reached {expectedVersion}.");
                        break;
                    }

                    log?.Invoke($"OTA: {label} {i + 1}/{targets.Count} did not reach {expectedVersion} within {RetryTimeout.TotalSeconds:0}s.");
                }

                if (!ok)
                {
                    allOk = false;
                }
            }

            return allOk;
        }

        private async Task<bool> WaitForTargetAsync<TId>(
            TId target,
            string expectedVersion,
            Func<TId, string> versionProvider,
            CancellationToken token)
        {
            DateTime deadline = DateTime.UtcNow.Add(RetryTimeout);
            while (DateTime.UtcNow < deadline)
            {
                token.ThrowIfCancellationRequested();
                if (TargetMatches(target, expectedVersion, versionProvider))
                {
                    return true;
                }
                await Task.Delay(PollInterval, token).ConfigureAwait(true);
            }
            return false;
        }

        private static bool TargetMatches<TId>(TId target, string expectedVersion, Func<TId, string> versionProvider)
        {
            string version = versionProvider(target);
            return string.Equals(version, expectedVersion, StringComparison.OrdinalIgnoreCase);
        }
    }
}
