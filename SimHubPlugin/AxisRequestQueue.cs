using System;
using System.Collections.Generic;
using System.Windows.Threading;
using ProtbufTest;

namespace DiyFfb
{
    public enum AxisRequestType
    {
        AxisConfig,
        FunctionConfig,
        ActiveFunction,
        DeviceInfo,
        Restart,
        Homing,
        StaticBalanceCalibration,
        AxisConfigUpload,
        FunctionConfigUpload
    }

    /// <summary>
    /// Interface for sending axis requests, extracted for testability.
    /// </summary>
    public interface IAxisRequestSender
    {
        bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload);
    }

    public class AxisRequestQueue
    {
        private class RequestItem
        {
            public AxisID AxisId;
            public AxisRequestType Type;
            public int RemainingRetries;
            public bool AwaitResponse;
            public DateTime NextSendUtc;
            public Message Payload;
        }

        private readonly IAxisRequestSender sender;
        private readonly DispatcherTimer timer;
        private readonly Queue<RequestItem> queue = new Queue<RequestItem>();
        private readonly object sync = new object();
        private RequestItem current;
        private bool hasCurrent;
        private readonly bool manualTick;
        private Func<DateTime> nowProvider = () => DateTime.UtcNow;

        private const int RetryDelayMs = 250;
        private const int MaxRetries = 3;

        // Minimum delay after sending an upload before processing the next queue item.
        // Gives ESP32 time to deserialize and apply configs (especially force curve splines).
        private const int PostUploadCooldownMs = 200;
        private DateTime _cooldownUntilUtc = DateTime.MinValue;

        public AxisRequestQueue(DiyFfbPluginUI ui)
            : this(new PluginUISender(ui), manualTick: false)
        {
        }

        /// <summary>
        /// Constructor for testing: accepts an IAxisRequestSender and allows manual tick control.
        /// </summary>
        internal AxisRequestQueue(IAxisRequestSender sender, bool manualTick)
        {
            this.sender = sender;
            this.manualTick = manualTick;
            if (!manualTick)
            {
                timer = new DispatcherTimer
                {
                    Interval = TimeSpan.FromMilliseconds(50)
                };
                timer.Tick += OnTick;
                timer.Start();
            }
        }

        /// <summary>
        /// For testing: override the time provider.
        /// </summary>
        internal void SetNowProvider(Func<DateTime> provider)
        {
            nowProvider = provider ?? (() => DateTime.UtcNow);
        }

        /// <summary>
        /// For testing: manually trigger the tick logic.
        /// </summary>
        internal void Tick()
        {
            OnTick(null, EventArgs.Empty);
        }

        /// <summary>
        /// For testing: get the current queue count.
        /// </summary>
        internal int QueueCount
        {
            get { lock (sync) return queue.Count; }
        }

        /// <summary>
        /// For testing: check if there's a current request being processed.
        /// </summary>
        internal bool HasCurrentRequest
        {
            get { lock (sync) return hasCurrent; }
        }

        private class PluginUISender : IAxisRequestSender
        {
            private readonly DiyFfbPluginUI ui;
            public PluginUISender(DiyFfbPluginUI ui) => this.ui = ui;
            public bool SendAxisRequest(AxisID axisId, AxisRequestType type, Message payload)
                => ui.SendAxisRequest(axisId, type, payload);
        }

        public void Enqueue(AxisID axisId, AxisRequestType type, Message payload = null)
        {
            lock (sync)
            {
                if (IsDuplicate(axisId, type))
                {
                    return;
                }
                queue.Enqueue(new RequestItem
                {
                    AxisId = axisId,
                    Type = type,
                    RemainingRetries = MaxRetries,
                    AwaitResponse = RequiresResponse(type),
                    NextSendUtc = nowProvider(),
                    Payload = payload
                });
            }
        }

        public void HandleResponse(Message msg)
        {
            AxisRequestType? responseType = null;
            AxisID responseAxis = AxisID.AxisUndefined;

            switch (msg.PayloadCase)
            {
                case Message.PayloadOneofCase.AxisConfig:
                    responseType = AxisRequestType.AxisConfig;
                    responseAxis = msg.AxisConfig.AxisId;
                    break;
                case Message.PayloadOneofCase.FunctionConfig:
                    responseType = AxisRequestType.FunctionConfig;
                    break;
                case Message.PayloadOneofCase.ActiveFunction:
                    responseType = AxisRequestType.ActiveFunction;
                    responseAxis = msg.ActiveFunction.AxisId;
                    break;
                case Message.PayloadOneofCase.DeviceInfo:
                    responseType = AxisRequestType.DeviceInfo;
                    if (msg.DeviceInfo.SourceCase == DeviceInfo.SourceOneofCase.AxisId)
                    {
                        responseAxis = msg.DeviceInfo.AxisId;
                    }
                    break;
                default:
                    break;
            }

            if (responseType == null)
            {
                return;
            }

            lock (sync)
            {
                if (!hasCurrent)
                {
                    return;
                }
                if (current.Type != responseType.Value)
                {
                    return;
                }
                if (current.Type == AxisRequestType.FunctionConfig)
                {
                    hasCurrent = false;
                    return;
                }
                if (responseAxis != AxisID.AxisUndefined && current.AxisId != responseAxis)
                {
                    return;
                }
                hasCurrent = false;
            }
        }

        private void OnTick(object timerSender, EventArgs e)
        {
            RequestItem item = null;
            lock (sync)
            {
                var now = nowProvider();
                if (!hasCurrent && now < _cooldownUntilUtc)
                {
                    return;
                }
                if (!hasCurrent && queue.Count > 0)
                {
                    current = queue.Dequeue();
                    hasCurrent = true;
                }
                if (!hasCurrent)
                {
                    return;
                }
                if (now < current.NextSendUtc)
                {
                    return;
                }
                item = current;
            }

            bool sent = sender.SendAxisRequest(item.AxisId, item.Type, item.Payload);
            lock (sync)
            {
                if (!hasCurrent)
                {
                    return;
                }
                if (!sent)
                {
                    current.RemainingRetries--;
                    if (current.RemainingRetries <= 0)
                    {
                        hasCurrent = false;
                        return;
                    }
                    current.NextSendUtc = nowProvider().AddMilliseconds(RetryDelayMs);
                    return;
                }

                if (!current.AwaitResponse)
                {
                    if (IsUploadType(current.Type))
                    {
                        _cooldownUntilUtc = nowProvider().AddMilliseconds(PostUploadCooldownMs);
                    }
                    hasCurrent = false;
                    return;
                }

                current.RemainingRetries--;
                if (current.RemainingRetries <= 0)
                {
                    hasCurrent = false;
                    return;
                }
                current.NextSendUtc = nowProvider().AddMilliseconds(RetryDelayMs);
            }
        }

        private bool RequiresResponse(AxisRequestType type)
        {
            switch (type)
            {
                case AxisRequestType.AxisConfig:
                case AxisRequestType.FunctionConfig:
                case AxisRequestType.ActiveFunction:
                case AxisRequestType.DeviceInfo:
                    return true;
                case AxisRequestType.Restart:
                case AxisRequestType.Homing:
                case AxisRequestType.StaticBalanceCalibration:
                case AxisRequestType.AxisConfigUpload:
                case AxisRequestType.FunctionConfigUpload:
                default:
                    return false;
            }
        }

        private static bool IsUploadType(AxisRequestType type)
        {
            return type == AxisRequestType.AxisConfigUpload ||
                   type == AxisRequestType.FunctionConfigUpload;
        }

        private bool IsDuplicate(AxisID axisId, AxisRequestType type)
        {
            if (type == AxisRequestType.AxisConfigUpload || type == AxisRequestType.FunctionConfigUpload)
            {
                return false;
            }
            if (hasCurrent && current != null && current.AxisId == axisId && current.Type == type)
            {
                return true;
            }
            foreach (var item in queue)
            {
                if (item.AxisId == axisId && item.Type == type)
                {
                    return true;
                }
            }
            return false;
        }
    }
}
