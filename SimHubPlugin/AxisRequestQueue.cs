using System;
using System.Collections.Generic;
using System.Windows.Threading;
using ProtbufTest;

namespace User.PluginSdkDemo
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

        private readonly DiyFfbPluginUI ui;
        private readonly DispatcherTimer timer;
        private readonly Queue<RequestItem> queue = new Queue<RequestItem>();
        private readonly object sync = new object();
        private RequestItem current;
        private bool hasCurrent;

        private const int RetryDelayMs = 250;
        private const int MaxRetries = 3;

        public AxisRequestQueue(DiyFfbPluginUI ui)
        {
            this.ui = ui;
            timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            timer.Tick += OnTick;
            timer.Start();
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
                    NextSendUtc = DateTime.UtcNow,
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

        private void OnTick(object sender, EventArgs e)
        {
            RequestItem item = null;
            lock (sync)
            {
                if (!hasCurrent && queue.Count > 0)
                {
                    current = queue.Dequeue();
                    hasCurrent = true;
                }
                if (!hasCurrent)
                {
                    return;
                }
                if (DateTime.UtcNow < current.NextSendUtc)
                {
                    return;
                }
                item = current;
            }

            bool sent = ui.SendAxisRequest(item.AxisId, item.Type, item.Payload);
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
                    current.NextSendUtc = DateTime.UtcNow.AddMilliseconds(RetryDelayMs);
                    return;
                }

                if (!current.AwaitResponse)
                {
                    hasCurrent = false;
                    return;
                }

                current.RemainingRetries--;
                if (current.RemainingRetries <= 0)
                {
                    hasCurrent = false;
                    return;
                }
                current.NextSendUtc = DateTime.UtcNow.AddMilliseconds(RetryDelayMs);
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
