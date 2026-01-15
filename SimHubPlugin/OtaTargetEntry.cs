using System.ComponentModel;

namespace User.PluginSdkDemo
{
    public class OtaTargetEntry : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        public AxisID AxisId { get; }
        public GatewayID GatewayId { get; }

        private bool selected;
        private bool isOnline;
        private string latestVersion = "-";
        private string latestLog = "-";

        public string Name { get; }

        public bool Selected
        {
            get => selected;
            set
            {
                if (selected != value)
                {
                    selected = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Selected)));
                }
            }
        }

        public bool IsOnline
        {
            get => isOnline;
            set
            {
                if (isOnline != value)
                {
                    isOnline = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsOnline)));
                }
            }
        }

        public string LatestVersion
        {
            get => latestVersion;
            set
            {
                if (latestVersion != value)
                {
                    latestVersion = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(LatestVersion)));
                }
            }
        }

        public string LatestLog
        {
            get => latestLog;
            set
            {
                if (latestLog != value)
                {
                    latestLog = value;
                    PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(LatestLog)));
                }
            }
        }

        public OtaTargetEntry(AxisID axisId)
        {
            AxisId = axisId;
            GatewayId = GatewayID.GatewayUndefined;
            Name = $"Axis {(int)axisId}";
        }

        public OtaTargetEntry(GatewayID gatewayId)
        {
            AxisId = AxisID.AxisUndefined;
            GatewayId = gatewayId;
            string label = gatewayId == GatewayID.GatewayUndefined ? "Gateway" : $"Gateway {(int)gatewayId}";
            Name = label;
        }
    }
}
