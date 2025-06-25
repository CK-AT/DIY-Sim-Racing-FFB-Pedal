using ProtbufTest;
using System;
using System.ComponentModel;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace User.PluginSdkDemo
{
    public class Axis : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        public delegate void OnlineStateChangedEventHandler(AxisID axis_id, bool new_online_state);
        public event OnlineStateChangedEventHandler OnlineStateChanged;
        private CancellationTokenSource _cancellationTokenSource = new CancellationTokenSource();
        public AxisConfig Config { get; set; }
        private AxisID _axisID;
        private string _axisName;
        private ProtobufSerial<Message> _serial_channel;
        public ProtobufSerial<Message> SerialChannel
        {
            get
            {
                return _serial_channel;
            }
            set
            {
                _serial_channel = value;
                if (value != null)
                {
                    IsOnline = true;
                }
            }
        }
        private bool isOnline;
        public bool IsOnline
        {
            get { return isOnline; }
            set
            {
                if (isOnline != value)
                {
                    isOnline = value;
                    PropertyChanged?.Invoke(this,
                        new PropertyChangedEventArgs(nameof(IsOnline)));
                    OnlineStateChanged?.Invoke(_axisID, isOnline);
                    Task.Delay(400).ContinueWith(t => RequestAxisConfig());
                    Task.Delay(500).ContinueWith(t => RequestFunctionConfig());
                    Task.Delay(600).ContinueWith(t => RequestActiveFunction());
                }
                if (value)
                {
                    _cancellationTokenSource.Cancel();
                    _cancellationTokenSource = new CancellationTokenSource();
                    Task.Delay(1000, _cancellationTokenSource.Token).ContinueWith(t => IsOnline = false, TaskContinuationOptions.NotOnCanceled);
                }
                else
                {
                    _serial_channel = null;
                }
            }
        }
        public bool SelectedToStore { get; set; }
        public bool SelectedToLoad { get; set; }
        public bool SelectableToLoad { get; set; }
        public AxisID ID { get { return _axisID; } }
        public string Name { get { return _axisName; } }

        public KinematicParameters KinematicParameters
        {
            get
            {
                return Config.KinematicParameters;
            }
        }

        public Axis(AxisID axis_id)
        {
            _axisID = axis_id;
            _axisName = String.Format("Axis {0}", (int)ID);
        }

        public void RequestAxisConfig()
        {
            if (IsOnline)
            {
                Message msg = new Message();
                msg.AxisAction = new AxisAction();
                msg.AxisAction.AxisId = ID;
                msg.AxisAction.ReturnAxisConfig = true;
                _serial_channel.WriteMessage(msg);
            }
        }

        public void RequestFunctionConfig()
        {
            if (IsOnline)
            {
                Message msg = new Message();
                msg.AxisAction = new AxisAction();
                msg.AxisAction.AxisId = ID;
                msg.AxisAction.ReturnFunctionConfig = true;
                _serial_channel.WriteMessage(msg);
            }
        }

        public void RequestActiveFunction()
        {
            if (IsOnline)
            {
                Message msg = new Message();
                msg.AxisAction = new AxisAction();
                msg.AxisAction.AxisId = ID;
                msg.AxisAction.ReturnActiveFunction = true;
                _serial_channel.WriteMessage(msg);
            }
        }

        public void UploadConfig(bool store)
        {
            if (IsOnline)
            {
                Message msg = new Message();
                Config.Store = store;
                msg.AxisConfig = Config;
                _serial_channel.WriteMessage(msg);
            }
        }

    }
}
