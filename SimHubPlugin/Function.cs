using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;

namespace DiyFfb
{
    public class Function : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;
        private FunctionConfig _Config;
        public FunctionConfig Config
        {
            get
            {
                return _Config;
            }
            set
            {
                _Config = value;
                OnAxisUpdate();
            }
        }
        private FunctionID _functionID;
        private string _functionName;
        private HashSet<AxisID> _axisIDs = new HashSet<AxisID>();
        private bool isOnline;
        public bool IsOnline
        {
            get { return isOnline; }
        }
        private bool isDirty;
        public bool IsDirty
        {
            get { return isDirty; }
        }

        private string _statusMessage = "No associated axis configured";
        public string StatusMessage
        {
            get { return _statusMessage; }
            private set
            {
                if (_statusMessage != value)
                {
                    _statusMessage = value;
                    PropertyChanged?.Invoke(this,
                        new PropertyChangedEventArgs(nameof(StatusMessage)));
                }
            }
        }
        public bool SelectedToStore { get; set; }
        public bool SelectedToLoad { get; set; }
        public bool SelectableToLoad { get; set; }
        public FunctionID ID { get { return _functionID; } }
        public string Name { get { return _functionName; } }

        public void OnAxisAdded(AxisID axis_id)
        {
            _axisIDs.Add(axis_id);
            OnAxisUpdate();
        }

        public void OnAxisRemoved(AxisID axis_id)
        {
            _axisIDs.Remove(axis_id);
            OnAxisUpdate();
        }

        public void OnAxisUpdate()
        {
            bool online = false;
            bool dirty = false;
            HashSet<AxisID> linked_axes = new HashSet<AxisID>();
            foreach (var axis in Config.Base.LinkedAxes)
            {
                if (axis == AxisID.AxisUndefined) break;
                linked_axes.Add(axis & AxisID.Mask);
            }
            if (linked_axes.Count == 0)
            {
                online = false;
                StatusMessage = "No associated axis configured";
            }
            else if (linked_axes.SetEquals(_axisIDs))
            {
                online = true;
                StatusMessage = "All associated axes online and reporting the correct function";
            }
            else
            {
                online = false;
                dirty = true;
                StatusMessage = String.Format("Associated axes reporting this function: {0}\nAssociated axes reporting other function or offline: {1}\nUnassociated axes reporting this function: {2}", linked_axes.Intersect(_axisIDs).Count(), linked_axes.Except(_axisIDs).Count(), _axisIDs.Except(linked_axes).Count());
            }
            if (online != isOnline)
            {
                isOnline = online;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsOnline)));
            }
            if (dirty != isDirty)
            {
                isDirty = dirty;
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(IsDirty)));
            }
        }

        public Function(FunctionID function_id)
        {
            _functionID = function_id;
            _functionName = function_id.ToString().CamelCaseToTitleCase();
        }
    }
}
