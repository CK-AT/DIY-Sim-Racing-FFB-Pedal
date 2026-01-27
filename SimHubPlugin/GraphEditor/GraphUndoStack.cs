using System;
using System.Collections.Generic;

namespace User.PluginSdkDemo.GraphEditor
{
    public sealed class GraphEditorViewState
    {
        public double ScaleX { get; set; }
        public double ScaleY { get; set; }
        public double TranslateX { get; set; }
        public double TranslateY { get; set; }
    }

    public sealed class GraphUndoSnapshot
    {
        public string GraphJson { get; set; }
        public GraphEditorViewState View { get; set; }
        public string SelectedNodeId { get; set; }
    }

    public sealed class GraphUndoStack
    {
        private readonly List<GraphUndoSnapshot> _snapshots = new List<GraphUndoSnapshot>();
        private int _index = -1;
        private int _baselineIndex = -1;

        public event EventHandler Changed;

        public bool CanUndo => _index > 0;
        public bool CanRedo => _index >= 0 && _index < _snapshots.Count - 1;
        public bool IsDirty => _index != _baselineIndex;
        public int Count => _snapshots.Count;
        public int CurrentIndex => _index;

        public GraphUndoSnapshot Current => (_index >= 0 && _index < _snapshots.Count) ? _snapshots[_index] : null;

        public void Reset(GraphUndoSnapshot snapshot)
        {
            _snapshots.Clear();
            if (snapshot != null)
            {
                _snapshots.Add(snapshot);
                _index = 0;
                _baselineIndex = 0;
            }
            else
            {
                _index = -1;
                _baselineIndex = -1;
            }
            OnChanged();
        }

        public void Push(GraphUndoSnapshot snapshot)
        {
            if (snapshot == null)
            {
                return;
            }

            if (_index < _snapshots.Count - 1 && _index >= 0)
            {
                _snapshots.RemoveRange(_index + 1, _snapshots.Count - _index - 1);
            }

            _snapshots.Add(snapshot);
            _index = _snapshots.Count - 1;
            OnChanged();
        }

        public GraphUndoSnapshot Undo()
        {
            if (!CanUndo)
            {
                return null;
            }

            _index--;
            OnChanged();
            return Current;
        }

        public GraphUndoSnapshot Redo()
        {
            if (!CanRedo)
            {
                return null;
            }

            _index++;
            OnChanged();
            return Current;
        }

        public void MarkClean()
        {
            _baselineIndex = _index;
            OnChanged();
        }

        private void OnChanged()
        {
            Changed?.Invoke(this, EventArgs.Empty);
        }
    }
}
