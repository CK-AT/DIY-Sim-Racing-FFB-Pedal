using System;
using System.Collections.Generic;
using System.Linq;

namespace DiyFfb.TieredConfig
{
    /// <summary>
    /// Represents a pending change to a config field.
    /// </summary>
    public class PendingChange
    {
        public int FunctionId { get; set; }
        public string FieldPath { get; set; }
        public object Value { get; set; }
        public ConfigLayer TargetLayer { get; set; }
        public DateTime Timestamp { get; set; }
    }

    /// <summary>
    /// Tracks pending config changes per layer before they're committed.
    /// Pure state container with no side effects.
    /// </summary>
    public class ChangeTracker
    {
        // Pending changes by layer: layer → function_id → field_path → value
        private readonly Dictionary<ConfigLayer, Dictionary<int, Dictionary<string, object>>> _pending
            = new Dictionary<ConfigLayer, Dictionary<int, Dictionary<string, object>>>
            {
                { ConfigLayer.User, new Dictionary<int, Dictionary<string, object>>() },
                { ConfigLayer.Profile, new Dictionary<int, Dictionary<string, object>>() },
                { ConfigLayer.Hardware, new Dictionary<int, Dictionary<string, object>>() }
            };

        /// <summary>
        /// Track a field change targeting a specific layer.
        /// </summary>
        public void TrackChange(int functionId, string fieldPath, object value, ConfigLayer targetLayer)
        {
            var layerChanges = _pending[targetLayer];

            if (!layerChanges.ContainsKey(functionId))
                layerChanges[functionId] = new Dictionary<string, object>();

            layerChanges[functionId][fieldPath] = value;
        }

        /// <summary>
        /// Track a field change, auto-routing to default layer.
        /// </summary>
        public void TrackChange(int functionId, string fieldPath, object value)
        {
            var targetLayer = FieldRouter.GetTargetLayer(fieldPath);
            TrackChange(functionId, fieldPath, value, targetLayer);
        }

        /// <summary>
        /// Check if there are any unsaved changes across all layers.
        /// </summary>
        public bool HasUnsavedChanges =>
            _pending.Any(layer => layer.Value.Any(func => func.Value.Any()));

        /// <summary>
        /// Check if there are unsaved changes for a specific layer.
        /// </summary>
        public bool HasUnsavedChangesForLayer(ConfigLayer layer)
        {
            return _pending[layer].Any(func => func.Value.Any());
        }

        /// <summary>
        /// Check if there are unsaved changes for a specific function.
        /// </summary>
        public bool HasUnsavedChangesForFunction(int functionId)
        {
            return _pending.Any(layer =>
                layer.Value.ContainsKey(functionId) &&
                layer.Value[functionId].Any());
        }

        /// <summary>
        /// Get pending changes for a specific layer.
        /// Returns: function_id → field_path → value
        /// </summary>
        public Dictionary<int, Dictionary<string, object>> GetPendingChanges(ConfigLayer layer)
        {
            return new Dictionary<int, Dictionary<string, object>>(_pending[layer]);
        }

        /// <summary>
        /// Get all pending changes across all layers.
        /// </summary>
        public List<PendingChange> GetAllPendingChanges()
        {
            var result = new List<PendingChange>();

            foreach (var layerEntry in _pending)
            {
                var layer = layerEntry.Key;
                foreach (var funcEntry in layerEntry.Value)
                {
                    var functionId = funcEntry.Key;
                    foreach (var fieldEntry in funcEntry.Value)
                    {
                        result.Add(new PendingChange
                        {
                            FunctionId = functionId,
                            FieldPath = fieldEntry.Key,
                            Value = fieldEntry.Value,
                            TargetLayer = layer
                        });
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Get pending changes for a specific function across all layers.
        /// </summary>
        public Dictionary<ConfigLayer, Dictionary<string, object>> GetPendingChanges(int functionId)
        {
            var result = new Dictionary<ConfigLayer, Dictionary<string, object>>();

            foreach (var layerEntry in _pending)
            {
                if (layerEntry.Value.TryGetValue(functionId, out var changes) && changes.Any())
                {
                    result[layerEntry.Key] = new Dictionary<string, object>(changes);
                }
            }

            return result;
        }

        /// <summary>
        /// Commit (clear) changes for a specific layer.
        /// Call this after successfully saving to that layer.
        /// </summary>
        public void CommitChanges(ConfigLayer layer)
        {
            _pending[layer].Clear();
        }

        /// <summary>
        /// Commit changes for a specific function in a specific layer.
        /// </summary>
        public void CommitChanges(ConfigLayer layer, int functionId)
        {
            if (_pending[layer].ContainsKey(functionId))
                _pending[layer].Remove(functionId);
        }

        /// <summary>
        /// Discard all pending changes across all layers.
        /// </summary>
        public void DiscardAll()
        {
            foreach (var layer in _pending.Values)
                layer.Clear();
        }

        /// <summary>
        /// Discard changes for a specific layer.
        /// </summary>
        public void DiscardChanges(ConfigLayer layer)
        {
            _pending[layer].Clear();
        }

        /// <summary>
        /// Discard changes for a specific function across all layers.
        /// </summary>
        public void DiscardChanges(int functionId)
        {
            foreach (var layer in _pending.Values)
            {
                if (layer.ContainsKey(functionId))
                    layer.Remove(functionId);
            }
        }

        /// <summary>
        /// Discard a specific field change.
        /// </summary>
        public void DiscardChange(int functionId, string fieldPath, ConfigLayer layer)
        {
            if (_pending[layer].TryGetValue(functionId, out var fields))
            {
                fields.Remove(fieldPath);
                if (!fields.Any())
                    _pending[layer].Remove(functionId);
            }
        }

        /// <summary>
        /// Re-route a pending change to a different layer.
        /// </summary>
        public void RerouteChange(int functionId, string fieldPath, ConfigLayer fromLayer, ConfigLayer toLayer)
        {
            if (!_pending[fromLayer].TryGetValue(functionId, out var fromFields))
                return;

            if (!fromFields.TryGetValue(fieldPath, out var value))
                return;

            // Remove from old layer
            fromFields.Remove(fieldPath);
            if (!fromFields.Any())
                _pending[fromLayer].Remove(functionId);

            // Add to new layer
            TrackChange(functionId, fieldPath, value, toLayer);
        }

        /// <summary>
        /// Get count of pending changes.
        /// </summary>
        public int Count => _pending.Sum(l => l.Value.Sum(f => f.Value.Count));
    }
}
