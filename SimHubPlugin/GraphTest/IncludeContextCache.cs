using System;
using System.Collections.Generic;

namespace DiyFfb.GraphTest
{
    /// <summary>
    /// Caches Include node evaluation contexts keyed by resolved path.
    /// Enables sub-graph previews to use real parent inputs.
    /// </summary>
    public sealed class IncludeContextCache
    {
        // Key: resolved include path (absolute), Value: list of contexts (one per Include node)
        private readonly Dictionary<string, List<IncludeCallContext>> _contexts
            = new Dictionary<string, List<IncludeCallContext>>(StringComparer.OrdinalIgnoreCase);

        private static readonly IReadOnlyList<IncludeCallContext> EmptyList = new List<IncludeCallContext>();

        public void Clear()
        {
            _contexts.Clear();
        }

        public void Add(string resolvedPath, IncludeCallContext context)
        {
            if (string.IsNullOrEmpty(resolvedPath) || context == null)
                return;

            if (!_contexts.TryGetValue(resolvedPath, out var list))
            {
                list = new List<IncludeCallContext>();
                _contexts[resolvedPath] = list;
            }
            list.Add(context);
        }

        public IReadOnlyList<IncludeCallContext> GetContexts(string resolvedPath)
        {
            if (string.IsNullOrEmpty(resolvedPath))
                return EmptyList;

            return _contexts.TryGetValue(resolvedPath, out var list) ? list : EmptyList;
        }

        public bool HasContexts(string resolvedPath)
        {
            if (string.IsNullOrEmpty(resolvedPath))
                return false;

            return _contexts.TryGetValue(resolvedPath, out var list) && list.Count > 0;
        }
    }
}
