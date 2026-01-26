using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace DiyFfb.GraphTest
{
    public sealed class GraphIncludeResolver : IGraphResolver
    {
        private readonly string _baseDirectory;
        private readonly GraphLoader _loader = new GraphLoader();
        private readonly GraphSaver _saver = new GraphSaver();
        private readonly Dictionary<string, GraphDefinition> _cache = new Dictionary<string, GraphDefinition>();
        private readonly string _libraryDirectory;
        private readonly Dictionary<string, GraphDefinition> _library = new Dictionary<string, GraphDefinition>();
        private readonly string _libraryIndexPath;
        private readonly BlockLibraryIndex _libraryIndex;

        public GraphIncludeResolver(string baseDirectory)
        {
            _baseDirectory = baseDirectory ?? "";
            _libraryDirectory = Path.Combine(_baseDirectory, "graphs", "_embedded");
            _libraryIndexPath = Path.Combine(_libraryDirectory, "index.json");
            _libraryIndex = LoadIndex();
        }

        public GraphDefinition GetGraph(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return null;
            }

            string resolved = path;
            if (!Path.IsPathRooted(resolved))
            {
                resolved = Path.Combine(_baseDirectory, path);
            }

            if (_cache.TryGetValue(resolved, out var cached))
            {
                return cached;
            }

            if (!File.Exists(resolved))
            {
                return null;
            }

            string json = File.ReadAllText(resolved);
            GraphDefinition graph = null;

            // Try runtime format first
            try
            {
                graph = _loader.LoadFromJson(json, out _);
            }
            catch
            {
                // Ignore - might be editor format
            }

            // Check if the loaded graph has valid nodes (not just non-empty count)
            // Runtime format should have nodes with non-empty Name for Input/Param/Output types
            bool hasValidNodes = graph != null && graph.Nodes.Count > 0 &&
                graph.Nodes.Values.Any(n =>
                    (n.Type != NodeType.Input && n.Type != NodeType.Param && n.Type != NodeType.Output) ||
                    !string.IsNullOrEmpty(n.Name));

            // If runtime format failed or has invalid nodes, try editor format conversion
            if (!hasValidNodes)
            {
                graph = TryLoadEditorFormat(json, resolved);
            }

            if (graph != null)
            {
                _cache[resolved] = graph;
                _library[resolved] = graph;
                RegisterBlock(resolved, graph);
            }
            return graph;
        }

        /// <summary>
        /// Delegate for converting editor-format JSON to runtime format.
        /// Set by the plugin to enable editor format support.
        /// Parameters: (json, resolvedFilePath) to allow path-relative include resolution.
        /// </summary>
        public Func<string, string, GraphDefinition> EditorFormatConverter { get; set; }

        private GraphDefinition TryLoadEditorFormat(string json, string resolvedPath)
        {
            return EditorFormatConverter?.Invoke(json, resolvedPath);
        }

        public GraphDefinition ResolveInclude(GraphNode includeNode)
        {
            if (includeNode == null)
            {
                return null;
            }

            if (includeNode.InlineGraph != null)
            {
                string path = includeNode.Path;
                if (string.IsNullOrWhiteSpace(path))
                {
                    string hash = ComputeGraphHash(includeNode.InlineGraph);
                    path = Path.Combine(_libraryDirectory, $"{hash}.json");
                }

                if (!_library.ContainsKey(path))
                {
                    Directory.CreateDirectory(Path.GetDirectoryName(path) ?? _libraryDirectory);
                    File.WriteAllText(path, _saver.SaveToJson(includeNode.InlineGraph));
                    _library[path] = includeNode.InlineGraph;
                    RegisterBlock(path, includeNode.InlineGraph);
                }

                _cache[path] = includeNode.InlineGraph;
                includeNode.Path = path;
                return includeNode.InlineGraph;
            }

            return GetGraph(includeNode.Path);
        }

        private static string ComputeGraphHash(GraphDefinition graph)
        {
            string json = new GraphSaver().SaveToJson(graph);
            using (var sha = System.Security.Cryptography.SHA256.Create())
            {
                var bytes = System.Text.Encoding.UTF8.GetBytes(json);
                var hash = sha.ComputeHash(bytes);
                return BitConverter.ToString(hash).Replace("-", "").ToLowerInvariant();
            }
        }

        private BlockLibraryIndex LoadIndex()
        {
            try
            {
                if (File.Exists(_libraryIndexPath))
                {
                    string json = File.ReadAllText(_libraryIndexPath);
                    var index = Newtonsoft.Json.JsonConvert.DeserializeObject<BlockLibraryIndex>(json);
                    if (index != null)
                    {
                        return index;
                    }
                }
            }
            catch
            {
                // ignore index parse errors
            }

            return new BlockLibraryIndex();
        }

        private void RegisterBlock(string path, GraphDefinition graph)
        {
            if (string.IsNullOrWhiteSpace(path) || graph == null)
            {
                return;
            }

            string hash = ComputeGraphHash(graph);
            var entry = _libraryIndex.FindByPath(path);
            if (entry == null)
            {
                entry = new BlockLibraryEntry
                {
                    Path = path,
                    Hash = hash,
                    Version = graph.Version,
                    UpdatedUtc = DateTime.UtcNow
                };
                _libraryIndex.Entries.Add(entry);
            }
            else
            {
                entry.Hash = hash;
                entry.Version = graph.Version;
                entry.UpdatedUtc = DateTime.UtcNow;
            }

            SaveIndex();
        }

        private void SaveIndex()
        {
            try
            {
                Directory.CreateDirectory(_libraryDirectory);
                string json = Newtonsoft.Json.JsonConvert.SerializeObject(_libraryIndex, Newtonsoft.Json.Formatting.Indented);
                File.WriteAllText(_libraryIndexPath, json);
            }
            catch
            {
                // ignore index write errors
            }
        }

        private sealed class BlockLibraryIndex
        {
            public List<BlockLibraryEntry> Entries { get; set; } = new List<BlockLibraryEntry>();

            public BlockLibraryEntry FindByPath(string path)
            {
                foreach (var entry in Entries)
                {
                    if (string.Equals(entry.Path, path, StringComparison.OrdinalIgnoreCase))
                    {
                        return entry;
                    }
                }

                return null;
            }
        }

        private sealed class BlockLibraryEntry
        {
            public string Path { get; set; } = string.Empty;
            public string Hash { get; set; } = string.Empty;
            public int Version { get; set; }
            public DateTime UpdatedUtc { get; set; }
        }
    }
}
