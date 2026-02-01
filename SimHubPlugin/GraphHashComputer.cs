using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;

namespace DiyFfb
{
    /// <summary>
    /// Computes content hashes of FFB graphs including all nested includes.
    /// Used to detect when graph files change to trigger parameter migration review.
    /// </summary>
    public static class GraphHashComputer
    {
        /// <summary>
        /// Computes a combined hash of the root graph file and all included files.
        /// </summary>
        /// <param name="graphPath">Absolute path to the root graph file.</param>
        /// <param name="graph">Loaded graph definition (for finding include paths).</param>
        /// <returns>Hex string hash, or null if graph path is invalid.</returns>
        public static string ComputeGraphTreeHash(string graphPath, GraphEditor.GraphDefinition graph)
        {
            if (string.IsNullOrWhiteSpace(graphPath) || !File.Exists(graphPath))
            {
                return null;
            }

            var allPaths = new List<string>();
            var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            CollectIncludePaths(graphPath, graph, visited, allPaths);

            // Sort paths for deterministic ordering
            allPaths.Sort(StringComparer.OrdinalIgnoreCase);

            // Compute hash of all file contents combined
            using (var sha = SHA256.Create())
            {
                foreach (var path in allPaths)
                {
                    if (File.Exists(path))
                    {
                        var bytes = File.ReadAllBytes(path);
                        sha.TransformBlock(bytes, 0, bytes.Length, bytes, 0);
                    }
                }
                sha.TransformFinalBlock(Array.Empty<byte>(), 0, 0);
                return BitConverter.ToString(sha.Hash).Replace("-", "").ToLowerInvariant();
            }
        }

        /// <summary>
        /// Recursively collects all file paths including the root graph and all includes.
        /// </summary>
        private static void CollectIncludePaths(
            string graphPath,
            GraphEditor.GraphDefinition graph,
            HashSet<string> visited,
            List<string> allPaths)
        {
            string normalizedPath = Path.GetFullPath(graphPath);
            if (visited.Contains(normalizedPath))
            {
                return;
            }
            visited.Add(normalizedPath);
            allPaths.Add(normalizedPath);

            if (graph?.Nodes == null)
            {
                return;
            }

            string graphDir = Path.GetDirectoryName(normalizedPath);

            foreach (var node in graph.Nodes)
            {
                if (node.Kind == GraphEditor.GraphNodeKind.Include
                    && !string.IsNullOrWhiteSpace(node.IncludePath))
                {
                    string includePath = ResolveIncludePath(graphDir, node.IncludePath);
                    if (!string.IsNullOrWhiteSpace(includePath) && File.Exists(includePath))
                    {
                        // Load include to find nested includes
                        var includedGraph = TryLoadGraph(includePath);
                        CollectIncludePaths(includePath, includedGraph, visited, allPaths);
                    }
                }
            }
        }

        private static string ResolveIncludePath(string baseDir, string includePath)
        {
            if (string.IsNullOrWhiteSpace(includePath))
            {
                return null;
            }

            if (Path.IsPathRooted(includePath))
            {
                return Path.GetFullPath(includePath);
            }

            if (!string.IsNullOrWhiteSpace(baseDir))
            {
                return Path.GetFullPath(Path.Combine(baseDir, includePath));
            }

            return includePath;
        }

        private static GraphEditor.GraphDefinition TryLoadGraph(string path)
        {
            try
            {
                string json = File.ReadAllText(path);
                return GraphEditor.GraphSerializer.Deserialize(json, out _);
            }
            catch
            {
                return null;
            }
        }
    }
}
