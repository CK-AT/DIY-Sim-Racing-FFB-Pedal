using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace User.PluginSdkDemo.GraphEditor
{
    /// <summary>
    /// Result of scanning graph usage across vehicles and includes.
    /// </summary>
    public sealed class GraphUsageReport
    {
        public string GraphPath { get; set; }
        public string CurrentVehicleKey { get; set; }

        /// <summary>
        /// Vehicle keys that directly reference this graph as their GraphPath.
        /// </summary>
        public List<string> DirectUsers { get; } = new List<string>();

        /// <summary>
        /// Graphs that include this graph (and their vehicle users).
        /// </summary>
        public List<IncludeUsage> IncludedBy { get; } = new List<IncludeUsage>();

        /// <summary>
        /// True if this graph is shared (used by multiple vehicles, or by
        /// any vehicle other than the current one, or included by other graphs).
        /// </summary>
        public bool IsShared
        {
            get
            {
                // If used by multiple vehicles directly
                if (DirectUsers.Count > 1)
                    return true;

                // If used by a vehicle other than the current one
                if (DirectUsers.Count == 1 && !string.IsNullOrEmpty(CurrentVehicleKey) &&
                    !string.Equals(DirectUsers[0], CurrentVehicleKey, StringComparison.OrdinalIgnoreCase))
                    return true;

                // If included by other graphs that have vehicle users
                if (IncludedBy.Any(i => i.VehicleKeys.Count > 0))
                    return true;

                return false;
            }
        }
    }

    /// <summary>
    /// Describes a parent graph that includes another graph.
    /// </summary>
    public sealed class IncludeUsage
    {
        public string IncludingGraphPath { get; set; }
        public List<string> VehicleKeys { get; } = new List<string>();
    }

    /// <summary>
    /// Scans graph usage to detect when graphs are shared between vehicles.
    /// </summary>
    public sealed class GraphUsageScanner
    {
        private readonly DiyFfbPlugin _plugin;
        private readonly string _baseDirectory;

        public GraphUsageScanner(DiyFfbPlugin plugin)
        {
            _plugin = plugin ?? throw new ArgumentNullException(nameof(plugin));
            _baseDirectory = AppDomain.CurrentDomain.BaseDirectory;
        }

        /// <summary>
        /// Gets a comprehensive usage report for the specified graph path.
        /// </summary>
        public GraphUsageReport GetUsageReport(string graphPath, string currentVehicleKey)
        {
            var report = new GraphUsageReport
            {
                GraphPath = graphPath,
                CurrentVehicleKey = currentVehicleKey
            };

            if (string.IsNullOrWhiteSpace(graphPath))
                return report;

            string normalizedPath = NormalizePath(graphPath);

            // Find direct users
            report.DirectUsers.AddRange(GetDirectUsers(normalizedPath));

            // Find graphs that include this one (and their vehicle users)
            var includingGraphs = GetIncludingGraphs(normalizedPath);
            foreach (var includingPath in includingGraphs)
            {
                var usage = new IncludeUsage { IncludingGraphPath = includingPath };
                usage.VehicleKeys.AddRange(GetDirectUsers(NormalizePath(includingPath)));
                report.IncludedBy.Add(usage);
            }

            return report;
        }

        /// <summary>
        /// Finds all vehicles that directly use this graph as their GraphPath.
        /// </summary>
        public List<string> GetDirectUsers(string normalizedGraphPath)
        {
            var users = new List<string>();

            var profiles = _plugin?.Settings?.AircraftFfbProfiles;
            if (profiles == null)
                return users;

            foreach (var kvp in profiles)
            {
                if (string.IsNullOrEmpty(kvp.Value?.GraphPath))
                    continue;

                string profilePath = NormalizePath(kvp.Value.GraphPath);
                if (string.Equals(profilePath, normalizedGraphPath, StringComparison.OrdinalIgnoreCase))
                {
                    users.Add(kvp.Key);
                }
            }

            return users;
        }

        /// <summary>
        /// Finds all graph files that include this graph (recursively scans parent graphs).
        /// </summary>
        public List<string> GetIncludingGraphs(string normalizedGraphPath)
        {
            var includingGraphs = new List<string>();
            var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            // Get all graph files we need to scan
            var graphFiles = GetAllGraphFiles();

            foreach (var graphFile in graphFiles)
            {
                if (visited.Contains(graphFile))
                    continue;

                visited.Add(graphFile);

                // Check if this graph includes the target
                var includes = GetGraphIncludes(graphFile);
                foreach (var includePath in includes)
                {
                    string normalizedInclude = NormalizePath(includePath, Path.GetDirectoryName(graphFile));
                    if (string.Equals(normalizedInclude, normalizedGraphPath, StringComparison.OrdinalIgnoreCase))
                    {
                        includingGraphs.Add(graphFile);
                        break;
                    }
                }
            }

            // Recursively find graphs that include the including graphs
            var additionalIncludes = new List<string>();
            foreach (var includingGraph in includingGraphs)
            {
                var parents = GetIncludingGraphsRecursive(NormalizePath(includingGraph), visited, graphFiles);
                additionalIncludes.AddRange(parents);
            }

            includingGraphs.AddRange(additionalIncludes);
            return includingGraphs.Distinct(StringComparer.OrdinalIgnoreCase).ToList();
        }

        private List<string> GetIncludingGraphsRecursive(string normalizedGraphPath, HashSet<string> visited, List<string> graphFiles)
        {
            var result = new List<string>();

            foreach (var graphFile in graphFiles)
            {
                if (visited.Contains(graphFile))
                    continue;

                var includes = GetGraphIncludes(graphFile);
                foreach (var includePath in includes)
                {
                    string normalizedInclude = NormalizePath(includePath, Path.GetDirectoryName(graphFile));
                    if (string.Equals(normalizedInclude, normalizedGraphPath, StringComparison.OrdinalIgnoreCase))
                    {
                        visited.Add(graphFile);
                        result.Add(graphFile);

                        // Recursively find parents of this graph
                        var parents = GetIncludingGraphsRecursive(NormalizePath(graphFile), visited, graphFiles);
                        result.AddRange(parents);
                        break;
                    }
                }
            }

            return result;
        }

        /// <summary>
        /// Gets all include paths from a graph file.
        /// </summary>
        private List<string> GetGraphIncludes(string graphFilePath)
        {
            var includes = new List<string>();

            try
            {
                if (!File.Exists(graphFilePath))
                    return includes;

                string json = File.ReadAllText(graphFilePath);
                var graph = GraphSerializer.Deserialize(json, out _);

                foreach (var node in graph.Nodes)
                {
                    if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
                    {
                        includes.Add(node.IncludePath);
                    }
                }
            }
            catch
            {
                // Ignore files that can't be parsed
            }

            return includes;
        }

        /// <summary>
        /// Gets all graph files in known locations.
        /// </summary>
        private List<string> GetAllGraphFiles()
        {
            var files = new List<string>();

            // Scan graphs/vehicles directory
            string vehiclesDir = Path.Combine(_baseDirectory, "graphs", "vehicles");
            if (Directory.Exists(vehiclesDir))
            {
                files.AddRange(Directory.GetFiles(vehiclesDir, "*.json", SearchOption.AllDirectories));
            }

            // Scan graphs/templates directory
            string templatesDir = Path.Combine(_baseDirectory, "graphs", "templates");
            if (Directory.Exists(templatesDir))
            {
                files.AddRange(Directory.GetFiles(templatesDir, "*.json", SearchOption.AllDirectories));
            }

            // Scan SimHubPlugin graphs directories (for embedded and templates)
            string pluginGraphsDir = Path.Combine(_baseDirectory, "SimHubPlugin", "graphs");
            if (Directory.Exists(pluginGraphsDir))
            {
                files.AddRange(Directory.GetFiles(pluginGraphsDir, "*.json", SearchOption.AllDirectories));
            }

            // Also scan from paths in profiles (they might be in custom locations)
            var profiles = _plugin?.Settings?.AircraftFfbProfiles;
            if (profiles != null)
            {
                foreach (var kvp in profiles)
                {
                    if (!string.IsNullOrEmpty(kvp.Value?.GraphPath))
                    {
                        string fullPath = ResolveGraphPath(kvp.Value.GraphPath);
                        if (File.Exists(fullPath) && !files.Contains(fullPath, StringComparer.OrdinalIgnoreCase))
                        {
                            files.Add(fullPath);
                        }
                    }
                }
            }

            return files.Distinct(StringComparer.OrdinalIgnoreCase).ToList();
        }

        /// <summary>
        /// Normalizes a graph path to an absolute path for comparison.
        /// </summary>
        private string NormalizePath(string path, string baseDir = null)
        {
            if (string.IsNullOrWhiteSpace(path))
                return "";

            // Normalize separators
            path = path.Replace('/', Path.DirectorySeparatorChar);

            if (Path.IsPathRooted(path))
                return Path.GetFullPath(path);

            // Try relative to provided base directory first
            if (!string.IsNullOrWhiteSpace(baseDir))
            {
                string combined = Path.Combine(baseDir, path);
                if (File.Exists(combined))
                    return Path.GetFullPath(combined);
            }

            // Try relative to app base directory
            string fromBase = Path.Combine(_baseDirectory, path);
            if (File.Exists(fromBase))
                return Path.GetFullPath(fromBase);

            // If file doesn't exist, still return normalized path
            return Path.GetFullPath(Path.Combine(baseDir ?? _baseDirectory, path));
        }

        /// <summary>
        /// Resolves a graph path (similar to plugin's ResolveGraphFilePath).
        /// </summary>
        private string ResolveGraphPath(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
                return "";

            path = path.Replace('/', Path.DirectorySeparatorChar);

            if (Path.IsPathRooted(path))
                return path;

            return Path.Combine(_baseDirectory, path);
        }
    }
}
