using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DiyFfb.GraphTest
{
    /// <summary>
    /// Debug logger for tracing graph evaluation flow.
    /// Writes to a dedicated log file for analysis.
    /// </summary>
    public static class GraphDebugLogger
    {
        private static readonly object _lock = new object();
        private static string _logPath;
        private static bool _enabled;

        public static bool Enabled
        {
            get => _enabled;
            set
            {
                _enabled = value;
                if (_enabled && string.IsNullOrEmpty(_logPath))
                {
                    _logPath = Path.Combine(
                        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                        "DiyFfb",
                        "graph_debug.log");
                    Directory.CreateDirectory(Path.GetDirectoryName(_logPath));
                }
            }
        }

        public static string LogPath => _logPath;

        public static void Log(string message)
        {
            if (!_enabled) return;

            lock (_lock)
            {
                try
                {
                    File.AppendAllText(_logPath, $"[{DateTime.Now:HH:mm:ss.fff}] {message}\n");
                }
                catch
                {
                    // Ignore logging errors
                }
            }
        }

        public static void LogSection(string title)
        {
            if (!_enabled) return;
            var separator = new string('=', 60);
            Log($"\n{separator}\n  {title}\n{separator}");
        }

        public static void LogDict(string name, IEnumerable<KeyValuePair<string, double>> dict)
        {
            if (!_enabled || dict == null) return;

            var sb = new StringBuilder();
            sb.AppendLine($"  {name}:");
            foreach (var kvp in dict)
            {
                sb.AppendLine($"    [{kvp.Key}] = {kvp.Value:F4}");
            }
            Log(sb.ToString());
        }

        public static void LogMap(string name, IDictionary<string, string> map)
        {
            if (!_enabled || map == null) return;

            var sb = new StringBuilder();
            sb.AppendLine($"  {name}:");
            foreach (var kvp in map)
            {
                sb.AppendLine($"    [{kvp.Key}] -> [{kvp.Value}]");
            }
            Log(sb.ToString());
        }

        public static void LogIndexMap(string name, Dictionary<string, int> indexMap, double[] values)
        {
            if (!_enabled || indexMap == null) return;

            var sb = new StringBuilder();
            sb.AppendLine($"  {name}:");
            foreach (var kvp in indexMap)
            {
                double value = kvp.Value >= 0 && kvp.Value < values.Length ? values[kvp.Value] : double.NaN;
                sb.AppendLine($"    [{kvp.Key}] idx={kvp.Value} val={value:F4}");
            }
            Log(sb.ToString());
        }

        public static void Clear()
        {
            if (!string.IsNullOrEmpty(_logPath))
            {
                lock (_lock)
                {
                    try
                    {
                        File.WriteAllText(_logPath, $"=== Graph Debug Log Started {DateTime.Now:yyyy-MM-dd HH:mm:ss} ===\n");
                    }
                    catch
                    {
                        // Ignore
                    }
                }
            }
        }
    }
}
