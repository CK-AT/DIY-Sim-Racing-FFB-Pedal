using System;
using System.IO;

namespace DiyFfb.GraphEditor
{
    /// <summary>
    /// Shared helpers for storing graph file references in vehicle profiles.
    /// Used by both the graph editor's "Save as Copy" flow and the Profile
    /// Browser's custom-graph picker so path-storage rules stay identical.
    /// </summary>
    public static class GraphPathUtil
    {
        /// <summary>
        /// Folder (relative to the plugin base dir, forward-slashed) where custom
        /// graphs copied into the managed library are stored.
        /// </summary>
        public const string CustomLibraryFolder = "graphs/custom";

        /// <summary>
        /// Makes a graph path relative to <paramref name="baseDir"/> (forward slashes)
        /// when the file lives under it; otherwise returns the absolute path unchanged.
        /// </summary>
        public static string MakeRelative(string fullPath, string baseDir)
        {
            if (string.IsNullOrEmpty(fullPath) || string.IsNullOrEmpty(baseDir))
                return fullPath;

            if (fullPath.StartsWith(baseDir, StringComparison.OrdinalIgnoreCase))
            {
                string relative = fullPath.Substring(baseDir.Length);
                return relative.TrimStart(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar)
                               .Replace(Path.DirectorySeparatorChar, '/');
            }

            return fullPath;
        }

        /// <summary>
        /// Copies a graph file into the plugin-managed custom-graph library
        /// (&lt;baseDir&gt;/graphs/custom), avoiding filename collisions by appending
        /// " (2)", " (3)", … Returns the full path of the created copy. Never
        /// overwrites an existing file. Throws on I/O failure.
        /// </summary>
        public static string CopyIntoCustomLibrary(string sourcePath, string baseDir)
        {
            if (string.IsNullOrEmpty(sourcePath))
                throw new ArgumentException("Source path is empty.", nameof(sourcePath));

            string targetDir = Path.Combine(baseDir ?? "",
                CustomLibraryFolder.Replace('/', Path.DirectorySeparatorChar));
            Directory.CreateDirectory(targetDir);

            string name = Path.GetFileNameWithoutExtension(sourcePath);
            string ext = Path.GetExtension(sourcePath);
            string candidate = Path.Combine(targetDir, name + ext);
            int n = 2;
            while (File.Exists(candidate))
            {
                candidate = Path.Combine(targetDir, $"{name} ({n}){ext}");
                n++;
            }

            File.Copy(sourcePath, candidate);
            return candidate;
        }
    }
}
