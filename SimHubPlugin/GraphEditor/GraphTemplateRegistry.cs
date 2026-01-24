using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace User.PluginSdkDemo.GraphEditor
{
    public static class GraphTemplateRegistry
    {
        private static readonly List<GraphTemplateEntry> AllTemplates = new List<GraphTemplateEntry>
        {
            new GraphTemplateEntry
            {
                Name = "Fixed-Wing Aircraft (Basic)",
                Description = "Basic FFB graph for fixed-wing aircraft with aerodynamic forces and trim.",
                Category = "Flight",
                TemplatePath = "graphs/templates/plane_default.json",
                GameIds = new[] { "X-Plane", "XPlane", "XPlane11", "XPlane12", "MSFS2020", "MSFS2024", "Prepar3D", "P3D", "FSX" }
            },
            new GraphTemplateEntry
            {
                Name = "Helicopter (Basic)",
                Description = "Basic FFB graph for helicopters with collective force feedback and rotor dynamics.",
                Category = "Flight",
                TemplatePath = "graphs/templates/heli_default.json",
                GameIds = new[] { "X-Plane", "XPlane", "XPlane11", "XPlane12", "MSFS2020", "MSFS2024" }
            }
        };

        /// <summary>
        /// Gets all available templates filtered by game ID.
        /// </summary>
        /// <param name="gameId">The game ID to filter by, or null to get all templates.</param>
        /// <param name="baseDirectory">Base directory for resolving template paths.</param>
        /// <returns>Enumerable of template entries that match the game ID.</returns>
        public static IEnumerable<GraphTemplateEntry> GetTemplates(string gameId, string baseDirectory)
        {
            var filtered = string.IsNullOrWhiteSpace(gameId)
                ? AllTemplates
                : AllTemplates.Where(t => t.GameIds != null &&
                                         t.GameIds.Any(g => string.Equals(g, gameId, StringComparison.OrdinalIgnoreCase)));

            // Verify template files exist and return only valid ones
            var result = new List<GraphTemplateEntry>();
            foreach (var template in filtered)
            {
                string resolvedPath = ResolveTemplatePath(template.TemplatePath, baseDirectory);
                if (!string.IsNullOrWhiteSpace(resolvedPath) && File.Exists(resolvedPath))
                {
                    result.Add(template);
                }
            }

            return result;
        }

        /// <summary>
        /// Gets the full path to a template file.
        /// </summary>
        /// <param name="templatePath">The template path (relative or absolute).</param>
        /// <param name="baseDirectory">Base directory for resolving relative paths.</param>
        /// <returns>Full path to the template file, or null if resolution fails.</returns>
        public static string ResolveTemplatePath(string templatePath, string baseDirectory)
        {
            if (string.IsNullOrWhiteSpace(templatePath))
            {
                return null;
            }

            if (Path.IsPathRooted(templatePath))
            {
                return templatePath;
            }

            // Try base directory first
            if (!string.IsNullOrWhiteSpace(baseDirectory))
            {
                string combined = Path.Combine(baseDirectory, templatePath);
                if (File.Exists(combined))
                {
                    return combined;
                }
            }

            // Try current directory
            if (File.Exists(templatePath))
            {
                return Path.GetFullPath(templatePath);
            }

            // Try plugin directory
            string pluginDir = AppDomain.CurrentDomain.BaseDirectory;
            if (!string.IsNullOrWhiteSpace(pluginDir))
            {
                string combined = Path.Combine(pluginDir, templatePath);
                if (File.Exists(combined))
                {
                    return combined;
                }
            }

            return null;
        }

        /// <summary>
        /// Checks if any templates are available for the given game.
        /// </summary>
        /// <param name="gameId">The game ID to check.</param>
        /// <param name="baseDirectory">Base directory for resolving template paths.</param>
        /// <returns>True if at least one template is available.</returns>
        public static bool HasTemplatesForGame(string gameId, string baseDirectory)
        {
            return GetTemplates(gameId, baseDirectory).Any();
        }
    }
}
