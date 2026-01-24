using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace User.PluginSdkDemo.GraphEditor
{
    public static class GraphSerializer
    {
        public const int CurrentVersion = 2;

        public static string Serialize(GraphDefinition graph)
        {
            if (graph == null)
            {
                throw new ArgumentNullException(nameof(graph));
            }

            graph.Version = CurrentVersion;
            var dto = GraphDefinitionDto.FromModel(graph);
            return JsonConvert.SerializeObject(dto, Formatting.Indented, JsonSettings());
        }

        public static GraphDefinition Deserialize(string json, out GraphValidationResult validation)
        {
            if (string.IsNullOrWhiteSpace(json))
            {
                throw new ArgumentException("Graph JSON is empty.", nameof(json));
            }

            GraphDefinitionDto dto = JsonConvert.DeserializeObject<GraphDefinitionDto>(json, JsonSettings());
            var graph = dto?.ToModel() ?? new GraphDefinition();

            // Migrate from v1 to v2: extract SignalGroup/SignalSuffix from legacy port names
            if (graph.Version < 2)
            {
                MigrateV1ToV2(graph);
                graph.Version = CurrentVersion;
            }

            validation = GraphValidator.Validate(graph);
            return graph;
        }

        private static void MigrateV1ToV2(GraphDefinition graph)
        {
            foreach (var node in graph.Nodes)
            {
                if (node.Kind == GraphNodeKind.Input)
                {
                    MigrateInputNode(node);
                }
                else if (node.Kind == GraphNodeKind.Output)
                {
                    MigrateOutputNode(node);
                }
                else if (node.Kind == GraphNodeKind.Param)
                {
                    MigrateParamNode(node);
                }
            }
        }

        private static void MigrateInputNode(GraphNode node)
        {
            // Try to infer SignalGroup from port names (e.g., "XPlane.IAS_kts" → group="XPlane", suffix="IAS_kts")
            foreach (var port in node.Ports)
            {
                if (port.Kind != GraphPortKind.Output || string.IsNullOrEmpty(port.Name))
                    continue;

                foreach (var group in GraphSignalCatalog.InputGroups)
                {
                    string prefix = group + ".";
                    if (port.Name.StartsWith(prefix, StringComparison.Ordinal))
                    {
                        if (string.IsNullOrEmpty(node.SignalGroup))
                            node.SignalGroup = group;
                        port.SignalSuffix = port.Name.Substring(prefix.Length);
                        port.Name = port.SignalSuffix;
                        break;
                    }
                }
            }

            // Default to first input group if not set
            if (string.IsNullOrEmpty(node.SignalGroup) && GraphSignalCatalog.InputGroups.Count > 0)
            {
                node.SignalGroup = GraphSignalCatalog.InputGroups[0];
            }
        }

        private static void MigrateOutputNode(GraphNode node)
        {
            // Try to infer SignalGroup from port names (e.g., "FlightStickPitch.SpringGain")
            foreach (var port in node.Ports)
            {
                if (port.Kind != GraphPortKind.Input || string.IsNullOrEmpty(port.Name))
                    continue;

                foreach (var group in GraphSignalCatalog.OutputGroups)
                {
                    string prefix = group + ".";
                    if (port.Name.StartsWith(prefix, StringComparison.Ordinal))
                    {
                        if (string.IsNullOrEmpty(node.SignalGroup))
                            node.SignalGroup = group;
                        port.SignalSuffix = port.Name.Substring(prefix.Length);
                        port.Name = port.SignalSuffix;
                        break;
                    }
                }
            }

            // Default to first output group if not set
            if (string.IsNullOrEmpty(node.SignalGroup) && GraphSignalCatalog.OutputGroups.Count > 0)
            {
                node.SignalGroup = GraphSignalCatalog.OutputGroups[0];
            }
        }

        private static void MigrateParamNode(GraphNode node)
        {
            // Try to infer SignalGroup from port names
            foreach (var port in node.Ports)
            {
                if (port.Kind != GraphPortKind.Output || string.IsNullOrEmpty(port.Name))
                    continue;

                foreach (var group in GraphSignalCatalog.ParamGroups)
                {
                    string prefix = group + ".";
                    if (port.Name.StartsWith(prefix, StringComparison.Ordinal))
                    {
                        if (string.IsNullOrEmpty(node.SignalGroup))
                            node.SignalGroup = group;
                        port.SignalSuffix = port.Name.Substring(prefix.Length);
                        port.Name = port.SignalSuffix;
                        break;
                    }
                }

                // For params, if no group matched, use the port name as suffix
                if (string.IsNullOrEmpty(port.SignalSuffix))
                {
                    port.SignalSuffix = port.Name;
                }
            }

            // Default to first param group if not set
            if (string.IsNullOrEmpty(node.SignalGroup) && GraphSignalCatalog.ParamGroups.Count > 0)
            {
                node.SignalGroup = GraphSignalCatalog.ParamGroups[0];
            }
        }

        private static JsonSerializerSettings JsonSettings()
        {
            return new JsonSerializerSettings
            {
                Converters = { new StringEnumConverter() }
            };
        }
    }

    public sealed class GraphValidationResult
    {
        public List<string> Errors { get; } = new List<string>();
        public List<string> Warnings { get; } = new List<string>();
        public bool IsValid => Errors.Count == 0;
    }

    public static class GraphValidator
    {
        public static GraphValidationResult Validate(GraphDefinition graph)
        {
            var result = new GraphValidationResult();
            if (graph == null)
            {
                result.Errors.Add("Graph is null.");
                return result;
            }

            var nodeIds = new HashSet<string>();
            foreach (var node in graph.Nodes)
            {
                if (string.IsNullOrWhiteSpace(node.Id))
                {
                    result.Errors.Add("Node has no ID.");
                    continue;
                }
                if (!nodeIds.Add(node.Id))
                {
                    result.Errors.Add($"Duplicate node ID '{node.Id}'.");
                }
            }

            foreach (var link in graph.Links)
            {
                if (!nodeIds.Contains(link.FromNodeId))
                {
                    result.Errors.Add($"Link references missing node '{link.FromNodeId}'.");
                }
                if (!nodeIds.Contains(link.ToNodeId))
                {
                    result.Errors.Add($"Link references missing node '{link.ToNodeId}'.");
                }
            }

            return result;
        }
    }

    internal sealed class GraphDefinitionDto
    {
        public int Version { get; set; } = 1;
        public List<GraphNodeDto> Nodes { get; set; } = new List<GraphNodeDto>();
        public List<GraphLinkDto> Links { get; set; } = new List<GraphLinkDto>();
        public List<GraphParamDto> Params { get; set; } = new List<GraphParamDto>();
        public Dictionary<string, double> ParamValues { get; set; }

        public static GraphDefinitionDto FromModel(GraphDefinition graph)
        {
            var dto = new GraphDefinitionDto
            {
                Version = graph.Version
            };
            foreach (var node in graph.Nodes)
            {
                dto.Nodes.Add(GraphNodeDto.FromModel(node));
            }
            foreach (var link in graph.Links)
            {
                dto.Links.Add(GraphLinkDto.FromModel(link));
            }
            foreach (var param in graph.Params.Values)
            {
                dto.Params.Add(GraphParamDto.FromModel(param));
            }
            if (graph.ParamValues != null && graph.ParamValues.Count > 0)
            {
                dto.ParamValues = new Dictionary<string, double>(graph.ParamValues);
            }
            return dto;
        }

        public GraphDefinition ToModel()
        {
            var graph = new GraphDefinition
            {
                Version = Version
            };
            if (Nodes != null)
            {
                foreach (var node in Nodes)
                {
                    graph.Nodes.Add(node.ToModel());
                }
            }
            if (Links != null)
            {
                foreach (var link in Links)
                {
                    graph.Links.Add(link.ToModel());
                }
            }
            if (Params != null)
            {
                foreach (var param in Params)
                {
                    graph.Params[param.Name] = param.ToModel();
                }
            }
            if (ParamValues != null)
            {
                foreach (var kvp in ParamValues)
                {
                    graph.ParamValues[kvp.Key] = kvp.Value;
                }
            }
            return graph;
        }
    }

    internal sealed class GraphNodeDto
    {
        public string Id { get; set; } = "";
        public string Title { get; set; } = "";
        public GraphNodeKind Kind { get; set; }
        public double X { get; set; }
        public double Y { get; set; }
        public List<GraphPortDto> Ports { get; set; } = new List<GraphPortDto>();
        public string Op { get; set; } = "";
        public string Func { get; set; } = "";
        public string IncludePath { get; set; } = "";
        public double ConstValue { get; set; }
        public string SignalGroup { get; set; } = "";

        // Conditional serialization: only include kind-specific fields when relevant
        public bool ShouldSerializeTitle() =>
            Kind != GraphNodeKind.Input && Kind != GraphNodeKind.Output && Kind != GraphNodeKind.Param;
        public bool ShouldSerializeOp() => Kind == GraphNodeKind.Op;
        public bool ShouldSerializeFunc() => Kind == GraphNodeKind.Func;
        public bool ShouldSerializeIncludePath() => Kind == GraphNodeKind.Include || Kind == GraphNodeKind.Func;
        public bool ShouldSerializeConstValue() => Kind == GraphNodeKind.Const;
        public bool ShouldSerializeSignalGroup() =>
            Kind == GraphNodeKind.Input || Kind == GraphNodeKind.Output || Kind == GraphNodeKind.Param;

        public static GraphNodeDto FromModel(GraphNode node)
        {
            var dto = new GraphNodeDto
            {
                Id = node.Id,
                Kind = node.Kind,
                X = node.X,
                Y = node.Y
            };

            // Only populate fields appropriate for this Kind (normalization)
            bool isSignalNode = node.Kind == GraphNodeKind.Input ||
                                node.Kind == GraphNodeKind.Output ||
                                node.Kind == GraphNodeKind.Param;
            if (isSignalNode)
            {
                dto.SignalGroup = node.SignalGroup;
            }
            else
            {
                dto.Title = node.Title;
                if (node.Kind == GraphNodeKind.Op)
                    dto.Op = node.Op;
                if (node.Kind == GraphNodeKind.Func)
                {
                    dto.Func = node.Func;
                    dto.IncludePath = node.IncludePath;
                }
                if (node.Kind == GraphNodeKind.Include)
                    dto.IncludePath = node.IncludePath;
                if (node.Kind == GraphNodeKind.Const)
                    dto.ConstValue = node.ConstValue;
            }

            foreach (var port in node.Ports)
            {
                dto.Ports.Add(GraphPortDto.FromModel(port, isSignalNode));
            }
            return dto;
        }

        public GraphNode ToModel()
        {
            var node = new GraphNode
            {
                Id = Id,
                Kind = Kind,
                X = X,
                Y = Y
            };

            // Only populate fields appropriate for this Kind (normalization)
            bool isSignalNode = Kind == GraphNodeKind.Input ||
                                Kind == GraphNodeKind.Output ||
                                Kind == GraphNodeKind.Param;
            if (isSignalNode)
            {
                node.SignalGroup = SignalGroup ?? "";
            }
            else
            {
                node.Title = Title ?? "";
                if (Kind == GraphNodeKind.Op)
                    node.Op = Op ?? "";
                if (Kind == GraphNodeKind.Func)
                {
                    node.Func = Func ?? "";
                    node.IncludePath = IncludePath ?? "";
                }
                if (Kind == GraphNodeKind.Include)
                    node.IncludePath = IncludePath ?? "";
                if (Kind == GraphNodeKind.Const)
                    node.ConstValue = ConstValue;
            }

            if (Ports != null)
            {
                foreach (var port in Ports)
                {
                    node.Ports.Add(port.ToModel(isSignalNode));
                }
            }
            return node;
        }
    }

    internal sealed class GraphPortDto
    {
        public string Name { get; set; } = "";
        public GraphPortKind Kind { get; set; }
        public string SignalSuffix { get; set; } = "";

        // Conditional serialization: Name for non-signal ports, SignalSuffix for signal ports
        public bool ShouldSerializeName() => string.IsNullOrEmpty(SignalSuffix);
        public bool ShouldSerializeSignalSuffix() => !string.IsNullOrEmpty(SignalSuffix);

        public static GraphPortDto FromModel(GraphPort port, bool isSignalNode)
        {
            var dto = new GraphPortDto { Kind = port.Kind };
            if (isSignalNode)
            {
                // Signal ports use SignalSuffix; Name is derived
                dto.SignalSuffix = port.SignalSuffix;
            }
            else
            {
                // Non-signal ports use Name only
                dto.Name = port.Name;
            }
            return dto;
        }

        /// <summary>
        /// Converts DTO to model. For signal ports, Name = SignalSuffix for link/ID matching.
        /// Full signal name is built by GraphRuntimeConverter from SignalGroup + SignalSuffix.
        /// </summary>
        public GraphPort ToModel(bool isSignalNode)
        {
            var port = new GraphPort { Kind = Kind };
            if (isSignalNode)
            {
                // Signal ports: Name = SignalSuffix (for link matching / ID generation)
                port.SignalSuffix = SignalSuffix ?? "";
                port.Name = port.SignalSuffix;
            }
            else
            {
                // Non-signal ports: Name is canonical
                port.Name = Name ?? "";
            }
            return port;
        }
    }

    internal sealed class GraphLinkDto
    {
        public string FromNodeId { get; set; } = "";
        public string FromPort { get; set; } = "";
        public string ToNodeId { get; set; } = "";
        public string ToPort { get; set; } = "";

        public static GraphLinkDto FromModel(GraphLink link)
        {
            return new GraphLinkDto
            {
                FromNodeId = link.FromNodeId,
                FromPort = link.FromPort,
                ToNodeId = link.ToNodeId,
                ToPort = link.ToPort
            };
        }

        public GraphLink ToModel()
        {
            return new GraphLink
            {
                FromNodeId = FromNodeId,
                FromPort = FromPort,
                ToNodeId = ToNodeId,
                ToPort = ToPort
            };
        }
    }

    internal sealed class GraphParamDto
    {
        public string Name { get; set; } = "";
        public double DefaultValue { get; set; }
        public double Min { get; set; }
        public double Max { get; set; }
        public GraphParamUiDto Ui { get; set; }

        public static GraphParamDto FromModel(GraphParam param)
        {
            return new GraphParamDto
            {
                Name = param.Name,
                DefaultValue = param.DefaultValue,
                Min = param.Min,
                Max = param.Max,
                Ui = param.Ui != null ? GraphParamUiDto.FromModel(param.Ui) : null
            };
        }

        public GraphParam ToModel()
        {
            return new GraphParam
            {
                Name = Name,
                DefaultValue = DefaultValue,
                Min = Min,
                Max = Max,
                Ui = Ui?.ToModel()
            };
        }
    }

    internal sealed class GraphParamUiDto
    {
        public string Widget { get; set; } = "";
        public string Label { get; set; } = "";
        public string Group { get; set; } = "";
        public string Units { get; set; } = "";
        public double? Step { get; set; }
        public int? Precision { get; set; }
        public bool LogScale { get; set; }
        public List<GraphParamOptionDto> Options { get; set; } = new List<GraphParamOptionDto>();

        public static GraphParamUiDto FromModel(GraphParamUi ui)
        {
            var dto = new GraphParamUiDto
            {
                Widget = ui.Widget,
                Label = ui.Label,
                Group = ui.Group,
                Units = ui.Units,
                Step = ui.Step,
                Precision = ui.Precision,
                LogScale = ui.LogScale
            };
            foreach (var option in ui.Options)
            {
                dto.Options.Add(GraphParamOptionDto.FromModel(option));
            }
            return dto;
        }

        public GraphParamUi ToModel()
        {
            var ui = new GraphParamUi
            {
                Widget = Widget ?? "",
                Label = Label ?? "",
                Group = Group ?? "",
                Units = Units ?? "",
                Step = Step,
                Precision = Precision,
                LogScale = LogScale
            };
            if (Options != null)
            {
                foreach (var option in Options)
                {
                    ui.Options.Add(option.ToModel());
                }
            }
            return ui;
        }
    }

    internal sealed class GraphParamOptionDto
    {
        public string Value { get; set; } = "";
        public string Label { get; set; } = "";

        public static GraphParamOptionDto FromModel(GraphParamOption option)
        {
            return new GraphParamOptionDto
            {
                Value = option.Value,
                Label = option.Label
            };
        }

        public GraphParamOption ToModel()
        {
            return new GraphParamOption
            {
                Value = Value ?? "",
                Label = Label ?? ""
            };
        }
    }
}
