using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace User.PluginSdkDemo.GraphEditor
{
    public static class GraphSerializer
    {
        public static string Serialize(GraphDefinition graph)
        {
            if (graph == null)
            {
                throw new ArgumentNullException(nameof(graph));
            }

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
            validation = GraphValidator.Validate(graph);
            return graph;
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

        public static GraphNodeDto FromModel(GraphNode node)
        {
            var dto = new GraphNodeDto
            {
                Id = node.Id,
                Title = node.Title,
                Kind = node.Kind,
                X = node.X,
                Y = node.Y,
                Op = node.Op,
                Func = node.Func,
                IncludePath = node.IncludePath,
                ConstValue = node.ConstValue
            };
            foreach (var port in node.Ports)
            {
                dto.Ports.Add(GraphPortDto.FromModel(port));
            }
            return dto;
        }

        public GraphNode ToModel()
        {
            var node = new GraphNode
            {
                Id = Id,
                Title = Title,
                Kind = Kind,
                X = X,
                Y = Y,
                Op = Op,
                Func = Func,
                IncludePath = IncludePath,
                ConstValue = ConstValue
            };
            if (Ports != null)
            {
                foreach (var port in Ports)
                {
                    node.Ports.Add(port.ToModel());
                }
            }
            return node;
        }
    }

    internal sealed class GraphPortDto
    {
        public string Name { get; set; } = "";
        public GraphPortKind Kind { get; set; }

        public static GraphPortDto FromModel(GraphPort port)
        {
            return new GraphPortDto
            {
                Name = port.Name,
                Kind = port.Kind
            };
        }

        public GraphPort ToModel()
        {
            return new GraphPort
            {
                Name = Name,
                Kind = Kind
            };
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
