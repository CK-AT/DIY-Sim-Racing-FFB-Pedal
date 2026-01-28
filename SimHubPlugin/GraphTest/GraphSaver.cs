using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace DiyFfb.GraphTest
{
    public sealed class GraphSaver
    {
        public string SaveToJson(GraphDefinition graph)
        {
            if (graph == null)
            {
                throw new ArgumentNullException(nameof(graph));
            }

            var dto = new GraphDefinitionDto
            {
                Version = graph.Version <= 0 ? 1 : graph.Version
            };

            foreach (var node in graph.Nodes.Values)
            {
                dto.Nodes.Add(new GraphNodeDto
                {
                    Id = node.Id,
                    Type = node.Type,
                    Name = node.Name,
                    ConstValue = node.ConstValue,
                    Op = node.Op,
                    Func = node.Func,
                    Args = new List<string>(node.Args),
                    ArgNegate = new List<bool>(node.ArgNegate),
                    Src = node.Src,
                    Path = node.Path,
                    Inputs = new Dictionary<string, string>(node.InputMap),
                    Outputs = new Dictionary<string, string>(node.OutputMap),
                    Inline = node.InlineGraph != null ? BuildDtoFromDefinition(node.InlineGraph) : null
                });
            }

            return JsonConvert.SerializeObject(dto, Formatting.Indented, new JsonSerializerSettings
            {
                Converters = { new StringEnumConverter() }
            });
        }

        internal GraphDefinitionDto BuildDtoFromDefinition(GraphDefinition graph)
        {
            var dto = new GraphDefinitionDto
            {
                Version = graph.Version <= 0 ? 1 : graph.Version
            };

            foreach (var node in graph.Nodes.Values)
            {
                dto.Nodes.Add(new GraphNodeDto
                {
                    Id = node.Id,
                    Type = node.Type,
                    Name = node.Name,
                    ConstValue = node.ConstValue,
                    Op = node.Op,
                    Func = node.Func,
                    Args = new List<string>(node.Args),
                    ArgNegate = new List<bool>(node.ArgNegate),
                    Src = node.Src,
                    Path = node.Path,
                    Inputs = new Dictionary<string, string>(node.InputMap),
                    Outputs = new Dictionary<string, string>(node.OutputMap),
                    Inline = node.InlineGraph != null ? BuildDtoFromDefinition(node.InlineGraph) : null
                });
            }

            return dto;
        }
    }
}
