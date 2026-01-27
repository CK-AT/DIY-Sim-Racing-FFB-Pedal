# Graph Editor Copy/Paste Implementation Plan

## Status: TO BE FIXED

Implementation completed January 2026.

## Overview

Add clipboard copy/paste functionality to the graph editor, enabling users to:
- Copy selected nodes and their interconnecting links
- Paste copied content into the same or different graph
- Support standard keyboard shortcuts (Ctrl+C, Ctrl+V)
- Preserve parameter settings when copying Param nodes
- Handle Include node paths correctly across different graph directories

## Implementation Summary

### Files Modified/Created

| File | Change Type | Description |
|------|-------------|-------------|
| `GraphEditor/GraphClipboard.cs` | **New** | Clipboard data class with Nodes, Links, Params, CenterX/Y |
| `GraphEditor/GraphSerializer.cs` | Modified | Added `GraphClipboardSerializer` class + `GraphClipboardDto` |
| `GraphEditor/GraphEditorControl.xaml.cs` | Modified | Copy/paste/cut logic, keyboard handlers, context menu |
| `DiyFfbPlugin.csproj` | Modified | Added GraphClipboard.cs to compile includes |

### Key Features

1. **Ctrl+C** - Copy selected nodes with internal links
2. **Ctrl+V** - Paste at mouse position (if over canvas) or with 50px offset
3. **Ctrl+X** - Cut (copy then delete)
4. **Context Menu** - Copy/Paste/Cut/Delete with keyboard shortcuts shown
5. **Param Copying** - GraphParam definitions are copied with Param nodes
6. **Include Path Handling** - Paths converted to absolute on copy, relative on paste

## Clipboard Data Structure

```csharp
public sealed class GraphClipboardData
{
    public const string ClipboardFormat = "DiyFfbGraphClipboard";

    public List<GraphNode> Nodes { get; set; }
    public List<GraphLink> Links { get; set; }
    public Dictionary<string, GraphParam> Params { get; set; }  // Parameter definitions
    public double CenterX { get; set; }
    public double CenterY { get; set; }
}
```

## Special Node Handling

### Param Nodes

When copying Param nodes:
- The `GraphParam` definition is cloned and included in clipboard
- On paste, params are merged into target graph (won't overwrite existing)
- Preserves: Name, DefaultValue, Min, Max, UI settings (Widget, Label, Group, Units, Step, Precision, LogScale, Options)

```csharp
// Copy: collect params for selected Param nodes
foreach (var nv in _selectedNodes)
{
    if (nv.Node.Kind == GraphNodeKind.Param)
    {
        foreach (var port in nv.Node.Ports.Where(p => p.Kind == GraphPortKind.Output))
        {
            string paramName = GetPortSignalName(nv.Node, port);
            if (_graph.Params.TryGetValue(paramName, out var param))
                copiedParams[paramName] = CloneParam(param);
        }
    }
}

// Paste: merge params (don't overwrite existing)
foreach (var kvp in clipboardData.Params)
{
    if (!_graph.Params.ContainsKey(kvp.Key))
        _graph.Params[kvp.Key] = kvp.Value;
}
```

### Include Nodes

When copying Include nodes:
- **On Copy**: Relative paths converted to absolute using source graph's `BaseDirectory`
- **On Paste**: Absolute paths converted back to relative using target graph's `BaseDirectory`

This ensures Include nodes work correctly when:
- Pasting into the same graph
- Pasting into a different graph in the same directory
- Pasting into a different graph in a different directory

```csharp
// Copy: convert to absolute
foreach (var node in copiedNodes)
{
    if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
        node.IncludePath = ResolveToAbsolutePath(node.IncludePath);
}

// Paste: convert back to relative
foreach (var node in clipboardData.Nodes)
{
    if (node.Kind == GraphNodeKind.Include && !string.IsNullOrWhiteSpace(node.IncludePath))
        node.IncludePath = MakeRelativePath(_baseDirectory, node.IncludePath);
}
```

## Testing Checklist

- [x] Copy single node (Ctrl+C)
- [x] Paste single node (Ctrl+V)
- [x] Copy multiple nodes with links
- [x] Paste preserves internal links
- [x] External links are NOT copied
- [x] Cut operation (Ctrl+X)
- [x] Paste at mouse position
- [x] Multiple paste creates offset copies
- [x] Copy from Graph A, paste to Graph B (different tab)
- [x] Paste empty clipboard (no-op)
- [x] Paste invalid data (no-op)
- [x] Copy Include nodes (paths handled correctly)
- [x] Copy Param nodes (settings preserved)
- [ ] Undo after paste (requires undo system)

## Build Commands

### Debug Build
```bash
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Debug /v:minimal /nologo
```

### Release Build
```bash
MSYS_NO_PATHCONV=1 "C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "d:\Projects\DIY-Sim-Racing-FFB-Pedal\SimHubPlugin\DiyFfbPlugin.csproj" /p:Configuration=Release /v:minimal /nologo
```

## Future Enhancements

1. **Undo/Redo Integration** - Add paste as undoable command when undo system exists
2. **Duplicate Command** (Ctrl+D) - Paste in place with offset
3. **Visual Feedback** - Flash/highlight pasted nodes briefly
4. **Paste Special** - Options dialog for paste behavior (position, naming)
