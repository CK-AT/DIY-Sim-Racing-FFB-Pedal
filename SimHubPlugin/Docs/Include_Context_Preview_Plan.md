# Include Context Preview - Implementation Plan

**Status**: COMPLETED (2026-01-25)

See [FFB_Graph_Progress.md](FFB_Graph_Progress.md) for implementation details.

## Overview

Enable sub-graph previews to show live evaluation results in the context of a specific Include node from the active parent graph. When editing a sub-graph that's included by the active graph, users can select which Include node's inputs to use for preview.

## Current State

- `GraphCompiledEvaluator.EvalInclude()` evaluates sub-graphs but doesn't expose the inputs passed to them
- Sub-graph tabs always use standalone preview (manual input sliders)
- No visibility into how a sub-graph behaves with real inputs from a parent

## Target State

- Context dropdown in inspector when editing a graph that's included by the active graph
- Dropdown shows: "(standalone)" + list of Include node titles from active graph
- Selecting a context applies live inputs from that Include node
- Tab label shows "(via IncludeTitle)" suffix when context is active
- Auto-fallback to standalone when active graph changes

---

## Data Structures

### 1. IncludeCallContext (new class in GraphTest namespace)

```csharp
namespace DiyFfb.GraphTest
{
    public sealed class IncludeCallContext
    {
        public string IncludeNodeId { get; set; }
        public string IncludeNodeTitle { get; set; }
        public string IncludePath { get; set; }  // Resolved absolute path
        public IReadOnlyDictionary<string, double> Inputs { get; set; }
        public IReadOnlyDictionary<string, double> Parameters { get; set; }
    }
}
```

### 2. IncludeContextCache (new class in GraphTest namespace)

```csharp
namespace DiyFfb.GraphTest
{
    public sealed class IncludeContextCache
    {
        // Key: resolved include path (absolute), Value: list of contexts (one per Include node)
        private readonly Dictionary<string, List<IncludeCallContext>> _contexts
            = new Dictionary<string, List<IncludeCallContext>>(StringComparer.OrdinalIgnoreCase);

        public void Clear();
        public void Add(string resolvedPath, IncludeCallContext context);
        public IReadOnlyList<IncludeCallContext> GetContexts(string resolvedPath);
        public bool HasContexts(string resolvedPath);
    }
}
```

---

## Implementation Steps

### Step 1: Add IncludeContextCache to GraphCompiledEvaluator

**File:** `SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs`

1. Add optional `IncludeContextCache` parameter to constructor
2. In `EvalInclude()`, after building `subInputs`, capture the context:

```csharp
private void EvalInclude(CompiledNode node, IReadOnlyDictionary<string, double> inputs,
    IReadOnlyDictionary<string, double> parameters)
{
    // ... existing code to resolve subGraph and key ...

    var subInputs = new Dictionary<string, double>();
    foreach (var mapping in node.Node.InputMap)
    {
        string inputName = shortToFullName.TryGetValue(mapping.Key, out var fullName) ? fullName : mapping.Key;
        subInputs[inputName] = ResolveById(mapping.Value);
    }

    // NEW: Capture context for preview
    if (_contextCache != null && !string.IsNullOrEmpty(key))
    {
        string resolvedPath = ResolveToAbsolutePath(key);
        _contextCache.Add(resolvedPath, new IncludeCallContext
        {
            IncludeNodeId = node.Node.Id,
            IncludeNodeTitle = node.Node.Name ?? node.Node.Id,
            IncludePath = resolvedPath,
            Inputs = new Dictionary<string, double>(subInputs),
            Parameters = new Dictionary<string, double>(parameters ?? new Dictionary<string, double>())
        });
    }

    var outputs = evaluator.Evaluate(subInputs, parameters);
    // ... rest of existing code ...
}
```

3. Add `ClearContextCache()` method called at start of `EvaluateWithTrace()`

**Important:** The cache must be cleared at the START of each evaluation cycle to avoid stale entries.

### Step 2: Wire Cache Through Plugin

**File:** `SimHubPlugin/DiyFfbPlugin.cs`

1. Add field: `private IncludeContextCache activeIncludeContextCache;`
2. Create cache when creating evaluator:

```csharp
activeIncludeContextCache = new IncludeContextCache();
activeGraphEvaluator = new GraphCompiledEvaluator(activeGraphRuntime, activeGraphResolver, activeIncludeContextCache);
```

3. Expose cache via property for UI access:

```csharp
public IncludeContextCache ActiveIncludeContextCache => activeIncludeContextCache;
```

4. Clear cache when active graph changes (in `ReloadActiveGraph()`):

```csharp
activeIncludeContextCache?.Clear();
```

### Step 3: Add Context Selection to GraphEditorControl

**File:** `SimHubPlugin/GraphEditor/GraphEditorControl.xaml`

Add to inspector panel (near top, visible when graph is loaded):

```xml
<StackPanel x:Name="PanelEvalContext" Orientation="Horizontal" Margin="0,4" Visibility="Collapsed">
    <TextBlock Text="Context:" VerticalAlignment="Center" Margin="0,0,8,0"/>
    <ComboBox x:Name="ComboEvalContext" Width="200" SelectionChanged="ComboEvalContext_SelectionChanged"/>
</StackPanel>
```

**File:** `SimHubPlugin/GraphEditor/GraphEditorControl.xaml.cs`

1. Add fields:

```csharp
private string _selectedContextId;  // null = standalone, else IncludeNodeId
private Func<string, IReadOnlyList<IncludeCallContext>> _contextProvider;
```

2. Add property for plugin to set:

```csharp
public Func<string, IReadOnlyList<IncludeCallContext>> ContextProvider
{
    get => _contextProvider;
    set
    {
        _contextProvider = value;
        RefreshContextDropdown();
    }
}
```

3. Add methods:

```csharp
private void RefreshContextDropdown()
{
    ComboEvalContext.Items.Clear();
    ComboEvalContext.Items.Add(new ComboBoxItem { Content = "(standalone)", Tag = null });

    if (_contextProvider == null || string.IsNullOrEmpty(_loadedFilePath))
    {
        PanelEvalContext.Visibility = Visibility.Collapsed;
        return;
    }

    var contexts = _contextProvider(_loadedFilePath);
    if (contexts == null || contexts.Count == 0)
    {
        PanelEvalContext.Visibility = Visibility.Collapsed;
        return;
    }

    foreach (var ctx in contexts)
    {
        ComboEvalContext.Items.Add(new ComboBoxItem
        {
            Content = ctx.IncludeNodeTitle,
            Tag = ctx.IncludeNodeId
        });
    }

    PanelEvalContext.Visibility = Visibility.Visible;

    // Restore selection or default to standalone
    var selected = ComboEvalContext.Items.Cast<ComboBoxItem>()
        .FirstOrDefault(i => (string)i.Tag == _selectedContextId);
    ComboEvalContext.SelectedItem = selected ?? ComboEvalContext.Items[0];
}

private void ComboEvalContext_SelectionChanged(object sender, SelectionChangedEventArgs e)
{
    if (ComboEvalContext.SelectedItem is ComboBoxItem item)
    {
        _selectedContextId = item.Tag as string;
        RefreshPreview();
        ContextChanged?.Invoke(this, _selectedContextId);
    }
}
```

4. Modify `RefreshPreview()` to use context inputs when selected:

```csharp
private void RefreshPreview()
{
    try
    {
        Dictionary<string, double> inputs;
        Dictionary<string, double> parameters;

        if (_selectedContextId != null && _contextProvider != null)
        {
            var contexts = _contextProvider(_loadedFilePath);
            var ctx = contexts?.FirstOrDefault(c => c.IncludeNodeId == _selectedContextId);
            if (ctx != null)
            {
                inputs = new Dictionary<string, double>(ctx.Inputs);
                parameters = new Dictionary<string, double>(ctx.Parameters);
            }
            else
            {
                // Context no longer available, fall back to standalone
                _selectedContextId = null;
                RefreshContextDropdown();
                return;
            }
        }
        else
        {
            // Standalone mode - use manual entries
            inputs = new Dictionary<string, double>();
            foreach (var entry in _previewInputEntries)
            {
                inputs[entry.Name] = entry.Value;
            }
            parameters = new Dictionary<string, double>();
            foreach (var entry in _previewParamEntries)
            {
                parameters[entry.Name] = entry.Value;
            }
        }

        var result = _previewEvaluator.Evaluate(_graph, inputs, parameters);
        UpdateNodeValues(result.NodeValues);
        TextPreviewStatus.Text = "";
    }
    catch (Exception ex)
    {
        UpdateNodeValues(null);
        TextPreviewStatus.Text = $"Preview error: {ex.Message}";
    }
}
```

5. Add event for tab manager to update label:

```csharp
public event EventHandler<string> ContextChanged;  // string = contextId or null
```

### Step 4: Update Tab Label in GraphEditorTabManager

**File:** `SimHubPlugin/GraphEditor/GraphEditorTabManager.cs`

1. Subscribe to `ContextChanged` event when creating editor:

```csharp
editor.ContextChanged += (s, contextId) => UpdateTabLabel(tab, editor, contextId);
```

2. Add method:

```csharp
private void UpdateTabLabel(TabItem tab, GraphEditorControl editor, string contextId)
{
    string baseName = Path.GetFileName(editor.LoadedFilePath ?? "Untitled");
    if (contextId != null)
    {
        var contexts = _plugin.ActiveIncludeContextCache?.GetContexts(editor.LoadedFilePath);
        var ctx = contexts?.FirstOrDefault(c => c.IncludeNodeId == contextId);
        if (ctx != null)
        {
            tab.Header = $"{baseName} (via {ctx.IncludeNodeTitle})";
            return;
        }
    }
    tab.Header = baseName;
}
```

### Step 5: Wire Plugin to Editor

**File:** `SimHubPlugin/DiyFfbPlugin.cs` or wherever editor tabs are created

When creating/configuring GraphEditorControl:

```csharp
editor.ContextProvider = path => ActiveIncludeContextCache?.GetContexts(path);
```

### Step 6: Refresh Contexts on Evaluation Cycle

The preview needs to refresh when the active graph evaluates (to get updated context values).

**Option A:** Timer-based refresh (simpler)
- Preview already has a timer for live inputs
- Extend to also call `RefreshContextDropdown()` and `RefreshPreview()`

**Option B:** Event-based refresh (cleaner)
- Plugin raises event after each evaluation cycle
- Editor subscribes and refreshes

Recommend **Option A** for simplicity - reuse existing `_liveInputsTimer`.

---

## Test Cases

### Unit Tests (GraphTestRunner.cs)

#### Test 1: IncludeContextCache Basic Operations
```csharp
[Test]
public void IncludeContextCache_AddAndRetrieve()
{
    var cache = new IncludeContextCache();
    var ctx = new IncludeCallContext
    {
        IncludeNodeId = "inc1",
        IncludeNodeTitle = "MyInclude",
        IncludePath = "C:/graphs/sub.json",
        Inputs = new Dictionary<string, double> { { "A", 1.0 } },
        Parameters = new Dictionary<string, double>()
    };

    cache.Add("C:/graphs/sub.json", ctx);

    var retrieved = cache.GetContexts("C:/graphs/sub.json");
    Assert.AreEqual(1, retrieved.Count);
    Assert.AreEqual("inc1", retrieved[0].IncludeNodeId);
    Assert.AreEqual(1.0, retrieved[0].Inputs["A"]);
}
```

#### Test 2: IncludeContextCache Case Insensitive Path
```csharp
[Test]
public void IncludeContextCache_PathCaseInsensitive()
{
    var cache = new IncludeContextCache();
    cache.Add("C:/Graphs/Sub.json", new IncludeCallContext { IncludeNodeId = "inc1" });

    var retrieved = cache.GetContexts("c:/graphs/sub.json");
    Assert.AreEqual(1, retrieved.Count);
}
```

#### Test 3: IncludeContextCache Multiple Includes Same Path
```csharp
[Test]
public void IncludeContextCache_MultipleIncludesSamePath()
{
    var cache = new IncludeContextCache();
    cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc1", IncludeNodeTitle = "First" });
    cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc2", IncludeNodeTitle = "Second" });

    var retrieved = cache.GetContexts("sub.json");
    Assert.AreEqual(2, retrieved.Count);
    Assert.IsTrue(retrieved.Any(c => c.IncludeNodeTitle == "First"));
    Assert.IsTrue(retrieved.Any(c => c.IncludeNodeTitle == "Second"));
}
```

#### Test 4: IncludeContextCache Clear
```csharp
[Test]
public void IncludeContextCache_Clear()
{
    var cache = new IncludeContextCache();
    cache.Add("sub.json", new IncludeCallContext { IncludeNodeId = "inc1" });
    cache.Clear();

    var retrieved = cache.GetContexts("sub.json");
    Assert.AreEqual(0, retrieved.Count);
}
```

#### Test 5: Evaluator Populates Cache
```csharp
[Test]
public void GraphCompiledEvaluator_PopulatesIncludeContextCache()
{
    // Create parent graph with Include node
    var parent = new GraphDefinition();
    parent.Nodes["in1"] = new GraphNode { Id = "in1", Type = NodeType.Input, Name = "Speed" };
    parent.Nodes["inc1"] = new GraphNode
    {
        Id = "inc1",
        Type = NodeType.Include,
        Name = "SubGraph",
        Path = "sub.json",
        InputMap = new Dictionary<string, string> { { "X", "in1" } },
        OutputMap = new Dictionary<string, string> { { "Y", "inc1:Y" } }
    };

    // Create sub-graph
    var sub = new GraphDefinition();
    sub.Nodes["x"] = new GraphNode { Id = "x", Type = NodeType.Input, Name = "X" };
    sub.Nodes["y"] = new GraphNode { Id = "y", Type = NodeType.Output, Name = "Y", Src = "x" };

    var resolver = new MockResolver();
    resolver.Register("sub.json", sub);

    var cache = new IncludeContextCache();
    var evaluator = new GraphCompiledEvaluator(parent, resolver, cache);

    var inputs = new Dictionary<string, double> { { "Speed", 42.0 } };
    evaluator.Evaluate(inputs, null);

    var contexts = cache.GetContexts("sub.json");
    Assert.AreEqual(1, contexts.Count);
    Assert.AreEqual("inc1", contexts[0].IncludeNodeId);
    Assert.AreEqual(42.0, contexts[0].Inputs["X"]);
}
```

#### Test 6: Cache Cleared Each Evaluation
```csharp
[Test]
public void GraphCompiledEvaluator_ClearsCacheEachEvaluation()
{
    // Setup similar to Test 5...

    var cache = new IncludeContextCache();
    var evaluator = new GraphCompiledEvaluator(parent, resolver, cache);

    // First evaluation
    evaluator.Evaluate(new Dictionary<string, double> { { "Speed", 10.0 } }, null);
    Assert.AreEqual(10.0, cache.GetContexts("sub.json")[0].Inputs["X"]);

    // Second evaluation - cache should have new values, not accumulated
    evaluator.Evaluate(new Dictionary<string, double> { { "Speed", 20.0 } }, null);
    var contexts = cache.GetContexts("sub.json");
    Assert.AreEqual(1, contexts.Count);  // Still 1, not 2
    Assert.AreEqual(20.0, contexts[0].Inputs["X"]);
}
```

### Integration Tests (Manual)

#### Test 7: Context Dropdown Appears
1. Load a parent graph with Include node pointing to "sub.json"
2. Set as active graph
3. Open "sub.json" in a tab
4. **Verify:** Context dropdown is visible with "(standalone)" and Include node title

#### Test 8: Context Selection Updates Preview
1. Same setup as Test 7
2. Set parent input "Speed" to 100 via live inputs
3. Select Include context in sub-graph tab
4. **Verify:** Sub-graph preview shows values based on Speed=100

#### Test 9: Live Updates
1. Same setup as Test 8
2. Change parent input "Speed" to 200
3. **Verify:** Sub-graph preview automatically updates

#### Test 10: Fallback to Standalone
1. Same setup as Test 8
2. Load a different active graph that doesn't include "sub.json"
3. **Verify:** Context dropdown hides or shows only "(standalone)"
4. **Verify:** Preview continues working in standalone mode

#### Test 11: Tab Label Updates
1. Same setup as Test 7
2. Select Include context
3. **Verify:** Tab label changes to "sub.json (via IncludeTitle)"
4. Select "(standalone)"
5. **Verify:** Tab label changes back to "sub.json"

---

## Potential Side Effects & Mitigations

### 1. Performance Impact
**Risk:** Capturing context on every Include evaluation adds overhead to the hot path (120Hz+ evaluation).

**Mitigation:**
- Only allocate new dictionaries if cache is non-null
- Use object pooling for IncludeCallContext if profiling shows GC pressure
- Consider making cache opt-in (only enabled when editor is open)

### 2. Memory Growth
**Risk:** Cache could grow unbounded if paths are not normalized consistently.

**Mitigation:**
- Use case-insensitive path comparison (already in design)
- Normalize paths to absolute before storing
- Clear cache at start of each evaluation cycle (already in design)

### 3. Thread Safety
**Risk:** Evaluator runs on game thread, UI runs on dispatcher thread.

**Mitigation:**
- Cache is written only during evaluation (single thread)
- UI reads cache on timer tick (may see partial state)
- Use `lock` or `ConcurrentDictionary` if issues arise
- Alternative: Copy cache snapshot to UI thread on timer tick

### 4. Stale Context Selection
**Risk:** User selects a context, then parent graph is edited to remove that Include node.

**Mitigation:**
- Already handled: `RefreshPreview()` checks if context still exists, falls back to standalone
- `RefreshContextDropdown()` rebuilds list from current cache state

### 5. Path Resolution Inconsistency
**Risk:** Include path "sub.json" vs ".\sub.json" vs "C:\full\path\sub.json" treated as different.

**Mitigation:**
- Normalize all paths to absolute in `EvalInclude()` before adding to cache
- Use `Path.GetFullPath()` for normalization
- Store both resolved path and original path in context if needed for display

### 6. Nested Includes
**Risk:** Graph A includes B, B includes C. When viewing C, should we show context from A→B→C chain?

**Mitigation:**
- Current design only shows direct parent contexts (B→C)
- This is intentional - keeps UI simple
- Future enhancement could add "call stack" view if needed

### 7. Circular Includes
**Risk:** Graph A includes B, B includes A (invalid but possible).

**Mitigation:**
- Existing evaluator already has cycle detection (throws on cycle)
- Cache population happens after cycle check, so won't cause issues

### 8. UI Flicker
**Risk:** Context dropdown rebuilds on every timer tick, causing selection to reset.

**Mitigation:**
- Preserve `_selectedContextId` across rebuilds
- Only rebuild dropdown if context list actually changed
- Compare context IDs before rebuilding

---

## File Checklist

| File | Changes |
|------|---------|
| `GraphTest/IncludeCallContext.cs` | NEW - Data class |
| `GraphTest/IncludeContextCache.cs` | NEW - Cache class |
| `GraphTest/GraphCompiledEvaluator.cs` | Add cache parameter, populate in EvalInclude |
| `GraphEditor/GraphRuntimeConverter.cs` | Update CreateResolver to accept optional cache |
| `GraphEditor/GraphEditorControl.xaml` | Add context dropdown UI |
| `GraphEditor/GraphEditorControl.xaml.cs` | Add context selection logic, modify RefreshPreview |
| `GraphEditor/GraphEditorTabManager.cs` | Update tab labels on context change |
| `DiyFfbPlugin.cs` | Create cache, wire to evaluator and UI |
| `GraphTest/GraphTestRunner.cs` | Add unit tests |

---

## Implementation Order

1. **IncludeCallContext.cs** - Simple data class, no dependencies
2. **IncludeContextCache.cs** - Simple cache, no dependencies
3. **Unit tests for cache** - Validate cache behavior
4. **GraphCompiledEvaluator.cs** - Add cache population
5. **Unit tests for evaluator** - Validate cache is populated
6. **GraphRuntimeConverter.cs** - Update factory (optional cache param)
7. **DiyFfbPlugin.cs** - Create and wire cache
8. **GraphEditorControl.xaml/.cs** - Add UI and logic
9. **GraphEditorTabManager.cs** - Tab label updates
10. **Integration testing** - Manual tests 7-11

---

## Open Questions for Implementer

1. Should the cache be completely opt-in (null by default) or always-on?
   - Recommendation: Always-on when evaluator is created with cache, overhead is minimal

2. Should we show context even when "Live Inputs" checkbox is unchecked?
   - Recommendation: Yes, context dropdown is independent of live inputs toggle

3. What happens to manual preview input values when switching to/from context?
   - Recommendation: Preserve manual values, just don't use them while context is active
