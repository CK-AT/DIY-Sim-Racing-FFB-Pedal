# Embedded Sub-Graphs

Today an Include node references an external `_embedded/*.json` file by
`IncludePath`. This adds a second form: an **embedded** sub-graph whose
definition lives inline in the parent graph (no path), edited in its
own editor tab. Reuse across templates stays file-based; embedding is
for one-off, template-specific clusters (the TR-gate, mu-buzz ramp,
ground-cues if not shared) that otherwise clutter the top level or get
dumped into `msfs_derivations`.

**Status:** implemented (L1–L3) and verified in the editor.
**Decided scope (user):** pathless creation + extract-to-file +
inline-a-file-include (promote/demote both directions).

**Post-implementation fixes (from editor testing):**

- Saving while on an embedded tab no longer prompts for a filename — it
  flushes the edit chain up and saves the root file tab.
- Closing an embedded tab no longer prompts to save (content already
  lives in the parent).
- Embedded Include nodes show their in/out ports on parent load
  (`SyncIncludePorts` derives from the inline graph, not just a file
  path); editing the sub-graph interface re-syncs the parent node's
  ports live.

**Done:**

- L1 data layer — editor `GraphNode.InlineGraph`, serializer nested
  `Inline` (de)serialize + ports-from-inline, recursive converter,
  resolver no longer spills inline to disk, validator recurses into
  inline blocks. Verified: 171 GraphTest tests pass (added an
  editor-format embedded round-trip test), 26 graphs validate, a
  negative inner-port test is caught.
- L2 tab system — embedded tabs clone the inline graph in, flush back
  into the parent node on edit (dirtying the parent), key on
  `(parentTab, nodeId)`, title-chain `file/outer/inner`, and close
  with their parent. Double-click on a path-less Include opens it via
  the new `EmbeddedOpenRequested` event.
- L3 UI — "Add Embedded Sub-Graph" context item; "Extract Embedded
  Sub-Graph to File…" and "Inline This Include (detach from file)"
  appear when a single Include node is selected.

**Note:** `TestBlockLibraryIndex` was retargeted to the file-include
path — inline no longer registers in the block library, since embedded
blocks are private to their parent by design.

---

## What already exists

The **runtime** layer supports inline sub-graphs end-to-end:

- `GraphNode.InlineGraph` ([GraphEvaluator.cs:55](../../SimHubPlugin/GraphTest/GraphEvaluator.cs#L55)),
  runtime DTO `Inline` ([GraphLoader.cs:453](../../SimHubPlugin/GraphTest/GraphLoader.cs#L453)),
  load/save round-trip ([GraphSaver.cs:38](../../SimHubPlugin/GraphTest/GraphSaver.cs#L38)).
- Compiled (production) evaluator evaluates inline with `"inline:"+nodeId`
  caching and **no disk spill** ([GraphCompiledEvaluator.cs:800](../../SimHubPlugin/GraphTest/GraphCompiledEvaluator.cs#L800)).
- Port names derived from `InlineGraph` ([GraphLoader.cs:237](../../SimHubPlugin/GraphTest/GraphLoader.cs#L237)).

The gap is entirely the **editor** layer (format, converter, UI) plus a
resolver cleanup.

---

## Layers (each independently verifiable)

### L1 — Data: format + model + converter + resolver + validator

- **Editor node model** ([GraphModel.cs:62](../../SimHubPlugin/GraphEditor/GraphModel.cs#L62)):
  add `GraphDefinition InlineGraph { get; set; }` to the editor
  `GraphNode`, mutually exclusive with `IncludePath`. An Include node is
  *embedded* iff `InlineGraph != null` (and `IncludePath` empty).
- **Editor serializer** ([GraphSerializer.cs](../../SimHubPlugin/GraphEditor/GraphSerializer.cs)):
  add a nested `inline` field to the Include node DTO; recursively
  (de)serialize. `PopulateIncludePorts` resolves ports from `InlineGraph`
  when present instead of reading a file.
- **Converter** ([GraphRuntimeConverter.cs:88](../../SimHubPlugin/GraphEditor/GraphRuntimeConverter.cs#L88)):
  when the editor node has `InlineGraph`, recursively `Convert()` it and
  set the runtime node's `InlineGraph` instead of `Path`.
- **Resolver** ([GraphIncludeResolver.cs:113](../../SimHubPlugin/GraphTest/GraphIncludeResolver.cs#L113)):
  drop the materialize-to-`{hash}.json` fallback — keep inline in memory
  like the compiled evaluator. Also stops the stray
  `_embedded/{hash}.json` + nested `index.json` generation seen earlier.
- **Validator** (`validate_graphs.py`): for a path-less Include with an
  inline block, resolve ports from that block recursively rather than
  from a file.

Verify: build; round-trip a graph with an embedded sub-graph
(editor→runtime→evaluate) in GraphTestRunner; `validate_graphs.py`
passes on a hand-authored embedded sample.

### L2 — Tab system

- **`GraphEditorTab`** ([GraphEditorTab.cs](../../SimHubPlugin/GraphEditor/GraphEditorTab.cs)):
  today keyed/saved by `FilePath`. Add an embedded mode keyed by
  `(parentTab, nodeId)`: loads from the node's `InlineGraph`, **saves
  back into that node's `InlineGraph` and marks the parent tab dirty**
  (no file of its own — persisting the parent file persists it). Title =
  `<parentTitle>/<nodeTitleOrId>`; nested embeds chain
  (`parent/child/grandchild`).
- **`GraphEditorTabManager`**: add `OpenEmbedded(parentTab, node)`
  alongside `OpenGraph(path)`; dedupe by `(parentTabId,nodeId)`.
- **Window** ([GraphEditorWindow.xaml.cs:947](../../SimHubPlugin/GraphEditor/GraphEditorWindow.xaml.cs#L947)):
  `OnIncludeOpenRequested` branches — path-less Include with inline →
  `OpenEmbedded`; else existing file path. Hierarchy tree shows embedded
  children under the parent by node name.

Verify: build; user opens an embedded sub-graph, edits, sees parent go
dirty, saves parent, reloads — change persists in the parent file.

### L3 — UI: create / extract / inline

- **Create**: adding an Include node with empty path creates an empty
  `InlineGraph`; double-click opens its (blank) tab.
- **Extract to file**: context-menu command on an embedded Include →
  write `InlineGraph` to a chosen `_embedded/*.json`, set `IncludePath`,
  clear `InlineGraph`.
- **Inline a file include**: inverse — read the file into `InlineGraph`,
  clear `IncludePath`. (Warn if the file is referenced by other graphs —
  inlining doesn't remove the shared file, just detaches this node.)

Verify: build; round-trip both conversions in the editor; extracted
file validates; inlined node still evaluates identically.

---

## Key decisions

- **Save-back, not separate file.** An embedded tab's saves mutate the
  parent node's `InlineGraph` and dirty the parent; there is no
  standalone file. This is why an embedded tab can't reuse the
  file-save path and needs its own track.
- **Identity without a path.** Embedded tabs key on
  `(parentTabId, nodeId)`, not a file path, for dedupe and titling.
- **Nesting allowed.** Embedded-within-embedded is just recursion in
  format/converter/evaluator; tab titles chain.
- **Reuse stays file-based.** Embedding deliberately has no reuse; if a
  cluster needs to appear in two templates, use a file include (or
  extract-to-file).
- **CRLF / no-trailing-newline** preserved per existing tooling.

## Risks

- Tab system is the deepest change; the save-back-to-parent + dirty
  propagation is the main correctness risk (L2 verify covers it).
- Headless limits: data layer (L1) and converters are build/test
  verifiable; tab + UI (L2/L3) need interactive testing by the user.
