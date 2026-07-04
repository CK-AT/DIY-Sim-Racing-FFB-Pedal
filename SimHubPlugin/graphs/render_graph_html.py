#!/usr/bin/env python3
"""Render an FFB graph JSON to a self-contained, shareable HTML file.

The output looks like the in-plugin graph editor (dark canvas, coloured node
title bars, blue/orange ports, grey bezier links) and needs no SimHub to view.
Pan with drag, zoom with the mouse wheel, "Fit" recenters.

Include nodes are drawn collapsed exactly as the editor shows them: their ports
are derived from the included sub-graph's Input/Output nodes. Every referenced
sub-graph is embedded recursively, so **double-clicking an Include node enters
it** (with a clickable breadcrumb to navigate back) -- all offline, one file.

Param nodes show their current default value. Include-port derivation and the
library-vs-top-level label rules mirror GraphEditorControl.xaml.cs.

Usage:
    python render_graph_html.py <graph.json> [-o out.html]
    python render_graph_html.py <dir>                # renders every *.json
    python render_graph_html.py <graph.json> --classification CONFIDENTIAL

Exit code 1 on a hard failure (missing input / parse error).
"""
import argparse
import json
import os
import sys

# ---------------------------------------------------------------------------
# Editor geometry + palette constants (mirror GraphEditorControl.xaml.cs).
# Kept in sync there: NodeFill 45,45,45 / TitleBar* / PortRowSpacing 25 / etc.
# ---------------------------------------------------------------------------
TITLE_BAR_COLORS = {
    "Input":        "rgb(60,120,180)",   # blue
    "Output":       "rgb(200,120,40)",   # orange
    "ConfigOut":    "rgb(200,120,40)",
    "ConfigIn":     "rgb(60,120,180)",
    "Param":        "rgb(140,80,180)",   # purple
    "Const":        "rgb(100,100,100)",  # gray
    "Op":           "rgb(80,150,80)",    # green
    "Func":         "rgb(60,140,160)",   # teal
    "Expr":         "rgb(60,140,160)",
    "Include":      "rgb(180,80,140)",   # magenta
    "LocalSend":    "rgb(190,110,60)",   # burnt-orange
    "LocalReceive": "rgb(70,170,130)",   # mint-green
}


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def port_id(port):
    """Identity used by links: Name first, else SignalSuffix (matches editor)."""
    return port.get("Name") or port.get("SignalSuffix") or ""


def inline_graph(node):
    """Embedded sub-graph dict for an Include node, or None."""
    return node.get("InlineGraph") or node.get("Inline")


def graph_io_ports(g):
    """Ordered (inputs, outputs) port-id lists a graph exposes via its
    Input nodes (output ports) and unscoped Output nodes (input ports)."""
    ins, outs = [], []
    for n in g.get("Nodes", []):
        kind = n.get("Kind")
        if kind == "Input":
            for p in n.get("Ports", []):
                if p.get("Kind") == "Output":
                    pid = port_id(p)
                    if pid and pid not in ins:
                        ins.append(pid)
        elif kind == "Output" and not n.get("Scoped"):
            for p in n.get("Ports", []):
                if p.get("Kind") == "Input":
                    pid = port_id(p)
                    if pid and pid not in outs:
                        outs.append(pid)
    return ins, outs


def reorder(names, order):
    """Apply an Include node's InputPortOrder/OutputPortOrder: listed names
    first (in that order), unknown/new names appended in derived order."""
    if not order:
        return names
    known = [n for n in order if n in names]
    rest = [n for n in names if n not in known]
    return known + rest


def display_label(node, port, is_library):
    """The on-canvas label for a port (mirrors GetPortDisplayLabel)."""
    kind = node.get("Kind")
    bus = port.get("BusName") or ""
    if kind == "LocalSend":
        return "▸ " + (bus or "(unnamed)")
    if kind == "LocalReceive":
        return (bus or "(unnamed)") + " ▸"
    name = port.get("Name") or ""
    suffix = port.get("SignalSuffix") or ""
    # ConfigOut/ConfigIn always use Name; in a library graph Input nodes and
    # unscoped Output nodes use their freeform Name too.
    if kind in ("ConfigOut", "ConfigIn") or \
       (is_library and (kind == "Input" or (kind == "Output" and not node.get("Scoped")))):
        label = name or suffix
    elif kind in ("Input", "Output") and suffix:
        label = suffix
    else:
        label = name or suffix
    if kind == "Op" and port.get("Kind") == "Input" and port.get("Negate"):
        label = "-" + label
    return label


def node_title(node, is_library):
    """Title-bar text (mirrors BuildNodeTitle)."""
    kind = node.get("Kind")
    group = node.get("SignalGroup") or ""
    uses_binding = kind in ("Input", "Output", "Param") and (not is_library or kind == "Param")
    if uses_binding and group:
        return group
    title = node.get("Title") or kind
    if kind == "Op" and node.get("Op"):
        return "%s (%s)" % (title, node["Op"])
    if kind == "Func" and node.get("Func"):
        return "%s (%s)" % (title, node["Func"])
    if kind == "Expr" and node.get("Expr"):
        return "%s (= %s)" % (title, node["Expr"])
    if kind == "Const" and not node.get("Title"):
        return _fmt_num(node.get("ConstValue", 0.0))
    return title


def _fmt_num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    if f == int(f):
        return str(int(f))
    return ("%.3f" % f).rstrip("0").rstrip(".")


def _fmt_param(value, ui):
    """Format a param value for display: enum -> option label, otherwise the
    number at the param's UI precision, plus units."""
    ui = ui or {}
    if (ui.get("Widget") or "").lower() == "enum":
        for opt in ui.get("Options", []):
            try:
                if abs(float(opt.get("Value")) - float(value)) < 1e-6:
                    return opt.get("Label") or _fmt_num(value)
            except (TypeError, ValueError):
                continue
    prec = ui.get("Precision")
    try:
        txt = ("%.*f" % (int(prec), float(value))) if isinstance(prec, int) else _fmt_num(value)
    except (TypeError, ValueError):
        txt = _fmt_num(value)
    units = ui.get("Units") or ""
    return ("%s %s" % (txt, units)).strip()


def param_display(graph, group, suffix, overrides):
    """What to show on a Param node's value box. Returns None when there is no
    matching param definition AND no override. Otherwise a dict with the
    effective value (override if present, else default), whether it is an
    override, and the default text (for the tooltip)."""
    key = "%s.%s" % (group, suffix) if group else suffix
    pdef = None
    for p in graph.get("Params", []):
        if p.get("Name") == key:
            pdef = p
            break
    overridden = key in (overrides or {})
    if pdef is None and not overridden:
        return None
    ui = (pdef or {}).get("Ui") or {}
    default = (pdef or {}).get("DefaultValue", 0.0)
    effective = overrides[key] if overridden else default
    return {
        "value": _fmt_param(effective, ui),
        "overridden": overridden,
        "default": _fmt_param(default, ui) if pdef is not None else None,
    }


class RegistryBuilder:
    """Builds a flat registry of render-ready graph models, keyed by a stable
    id. File includes dedupe by absolute path; inline includes get a fresh key.
    Cycles are guarded so a graph that (directly or transitively) includes
    itself can't recurse forever."""

    def __init__(self, overrides=None):
        self.registry = {}      # key -> {nodes, links, name, isLibrary}
        self._building = set()  # keys mid-construction (cycle guard)
        self._inline = 0
        self.overrides = overrides or {}   # param-name -> override value

    def add_root(self, graph, base_dir, name):
        key = "root"
        is_lib = bool(graph.get("IsLibraryGraph"))
        self._building.add(key)
        self.registry[key] = self._build_graph(graph, base_dir, name, is_lib)
        self._building.discard(key)
        return key

    def _ensure_child(self, node, base_dir):
        """Resolve an Include node to a registry key, building the sub-graph if
        needed. Returns (key_or_None, error_or_None)."""
        g = inline_graph(node)
        if g is not None:
            self._inline += 1
            key = "inline_%d" % self._inline
            name = node.get("Title") or "(embedded)"
            child_base, child_json = base_dir, g
        else:
            rel = (node.get("IncludePath") or "").replace("\\", os.sep)
            if not rel:
                return None, "include has no path"
            inc = os.path.normpath(os.path.join(base_dir, rel))
            if not os.path.isfile(inc):
                return None, "missing include: %s" % rel
            key = os.path.normcase(os.path.abspath(inc))
            if key in self.registry or key in self._building:
                return key, None  # already built / building (shared or cyclic)
            try:
                child_json = load(inc)
            except (json.JSONDecodeError, OSError) as e:
                return None, "bad include %s: %s" % (rel, e)
            child_base = os.path.dirname(inc)
            name = os.path.splitext(os.path.basename(inc))[0]

        is_lib = bool(child_json.get("IsLibraryGraph"))
        self._building.add(key)
        self.registry[key] = self._build_graph(child_json, child_base, name, is_lib)
        self._building.discard(key)
        return key, None

    def _build_node(self, node, graph, base_dir, is_library):
        kind = node.get("Kind")
        error = None
        child_key = None
        if kind == "Include":
            g = inline_graph(node)
            if g is not None:
                in_keys, out_keys = graph_io_ports(g)
            else:
                rel = (node.get("IncludePath") or "").replace("\\", os.sep)
                inc = os.path.normpath(os.path.join(base_dir, rel))
                if os.path.isfile(inc):
                    try:
                        in_keys, out_keys = graph_io_ports(load(inc))
                    except (json.JSONDecodeError, OSError):
                        in_keys, out_keys = [], []
                else:
                    in_keys, out_keys = [], []
            in_keys = reorder(in_keys, node.get("InputPortOrder"))
            out_keys = reorder(out_keys, node.get("OutputPortOrder"))
            inputs = [{"key": k, "label": k} for k in in_keys]
            outputs = [{"key": k, "label": k} for k in out_keys]
            child_key, error = self._ensure_child(node, base_dir)
        else:
            inputs, outputs = [], []
            for p in node.get("Ports", []):
                entry = {"key": port_id(p), "label": display_label(node, p, is_library)}
                if p.get("Kind") == "Input":
                    # Output nodes are signal sinks: prefix each input port with
                    # the group it resolves to (highlighted inline by JS). A
                    # scoped Output has no group of its own — it's qualified by
                    # the scope of the Include it was entered through.
                    if kind == "Output" and (p.get("SignalSuffix") or ""):
                        grp = node.get("SignalGroup") or ""
                        if grp:
                            entry["prefix"] = grp           # static group
                        elif node.get("Scoped"):
                            entry["scoped"] = True          # prefix = active scope (runtime)
                    inputs.append(entry)
                else:
                    if kind == "Param":
                        disp = param_display(graph, node.get("SignalGroup", ""),
                                             p.get("SignalSuffix", ""), self.overrides)
                        if disp is not None:
                            entry["value"] = disp["value"]
                            entry["overridden"] = disp["overridden"]
                            if disp["default"] is not None:
                                entry["default"] = disp["default"]
                    outputs.append(entry)

        return {
            "id": node.get("Id"),
            "kind": kind,
            "title": node_title(node, is_library),
            "x": float(node.get("X", 0.0)),
            "y": float(node.get("Y", 0.0)),
            "inputs": inputs,
            "outputs": outputs,
            "error": error,
            "childKey": child_key,
            "scope": node.get("FunctionScope") or "",   # scoped Include -> where its scoped outputs land
        }

    def _build_graph(self, graph, base_dir, name, is_library):
        nodes = [self._build_node(n, graph, base_dir, is_library)
                 for n in graph.get("Nodes", [])]
        links = [{
            "fromNode": l.get("FromNodeId"), "fromPort": l.get("FromPort"),
            "toNode": l.get("ToNodeId"), "toPort": l.get("ToPort"),
        } for l in graph.get("Links", [])]
        return {"nodes": nodes, "links": links, "name": name, "isLibrary": is_library}


def build_model(graph, base_dir, name, overrides=None, subtitle=""):
    b = RegistryBuilder(overrides)
    root = b.add_root(graph, base_dir, name)
    return {"name": name, "subtitle": subtitle, "rootKey": root,
            "graphs": b.registry, "colors": TITLE_BAR_COLORS}


def resolve_graph_path(graph_path, profile_path):
    """Locate the template a profile references. Tries the stored (possibly
    absolute, Windows-style) path, then the basename next to this script's
    templates/ dir, then next to the profile file."""
    if not graph_path:
        return None
    candidates = [graph_path, graph_path.replace("\\", os.sep)]
    base = os.path.basename(graph_path.replace("\\", os.sep))
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "templates", base))
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(profile_path)), base))
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return None


def load_input(path):
    """Returns (graph_dict, graph_base_dir, name, overrides, subtitle).

    Accepts either a graph file (has top-level "Nodes") or a profile export
    (has a "Profile" with "GraphParamValues" and a "GraphPath" to a template)."""
    data = load(path)
    if "Nodes" in data:  # plain graph
        name = os.path.splitext(os.path.basename(path))[0]
        return data, os.path.dirname(os.path.abspath(path)), name, {}, ""

    prof = data.get("Profile") or data
    overrides = prof.get("GraphParamValues") or {}
    gp = data.get("GraphPath") or prof.get("GraphPath") or ""
    resolved = resolve_graph_path(gp, path)
    if resolved is None:
        raise FileNotFoundError(
            "profile references a graph that could not be found: %r\n"
            "  (looked next to this script's templates/ and beside the profile)" % gp)
    graph = load(resolved)
    name = data.get("ProfileKey") or os.path.splitext(os.path.basename(path))[0]
    subtitle = "profile · %s" % os.path.splitext(os.path.basename(resolved))[0]
    return graph, os.path.dirname(os.path.abspath(resolved)), name, overrides, subtitle


# ---------------------------------------------------------------------------
# HTML template. The JS renderer reproduces the editor geometry exactly:
#   nodeWidth = max(80, max(titleW, maxIn+maxOut+18)[+120 for param] + 8)
#   nodeHeight = 42 + max(1, max(inN,outN)) * 25
#   port row y = 34 + idx*25 + 5 ; input x = nodeX, output x = nodeX+width
#   link bezier: scaled = max(40,|dx|*0.5)*0.5 ; c1=(fx+s,fy) c2=(tx-s,ty)
# ---------------------------------------------------------------------------
HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>__TITLE__</title>
<style>
  html, body { margin: 0; height: 100%; background: #1e1e1e;
    font-family: "Segoe UI", system-ui, sans-serif; color: #ddd; overflow: hidden; }
  #toolbar { position: fixed; top: 0; left: 0; right: 0; min-height: 40px;
    display: flex; align-items: center; gap: 14px; padding: 0 14px;
    background: #2d2d2d; border-bottom: 1px solid #4a4a4a; z-index: 10;
    box-sizing: border-box; }
  #crumbs { display: flex; align-items: center; gap: 4px; font-weight: 600;
    flex-wrap: nowrap; min-width: 0; overflow: hidden; }
  #crumbs .crumb { cursor: pointer; color: #6cb6ff; white-space: nowrap; }
  #crumbs .crumb.last { color: #ddd; cursor: default; }
  #crumbs .sep { color: #777; }
  #subtitle { color: #888; font-size: 11px; min-width: 0; overflow: hidden;
    white-space: nowrap; text-overflow: ellipsis; flex-shrink: 1; }
  #toolbar .spacer { flex: 1; min-width: 0; }
  #nav, #names, #fit, #reset, #leg { flex-shrink: 0; }   /* keep controls visible; shrink crumbs/subtitle instead */
  #names.off, #leg.off { opacity: 0.5; }
  #toolbar button { background: #1e1e1e; color: #ddd; border: 1px solid #4a4a4a;
    border-radius: 4px; padding: 4px 10px; cursor: pointer; font-size: 12px; }
  #toolbar button:hover:not(:disabled) { background: #3a3a3a; }
  #toolbar button:disabled { opacity: 0.35; cursor: default; }
  #nav button { padding: 4px 8px; }
  #legend { position: fixed; left: 12px; bottom: 12px; z-index: 9;
    background: rgba(45,45,45,0.92); border: 1px solid #4a4a4a; border-radius: 6px;
    padding: 8px 11px; display: flex; flex-direction: column; gap: 5px;
    font-size: 11px; color: #bbb; }
  #legend .title { font-weight: 600; color: #ddd; margin-bottom: 2px; }
  #legend span { display: inline-flex; align-items: center; gap: 6px; }
  #legend i { width: 11px; height: 11px; border-radius: 2px; display: inline-block; }
  #legend.empty, body.legend-hidden #legend { display: none; }
  #banner { position: fixed; top: 40px; left: 0; right: 0; text-align: center;
    font-size: 12px; font-weight: 700; letter-spacing: 1px; padding: 3px 0; z-index: 9; }
  #stage { position: absolute; inset: 40px 0 0 0; cursor: grab; touch-action: none; }
  #stage.panning { cursor: grabbing; }
  svg { width: 100%; height: 100%; display: block; }
  .node-title { font-size: 11px; fill: #fff; }
  .port-label { font-size: 10px; fill: #d3d3d3; }
  .param-val { font-size: 10px; fill: #fff; }
  .enterable { cursor: pointer; }
  .link { fill: none; stroke: rgb(120,120,120); stroke-width: 2; }
  .link:hover { stroke: rgb(160,200,255); stroke-width: 2.6; }
  #empty { position: absolute; inset: 0; display: flex; align-items: center;
    justify-content: center; color: #777; }
  #hint { position: fixed; bottom: 10px; right: 14px; font-size: 11px; color: #777; z-index: 9; }
  @media (max-width: 640px) {
    #toolbar { gap: 8px; padding: 0 8px; }
    #toolbar button { padding: 4px 7px; }
    #subtitle, #hint, #names, #reset { display: none; }   /* free room for the breadcrumb */
  }
</style>
</head>
<body>
<div id="toolbar">
  <span id="nav">
    <button id="back" title="Back (Alt+Left / mouse back)">◀</button>
    <button id="fwd" title="Forward (Alt+Right / mouse forward)">▶</button>
  </span>
  <div id="crumbs"></div>
  <span id="subtitle"></span>
  <span class="spacer"></span>
  <button id="names" title="Toggle fully-qualified signal names on output ports">Names</button>
  <button id="fit">Fit</button>
  <button id="reset">100%</button>
  <button id="leg" title="Show / hide the legend">Legend</button>
</div>
<div id="stage">
  <svg id="svg" xmlns="http://www.w3.org/2000/svg">
    <g id="viewport">
      <g id="links"></g>
      <g id="nodes"></g>
    </g>
  </svg>
</div>
<div id="legend"></div>
<div id="hint">drag / 1-finger = pan · wheel / pinch = zoom · double-click an Include to enter</div>
<script>
const MODEL = __MODEL__;
const GRAPHS = MODEL.graphs;
const SVGNS = "http://www.w3.org/2000/svg";

// --- text measurement matching Segoe UI on a 2D canvas -------------------
const measureCanvas = document.createElement("canvas").getContext("2d");
const measCache = new Map();
function measure(text, px) {
  if (!text) return 0;
  const key = px + "|" + text;
  let w = measCache.get(key);
  if (w === undefined) {
    measureCanvas.font = px + 'px "Segoe UI", system-ui, sans-serif';
    w = measureCanvas.measureText(text).width;
    measCache.set(key, w);
  }
  return w;
}

const NODE_MIN_W = 80, NODE_PAD = 4, PORT_LABEL_PAD = 6, PORT_ROW = 25;
const PARAM_CTRL_W = 120, TITLE_PX = 11, PORT_PX = 10, TITLE_BAR_H = 24;

function portText(p, activeScope) {
  const pre = showNames ? (p.prefix || (p.scoped ? activeScope : "")) : "";
  return pre ? pre + "." + p.label : p.label;
}
function nodeWidth(n, activeScope) {
  const enterable = !!(n.childKey && GRAPHS[n.childKey]);
  const titleText = n.title + (enterable ? "  ⤢" : (n.error ? "  ⚠" : ""));
  let titleW = measure(titleText, TITLE_PX);
  if (n.scope) titleW += 10 + scopeBadgeW(n.scope) + 6;   // gap + pill + right margin
  let maxIn = 0, maxOut = 0;
  for (const p of n.inputs) maxIn = Math.max(maxIn, measure(portText(p, activeScope), PORT_PX));
  for (const p of n.outputs) maxOut = Math.max(maxOut, measure(p.label, PORT_PX));
  let content = Math.max(titleW, maxIn + maxOut + 18);
  if (n.kind === "Param" && n.outputs.length)
    content = Math.max(content, maxIn + maxOut + PARAM_CTRL_W + 18);
  return Math.max(NODE_MIN_W, content + NODE_PAD * 2);
}
function nodeHeight(n) {
  const rows = Math.max(1, Math.max(n.inputs.length, n.outputs.length));
  return 42 + rows * PORT_ROW;
}
function portCenter(n, key, isInput) {
  const list = isInput ? n.inputs : n.outputs;
  const idx = list.findIndex(p => p.key === key);
  if (idx < 0) return null;
  const y = n.y + 34 + idx * PORT_ROW + 5;
  const x = isInput ? n.x : n.x + n._w;
  return { x, y };
}
function el(tag, attrs) {
  const e = document.createElementNS(SVGNS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  return e;
}
function color(kind) { return MODEL.colors[kind] || "rgb(100,100,100)"; }

// Scope (FunctionScope) badge styling for scoped Include nodes.
const SCOPE_ABBR = { FlightStickPitch: "Pitch", FlightStickRoll: "Roll",
  FlightStickCollective: "Coll", FlightStickYaw: "Yaw", FlightPedals: "Pedals" };
const SCOPE_COLORS = { FlightStickPitch: "rgb(220,90,140)", FlightStickRoll: "rgb(90,150,220)",
  FlightStickCollective: "rgb(120,180,90)", FlightStickYaw: "rgb(160,110,210)",
  FlightPedals: "rgb(210,150,60)" };
function scopeAbbrev(s) { return SCOPE_ABBR[s] || s.replace(/^FlightStick/, "").replace(/^Flight/, "") || s; }
function scopeColor(s) { return SCOPE_COLORS[s] || "rgb(150,150,150)"; }
function scopeBadgeW(s) { return measure(scopeAbbrev(s), 9) + 12; }   // pill width
// Highlight colour for an Output-port prefix: scope colour if it's a known
// scope (ties to the badge), else a generic accent.
function prefixColor(p) { return SCOPE_COLORS[p] || "rgb(150,185,225)"; }
let showNames = true;   // toggle: prefix Output ports with their resolved group

// --- navigation -----------------------------------------------------------
// Each level (a breadcrumb path: array of {key,label}) is pushed as a browser
// history entry via pushState. We render from popstate, so the mouse
// back/forward buttons, Alt+Arrows, and the browser's own back/forward all
// work natively in every browser (Firefox doesn't deliver DOM mouse events for
// the side buttons, so cooperating with history is the only reliable way).
// Per-level zoom/pan lives in viewByPath, keyed by path + orientation.
const ROOT_STACK = [{ key: MODEL.rootKey, label: MODEL.name, scope: "" }];
let currentStack = ROOT_STACK;
let curSeq = 0, maxSeq = 0;   // seq of current entry / highest reachable (for forward-enable)
const linksG = document.getElementById("links");
const nodesG = document.getElementById("nodes");
const crumbsEl = document.getElementById("crumbs");
const legendEl = document.getElementById("legend");
const backBtn = document.getElementById("back");
const fwdBtn = document.getElementById("fwd");

function curStack() { return currentStack; }
function sameStack(a, b) { return a.length === b.length && a.every((s, i) => s.key === b[i].key); }
function show(stack, seq) { currentStack = stack; curSeq = seq; renderGraph(); }
function navTo(stack) {
  if (sameStack(stack, currentStack)) return;
  curSeq = maxSeq = curSeq + 1;
  window.history.pushState({ stack, seq: curSeq }, "");
  show(stack, curSeq);
}
function enterInclude(node) {
  if (!node.childKey || !GRAPHS[node.childKey]) return;
  // carry the active scope: this include's own scope, else inherit the parent's
  const scope = node.scope || currentStack[currentStack.length - 1].scope || "";
  navTo(currentStack.concat([{ key: node.childKey, label: node.title, scope }]));
}
function gotoLevel(i) { navTo(currentStack.slice(0, i + 1)); }

window.addEventListener("popstate", e => {
  // Only react to our own states. Mobile browsers can fire a spurious, stateless
  // popstate on orientation change; ignoring it (rather than navigating) keeps it
  // from shuffling the history position and "eating" the next back press.
  const st = e.state;
  if (st && st.stack) show(st.stack, st.seq);
});

function renderCrumbs() {
  crumbsEl.innerHTML = "";
  backBtn.disabled = curSeq <= 0;
  fwdBtn.disabled = curSeq >= maxSeq;
  const stack = currentStack;
  // On a narrow screen the full path doesn't fit; show only the current level
  // (use ◀ / ▶ to move between levels) and let it ellipsize.
  if (window.matchMedia("(max-width: 640px)").matches) {
    const c = document.createElement("span");
    c.className = "crumb last";
    c.style.cssText = "display:block;overflow:hidden;text-overflow:ellipsis";
    c.textContent = stack[stack.length - 1].label;
    crumbsEl.appendChild(c);
    return;
  }
  stack.forEach((s, i) => {
    if (i > 0) {
      const sep = document.createElement("span");
      sep.className = "sep"; sep.textContent = "›";
      crumbsEl.appendChild(sep);
    }
    const c = document.createElement("span");
    c.className = "crumb" + (i === stack.length - 1 ? " last" : "");
    c.textContent = s.label;
    if (i < stack.length - 1) c.addEventListener("click", () => gotoLevel(i));
    crumbsEl.appendChild(c);
  });
}

function renderGraph() {
  const top = curStack()[curStack().length - 1];
  const g = GRAPHS[top.key];
  const activeScope = top.scope || "";   // scope of the Include we entered through
  linksG.textContent = ""; nodesG.textContent = "";
  const nodeById = new Map();
  for (const n of g.nodes) { n._w = nodeWidth(n, activeScope); n._h = nodeHeight(n); nodeById.set(n.id, n); }

  for (const lk of g.links) {
    const a = nodeById.get(lk.fromNode), b = nodeById.get(lk.toNode);
    if (!a || !b) continue;
    const from = portCenter(a, lk.fromPort, false);
    const to = portCenter(b, lk.toPort, true);
    if (!from || !to) continue;
    const s = Math.max(40, Math.abs(to.x - from.x) * 0.5) * 0.5;
    const d = `M ${from.x} ${from.y} C ${from.x + s} ${from.y}, ${to.x - s} ${to.y}, ${to.x} ${to.y}`;
    const path = el("path", { class: "link", d });
    const tip = el("title", {});
    tip.textContent = `${lk.fromNode}.${lk.fromPort} → ${lk.toNode}.${lk.toPort}`;
    path.appendChild(tip);
    linksG.appendChild(path);
  }

  for (const n of g.nodes) {
    const w = n._w, h = n._h;
    const enterable = !!(n.childKey && GRAPHS[n.childKey]);
    const grp = el("g", { transform: `translate(${n.x},${n.y})` });
    if (enterable) {
      grp.setAttribute("class", "enterable");
      grp.addEventListener("dblclick", () => enterInclude(n));
    }
    grp.appendChild(el("rect", { x: 0, y: 0, width: w, height: h, rx: 6, ry: 6,
      fill: "rgb(45,45,45)", stroke: enterable ? "rgb(180,80,140)" : "rgb(20,20,20)",
      "stroke-width": enterable ? 1.5 : 1 }));
    grp.appendChild(el("rect", { x: 0, y: 0, width: w, height: TITLE_BAR_H, rx: 6, ry: 6, fill: color(n.kind) }));
    grp.appendChild(el("rect", { x: 0, y: 12, width: w, height: 12, fill: color(n.kind) }));
    const t = el("text", { x: 8, y: 16, class: "node-title" });
    t.textContent = n.title + (enterable ? "  ⤢" : (n.error ? "  ⚠" : ""));
    if (n.error) { const tt = el("title", {}); tt.textContent = n.error; t.appendChild(tt); }
    else if (enterable) { const tt = el("title", {}); tt.textContent = "Double-click to open"; t.appendChild(tt); }
    grp.appendChild(t);

    if (n.scope) {   // scoped Include -> color-coded scope pill on the title bar
      const bw = scopeBadgeW(n.scope), bx = w - bw - 6;
      const badge = el("rect", { x: bx, y: 4, width: bw, height: 16, rx: 8, ry: 8,
        fill: scopeColor(n.scope), stroke: "rgba(0,0,0,0.35)", "stroke-width": 1 });
      const lab = el("text", { x: bx + bw / 2, y: 16, "text-anchor": "middle",
        "font-size": 9, "font-weight": 600, fill: "#fff" });
      lab.textContent = scopeAbbrev(n.scope);
      const tip = el("title", {}); tip.textContent = "scope: " + n.scope;
      badge.appendChild(tip); lab.appendChild(tip.cloneNode(true));
      grp.appendChild(badge); grp.appendChild(lab);
    }

    const draw = (p, idx, isInput) => {
      const cy = 34 + idx * PORT_ROW + 5;
      const cx = isInput ? 0 : w;
      grp.appendChild(el("circle", { cx, cy, r: 5,
        fill: isInput ? "rgb(0,191,255)" : "rgb(255,165,0)", stroke: "black", "stroke-width": 1 }));
      const lbl = el("text", { class: "port-label", y: cy + 3 });
      const pre = isInput && showNames ? (p.prefix || (p.scoped ? activeScope : "")) : "";
      if (pre) {   // highlighted resolved-group prefix on Output-node sink ports
        const ts1 = el("tspan", { fill: prefixColor(pre) }); ts1.textContent = pre + ".";
        const ts2 = el("tspan", {}); ts2.textContent = p.label;
        lbl.appendChild(ts1); lbl.appendChild(ts2);
        const tip = el("title", {}); tip.textContent = pre + "." + p.label; lbl.appendChild(tip);
      } else {
        lbl.textContent = p.label;
      }
      if (isInput) { lbl.setAttribute("x", PORT_LABEL_PAD); }
      else {
        lbl.setAttribute("text-anchor", "end");
        const rightPad = (n.kind === "Param") ? PARAM_CTRL_W + PORT_LABEL_PAD + 6 : PORT_LABEL_PAD;
        lbl.setAttribute("x", w - rightPad);
      }
      grp.appendChild(lbl);
      if (!isInput && p.value !== undefined) {
        const bw = PARAM_CTRL_W, bx = w - bw - PORT_LABEL_PAD, ov = p.overridden;
        grp.appendChild(el("rect", { x: bx, y: cy - 8, width: bw, height: 16, rx: 3, ry: 3,
          fill: ov ? "rgb(60,48,20)" : "rgb(30,30,30)",
          stroke: ov ? "rgb(240,180,70)" : "rgb(74,74,74)", "stroke-width": 1 }));
        const v = el("text", { class: "param-val", x: bx + 5, y: cy + 3 });
        if (ov) v.setAttribute("fill", "rgb(255,200,90)");
        v.textContent = p.value;
        const maxw = bw - 10;
        if (measure(p.value, PORT_PX) > maxw) { v.setAttribute("textLength", maxw); v.setAttribute("lengthAdjust", "spacingAndGlyphs"); }
        if (ov && p.default !== undefined) { const tt = el("title", {}); tt.textContent = "default: " + p.default; v.appendChild(tt); }
        grp.appendChild(v);
      }
    };
    n.inputs.forEach((p, i) => draw(p, i, true));
    n.outputs.forEach((p, i) => draw(p, i, false));
    nodesG.appendChild(grp);
  }

  // legend reflects the kinds present in the current graph
  legendEl.innerHTML = '<div class="title">Legend</div>';
  for (const k of [...new Set(g.nodes.map(n => n.kind))].sort()) {
    const s = document.createElement("span");
    s.innerHTML = `<i style="background:${color(k)}"></i>${k}`;
    legendEl.appendChild(s);
  }
  const scopes = [...new Set(g.nodes.filter(n => n.scope).map(n => n.scope))].sort();
  if (scopes.length) {
    const hdr = document.createElement("div");
    hdr.className = "title"; hdr.style.marginTop = "5px"; hdr.textContent = "Scopes";
    legendEl.appendChild(hdr);
    for (const sc of scopes) {
      const s = document.createElement("span");
      s.innerHTML = `<i style="background:${scopeColor(sc)};border-radius:7px"></i>${sc}`;
      legendEl.appendChild(s);
    }
  }
  legendEl.classList.toggle("empty", !g.nodes.length);   // CSS controls visibility (so the mobile breakpoint can hide it)
  renderCrumbs();
  if (!g.nodes.length) document.getElementById("stage").insertAdjacentHTML("beforeend", '<div id="empty">(empty graph)</div>');
  applyEntryView();
}

// --- pan / zoom ----------------------------------------------------------
const vp = document.getElementById("viewport");
const stage = document.getElementById("stage");
let scale = 1, tx = 0, ty = 0, panning = false, sx = 0, sy = 0;
// Views are remembered both on the exact history entry (for back/forward) and
// per breadcrumb-path (so a *new* entry from a crumb click — which has no entry
// view yet — still restores where you last were at that level instead of refitting).
const viewByPath = {};
function pathKey(stack) { return stack.map(s => s.key).join(">"); }
function orient() { return stage.clientWidth >= stage.clientHeight ? "L" : "P"; }
function apply() {
  vp.setAttribute("transform", `translate(${tx},${ty}) scale(${scale})`);
  viewByPath[pathKey(currentStack)] = { scale, tx, ty, o: orient() };   // remember view + the orientation it fits
}
function applyEntryView() {
  const v = viewByPath[pathKey(currentStack)];
  // restore only if it was captured in the current orientation; a portrait view
  // doesn't fit landscape, so re-fit instead of restoring a stale transform.
  if (v && v.o === orient()) { scale = v.scale; tx = v.tx; ty = v.ty; apply(); }
  else fit();
}
function bounds() {
  const g = GRAPHS[curStack()[curStack().length - 1].key];
  if (!g.nodes.length) return null;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const n of g.nodes) {
    minX = Math.min(minX, n.x); minY = Math.min(minY, n.y);
    maxX = Math.max(maxX, n.x + n._w); maxY = Math.max(maxY, n.y + n._h);
  }
  return { minX, minY, maxX, maxY };
}
function fit() {
  const b = bounds(); if (!b) { scale = 1; tx = 0; ty = 0; apply(); return; }
  const pad = 60, vw = stage.clientWidth, vh = stage.clientHeight;
  const gw = (b.maxX - b.minX) + pad * 2, gh = (b.maxY - b.minY) + pad * 2;
  scale = Math.min(vw / gw, vh / gh, 1.5);
  tx = -(b.minX - pad) * scale + (vw - gw * scale) / 2;
  ty = -(b.minY - pad) * scale + (vh - gh * scale) / 2;
  apply();
}
stage.addEventListener("mousedown", e => { if (e.button !== 0) return; panning = true; sx = e.clientX - tx; sy = e.clientY - ty; stage.classList.add("panning"); });
window.addEventListener("mousemove", e => { if (!panning) return; tx = e.clientX - sx; ty = e.clientY - sy; apply(); });
window.addEventListener("mouseup", () => { panning = false; stage.classList.remove("panning"); });
stage.addEventListener("wheel", e => {
  e.preventDefault();
  const r = stage.getBoundingClientRect();
  const mx = e.clientX - r.left, my = e.clientY - r.top;
  const f = e.deltaY < 0 ? 1.1 : 1 / 1.1;
  const ns = Math.min(4, Math.max(0.05, scale * f));
  tx = mx - (mx - tx) * (ns / scale); ty = my - (my - ty) * (ns / scale);
  scale = ns; apply();
}, { passive: false });

// --- touch: one finger pans, two fingers pinch-zoom (centred between them) ---
let pinch = null;   // { d0, ax, ay } at gesture start; ax/ay = graph anchor under centroid
function touchRel(t) { const r = stage.getBoundingClientRect(); return { x: t.clientX - r.left, y: t.clientY - r.top }; }
function touchMid(a, b) { const r = stage.getBoundingClientRect(); return { x: (a.clientX + b.clientX) / 2 - r.left, y: (a.clientY + b.clientY) / 2 - r.top }; }
function touchDist(a, b) { return Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY); }
function startPan(t) { panning = true; sx = t.clientX - tx; sy = t.clientY - ty; }
function startPinch(a, b) {
  panning = false;
  const m = touchMid(a, b);
  pinch = { d0: touchDist(a, b) || 1, ax: (m.x - tx) / scale, ay: (m.y - ty) / scale };
}
stage.addEventListener("touchstart", e => {
  if (e.touches.length >= 2) startPinch(e.touches[0], e.touches[1]);
  else if (e.touches.length === 1) { pinch = null; startPan(e.touches[0]); }
}, { passive: true });
stage.addEventListener("touchmove", e => {
  // touch-action:none already blocks browser scroll/zoom, so we stay passive
  // (no preventDefault) — that keeps tap -> dblclick working to enter includes.
  if (pinch && e.touches.length >= 2) {
    const m = touchMid(e.touches[0], e.touches[1]);
    const ns = Math.min(4, Math.max(0.05, scale * (touchDist(e.touches[0], e.touches[1]) / pinch.d0)));
    pinch.d0 = touchDist(e.touches[0], e.touches[1]);   // incremental so it tracks smoothly
    tx = m.x - pinch.ax * ns; ty = m.y - pinch.ay * ns; scale = ns;
    apply();
  } else if (panning && e.touches.length === 1) {
    tx = e.touches[0].clientX - sx; ty = e.touches[0].clientY - sy;
    apply();
  }
}, { passive: true });
function endTouch(e) {
  if (e.touches.length === 0) { panning = false; pinch = null; }
  else if (e.touches.length === 1) { pinch = null; startPan(e.touches[0]); }   // pinch -> pan handoff
  else if (e.touches.length >= 2) startPinch(e.touches[0], e.touches[1]);
}
stage.addEventListener("touchend", endTouch, { passive: true });
stage.addEventListener("touchcancel", endTouch, { passive: true });
document.getElementById("fit").addEventListener("click", fit);
document.getElementById("reset").addEventListener("click", () => { scale = 1; tx = 0; ty = 0; apply(); });
const namesBtn = document.getElementById("names");
namesBtn.addEventListener("click", () => {
  showNames = !showNames;
  namesBtn.classList.toggle("off", !showNames);
  renderGraph();
});
// Legend toggle: hidden by default on a phone-sized screen, on-demand elsewhere.
const legBtn = document.getElementById("leg");
function syncLegBtn() { legBtn.classList.toggle("off", document.body.classList.contains("legend-hidden")); }
legBtn.addEventListener("click", () => { document.body.classList.toggle("legend-hidden"); syncLegBtn(); });
if (window.matchMedia("(max-width: 640px)").matches) document.body.classList.add("legend-hidden");
syncLegBtn();
// orientation / resize: breadcrumb mode depends on width, and a portrait<->landscape
// flip makes the current transform stale, so re-fit the current level on flip.
// (Minor same-orientation resizes — e.g. the mobile address bar — are left alone.)
let lastOrient = orient();
window.addEventListener("resize", () => {
  renderCrumbs();
  const o = orient();
  if (o !== lastOrient) { lastOrient = o; fit(); }
});

// Toolbar back/forward delegate to the browser so they share one code path
// with the mouse side buttons / Alt+Arrows (all of which fire popstate).
backBtn.addEventListener("click", () => window.history.back());
fwdBtn.addEventListener("click", () => window.history.forward());

// Seed history: the root as the current entry. (Back at the root level then
// behaves like a normal page back, which avoids the fragile self-navigation
// that interfered with orientation-change popstates.)
window.history.replaceState({ stack: ROOT_STACK, seq: 0 }, "");
document.getElementById("subtitle").textContent = MODEL.subtitle || "";
show(ROOT_STACK, 0);
</script>
__BANNER__
</body>
</html>
"""


def render_html(model, classification=None):
    name = model["name"]
    html = HTML_TEMPLATE
    html = html.replace("__TITLE__", _esc(name))
    html = html.replace("__MODEL__", json.dumps(model, ensure_ascii=False))
    if classification:
        bg = {"PUBLIC": "#2e7d32", "CONFIDENTIAL": "#b8860b",
              "STRICTLY CONFIDENTIAL": "#b00020"}.get(classification.upper(), "#555")
        banner = ('<div id="banner" style="background:%s;color:#fff">%s</div>'
                  % (bg, _esc(classification.upper())))
        html = html.replace("inset: 40px 0 0 0;", "inset: 62px 0 0 0;")
        html = html.replace("__BANNER__", banner)
    else:
        html = html.replace("__BANNER__", "")
    return html


def _esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace('"', "&quot;"))


def render_file(in_path, out_path, classification):
    graph, base_dir, name, overrides, subtitle = load_input(in_path)
    model = build_model(graph, base_dir, name, overrides, subtitle)
    html = render_html(model, classification)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    root = model["graphs"][model["rootKey"]]
    n_err = sum(1 for n in root["nodes"] if n["error"])
    bits = []
    if n_err:
        bits.append("%d include warning(s)" % n_err)
    if overrides:
        applied = sum(1 for g in model["graphs"].values() for n in g["nodes"]
                      for p in n["outputs"] if p.get("overridden"))
        bits.append("%d/%d overrides shown" % (applied, len(overrides)))
    note = "  (%s)" % "; ".join(bits) if bits else ""
    print("rendered %s -> %s  [%d nodes, %d links, %d sub-graph(s)]%s"
          % (in_path, out_path, len(root["nodes"]), len(root["links"]),
             len(model["graphs"]) - 1, note))


def main(argv):
    ap = argparse.ArgumentParser(description="Render an FFB graph JSON to self-contained HTML.")
    ap.add_argument("input", help="graph .json file or a directory of them")
    ap.add_argument("-o", "--output", help="output .html (single-file input only)")
    ap.add_argument("--classification", help="banner label, e.g. PUBLIC / CONFIDENTIAL / 'STRICTLY CONFIDENTIAL'")
    args = ap.parse_args(argv)

    if os.path.isdir(args.input):
        if args.output:
            ap.error("-o is not allowed with a directory input")
        files = [os.path.join(args.input, f) for f in sorted(os.listdir(args.input))
                 if f.endswith(".json") and "index" not in f]
        if not files:
            print("no .json files in %s" % args.input)
            return 1
        for f in files:
            render_file(f, os.path.splitext(f)[0] + ".html", args.classification)
        return 0

    if not os.path.isfile(args.input):
        print("error: no such file: %s" % args.input, file=sys.stderr)
        return 1
    out = args.output or (os.path.splitext(args.input)[0] + ".html")
    render_file(args.input, out, args.classification)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
