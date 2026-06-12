#!/usr/bin/env python3
"""Structural validator for FFB graph JSON files.

Checks, per graph file:
  - every Link's FromNodeId/ToNodeId exists
  - every Link's FromPort/ToPort exists on the referenced node, with the
    right direction (Output for From, Input for To)
  - Include node ports are resolved from the included file's Input/Output
    nodes (port id = Name or SignalSuffix)
  - every LocalReceive BusName has a matching LocalSend BusName in the
    same top-level graph
  - duplicate node ids

Usage: python validate_graphs.py <file-or-dir> [...]
Exit code 1 on any error. Warnings don't fail the run.
"""
import json
import os
import sys

errors = []
warnings = []


def port_id(port):
    return port.get("Name") or port.get("SignalSuffix")


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def include_ports(inc_path):
    """Returns (input_port_names, output_port_names) exposed by an include."""
    g = load(inc_path)
    ins, outs = set(), set()
    for n in g.get("Nodes", []):
        kind = n.get("Kind")
        if kind == "Input":
            for p in n.get("Ports", []):
                if p.get("Kind") == "Output":
                    ins.add(port_id(p))
        elif kind == "Output":
            for p in n.get("Ports", []):
                if p.get("Kind") == "Input":
                    outs.add(port_id(p))
    return ins, outs


def node_ports(node, base_dir, fname):
    """Returns (input_names, output_names) for a node."""
    kind = node.get("Kind")
    if kind == "Include":
        rel = node.get("IncludePath", "").replace("\\", os.sep)
        inc_path = os.path.normpath(os.path.join(base_dir, rel))
        if not os.path.isfile(inc_path):
            errors.append(f"{fname}: include {node['Id']} -> missing file {inc_path}")
            return set(), set()
        return include_ports(inc_path)
    ins = {port_id(p) for p in node.get("Ports", []) if p.get("Kind") == "Input"}
    outs = {port_id(p) for p in node.get("Ports", []) if p.get("Kind") == "Output"}
    return ins, outs


def validate(path):
    fname = os.path.relpath(path)
    g = load(path)
    base_dir = os.path.dirname(path)
    nodes = {}
    for n in g.get("Nodes", []):
        if n["Id"] in nodes:
            errors.append(f"{fname}: duplicate node id {n['Id']}")
        nodes[n["Id"]] = n

    ports = {nid: node_ports(n, base_dir, fname) for nid, n in nodes.items()}

    for l in g.get("Links", []):
        src, dst = l.get("FromNodeId"), l.get("ToNodeId")
        if src not in nodes:
            errors.append(f"{fname}: link from unknown node {src}")
            continue
        if dst not in nodes:
            errors.append(f"{fname}: link to unknown node {dst}")
            continue
        if l.get("FromPort") not in ports[src][1]:
            errors.append(
                f"{fname}: {src}.{l.get('FromPort')} is not an output port "
                f"(has: {sorted(p for p in ports[src][1] if p)})")
        if l.get("ToPort") not in ports[dst][0]:
            errors.append(
                f"{fname}: {dst}.{l.get('ToPort')} is not an input port "
                f"(has: {sorted(p for p in ports[dst][0] if p)})")

    sends = {p.get("BusName") for n in nodes.values() if n.get("Kind") == "LocalSend"
             for p in n.get("Ports", [])}
    recvs = {p.get("BusName") for n in nodes.values() if n.get("Kind") == "LocalReceive"
             for p in n.get("Ports", [])}
    for b in sorted(recvs - sends):
        warnings.append(f"{fname}: bus '{b}' received but never sent")
    for b in sorted(sends - recvs):
        warnings.append(f"{fname}: bus '{b}' sent but never received")

    # links wired into Input-kind ports more than once
    seen = {}
    for l in g.get("Links", []):
        key = (l.get("ToNodeId"), l.get("ToPort"))
        if key in seen:
            errors.append(f"{fname}: input {key[0]}.{key[1]} driven by multiple links")
        seen[key] = True


def main(args):
    files = []
    for a in args:
        if os.path.isdir(a):
            for root, _, names in os.walk(a):
                files += [os.path.join(root, n) for n in names if n.endswith(".json")
                          and "index" not in n]
        else:
            files.append(a)
    for f in files:
        try:
            validate(f)
        except (json.JSONDecodeError, KeyError) as e:
            errors.append(f"{f}: parse failure: {e}")
    for w in warnings:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    print(f"{len(files)} file(s), {len(errors)} error(s), {len(warnings)} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or ["."]))
