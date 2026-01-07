import argparse
import json
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


def _get_field(data, *names, default=None):
    for name in names:
        if name in data:
            return data[name]
    return default


def _looks_like_shifter_config(data):
    return any(
        key in data
        for key in (
            "posXMin",
            "pos_x_min",
            "gateSegments",
            "gate_segments",
            "detents",
        )
    )


def _extract_shifter_node(node):
    node = node.get("functionConfig", node)
    node = node.get("function_config", node)
    if "shifter" in node:
        return node["shifter"]
    return None


def _extract_detect_node(node):
    node = node.get("functionConfig", node)
    node = node.get("function_config", node)
    aux = node.get("auxFunction") or node.get("aux_function") or {}
    detect = aux.get("shifterDetect") or aux.get("shifter_detect")
    return detect


def _load_configs(path, item_index):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    if _looks_like_shifter_config(data):
        return data, {}

    def try_items(items):
        for it in items:
            found = _extract_shifter_node(it)
            det = _extract_detect_node(it) if found is not None else None
            if found is not None:
                return found, det or {}
        return None, None

    if "configItems" in data:
        items = data["configItems"]
        if not items:
            raise ValueError("configItems is empty.")
        chosen = None
        detect = None
        # Prefer the requested index if it contains a shifter; otherwise scan all.
        if 0 <= item_index < len(items):
            chosen = _extract_shifter_node(items[item_index])
            detect = _extract_detect_node(items[item_index])
        if chosen is None:
            chosen, detect = try_items(items)
        if chosen is None:
            raise ValueError("No shifter config found in configItems.")
        return chosen, detect or {}

    shifter = _extract_shifter_node(data)
    detect = _extract_detect_node(data) or {}
    if shifter is None:
        raise ValueError("No shifter config found in JSON.")
    return shifter, detect


def _to_mm_01(value):
    return 0.1 * float(value)


@dataclass
class GateSegPre:
    horizontal: bool
    hw: float
    line: float
    a0_cap: float
    a1_cap: float


@dataclass
class DetentPre:
    x_mm: float
    y_mm: float
    radius_mm: float
    spring_n_per_mm: float
    lane_v: int = -1
    lane_h: int = -1


def _order2(a, b):
    return (a, b) if a <= b else (b, a)


def _make_seg_pre(seg) -> GateSegPre:
    x0 = _to_mm_01(seg.get("x0", 0))
    y0 = _to_mm_01(seg.get("y0", 0))
    x1 = _to_mm_01(seg.get("x1", x0))
    y1 = _to_mm_01(seg.get("y1", y0))
    hw = _to_mm_01(seg.get("halfWidth", seg.get("half_width", 0)))
    horizontal = abs(y1 - y0) < 1e-6
    if horizontal:
        xmin, xmax = _order2(x0, x1)
        line = y0
        a0_cap = xmin - hw
        a1_cap = xmax + hw
    else:
        ymin, ymax = _order2(y0, y1)
        line = x0
        a0_cap = ymin - hw
        a1_cap = ymax + hw
    return GateSegPre(horizontal, hw, line, a0_cap, a1_cap)


def _make_detent_pre(det) -> DetentPre:
    return DetentPre(
        x_mm=_to_mm_01(det.get("x", 0)),
        y_mm=_to_mm_01(det.get("y", 0)),
        radius_mm=_to_mm_01(det.get("radius", 0)),
        spring_n_per_mm=float(det.get("spring", 0.0)),
    )


def _absf(v):
    return v if v >= 0.0 else -v


def _inside(seg: GateSegPre, x_mm: float, y_mm: float, margin_mm: float) -> bool:
    hw = seg.hw + margin_mm
    if seg.horizontal:
        return (_absf(y_mm - seg.line) <= hw) and (x_mm >= seg.a0_cap - margin_mm) and (x_mm <= seg.a1_cap + margin_mm)
    return (_absf(x_mm - seg.line) <= hw) and (y_mm >= seg.a0_cap - margin_mm) and (y_mm <= seg.a1_cap + margin_mm)


def _perp_dist(seg: GateSegPre, x_mm: float, y_mm: float) -> float:
    return _absf(y_mm - seg.line) if seg.horizontal else _absf(x_mm - seg.line)


def _interval_from_seg(seg: GateSegPre, axis: str, g_min: float, g_max: float) -> Tuple[float, float]:
    if axis == "x":
        iv = (seg.a0_cap, seg.a1_cap) if seg.horizontal else (seg.line - seg.hw, seg.line + seg.hw)
    else:
        iv = (seg.a0_cap, seg.a1_cap) if not seg.horizontal else (seg.line - seg.hw, seg.line + seg.hw)
    return max(iv[0], g_min), min(iv[1], g_max)


def _union_insert(unions: List[Tuple[float, float]], iv: Tuple[float, float]) -> List[Tuple[float, float]]:
    lo, hi = iv
    if lo > hi:
        return unions
    merged = (lo, hi)
    out = []
    for ulo, uhi in unions:
        if merged[1] < ulo or merged[0] > uhi:
            out.append((ulo, uhi))
            continue
        merged = (min(merged[0], ulo), max(merged[1], uhi))
    out.append(merged)
    return out


def _pick_union_best(unions: Sequence[Tuple[float, float]], coord: float, fallback: Tuple[float, float]) -> Tuple[float, float]:
    if not unions:
        return fallback
    best = None
    best_d = 1e9
    for lo, hi in unions:
        if coord < lo:
            d = lo - coord
        elif coord > hi:
            d = coord - hi
        else:
            d = 0.0
        if d < best_d:
            best_d = d
            best = (lo, hi)
    return best if best is not None else fallback


def _assign_detents_to_lanes(segments: List[GateSegPre], detents: List[DetentPre], det_lane_tol_mm: float = 0.8):
    seg_index_v = [i for i, s in enumerate(segments) if not s.horizontal]
    seg_index_h = [i for i, s in enumerate(segments) if s.horizontal]
    det_by_v = {i: [] for i in range(len(seg_index_v))}
    det_by_h = {i: [] for i in range(len(seg_index_h))}

    for det in detents:
        # vertical (X lanes)
        best_v = -1
        best_dx = 1e9
        for idx, si in enumerate(seg_index_v):
            lane = segments[si]
            dx = _absf(det.x_mm - lane.line)
            if dx < best_dx:
                best_dx = dx
                best_v = idx
        if best_v >= 0 and best_dx <= det_lane_tol_mm:
            det.lane_v = best_v
            det_by_v[best_v].append(det)

        # horizontal (Y lanes)
        best_h = -1
        best_dy = 1e9
        for idx, si in enumerate(seg_index_h):
            lane = segments[si]
            dy = _absf(det.y_mm - lane.line)
            if dy < best_dy:
                best_dy = dy
                best_h = idx
        if best_h >= 0 and best_dy <= det_lane_tol_mm:
            det.lane_h = best_h
            det_by_h[best_h].append(det)

    return seg_index_v, seg_index_h, det_by_v, det_by_h


def _update_axis_context(x_mm: float, y_mm: float, axis: str, segments: List[GateSegPre], membership_margin_mm: float, g_min: float, g_max: float):
    unions: List[Tuple[float, float]] = []
    for seg in segments:
        if not _inside(seg, x_mm, y_mm, membership_margin_mm):
            continue
        unions = _union_insert(unions, _interval_from_seg(seg, axis, g_min, g_max))

    # oriented lane selection (no hysteresis; pick closest containing lane of correct orientation)
    if axis == "x":
        oriented = [i for i, s in enumerate(segments) if s.horizontal]
    else:
        oriented = [i for i, s in enumerate(segments) if not s.horizontal]
    best_lane = -1
    best_d = 1e9
    for idx in oriented:
        seg = segments[idx]
        if not _inside(seg, x_mm, y_mm, membership_margin_mm):
            continue
        d = _perp_dist(seg, x_mm, y_mm)
        if d < best_d:
            best_d = d
            best_lane = idx

    coord = x_mm if axis == "x" else y_mm
    soft = _pick_union_best(unions, coord, (g_min, g_max))
    return soft, best_lane


def _find_best_horizontal_gate(segs: List[GateSegPre]):
    best = -1
    best_span = -1.0
    for i, s in enumerate(segs):
        if not s.horizontal:
            continue
        span = s.a1_cap - s.a0_cap
        if span > best_span:
            best_span = span
            best = i
    return best


def _centering_anchor(detect_cfg, segments: List[GateSegPre]):
    # prefer neutral, else gear 3 + best horizontal gate
    gear_slots = detect_cfg.get("gearSlots") or detect_cfg.get("gear_slots") or []
    anchor = {"valid": False, "x": 0.0, "y": 0.0}
    for slot in gear_slots:
        gear = slot.get("gear")
        if gear in ("SHIFTER_GEAR_NEUTRAL", 0):
            anchor["x"] = 0.1 * float(slot.get("centerX", slot.get("center_x", 0)))
            anchor["y"] = 0.1 * float(slot.get("centerY", slot.get("center_y", 0)))
            anchor["valid"] = True
            return anchor

    gear3 = None
    for slot in gear_slots:
        gear = slot.get("gear")
        if gear in ("SHIFTER_GEAR_3", 3):
            gear3 = slot
            break
    h_idx = _find_best_horizontal_gate(segments)
    if gear3 is not None and h_idx >= 0:
        anchor["x"] = 0.1 * float(gear3.get("centerX", gear3.get("center_x", 0)))
        anchor["y"] = segments[h_idx].line
        anchor["valid"] = True
    return anchor


def _compute_field(config, detect_cfg, axis: str, step_mm: float, membership_margin_mm: float = 0.2, det_lane_tol_mm: float = 0.8):
    segments = [_make_seg_pre(seg) for seg in config.get("gateSegments", config.get("gate_segments", []))]
    detents = [_make_detent_pre(det) for det in config.get("detents", [])]
    seg_index_v, seg_index_h, det_by_v, det_by_h = _assign_detents_to_lanes(segments, detents, det_lane_tol_mm)

    x_min = float(_get_field(config, "posXMin", "pos_x_min"))
    x_max = float(_get_field(config, "posXMax", "pos_x_max"))
    y_min = float(_get_field(config, "posYMin", "pos_y_min"))
    y_max = float(_get_field(config, "posYMax", "pos_y_max"))

    x_vals = np.arange(x_min, x_max + 0.5 * step_mm, step_mm, dtype=float)
    y_vals = np.arange(y_min, y_max + 0.5 * step_mm, step_mm, dtype=float)
    soft_lo = np.zeros((len(y_vals), len(x_vals)), dtype=float)
    soft_hi = np.zeros_like(soft_lo)
    det_force = np.zeros_like(soft_lo)
    lane_idx = -np.ones_like(soft_lo, dtype=int)

    anchor = _centering_anchor(detect_cfg or {}, segments)
    centering_k = float(_get_field(config, "maxForce", "max_force", default=0.0))

    for iy, y in enumerate(y_vals):
        for ix, x in enumerate(x_vals):
            g_min = x_min if axis == "x" else y_min
            g_max = x_max if axis == "x" else y_max
            soft, lane = _update_axis_context(x, y, axis, segments, membership_margin_mm, g_min, g_max)
            soft_lo[iy, ix] = soft[0]
            soft_hi[iy, ix] = soft[1]
            lane_idx[iy, ix] = lane

            # active detents for lane
            if lane >= 0:
                if axis == "y":
                    list_idx = seg_index_v.index(lane) if lane in seg_index_v else -1
                    active = det_by_v.get(list_idx, []) if list_idx >= 0 else []
                    coord = y
                else:
                    list_idx = seg_index_h.index(lane) if lane in seg_index_h else -1
                    active = det_by_h.get(list_idx, []) if list_idx >= 0 else []
                    coord = x
                f_sum = 0.0
                for det in active:
                    det_center = det.y_mm if axis == "y" else det.x_mm
                    z = (coord - det_center) / det.radius_mm
                    if -1.0 < z < 1.0:
                        f_sum += det.spring_n_per_mm * math.sin(math.pi * z)
                det_force[iy, ix] = f_sum
            else:
                det_force[iy, ix] = 0.0

            # Centering spring (axis-aligned)
            if anchor["valid"] and centering_k > 0.0:
                target = anchor["x"] if axis == "x" else anchor["y"]
                coord_axis = x if axis == "x" else y
                det_force[iy, ix] += -centering_k * (coord_axis - target)

    return x_vals, y_vals, soft_lo, soft_hi, det_force, lane_idx, segments, detents, anchor


def _compute_lane_curves(axis: str, segments: List[GateSegPre], detents: List[DetentPre], seg_index_v, seg_index_h, det_by_v, det_by_h, step_mm: float, centering=None, centering_k=0.0):
    curves = []
    lane_pairs = []
    if axis == "x":
        lane_pairs = [(li, segments[seg_index_h[li]], det_by_h.get(li, [])) for li in range(len(seg_index_h))]
    else:
        lane_pairs = [(li, segments[seg_index_v[li]], det_by_v.get(li, [])) for li in range(len(seg_index_v))]

    for li, seg, lane_detents in lane_pairs:
        lo = seg.a0_cap
        hi = seg.a1_cap
        xs = np.arange(lo, hi + 0.5 * step_mm, step_mm, dtype=float)
        force = np.zeros_like(xs)
        if lane_detents and axis == "y":
            for det in lane_detents:
                center = det.y_mm if axis == "y" else det.x_mm
                radius = det.radius_mm if det.radius_mm > 1e-6 else 1.0
                z = (xs - center) / radius
                mask = (z > -1.0) & (z < 1.0)
                force[mask] += det.spring_n_per_mm * np.sin(np.pi * z[mask])
        if centering and centering.get("valid", False) and centering_k > 0.0:
            target = centering["x"] if axis == "x" else centering["y"]
            force += -centering_k * (xs - target)
        curves.append({"lane": li, "coord": xs, "force": force})
    return curves


def _overlay_gate_segments(ax, segments: List[GateSegPre]):
    for seg in segments:
        xs = [seg.a0_cap, seg.a1_cap] if seg.horizontal else [seg.line, seg.line]
        ys = [seg.line, seg.line] if seg.horizontal else [seg.a0_cap, seg.a1_cap]
        ax.plot(xs, ys, color="black", linewidth=1.4, alpha=0.8)


def _overlay_detents(ax, detents: List[DetentPre]):
    if not detents:
        return
    ax.scatter([d.x_mm for d in detents], [d.y_mm for d in detents], color="tab:red", s=26, alpha=0.9, label="Detents")


def _plot_heat(ax, x_vals, y_vals, data, title, cmap, vmin=None, vmax=None):
    extent = (x_vals[0], x_vals[-1], y_vals[0], y_vals[-1])
    im = ax.imshow(data, origin="lower", extent=extent, aspect="equal", cmap=cmap, vmin=vmin, vmax=vmax)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(title)
    return im


def main():
    parser = argparse.ArgumentParser(description="Visualize shifter detent force for the current physical model.")
    parser.add_argument(
        "--config-json",
        default="SimhubPlugin/shifter_hpattern_demo_config.json",
        help="Path to a JSON file containing a shifter config or configItems list.",
    )
    parser.add_argument(
        "--item-index",
        type=int,
        default=0,
        help="Index into configItems when present.",
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y"),
        default="x",
        help="Axis role to visualize (matches firmware AxisRole).",
    )
    parser.add_argument(
        "--step-mm",
        type=float,
        default=5.0,
        help="Sampling step in mm.",
    )
    parser.add_argument(
        "--membership-margin-mm",
        type=float,
        default=0.2,
        help="Margin used for corridor membership (matches firmware default).",
    )
    parser.add_argument(
        "--curve-step-mm",
        type=float,
        default=1.0,
        help="Sampling step in mm for per-lane force curves.",
    )
    parser.add_argument(
        "--no-curves",
        action="store_true",
        help="Skip plotting per-gate detent force curves (including centering spring).",
    )
    parser.add_argument(
        "--output",
        help="Output image path (png/svg). If omitted, opens a window.",
    )
    args = parser.parse_args()

    config, detect_cfg = _load_configs(args.config_json, args.item_index)
    (
        x_vals,
        y_vals,
        soft_lo,
        soft_hi,
        det_force,
        lane_idx,
        segments,
        detents,
        anchor,
    ) = _compute_field(config, detect_cfg, args.axis, args.step_mm, args.membership_margin_mm)

    seg_index_v, seg_index_h, det_by_v, det_by_h = _assign_detents_to_lanes(segments, detents)
    lane_curves = [] if args.no_curves else _compute_lane_curves(args.axis, segments, detents, seg_index_v, seg_index_h, det_by_v, det_by_h, args.curve_step_mm, centering=anchor, centering_k=float(_get_field(config, "maxForce", "max_force", default=0.0)))

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting. Install it with: pip install matplotlib") from exc

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    vlim = np.max(np.abs(det_force)) or 1.0
    im0 = _plot_heat(axes[0], x_vals, y_vals, det_force, f"Detent force along {args.axis.upper()} (N)", cmap="coolwarm", vmin=-vlim, vmax=vlim)
    _overlay_gate_segments(axes[0], segments)
    _overlay_detents(axes[0], detents)
    fig.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    span = soft_hi - soft_lo
    im1 = _plot_heat(axes[1], x_vals, y_vals, span, f"Soft travel span for {args.axis.upper()} (mm)", cmap="viridis")
    _overlay_gate_segments(axes[1], segments)
    fig.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    axes[0].legend(loc="upper right")
    fig.suptitle(f"Step {args.step_mm:.2f} mm, membership margin {args.membership_margin_mm:.2f} mm")

    curve_fig = None
    if lane_curves:
        curve_fig, curve_ax = plt.subplots(1, 1, figsize=(8, 4), constrained_layout=True)
        for curve in lane_curves:
            curve_ax.plot(curve["coord"], curve["force"], label=f"Lane {curve['lane']}")
        curve_ax.axhline(0.0, color="black", linewidth=0.8)
        curve_ax.set_xlabel(f"{args.axis.upper()} position (mm)")
        curve_ax.set_ylabel("Detent + centering force (N)")
        curve_ax.set_title("Per-gate force curves")
        curve_ax.legend()

    if args.output:
        fig.savefig(args.output, dpi=150)
        if curve_fig:
            stem, ext = (args.output.rsplit(".", 1) + ["png"])[:2]
            curve_fig.savefig(f"{stem}_curves.{ext}", dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
