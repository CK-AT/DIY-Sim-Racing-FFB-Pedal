import argparse
import json
import math

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


def _load_shifter_config(path, item_index):
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    if _looks_like_shifter_config(data):
        return data

    if "configItems" in data:
        items = data["configItems"]
        if not items:
            raise ValueError("configItems is empty.")
        if item_index < 0 or item_index >= len(items):
            raise ValueError(f"configItems index {item_index} is out of range.")
        data = items[item_index]

    data = data.get("functionConfig", data)
    data = data.get("function_config", data)

    if "shifter" not in data:
        raise ValueError("No shifter config found in JSON.")
    return data["shifter"]


def _count_for_range(min_val, max_val, step):
    if step <= 0.0:
        return 0
    if max_val < min_val:
        min_val, max_val = max_val, min_val
    return int(max(1.0, math.floor((max_val - min_val) / step) + 1.0))


def _build_gate_segments(config):
    segments = []
    for seg in config.get("gateSegments", config.get("gate_segments", [])):
        segments.append(
            {
                "x0": 0.1 * float(seg["x0"]),
                "y0": 0.1 * float(seg["y0"]),
                "x1": 0.1 * float(seg["x1"]),
                "y1": 0.1 * float(seg["y1"]),
                "half_width": 0.1 * float(seg["halfWidth"]),
                "spring_center": float(seg["springCenter"]),
                "spring_wall": float(seg["springWall"]),
            }
        )
    return segments


def _build_detents(config):
    detents = []
    for detent in config.get("detents", []):
        detents.append(
            {
                "x": 0.1 * float(detent["x"]),
                "y": 0.1 * float(detent["y"]),
                "radius": 0.1 * float(detent["radius"]),
                "spring": float(detent["spring"]),
            }
        )
    return detents


def _compute_neutral_guide(config, segments):
    if _get_field(config, "sequential", "sequential", default=False):
        return None
    if not segments:
        return None

    horizontal_idx = None
    horizontal_len = 0.0
    horizontal_y = 0.0
    horizontal_half_width = 0.0
    horizontal_spring = 0.0
    vertical_sum_x = 0.0
    vertical_count = 0.0

    for idx, seg in enumerate(segments):
        dx = seg["x1"] - seg["x0"]
        dy = seg["y1"] - seg["y0"]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        if abs(dy) <= abs(dx):
            mid_y = 0.5 * (seg["y0"] + seg["y1"])
            if horizontal_idx is None or length > horizontal_len or (
                length == horizontal_len and abs(mid_y) < abs(horizontal_y)
            ):
                horizontal_idx = idx
                horizontal_len = length
                horizontal_y = mid_y
                horizontal_half_width = seg["half_width"]
                horizontal_spring = seg["spring_center"]
        if abs(dx) <= abs(dy):
            vertical_sum_x += 0.5 * (seg["x0"] + seg["x1"])
            vertical_count += 1.0

    if horizontal_idx is None or vertical_count < 0.5 or horizontal_half_width <= 1e-6:
        return None

    avg_x = vertical_sum_x / vertical_count
    vertical_idx = None
    vertical_x = 0.0
    vertical_spring = 0.0
    best_dist = 0.0

    for idx, seg in enumerate(segments):
        dx = seg["x1"] - seg["x0"]
        dy = seg["y1"] - seg["y0"]
        if abs(dx) > abs(dy):
            continue
        mid_x = 0.5 * (seg["x0"] + seg["x1"])
        dist = abs(mid_x - avg_x)
        if vertical_idx is None or dist < best_dist:
            vertical_idx = idx
            best_dist = dist
            vertical_x = mid_x
            vertical_spring = seg["spring_center"]

    if vertical_idx is None:
        return None

    return {
        "x": vertical_x,
        "y": horizontal_y,
        "half_width": horizontal_half_width,
        "spring": 0.5 * (horizontal_spring + vertical_spring),
    }


def _compute_force_map(config, step_override, max_points):
    x_min = float(_get_field(config, "posXMin", "pos_x_min"))
    x_max = float(_get_field(config, "posXMax", "pos_x_max"))
    y_min = float(_get_field(config, "posYMin", "pos_y_min"))
    y_max = float(_get_field(config, "posYMax", "pos_y_max"))
    max_force = float(_get_field(config, "maxForce", "max_force", default=0.0))

    if step_override is not None:
        step = max(step_override, 0.1)
    else:
        grid_step = int(_get_field(config, "gridStep", "grid_step", default=1))
        step = max(1, grid_step) * 0.1
        step = max(step, 0.1)

    x_count = _count_for_range(x_min, x_max, step)
    y_count = _count_for_range(y_min, y_max, step)
    total = x_count * y_count
    if total > max_points and total > 0:
        scale = math.sqrt(total / max_points)
        step = max(0.1, step * scale)
        x_count = _count_for_range(x_min, x_max, step)
        y_count = _count_for_range(y_min, y_max, step)
        total = x_count * y_count

    x_vals = x_min + step * np.arange(x_count, dtype=float)
    y_vals = y_min + step * np.arange(y_count, dtype=float)
    fx = np.zeros((y_count, x_count), dtype=float)
    fy = np.zeros((y_count, x_count), dtype=float)

    segments = _build_gate_segments(config)
    detents = _build_detents(config)
    neutral = _compute_neutral_guide(config, segments)

    for iy, y in enumerate(y_vals):
        for ix, x in enumerate(x_vals):
            fx_val = 0.0
            fy_val = 0.0

            if segments:
                has_segment = False
                best_metric = 0.0
                best_dist = 0.0
                best_dx = 0.0
                best_dy = 0.0
                best_half_width = 0.0
                best_spring_center = 0.0
                best_spring_wall = 0.0

                for seg in segments:
                    vx = seg["x1"] - seg["x0"]
                    vy = seg["y1"] - seg["y0"]
                    len_sq = (vx * vx) + (vy * vy)
                    t = 0.0
                    if len_sq > 1e-6:
                        t = ((x - seg["x0"]) * vx + (y - seg["y0"]) * vy) / len_sq
                        t = max(0.0, min(1.0, t))
                    nx = seg["x0"] + (t * vx)
                    ny = seg["y0"] + (t * vy)
                    dx = x - nx
                    dy = y - ny
                    dist = math.sqrt((dx * dx) + (dy * dy))
                    metric = max(0.0, dist - seg["half_width"])
                    if not has_segment or metric < best_metric or (metric == best_metric and dist < best_dist):
                        has_segment = True
                        best_metric = metric
                        best_dist = dist
                        best_dx = dx
                        best_dy = dy
                        best_half_width = seg["half_width"]
                        best_spring_center = seg["spring_center"]
                        best_spring_wall = seg["spring_wall"]

                if has_segment and best_dist > 1e-6:
                    inv_dist = 1.0 / best_dist
                    nx = -best_dx * inv_dist
                    ny = -best_dy * inv_dist
                    if best_dist <= best_half_width:
                        force_mag = best_spring_center * best_dist
                    else:
                        outside = best_dist - best_half_width
                        force_mag = (best_spring_wall * outside) + (best_spring_center * best_half_width)
                    fx_val += nx * force_mag
                    fy_val += ny * force_mag

            if neutral is not None:
                dy = abs(y - neutral["y"])
                if dy <= neutral["half_width"]:
                    weight = 1.0 - (dy / neutral["half_width"])
                    fx_val += (neutral["x"] - x) * neutral["spring"] * weight

            if detents:
                for detent in detents:
                    dx = x - detent["x"]
                    dy = y - detent["y"]
                    dist = math.sqrt((dx * dx) + (dy * dy))
                    if dist < 1e-6 or dist > detent["radius"]:
                        continue
                    force_mag = detent["spring"] * dist
                    inv_dist = 1.0 / dist
                    fx_val += (-dx * inv_dist) * force_mag
                    fy_val += (-dy * inv_dist) * force_mag

            if max_force > 0.0:
                mag = math.sqrt((fx_val * fx_val) + (fy_val * fy_val))
                if mag > max_force and mag > 1e-6:
                    scale = max_force / mag
                    fx_val *= scale
                    fy_val *= scale

            fx[iy, ix] = fx_val
            fy[iy, ix] = fy_val

    return x_vals, y_vals, fx, fy, step


def _sample_surface(x_vals, y_vals, z_vals, x, y):
    if len(x_vals) < 2 or len(y_vals) < 2:
        return float(z_vals[0][0])
    step_x = float(x_vals[1] - x_vals[0])
    step_y = float(y_vals[1] - y_vals[0])
    if step_x <= 0.0 or step_y <= 0.0:
        return float(z_vals[0][0])

    x_idx = (x - x_vals[0]) / step_x
    y_idx = (y - y_vals[0]) / step_y
    x_idx = max(0.0, min(float(len(x_vals) - 1), x_idx))
    y_idx = max(0.0, min(float(len(y_vals) - 1), y_idx))

    x0 = int(math.floor(x_idx))
    y0 = int(math.floor(y_idx))
    x1 = min(x0 + 1, len(x_vals) - 1)
    y1 = min(y0 + 1, len(y_vals) - 1)
    tx = x_idx - x0
    ty = y_idx - y0

    z00 = float(z_vals[y0, x0])
    z10 = float(z_vals[y0, x1])
    z01 = float(z_vals[y1, x0])
    z11 = float(z_vals[y1, x1])
    z0 = z00 + (z10 - z00) * tx
    z1 = z01 + (z11 - z01) * tx
    return z0 + (z1 - z0) * ty


def _overlay_gate_segments(ax, x_vals, y_vals, z_vals, config, z_offset):
    segments = _build_gate_segments(config)
    for seg in segments:
        samples = 60
        t_vals = np.linspace(0.0, 1.0, samples)
        xs = seg["x0"] + (seg["x1"] - seg["x0"]) * t_vals
        ys = seg["y0"] + (seg["y1"] - seg["y0"]) * t_vals
        zs = np.array([_sample_surface(x_vals, y_vals, z_vals, x, y) for x, y in zip(xs, ys)])
        ax.plot(xs, ys, zs + z_offset, color="black", linewidth=1.6, alpha=0.85)


def _overlay_detents(ax, x_vals, y_vals, z_vals, config, z_offset):
    detents = _build_detents(config)
    if not detents:
        return
    xs = []
    ys = []
    zs = []
    for detent in detents:
        xs.append(detent["x"])
        ys.append(detent["y"])
        zs.append(_sample_surface(x_vals, y_vals, z_vals, detent["x"], detent["y"]) + z_offset)
    ax.scatter(xs, ys, zs, color="tab:red", s=18, depthshade=False)


def _overlay_force_vectors(ax, x_vals, y_vals, fx, fy, z_vals, step, vector_step_mm, scale):
    if vector_step_mm <= 0.0:
        return
    stride = max(1, int(round(vector_step_mm / step)))
    xs = x_vals[::stride]
    ys = y_vals[::stride]
    x_grid, y_grid = np.meshgrid(xs, ys)
    fx_grid = fx[::stride, ::stride]
    fy_grid = fy[::stride, ::stride]
    z_grid = z_vals[::stride, ::stride]

    ax.quiver(
        x_grid,
        y_grid,
        z_grid,
        fx_grid * scale,
        fy_grid * scale,
        np.zeros_like(fx_grid),
        length=1.0,
        normalize=False,
        color="white",
        linewidth=0.6,
        arrow_length_ratio=0.2,
    )


def _plot_surface(ax, x_vals, y_vals, z_vals, title):
    x_grid, y_grid = np.meshgrid(x_vals, y_vals)
    surface = ax.plot_surface(
        x_grid,
        y_grid,
        z_vals,
        cmap="viridis",
        linewidth=0,
        antialiased=True,
        rstride=1,
        cstride=1,
    )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Force (N)")
    ax.set_title(title)
    return surface


def main():
    parser = argparse.ArgumentParser(
        description="Plot shifter force fields as 3D surfaces.",
    )
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
        "--component",
        choices=("fx", "fy", "mag", "all"),
        default="mag",
        help="Force component to plot.",
    )
    parser.add_argument(
        "--no-gates",
        action="store_true",
        help="Hide gate segment overlay.",
    )
    parser.add_argument(
        "--no-detents",
        action="store_true",
        help="Hide detent overlay.",
    )
    parser.add_argument(
        "--no-vectors",
        action="store_true",
        help="Hide force vector overlay.",
    )
    parser.add_argument(
        "--vector-step-mm",
        type=float,
        default=10.0,
        help="Grid spacing for force vectors in mm.",
    )
    parser.add_argument(
        "--vector-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to auto-sized force vectors.",
    )
    parser.add_argument(
        "--step-mm",
        type=float,
        default=None,
        help="Override grid step in mm (default uses shifter gridStep).",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=20000,
        help="Maximum grid points before auto-scaling the step size.",
    )
    parser.add_argument(
        "--output",
        help="Output image path (png/svg). If omitted, opens a window.",
    )
    args = parser.parse_args()

    config = _load_shifter_config(args.config_json, args.item_index)
    x_vals, y_vals, fx, fy, step = _compute_force_map(config, args.step_mm, args.max_points)
    magnitude = np.sqrt((fx * fx) + (fy * fy))

    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting. Install it with: pip install matplotlib") from exc

    if args.component == "all":
        fig = plt.figure(figsize=(15, 5))
        axes = [
            fig.add_subplot(1, 3, 1, projection="3d"),
            fig.add_subplot(1, 3, 2, projection="3d"),
            fig.add_subplot(1, 3, 3, projection="3d"),
        ]
        surface_items = [
            (fx, "Fx (N)"),
            (fy, "Fy (N)"),
            (magnitude, "Force magnitude (N)"),
        ]
        surfaces = []
        for ax, (z_vals, label) in zip(axes, surface_items):
            surfaces.append(_plot_surface(ax, x_vals, y_vals, z_vals, label))
        for ax, surface in zip(axes, surfaces):
            fig.colorbar(surface, ax=ax, shrink=0.6, pad=0.08)

        for ax, z_vals in zip(axes, (fx, fy, magnitude)):
            z_range = float(np.nanmax(z_vals) - np.nanmin(z_vals))
            z_offset = max(0.2, 0.02 * z_range)
            if not args.no_gates:
                _overlay_gate_segments(ax, x_vals, y_vals, z_vals, config, z_offset)
            if not args.no_detents:
                _overlay_detents(ax, x_vals, y_vals, z_vals, config, z_offset)
            if not args.no_vectors:
                max_force = float(np.nanmax(np.sqrt((fx * fx) + (fy * fy))))
                range_xy = max(float(x_vals[-1] - x_vals[0]), float(y_vals[-1] - y_vals[0]), 1.0)
                auto_scale = (0.15 * range_xy / max_force) if max_force > 1e-6 else 0.0
                _overlay_force_vectors(
                    ax,
                    x_vals,
                    y_vals,
                    fx,
                    fy,
                    z_vals,
                    step,
                    args.vector_step_mm,
                    auto_scale * args.vector_scale,
                )
    else:
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(1, 1, 1, projection="3d")
        if args.component == "fx":
            z_vals = fx
            surface = _plot_surface(ax, x_vals, y_vals, z_vals, "Fx (N)")
        elif args.component == "fy":
            z_vals = fy
            surface = _plot_surface(ax, x_vals, y_vals, z_vals, "Fy (N)")
        else:
            z_vals = magnitude
            surface = _plot_surface(ax, x_vals, y_vals, z_vals, "Force magnitude (N)")
        fig.colorbar(surface, ax=ax, shrink=0.7, pad=0.1)

        z_range = float(np.nanmax(z_vals) - np.nanmin(z_vals))
        z_offset = max(0.2, 0.02 * z_range)
        if not args.no_gates:
            _overlay_gate_segments(ax, x_vals, y_vals, z_vals, config, z_offset)
        if not args.no_detents:
            _overlay_detents(ax, x_vals, y_vals, z_vals, config, z_offset)
        if not args.no_vectors:
            max_force = float(np.nanmax(np.sqrt((fx * fx) + (fy * fy))))
            range_xy = max(float(x_vals[-1] - x_vals[0]), float(y_vals[-1] - y_vals[0]), 1.0)
            auto_scale = (0.15 * range_xy / max_force) if max_force > 1e-6 else 0.0
            _overlay_force_vectors(
                ax,
                x_vals,
                y_vals,
                fx,
                fy,
                z_vals,
                step,
                args.vector_step_mm,
                auto_scale * args.vector_scale,
            )

    fig.suptitle(f"Grid step: {step:.2f} mm, points: {len(x_vals)} x {len(y_vals)}")
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
