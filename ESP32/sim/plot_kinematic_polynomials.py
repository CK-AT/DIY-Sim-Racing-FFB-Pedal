import argparse
import json

import numpy as np

import diy_ffb_protocol_pb2 as ffb_protocol
import general_kinematics


def _build_sample_config(travel):
    config = ffb_protocol.GeneralKinematicConfig()
    config.rail_travel_negative = travel / 2.0
    config.rail_travel_positive = travel / 2.0

    config.pins.add(pin_id=1, x=0.0, y=0.0, grounded=True)
    config.pins.add(pin_id=2, x=100.0, y=0.0, is_rail_interface=True)
    config.pins.add(pin_id=3, x=50.0, y=50.0, is_contact_point=True)

    metering = config.bars.add()
    metering.pin_ids.extend([2, 3])
    metering.is_metering = True

    link = config.bars.add()
    link.pin_ids.extend([1, 3])
    return config


def _build_config_from_json(data):
    config = ffb_protocol.GeneralKinematicConfig()
    if "rail_travel_negative" not in data or "rail_travel_positive" not in data:
        raise ValueError("JSON config requires rail_travel_negative and rail_travel_positive.")
    config.rail_travel_negative = float(data["rail_travel_negative"])
    config.rail_travel_positive = float(data["rail_travel_positive"])

    for pin in data.get("pins", []):
        config.pins.add(
            pin_id=int(pin["pin_id"]),
            x=float(pin["x"]),
            y=float(pin["y"]),
            grounded=bool(pin.get("grounded", False)),
            is_contact_point=bool(pin.get("is_contact_point", False)),
            is_rail_interface=bool(pin.get("is_rail_interface", False)),
        )

    for bar in data.get("bars", []):
        entry = config.bars.add()
        entry.pin_ids.extend([int(pin_id) for pin_id in bar["pin_ids"]])
        entry.is_metering = bool(bar.get("is_metering", False))

    return config


def _params_from_json(data):
    params = ffb_protocol.KinematicParameters()
    params.contact_point_pos_min_abs = int(data["contact_point_pos_min_abs"])
    params.contact_point_pos_max_abs = int(data["contact_point_pos_max_abs"])
    params.coeffs_sled_pos_over_contact_point_pos.extend(data["coeffs_sled_pos_over_contact_point_pos"])
    params.coeffs_force_factor_over_contact_point_pos.extend(data["coeffs_force_factor_over_contact_point_pos"])
    return params


def _polyval(coeffs, x):
    return np.polynomial.polynomial.polyval(x, coeffs)


def _compute_layout_frames(config, frames):
    if frames < 2:
        raise ValueError("Animation requires at least 2 frames.")

    pins, contact_idx, rail_idx, id_to_index = general_kinematics._build_pins(config)
    (
        constraints,
        bar_lines,
        pin_bar_index,
        pin_bar_local_x,
        pin_bar_local_y,
        metering_idx,
    ) = general_kinematics._build_constraints(config, pins, id_to_index)
    var_index_x, var_index_y, variables, bar_var_base = general_kinematics._build_variable_map(
        pins, rail_idx, contact_idx, bar_lines, pin_bar_index
    )

    travel_negative = float(config.rail_travel_negative)
    travel_positive = float(config.rail_travel_positive)
    if travel_negative < 0.0 or travel_positive < 0.0:
        raise ValueError("GeneralKinematicConfig.rail_travel_negative/rail_travel_positive must be >= 0.")
    travel_total = travel_negative + travel_positive
    if travel_total <= 0.0:
        raise ValueError("GeneralKinematicConfig.rail_travel_negative/rail_travel_positive must sum to > 0.")

    rail_offset_min = -travel_negative
    rail_step = travel_total / (frames - 1)

    positions = []
    for i in range(frames):
        rail_offset = rail_offset_min + rail_step * i
        if not general_kinematics._solve_positions(
            pins,
            constraints,
            rail_idx,
            rail_offset,
            var_index_x,
            var_index_y,
            bar_var_base,
            variables,
            pin_bar_index,
            pin_bar_local_x,
            pin_bar_local_y,
        ):
            raise RuntimeError("General kinematics solver failed to converge for animation.")
        positions.append(
            general_kinematics._fill_positions(
                pins,
                rail_idx,
                rail_offset,
                var_index_x,
                var_index_y,
                bar_var_base,
                variables,
                pin_bar_index,
                pin_bar_local_x,
                pin_bar_local_y,
            )
        )

    segments = []
    for idx, constraint in enumerate(constraints):
        if constraint["type"] != "distance":
            continue
        segments.append((constraint["a"], constraint["b"], idx == metering_idx))
    for bar in bar_lines:
        pin_indices = bar["pin_indices"]
        for i in range(1, len(pin_indices)):
            segments.append((pin_indices[i - 1], pin_indices[i], False))

    return pins, positions, segments, rail_idx, contact_idx, rail_offset_min, rail_step


def _setup_layout_animation(config, frames, interval_ms):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting. Install it with: pip install matplotlib") from exc

    pins, positions, segments, rail_idx, contact_idx, rail_offset_min, rail_step = _compute_layout_frames(config, frames)
    fig, ax = plt.subplots(figsize=(7, 7))

    rail_y = pins[rail_idx]["y"]
    rail_x_min = pins[rail_idx]["x"] + rail_offset_min
    rail_x_max = pins[rail_idx]["x"] + rail_offset_min + rail_step * (frames - 1)
    ax.plot([rail_x_min, rail_x_max], [rail_y, rail_y], color="gray", linestyle="--", linewidth=1, label="rail")

    line_objs = []
    for _, _, is_metering in segments:
        color = "tab:red" if is_metering else "tab:blue"
        width = 2.5 if is_metering else 1.5
        label = "metering bar" if is_metering else ""
        line, = ax.plot([], [], color=color, linewidth=width, label=label)
        line_objs.append(line)

    ground_idx = [i for i, pin in enumerate(pins) if pin["grounded"]]
    other_idx = [i for i in range(len(pins)) if i not in ground_idx and i not in (contact_idx, rail_idx)]

    scatter_ground = ax.scatter([], [], s=40, c="black", marker="s", label="ground")
    scatter_other = ax.scatter([], [], s=35, c="tab:blue", label="pins")
    scatter_contact = ax.scatter([], [], s=60, c="tab:red", label="contact")
    scatter_rail = ax.scatter([], [], s=60, c="tab:orange", label="rail")

    positions_stack = np.vstack(positions)
    x_min = float(positions_stack[:, 0].min())
    x_max = float(positions_stack[:, 0].max())
    y_min = float(positions_stack[:, 1].min())
    y_max = float(positions_stack[:, 1].max())
    pad = 0.1 * max(x_max - x_min, y_max - y_min, 1.0)
    ax.set_xlim(x_min - pad, x_max + pad)
    ax.set_ylim(y_min - pad, y_max + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax.legend(loc="upper right")

    title = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    def _scatter_offsets(pos, indices):
        if not indices:
            return np.empty((0, 2))
        return pos[np.array(indices)]

    def update(frame_idx):
        pos = positions[frame_idx]
        for line, (pin_a, pin_b, _) in zip(line_objs, segments):
            line.set_data([pos[pin_a, 0], pos[pin_b, 0]], [pos[pin_a, 1], pos[pin_b, 1]])

        scatter_ground.set_offsets(_scatter_offsets(pos, ground_idx))
        scatter_other.set_offsets(_scatter_offsets(pos, other_idx))
        scatter_contact.set_offsets(pos[[contact_idx]])
        scatter_rail.set_offsets(pos[[rail_idx]])

        rail_offset = rail_offset_min + rail_step * frame_idx
        title.set_text(f"Rail offset: {rail_offset:.2f} mm")
        return line_objs + [scatter_ground, scatter_other, scatter_contact, scatter_rail, title]

    animation = FuncAnimation(fig, update, frames=frames, interval=interval_ms, blit=False)
    return fig, animation


def _plot(params, points):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting. Install it with: pip install matplotlib") from exc

    x_min = params.contact_point_pos_min_abs / 10.0
    x_max = params.contact_point_pos_max_abs / 10.0
    x_vals = np.linspace(x_min, x_max, points)

    sled = _polyval(params.coeffs_sled_pos_over_contact_point_pos, x_vals)
    force = _polyval(params.coeffs_force_factor_over_contact_point_pos, x_vals)

    fig, (ax_sled, ax_force) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    ax_sled.plot(x_vals, sled, label="sled_pos_over_contact_pos")
    ax_sled.axvline(0.0, color="gray", linestyle="--", linewidth=1)
    ax_sled.set_ylabel("Sled position (mm)")
    ax_sled.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax_sled.legend()

    ax_force.plot(x_vals, force, label="force_factor_over_contact_pos")
    ax_force.axvline(0.0, color="gray", linestyle="--", linewidth=1)
    ax_force.set_xlabel("Contact position (mm)")
    ax_force.set_ylabel("Force conversion factor")
    ax_force.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    ax_force.legend()

    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot kinematic conversion polynomials.")
    parser.add_argument("--config-json", help="Path to GeneralKinematicConfig JSON.")
    parser.add_argument("--params-json", help="Path to KinematicParameters JSON.")
    parser.add_argument("--points", type=int, default=400, help="Number of points for the plot.")
    parser.add_argument("--output", help="Output image file (png/svg). If omitted, shows a window.")
    parser.add_argument("--animate", action="store_true", help="Show animated kinematic layout.")
    parser.add_argument("--layout-frames", type=int, default=120, help="Number of frames for layout animation.")
    parser.add_argument("--layout-interval", type=int, default=60, help="Frame interval in milliseconds.")
    args = parser.parse_args()

    if args.params_json and args.config_json:
        raise SystemExit("Use only one of --params-json or --config-json.")

    config = None
    if args.params_json:
        with open(args.params_json, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        params = _params_from_json(data)
    elif args.config_json:
        with open(args.config_json, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        config = _build_config_from_json(data)
        params = general_kinematics.calc_kinematic_parameters(config)
    else:
        config = _build_sample_config(40.0)
        params = general_kinematics.calc_kinematic_parameters(config)

    if args.animate and config is None:
        raise SystemExit("Animation requires a GeneralKinematicConfig; use --config-json or omit --params-json.")

    fig = _plot(params, args.points)
    if args.output:
        fig.savefig(args.output, dpi=150)

    animation = None
    if args.animate:
        _, animation = _setup_layout_animation(config, args.layout_frames, args.layout_interval)

    if args.animate or args.output is None:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
