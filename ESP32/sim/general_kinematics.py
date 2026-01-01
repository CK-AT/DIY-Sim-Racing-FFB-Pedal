import math
import numpy as np

import diy_ffb_protocol_pb2 as ffb_protocol

_SAMPLE_COUNT = 200
_MAX_ITERATIONS = 100
_TOL = 1e-6
_DAMPING = 1e-6
_EPS = 1e-9
_COLLINEAR_TOL = 1e-4
_DAMPING_MIN = 1e-8
_DAMPING_MAX = 1e3
_MAX_LM_TRIES = 8
_MAX_STEP = 5.0
_MAX_THETA_STEP = 0.2
_COND_LIMIT = 1e8


def calc_kinematic_parameters(config, sample_count=_SAMPLE_COUNT):
    pins, contact_idx, rail_idx, id_to_index = _build_pins(config)
    constraints, bar_lines, pin_bar_index, pin_bar_s, metering_constraint_idx = _build_constraints(
        config, pins, id_to_index
    )
    if metering_constraint_idx < 0:
        raise ValueError("GeneralKinematicConfig requires exactly one 2-pin metering bar.")

    var_index_x, var_index_y, variables, bar_var_base = _build_variable_map(
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
    rail_step = travel_total / (sample_count - 1)

    contact_x = np.zeros(sample_count)
    contact_y = np.zeros(sample_count)
    sled_pos = np.zeros(sample_count)
    positions = []
    rail_offsets = []

    for i in range(sample_count):
        rail_offset = rail_offset_min + rail_step * i
        if not _solve_positions(
            pins,
            constraints,
            rail_idx,
            rail_offset,
            var_index_x,
            var_index_y,
            bar_var_base,
            variables,
            pin_bar_index,
            pin_bar_s,
        ):
            raise RuntimeError("General kinematics solver failed to converge.")

        pos = _fill_positions(
            pins,
            rail_idx,
            rail_offset,
            var_index_x,
            var_index_y,
            bar_var_base,
            variables,
            pin_bar_index,
            pin_bar_s,
        )
        positions.append(pos)
        rail_offsets.append(rail_offset)
        contact_x[i] = pos[contact_idx, 0]
        contact_y[i] = pos[contact_idx, 1]
        sled_pos[i] = rail_offset + travel_negative

    contact_pos = _build_contact_path(contact_x, contact_y)
    center_index = (-rail_offset_min) / rail_step
    center_offset = _interpolate_index(contact_pos, center_index)
    contact_pos = contact_pos - center_offset
    force_factors = np.zeros(sample_count)

    for i in range(sample_count):
        tx, ty = _contact_tangent(contact_x, contact_y, i)
        f_ext = np.zeros(len(variables))
        bar_cos, bar_sin = _bar_trig_from_positions(positions[i], bar_lines)
        _apply_force_on_pin(
            f_ext,
            contact_idx,
            tx,
            ty,
            var_index_x,
            var_index_y,
            pin_bar_index,
            pin_bar_s,
            bar_var_base,
            bar_cos,
            bar_sin,
        )

        j = _build_jacobian(
            constraints,
            positions[i],
            rail_offsets[i],
            pins,
            var_index_x,
            var_index_y,
            pin_bar_index,
            pin_bar_s,
            bar_var_base,
            bar_cos,
            bar_sin,
            len(variables),
        )
        a = j @ j.T + _DAMPING * np.eye(j.shape[0])
        b = -(j @ f_ext)
        try:
            lam = np.linalg.solve(a, b)
        except np.linalg.LinAlgError:
            lam = np.linalg.lstsq(a, b, rcond=None)[0]

        meter = lam[metering_constraint_idx]
        if abs(meter) < _EPS:
            meter = -_EPS if meter < 0.0 else _EPS
        force_factors[i] = 1.0 / meter

    coeffs_sled = _fit_polynomial(contact_pos, sled_pos, 4)
    coeffs_force = _fit_polynomial(contact_pos, force_factors, 4)

    params = ffb_protocol.KinematicParameters()
    params.coeffs_sled_pos_over_contact_point_pos.extend(coeffs_sled.tolist())
    params.coeffs_force_factor_over_contact_point_pos.extend(coeffs_force.tolist())
    params.contact_point_pos_min_abs = int(contact_pos[0] * 10.0)
    params.contact_point_pos_max_abs = int(contact_pos[-1] * 10.0)
    return params


def _build_pins(config):
    pins = []
    id_to_index = {}
    contact_idx = -1
    rail_idx = -1

    for pin in config.pins:
        if pin.pin_id in id_to_index:
            raise ValueError("Duplicate pin_id in GeneralKinematicConfig.")
        info = {
            "id": int(pin.pin_id),
            "x": float(pin.x),
            "y": float(pin.y),
            "grounded": bool(pin.grounded),
            "is_contact": bool(pin.is_contact_point),
            "is_rail": bool(pin.is_rail_interface),
        }
        if info["grounded"] and info["is_rail"]:
            raise ValueError("Rail interface pin cannot be grounded.")
        if info["grounded"] and info["is_contact"]:
            raise ValueError("Contact point pin cannot be grounded.")
        if info["is_contact"] and info["is_rail"]:
            raise ValueError("Contact point pin cannot be the rail interface pin.")

        if info["is_contact"]:
            if contact_idx >= 0:
                raise ValueError("Only one contact point pin is allowed.")
            contact_idx = len(pins)
        if info["is_rail"]:
            if rail_idx >= 0:
                raise ValueError("Only one rail interface pin is allowed.")
            rail_idx = len(pins)

        id_to_index[info["id"]] = len(pins)
        pins.append(info)

    if contact_idx < 0:
        raise ValueError("GeneralKinematicConfig requires exactly one contact point pin.")
    if rail_idx < 0:
        raise ValueError("GeneralKinematicConfig requires exactly one rail interface pin.")

    return pins, contact_idx, rail_idx, id_to_index


def _build_constraints(config, pins, id_to_index):
    constraints = []
    bar_lines = []
    pin_bar_index = [-1] * len(pins)
    pin_bar_s = [0.0] * len(pins)
    metering_constraint_idx = -1
    metering_found = False

    for bar in config.bars:
        unique_ids = list(dict.fromkeys(bar.pin_ids))
        if len(unique_ids) < 2:
            raise ValueError("Each bar must reference at least two pins.")
        if bar.is_metering:
            if metering_found:
                raise ValueError("Only one metering bar is allowed.")
            if len(unique_ids) != 2:
                raise ValueError("Metering bar must have exactly two pins.")
            metering_found = True

        if len(unique_ids) == 2:
            pin_a, pin_b = unique_ids
            if pin_a not in id_to_index or pin_b not in id_to_index:
                raise ValueError("Bar references unknown pin id.")
            idx_a = id_to_index[pin_a]
            idx_b = id_to_index[pin_b]
            dx = pins[idx_a]["x"] - pins[idx_b]["x"]
            dy = pins[idx_a]["y"] - pins[idx_b]["y"]
            length = float(np.hypot(dx, dy))
            if length < _EPS:
                raise ValueError("Bar length must be > 0.")
            constraints.append({"type": "distance", "a": idx_a, "b": idx_b, "length": length})
            if bar.is_metering:
                metering_constraint_idx = len(constraints) - 1
            continue

        if bar.is_metering:
            raise ValueError("Metering bar must have exactly two pins.")

        pin_indices = []
        for pin_id in unique_ids:
            if pin_id not in id_to_index:
                raise ValueError("Bar references unknown pin id.")
            idx = id_to_index[pin_id]
            if pin_bar_index[idx] >= 0:
                raise ValueError("Pin participates in multiple collinear bars.")
            pin_indices.append(idx)

        ref_idx = pin_indices[0]
        axis_idx = pin_indices[1]
        dx = pins[axis_idx]["x"] - pins[ref_idx]["x"]
        dy = pins[axis_idx]["y"] - pins[ref_idx]["y"]
        axis_len = float(np.hypot(dx, dy))
        if axis_len < _EPS:
            raise ValueError("Bar length must be > 0.")
        dir_x = dx / axis_len
        dir_y = dy / axis_len

        for idx in pin_indices:
            px = pins[idx]["x"] - pins[ref_idx]["x"]
            py = pins[idx]["y"] - pins[ref_idx]["y"]
            s = px * dir_x + py * dir_y
            perp = px * dir_y - py * dir_x
            if abs(perp) > _COLLINEAR_TOL * axis_len:
                raise ValueError("Collinear bar pins must lie on the same line.")
            pin_bar_index[idx] = len(bar_lines)
            pin_bar_s[idx] = s

        bar_lines.append(
            {
                "pin_indices": pin_indices,
                "ref_pin": ref_idx,
                "axis_pin": axis_idx,
                "dir_x": dir_x,
                "dir_y": dir_y,
            }
        )

    for idx, pin in enumerate(pins):
        if pin_bar_index[idx] < 0:
            continue
        if pin["grounded"] or pin["is_rail"]:
            constraints.append({"type": "fix", "pin": idx, "axis": 0})
            constraints.append({"type": "fix", "pin": idx, "axis": 1})

    return constraints, bar_lines, pin_bar_index, pin_bar_s, metering_constraint_idx


def _build_variable_map(pins, rail_idx, contact_idx, bar_lines, pin_bar_index):
    var_index_x = [-1] * len(pins)
    var_index_y = [-1] * len(pins)
    variables = []

    for i, pin in enumerate(pins):
        if pin_bar_index[i] >= 0:
            continue
        if i == rail_idx or pin["grounded"]:
            continue
        var_index_x[i] = len(variables)
        var_index_y[i] = len(variables) + 1
        variables.extend([pin["x"], pin["y"]])

    bar_var_base = []
    for bar in bar_lines:
        base = len(variables)
        bar_var_base.append(base)
        ref_idx = bar["ref_pin"]
        theta = math.atan2(bar["dir_y"], bar["dir_x"])
        variables.extend([pins[ref_idx]["x"], pins[ref_idx]["y"], theta])

    return var_index_x, var_index_y, np.array(variables), bar_var_base


def _bar_pose_from_variables(variables, bar_var_base):
    bar_count = len(bar_var_base)
    bar_x0 = np.zeros(bar_count)
    bar_y0 = np.zeros(bar_count)
    bar_cos = np.zeros(bar_count)
    bar_sin = np.zeros(bar_count)
    for i, base in enumerate(bar_var_base):
        bar_x0[i] = variables[base]
        bar_y0[i] = variables[base + 1]
        theta = variables[base + 2]
        bar_cos[i] = math.cos(theta)
        bar_sin[i] = math.sin(theta)
    return bar_x0, bar_y0, bar_cos, bar_sin


def _bar_trig_from_positions(pos, bar_lines):
    bar_cos = np.zeros(len(bar_lines))
    bar_sin = np.zeros(len(bar_lines))
    for i, bar in enumerate(bar_lines):
        ref_idx = bar["ref_pin"]
        axis_idx = bar["axis_pin"]
        dx = pos[axis_idx, 0] - pos[ref_idx, 0]
        dy = pos[axis_idx, 1] - pos[ref_idx, 1]
        dist = float(np.hypot(dx, dy))
        if dist < _EPS:
            dist = _EPS
        bar_cos[i] = dx / dist
        bar_sin[i] = dy / dist
    return bar_cos, bar_sin


def _fill_positions_cached(
    pins,
    rail_idx,
    rail_offset,
    var_index_x,
    var_index_y,
    variables,
    pin_bar_index,
    pin_bar_s,
    bar_x0,
    bar_y0,
    bar_cos,
    bar_sin,
):
    pos = np.zeros((len(pins), 2))
    for i, pin in enumerate(pins):
        bar_idx = pin_bar_index[i]
        if bar_idx >= 0:
            s = pin_bar_s[i]
            pos[i, 0] = bar_x0[bar_idx] + s * bar_cos[bar_idx]
            pos[i, 1] = bar_y0[bar_idx] + s * bar_sin[bar_idx]
        elif i == rail_idx:
            pos[i, 0] = pin["x"] + rail_offset
            pos[i, 1] = pin["y"]
        elif pin["grounded"]:
            pos[i, 0] = pin["x"]
            pos[i, 1] = pin["y"]
        else:
            pos[i, 0] = variables[var_index_x[i]]
            pos[i, 1] = variables[var_index_y[i]]
    return pos


def _fill_positions(
    pins,
    rail_idx,
    rail_offset,
    var_index_x,
    var_index_y,
    bar_var_base,
    variables,
    pin_bar_index,
    pin_bar_s,
):
    bar_x0, bar_y0, bar_cos, bar_sin = _bar_pose_from_variables(variables, bar_var_base)
    return _fill_positions_cached(
        pins,
        rail_idx,
        rail_offset,
        var_index_x,
        var_index_y,
        variables,
        pin_bar_index,
        pin_bar_s,
        bar_x0,
        bar_y0,
        bar_cos,
        bar_sin,
    )


def _accumulate_pin_jacobian(
    row,
    pin_idx,
    weight_x,
    weight_y,
    var_index_x,
    var_index_y,
    pin_bar_index,
    pin_bar_s,
    bar_var_base,
    bar_cos,
    bar_sin,
):
    bar_idx = pin_bar_index[pin_idx]
    if bar_idx >= 0:
        base = bar_var_base[bar_idx]
        row[base] += weight_x
        row[base + 1] += weight_y
        s = pin_bar_s[pin_idx]
        row[base + 2] += weight_x * (-s * bar_sin[bar_idx]) + weight_y * (s * bar_cos[bar_idx])
        return

    ix = var_index_x[pin_idx]
    iy = var_index_y[pin_idx]
    if ix >= 0:
        row[ix] += weight_x
    if iy >= 0:
        row[iy] += weight_y


def _apply_force_on_pin(
    f_ext,
    pin_idx,
    fx,
    fy,
    var_index_x,
    var_index_y,
    pin_bar_index,
    pin_bar_s,
    bar_var_base,
    bar_cos,
    bar_sin,
):
    bar_idx = pin_bar_index[pin_idx]
    if bar_idx >= 0:
        base = bar_var_base[bar_idx]
        f_ext[base] += fx
        f_ext[base + 1] += fy
        s = pin_bar_s[pin_idx]
        f_ext[base + 2] += fx * (-s * bar_sin[bar_idx]) + fy * (s * bar_cos[bar_idx])
        return

    ix = var_index_x[pin_idx]
    iy = var_index_y[pin_idx]
    if ix >= 0:
        f_ext[ix] += fx
    if iy >= 0:
        f_ext[iy] += fy


def _build_residuals_and_jacobian(
    constraints,
    pins,
    pos,
    rail_offset,
    var_index_x,
    var_index_y,
    pin_bar_index,
    pin_bar_s,
    bar_var_base,
    bar_cos,
    bar_sin,
    var_count,
):
    res = np.zeros(len(constraints))
    j = np.zeros((len(constraints), var_count))
    for ci, constraint in enumerate(constraints):
        if constraint["type"] == "distance":
            idx_a = constraint["a"]
            idx_b = constraint["b"]
            length = constraint["length"]
            dx = pos[idx_a, 0] - pos[idx_b, 0]
            dy = pos[idx_a, 1] - pos[idx_b, 1]
            dist = float(np.hypot(dx, dy))
            if dist < _EPS:
                dist = _EPS
            res[ci] = dist - length
            ux = dx / dist
            uy = dy / dist
            _accumulate_pin_jacobian(
                j[ci],
                idx_a,
                ux,
                uy,
                var_index_x,
                var_index_y,
                pin_bar_index,
                pin_bar_s,
                bar_var_base,
                bar_cos,
                bar_sin,
            )
            _accumulate_pin_jacobian(
                j[ci],
                idx_b,
                -ux,
                -uy,
                var_index_x,
                var_index_y,
                pin_bar_index,
                pin_bar_s,
                bar_var_base,
                bar_cos,
                bar_sin,
            )
        else:
            pin_idx = constraint["pin"]
            axis = constraint["axis"]
            if axis == 0:
                res[ci] = pos[pin_idx, 0] - (
                    pins[pin_idx]["x"] + (rail_offset if pins[pin_idx]["is_rail"] else 0.0)
                )
                weight_x = 1.0
                weight_y = 0.0
            else:
                res[ci] = pos[pin_idx, 1] - pins[pin_idx]["y"]
                weight_x = 0.0
                weight_y = 1.0
            _accumulate_pin_jacobian(
                j[ci],
                pin_idx,
                weight_x,
                weight_y,
                var_index_x,
                var_index_y,
                pin_bar_index,
                pin_bar_s,
                bar_var_base,
                bar_cos,
                bar_sin,
            )
    return res, j


def _solve_positions(
    pins,
    constraints,
    rail_idx,
    rail_offset,
    var_index_x,
    var_index_y,
    bar_var_base,
    variables,
    pin_bar_index,
    pin_bar_s,
):
    var_count = len(variables)
    if var_count == 0:
        bar_x0, bar_y0, bar_cos, bar_sin = _bar_pose_from_variables(variables, bar_var_base)
        pos = _fill_positions_cached(
            pins,
            rail_idx,
            rail_offset,
            var_index_x,
            var_index_y,
            variables,
            pin_bar_index,
            pin_bar_s,
            bar_x0,
            bar_y0,
            bar_cos,
            bar_sin,
        )
        res, _ = _build_residuals_and_jacobian(
            constraints,
            pins,
            pos,
            rail_offset,
            var_index_x,
            var_index_y,
            pin_bar_index,
            pin_bar_s,
            bar_var_base,
            bar_cos,
            bar_sin,
            var_count,
        )
        return np.linalg.norm(res) < _TOL

    damping = _DAMPING
    step_limits = _build_step_limits(var_count, bar_var_base)

    for _ in range(_MAX_ITERATIONS):
        bar_x0, bar_y0, bar_cos, bar_sin = _bar_pose_from_variables(variables, bar_var_base)
        pos = _fill_positions_cached(
            pins,
            rail_idx,
            rail_offset,
            var_index_x,
            var_index_y,
            variables,
            pin_bar_index,
            pin_bar_s,
            bar_x0,
            bar_y0,
            bar_cos,
            bar_sin,
        )
        res, j = _build_residuals_and_jacobian(
            constraints,
            pins,
            pos,
            rail_offset,
            var_index_x,
            var_index_y,
            pin_bar_index,
            pin_bar_s,
            bar_var_base,
            bar_cos,
            bar_sin,
            var_count,
        )

        res_norm = np.linalg.norm(res)
        if res_norm < _TOL:
            return True

        cond = _estimate_condition(j)
        local_damping = damping
        if cond > _COND_LIMIT:
            scale = min(cond / _COND_LIMIT, 1e4)
            local_damping = max(local_damping, _DAMPING * scale)

        accepted = False
        last_delta = np.zeros_like(variables)
        for _ in range(_MAX_LM_TRIES):
            a = j.T @ j + local_damping * np.eye(var_count)
            b = -(j.T @ res)
            try:
                delta = np.linalg.solve(a, b)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(a, b, rcond=None)[0]
            delta = _apply_step_limits(delta, step_limits)
            last_delta = delta

            trial_vars = variables + delta
            t_bar_x0, t_bar_y0, t_bar_cos, t_bar_sin = _bar_pose_from_variables(trial_vars, bar_var_base)
            t_pos = _fill_positions_cached(
                pins,
                rail_idx,
                rail_offset,
                var_index_x,
                var_index_y,
                trial_vars,
                pin_bar_index,
                pin_bar_s,
                t_bar_x0,
                t_bar_y0,
                t_bar_cos,
                t_bar_sin,
            )
            t_res, _ = _build_residuals_and_jacobian(
                constraints,
                pins,
                t_pos,
                rail_offset,
                var_index_x,
                var_index_y,
                pin_bar_index,
                pin_bar_s,
                bar_var_base,
                t_bar_cos,
                t_bar_sin,
                var_count,
            )
            if np.linalg.norm(t_res) <= res_norm:
                variables[:] = trial_vars
                damping = max(_DAMPING_MIN, local_damping / 10.0)
                accepted = True
                break

            local_damping = min(_DAMPING_MAX, local_damping * 10.0)
            if np.linalg.norm(delta) < _TOL:
                break

        if not accepted:
            return False

        if np.linalg.norm(last_delta) < _TOL:
            return True

    return False


def _build_jacobian(
    constraints,
    pos,
    rail_offset,
    pins,
    var_index_x,
    var_index_y,
    pin_bar_index,
    pin_bar_s,
    bar_var_base,
    bar_cos,
    bar_sin,
    var_count,
):
    _, j = _build_residuals_and_jacobian(
        constraints,
        pins,
        pos,
        rail_offset,
        var_index_x,
        var_index_y,
        pin_bar_index,
        pin_bar_s,
        bar_var_base,
        bar_cos,
        bar_sin,
        var_count,
    )
    return j


def _build_step_limits(var_count, bar_var_base):
    limits = np.full(var_count, _MAX_STEP, dtype=float)
    for base in bar_var_base:
        if base + 2 < var_count:
            limits[base + 2] = _MAX_THETA_STEP
    return limits


def _apply_step_limits(delta, limits):
    if delta.size == 0:
        return delta
    return np.clip(delta, -limits, limits)


def _estimate_condition(j):
    if j.size == 0:
        return 0.0
    try:
        return float(np.linalg.cond(j))
    except np.linalg.LinAlgError:
        return float("inf")


def _build_contact_path(contact_x, contact_y):
    path = np.zeros_like(contact_x)
    for i in range(1, len(contact_x)):
        dx = contact_x[i] - contact_x[i - 1]
        dy = contact_y[i] - contact_y[i - 1]
        path[i] = path[i - 1] + np.hypot(dx, dy)
    return path


def _contact_tangent(contact_x, contact_y, index):
    last = len(contact_x) - 1
    if index <= 0:
        tx = contact_x[1] - contact_x[0]
        ty = contact_y[1] - contact_y[0]
    elif index >= last:
        tx = contact_x[last] - contact_x[last - 1]
        ty = contact_y[last] - contact_y[last - 1]
    else:
        tx = contact_x[index + 1] - contact_x[index - 1]
        ty = contact_y[index + 1] - contact_y[index - 1]

    length = float(np.hypot(tx, ty))
    if length < _EPS:
        return 1.0, 0.0
    return tx / length, ty / length


def _interpolate_index(values, index):
    if len(values) == 0:
        return 0.0
    if index <= 0.0:
        return float(values[0])
    last = len(values) - 1
    if index >= last:
        return float(values[last])
    low = int(np.floor(index))
    high = low + 1
    t = index - low
    return float(values[low] + (values[high] - values[low]) * t)


def _fit_polynomial(x, y, order):
    poly = np.polynomial.Polynomial.fit(x, y, order)
    return poly.convert().coef
