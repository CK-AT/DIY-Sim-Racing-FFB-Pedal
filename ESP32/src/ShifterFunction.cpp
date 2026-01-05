#include "ShifterFunction.h"

#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "CommManager.h"

ShifterFunction::ShifterFunction(void) {
    disable();
    add_element(&_damper);
    add_element(&_centering_spring);
}

void ShifterFunction::update_config(const ShifterConfig &config, const ShifterDetectConfig &detect_config, CommManager &comm_manager, const AxisID *linked_axes) {
    _comm_manager = &comm_manager;
    _axis_role = resolve_axis_role(linked_axes);
    _damper.set_k(config.damping);

    GateSegPre segsPre[sizeof(config.gate_segments) / sizeof(config.gate_segments[0])];
    DetentPre detsPre[sizeof(config.detents) / sizeof(config.detents[0])];
    int segCount = 0;
    int detCount = 0;

    for (segCount = 0; segCount < config.gate_segments_count; segCount++) {
        auto &gs = config.gate_segments[segCount];
        segsPre[segCount] = makeSegPre(gs.x0, gs.y0, gs.x1, gs.y1, gs.half_width);
    }

    for (detCount = 0; detCount < config.detents_count; detCount++) {
        auto &d = config.detents[detCount];
        detsPre[detCount] = makeDetPre(d.x, d.y, d.radius, d.spring);
    }

    gateRt.buildPrecompute(segsPre, segCount, detsPre, detCount,
                           /* global bounds (mm) */
                           config.pos_x_min, config.pos_x_max, config.pos_y_min, config.pos_y_max);

    auto spring_center = computeCenteringAnchorMm(detect_config, segsPre, segCount);

    if (spring_center.valid) {
        _centering_spring.set_offset((_axis_role == AxisRole::X ? spring_center.x_mm : spring_center.y_mm));
    }

    // TODO: max_force is repurposed for now 
    _centering_spring.set_k(config.max_force);

     if (_axis_role == AxisRole::X) {
        _x_min = config.pos_x_min;
        _x_max = config.pos_x_max;
     } else if (_axis_role == AxisRole::Y) {
        _x_min = config.pos_y_min;
        _x_max = config.pos_y_max;
     } else {
        _x_min = 0.0f;
        _x_max = 0.0f;
     }


    // rebuild_map();
    // float sequential_x_center = 0.5f * (float(_config.pos_x_min) + float(_config.pos_x_max));
    // bool use_fixed_x = _config.sequential;
    // _map_force.configure(_comm_manager, &_force_map, _axis_role, _axis_id_x, _axis_id_y, _invert_x, _invert_y, use_fixed_x,
    //                      sequential_x_center);
}

AxisRole ShifterFunction::resolve_axis_role(const AxisID *linked_axes) {
    AxisID self_axis = _comm_manager ? _comm_manager->get_axis_id() : AxisID_AXIS_UNDEFINED;
    AxisID axis_0 = AxisID(linked_axes[0] & AxisID_AXIS_ID_MASK);
    AxisID axis_1 = AxisID(linked_axes[1] & AxisID_AXIS_ID_MASK);
    bool invert_0 = (linked_axes[0] & AxisID_AXIS_SUBTRACTIVE);
    bool invert_1 = (linked_axes[1] & AxisID_AXIS_SUBTRACTIVE);
    _axis_id_x = axis_0;
    _axis_id_y = axis_1;
    _invert_x = invert_0;
    _invert_y = invert_1;
    if (self_axis == _axis_id_x) return AxisRole::X;
    if (self_axis == _axis_id_y) return AxisRole::Y;
    return AxisRole::Unknown;
}

void ShifterFunction::rebuild_map(void) {
    /*
    float step = max(uint32_t(1), static_cast<uint32_t>(_config.grid_step)) * k_tenth_mm;
    step = max(step, k_min_step);
    float x_min = float(_config.pos_x_min);
    float x_max = float(_config.pos_x_max);
    float y_min = float(_config.pos_y_min);
    float y_max = float(_config.pos_y_max);

    uint16_t x_count = count_for_range(x_min, x_max, step);
    uint16_t y_count = count_for_range(y_min, y_max, step);
    uint32_t total = uint32_t(x_count) * uint32_t(y_count);
    if (total > k_max_map_points && total > 0) {
        float scale = sqrtf(float(total) / float(k_max_map_points));
        step = max(k_min_step, step * scale);
        x_count = count_for_range(x_min, x_max, step);
        y_count = count_for_range(y_min, y_max, step);
        total = uint32_t(x_count) * uint32_t(y_count);
    }

    _force_map.x_min = x_min;
    _force_map.y_min = y_min;
    _force_map.step = step;
    _force_map.max_force = _config.max_force;
    _force_map.x_count = x_count;
    _force_map.y_count = y_count;
    _force_map.fx.assign(total, 0.0f);
    _force_map.fy.assign(total, 0.0f);

    if (total == 0) return;

    NeutralGuide neutral = {};
    if (!_config.sequential && _config.gate_segments_count > 0) {
        int horizontal_idx = -1;
        float horizontal_len = 0.0f;
        float horizontal_y = 0.0f;
        float horizontal_half_width = 0.0f;
        float horizontal_spring = 0.0f;
        float vertical_sum_x = 0.0f;
        float vertical_count = 0.0f;

        for (uint32_t seg_idx = 0; seg_idx < _config.gate_segments_count; seg_idx++) {
            const ShifterGateSegment &segment = _config.gate_segments[seg_idx];
            float x0 = to_mm(segment.x0);
            float y0 = to_mm(segment.y0);
            float x1 = to_mm(segment.x1);
            float y1 = to_mm(segment.y1);
            float dx = x1 - x0;
            float dy = y1 - y0;
            float len = sqrtf((dx * dx) + (dy * dy));
            if (len < 1e-6f) {
                continue;
            }
            if (fabsf(dy) <= fabsf(dx)) {
                float mid_y = 0.5f * (y0 + y1);
                if (horizontal_idx < 0 || len > horizontal_len || (len == horizontal_len && fabsf(mid_y) < fabsf(horizontal_y))) {
                    horizontal_idx = static_cast<int>(seg_idx);
                    horizontal_len = len;
                    horizontal_y = mid_y;
                    horizontal_half_width = to_mm_unsigned(segment.half_width);
                    horizontal_spring = segment.spring_center;
                }
            }
            if (fabsf(dx) <= fabsf(dy)) {
                float mid_x = 0.5f * (x0 + x1);
                vertical_sum_x += mid_x;
                vertical_count += 1.0f;
            }
        }

        if (horizontal_idx >= 0 && vertical_count > 0.5f) {
            float avg_x = vertical_sum_x / vertical_count;
            int vertical_idx = -1;
            float vertical_x = 0.0f;
            float vertical_spring = 0.0f;
            float best_dist = 0.0f;
            for (uint32_t seg_idx = 0; seg_idx < _config.gate_segments_count; seg_idx++) {
                const ShifterGateSegment &segment = _config.gate_segments[seg_idx];
                float x0 = to_mm(segment.x0);
                float y0 = to_mm(segment.y0);
                float x1 = to_mm(segment.x1);
                float y1 = to_mm(segment.y1);
                float dx = x1 - x0;
                float dy = y1 - y0;
                if (fabsf(dx) > fabsf(dy)) {
                    continue;
                }
                float mid_x = 0.5f * (x0 + x1);
                float dist = fabsf(mid_x - avg_x);
                if (vertical_idx < 0 || dist < best_dist) {
                    vertical_idx = static_cast<int>(seg_idx);
                    best_dist = dist;
                    vertical_x = mid_x;
                    vertical_spring = segment.spring_center;
                }
            }

            if (vertical_idx >= 0 && horizontal_half_width > k_neutral_band_min) {
                neutral.has_value = true;
                neutral.x_center = vertical_x;
                neutral.y_center = horizontal_y;
                neutral.half_width = horizontal_half_width;
                neutral.spring = 0.5f * (horizontal_spring + vertical_spring);
            }
        }
    }

    for (uint16_t iy = 0; iy < y_count; iy++) {
        float y = y_min + (float(iy) * step);
        for (uint16_t ix = 0; ix < x_count; ix++) {
            float x = x_min + (float(ix) * step);
            float fx = 0.0f;
            float fy = 0.0f;

            if (_config.gate_segments_count > 0) {
                bool has_segment = false;
                float best_metric = 0.0f;
                float best_dist = 0.0f;
                float best_dx = 0.0f;
                float best_dy = 0.0f;
                float best_half_width = 0.0f;
                float best_spring_center = 0.0f;
                float best_spring_wall = 0.0f;
                for (uint32_t seg_idx = 0; seg_idx < _config.gate_segments_count; seg_idx++) {
                    const ShifterGateSegment &segment = _config.gate_segments[seg_idx];
                    float x0 = to_mm(segment.x0);
                    float y0 = to_mm(segment.y0);
                    float x1 = to_mm(segment.x1);
                    float y1 = to_mm(segment.y1);
                    float vx = x1 - x0;
                    float vy = y1 - y0;
                    float len_sq = (vx * vx) + (vy * vy);
                    float t = 0.0f;
                    if (len_sq > 1e-6f) {
                        t = ((x - x0) * vx + (y - y0) * vy) / len_sq;
                        t = clampf(t, 0.0f, 1.0f);
                    }
                    float nx = x0 + (t * vx);
                    float ny = y0 + (t * vy);
                    float dx = x - nx;
                    float dy = y - ny;
                    float dist = sqrtf((dx * dx) + (dy * dy));
                    float half_width = to_mm_unsigned(segment.half_width);
                    float metric = max(0.0f, dist - half_width);
                    if (!has_segment || metric < best_metric || ((metric == best_metric) && (dist < best_dist))) {
                        has_segment = true;
                        best_metric = metric;
                        best_dist = dist;
                        best_dx = dx;
                        best_dy = dy;
                        best_half_width = half_width;
                        best_spring_center = segment.spring_center;
                        best_spring_wall = segment.spring_wall;
                    }
                }
                if (has_segment && best_dist > 1e-6f) {
                    float inv_dist = 1.0f / best_dist;
                    float nx = -best_dx * inv_dist;
                    float ny = -best_dy * inv_dist;
                    float force_mag = 0.0f;
                    if (best_dist <= best_half_width) {
                        force_mag = best_spring_center * best_dist;
                    } else {
                        float outside = best_dist - best_half_width;
                        force_mag = (best_spring_wall * outside) + (best_spring_center * best_half_width);
                    }
                    fx += nx * force_mag;
                    fy += ny * force_mag;
                }
            }

            if (neutral.has_value) {
                float dy = fabsf(y - neutral.y_center);
                if (dy <= neutral.half_width) {
                    float weight = 1.0f - (dy / neutral.half_width);
                    fx += (neutral.x_center - x) * neutral.spring * weight;
                }
            }

            if (_config.detents_count > 0) {
                for (uint32_t detent_idx = 0; detent_idx < _config.detents_count; detent_idx++) {
                    const ShifterDetentPoint &detent = _config.detents[detent_idx];
                    float cx = to_mm(detent.x);
                    float cy = to_mm(detent.y);
                    float radius = to_mm_unsigned(detent.radius);
                    float dx = x - cx;
                    float dy = y - cy;
                    float dist = sqrtf((dx * dx) + (dy * dy));
                    if (dist < 1e-6f || dist > radius) continue;
                    float force_mag = detent.spring * dist;
                    float inv_dist = 1.0f / dist;
                    fx += (-dx * inv_dist) * force_mag;
                    fy += (-dy * inv_dist) * force_mag;
                }
            }

            if (_force_map.max_force > 0.0f) {
                float mag = sqrtf((fx * fx) + (fy * fy));
                if (mag > _force_map.max_force && mag > 1e-6f) {
                    float scale = _force_map.max_force / mag;
                    fx *= scale;
                    fy *= scale;
                }
            }

            uint32_t idx = uint32_t(iy) * x_count + ix;
            _force_map.fx[idx] = fx;
            _force_map.fy[idx] = fy;
        }
    }
    */
}

float ShifterFunction::get_x_contact_point_min(void) {
    return _x_min;
}

float ShifterFunction::get_x_contact_point_max(void) {
    return _x_max;
}

void ShifterFunction::on_ffb_action(const FFBAction &ffb_action) {
}

void ShifterFunction::update(Sim *sim, float &f_sum) {
    if (!_enabled) return;
    float x_pos = 0.0f, y_pos = 0.0f;
    _comm_manager->get_position(_axis_id_x, x_pos);
    _comm_manager->get_position(_axis_id_y, y_pos);

    auto ctx = gateRt.updateAxisContext(x_pos, y_pos, _axis_role);

    // soft limits for integration
    float ySoftMin = ctx.soft.lo;
    float ySoftMax = ctx.soft.hi;

    // active detents for current lane
    auto detSpan = gateRt.detentsForLane(ctx, _axis_role);

    for (uint8_t i = 0; i < detSpan.count; i++) {
        const DetentPre& d = gateRt.dets[ detSpan.indices[i] ];
        float z = (sim->get_x() - (_axis_role == AxisRole::X ? d.x_mm : d.y_mm)) / d.radius_mm;
        if (z <= -1.0f || z >= 1.0f) return;
        f_sum += d.spring_N_per_mm * fastmath::fast_sinf(float(PI) * z);
    }
    
    CompoundElement::update(sim, f_sum);
}
