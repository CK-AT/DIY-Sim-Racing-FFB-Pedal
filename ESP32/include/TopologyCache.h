#pragma once

#include <stddef.h>
#include <stdint.h>

#include "diy_ffb_protocol.pb.h"

// Precomputed view of FunctionBase.linked_axes from the perspective of a
// single axis. Replaces the per-tick walks previously done in
// CommManager::calc_input_force_sum / calc_final_position. Populated on
// config update, read by the FFB hot path.

enum PositionMode {
    POSITION_MODE_USE_OWN,                // independent or primary
    POSITION_MODE_FETCH_PRIMARY,          // linked-additive
    POSITION_MODE_FETCH_PRIMARY_MIRRORED, // linked-subtractive
    POSITION_MODE_NOT_MEMBER,             // own axis not present in linked_axes
};

struct ForceFetchEntry {
    AxisID axis_id;
    float sign;
};

// linked_axes is fixed_count:4 in the protobuf options, so the fetch list
// can never exceed 4 entries.
constexpr size_t TOPOLOGY_MAX_FETCH = 4;

inline void compute_topology(
    AxisID own_axis_id,
    const AxisID *linked_axes,
    size_t n,
    PositionMode &out_mode,
    AxisID &out_primary_axis_id,
    ForceFetchEntry *out_force_fetch,
    uint8_t &out_force_fetch_count
) {
    out_mode = POSITION_MODE_USE_OWN;
    out_primary_axis_id = AxisID_AXIS_UNDEFINED;
    out_force_fetch_count = 0;

    // Step 1: is own axis flagged INDEPENDENT in any entry?
    bool own_independent = false;
    for (size_t i = 0; i < n; i++) {
        AxisID id = AxisID(linked_axes[i] & AxisID_AXIS_ID_MASK);
        if (id == AxisID_AXIS_UNDEFINED) break;
        if (id == own_axis_id && (linked_axes[i] & AxisID_AXIS_INDEPENDENT)) {
            own_independent = true;
            break;
        }
    }

    // Step 2: position mode (matches calc_final_position decision tree).
    AxisID primary_id = AxisID(linked_axes[0] & AxisID_AXIS_ID_MASK);
    bool primary_independent = (linked_axes[0] & AxisID_AXIS_INDEPENDENT) != 0;

    if (own_independent || primary_independent || primary_id == own_axis_id) {
        out_mode = POSITION_MODE_USE_OWN;
        out_primary_axis_id = AxisID_AXIS_UNDEFINED;
    } else {
        // Default to NOT_MEMBER; promoted to FETCH_PRIMARY[_MIRRORED] when
        // own_axis_id is found in entries 1..n-1.
        out_mode = POSITION_MODE_NOT_MEMBER;
        out_primary_axis_id = primary_id;
        for (size_t i = 1; i < n; i++) {
            AxisID id = AxisID(linked_axes[i] & AxisID_AXIS_ID_MASK);
            if (id == AxisID_AXIS_UNDEFINED) break;
            if (id == own_axis_id) {
                if (linked_axes[i] & AxisID_AXIS_SUBTRACTIVE) {
                    out_mode = POSITION_MODE_FETCH_PRIMARY_MIRRORED;
                } else {
                    out_mode = POSITION_MODE_FETCH_PRIMARY;
                }
                break;
            }
        }
    }

    // Step 3: force fetch list (matches calc_input_force_sum decision tree).
    // Same order as the original walk so floating-point summation is
    // bit-identical.
    for (size_t i = 0; i < n; i++) {
        AxisID id = AxisID(linked_axes[i] & AxisID_AXIS_ID_MASK);
        if (id == AxisID_AXIS_UNDEFINED) break;
        bool entry_independent = (linked_axes[i] & AxisID_AXIS_INDEPENDENT) != 0;
        if (own_independent) {
            if (id != own_axis_id) continue;
        } else if (entry_independent && id != own_axis_id) {
            continue;
        }
        if (out_force_fetch_count < TOPOLOGY_MAX_FETCH) {
            out_force_fetch[out_force_fetch_count].axis_id = id;
            out_force_fetch[out_force_fetch_count].sign =
                (linked_axes[i] & AxisID_AXIS_SUBTRACTIVE) ? -1.0f : +1.0f;
            out_force_fetch_count++;
        }
    }
}
