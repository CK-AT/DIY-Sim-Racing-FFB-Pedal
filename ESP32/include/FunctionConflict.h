#pragma once

#include <stddef.h>
#include <stdint.h>

#include "diy_ffb_protocol.pb.h"

// Helpers for deciding when two functions cannot coexist on the gateway's
// function lookup table. A physical servo axis and a controller (HID) output
// channel each belong to exactly one function at a time, so a newly received
// function config supersedes any prior function it conflicts with. This lets
// the gateway self-heal stale entries (e.g. a flight-pedals function lingering
// after switching to an automotive profile) without the host having to track
// what it previously uploaded.

// Bitmask of the physical axes a function binds. linked_axes is fixed_count:4
// in the protobuf; entries carry flags in the high bits and AXIS_UNDEFINED
// terminates the list, so mask off the flag bits and stop at the terminator.
inline uint32_t function_axis_mask(const FunctionBase &base) {
    uint32_t mask = 0;
    for (size_t i = 0; i < 4; i++) {
        AxisID id = AxisID(base.linked_axes[i] & AxisID_AXIS_ID_MASK);
        if (id == AxisID_AXIS_UNDEFINED) break;
        mask |= (1u << static_cast<uint32_t>(id));
    }
    return mask;
}

// True if functions a and b conflict: they bind a common physical axis, OR they
// drive the same controller output axis. An UNDEFINED controller output axis
// (e.g. a cleared config) never counts as an output conflict.
inline bool functions_conflict(const FunctionBase &a, const FunctionBase &b) {
    if ((function_axis_mask(a) & function_axis_mask(b)) != 0) {
        return true;
    }
    ControllerAxis out_a = a.controller_output_axis;
    ControllerAxis out_b = b.controller_output_axis;
    if (out_a != ControllerAxis_CONTROLLER_AXIS_UNDEFINED && out_a == out_b) {
        return true;
    }
    return false;
}
