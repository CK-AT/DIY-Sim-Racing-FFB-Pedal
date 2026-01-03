#include "ShifterDetect.h"

#include <math.h>

#include "CommManager.h"

namespace {
constexpr float k_tenth_mm = 0.1f;
constexpr ShifterGear kGearNeutral = ShifterGear_SHIFTER_GEAR_NEUTRAL;
constexpr ShifterGear kGearMax = ShifterGear_SHIFTER_GEAR_7;
constexpr ShifterGear kGearReverse = ShifterGear_SHIFTER_GEAR_REVERSE;
constexpr ShifterGear kGearSeqUp = ShifterGear_SHIFTER_GEAR_SEQUENTIAL_UP;
constexpr ShifterGear kGearSeqDown = ShifterGear_SHIFTER_GEAR_SEQUENTIAL_DOWN;

float from_tenth_mm(int32_t value) {
    return static_cast<float>(value) * k_tenth_mm;
}

float from_tenth_mm_unsigned(uint32_t value) {
    return static_cast<float>(value) * k_tenth_mm;
}

bool is_gear_slot(ShifterGear gear) {
    uint32_t gear_value = static_cast<uint32_t>(gear);
    return gear_value == static_cast<uint32_t>(kGearNeutral) ||
        (gear_value >= static_cast<uint32_t>(ShifterGear_SHIFTER_GEAR_1) &&
         gear_value <= static_cast<uint32_t>(kGearMax)) ||
        gear_value == static_cast<uint32_t>(kGearReverse);
}

bool map_gear_to_button(ShifterGear gear, uint8_t &button_index) {
    uint32_t gear_value = static_cast<uint32_t>(gear);
    if (gear_value <= static_cast<uint32_t>(kGearSeqDown)) {
        button_index = static_cast<uint8_t>(gear_value);
        return true;
    }
    return false;
}
}  // namespace

void ShifterDetect::process(CommManager &comm_manager, const AuxFunctionConfig &config) {
    if (config.which_specific != AuxFunctionConfig_shifter_detect_tag) return;
    AxisID axis_x = AxisID_AXIS_UNDEFINED;
    AxisID axis_y = AxisID_AXIS_UNDEFINED;
    bool invert_x = false;
    bool invert_y = false;
    axis_x = AxisID(config.linked_axes[0] & AxisID_AXIS_ID_MASK);
    axis_y = AxisID(config.linked_axes[1] & AxisID_AXIS_ID_MASK);
    invert_x = (config.linked_axes[0] & AxisID_AXIS_SUBTRACTIVE);
    invert_y = (config.linked_axes[1] & AxisID_AXIS_SUBTRACTIVE);
    if (axis_x == AxisID_AXIS_UNDEFINED || axis_y == AxisID_AXIS_UNDEFINED) {
        _active_slot_index = -1;
        return;
    }

    float x_pos = 0.0f;
    float y_pos = 0.0f;
    if (!comm_manager.get_position(axis_x, x_pos) || !comm_manager.get_position(axis_y, y_pos)) {
        _active_slot_index = -1;
        return;
    }
    if (invert_x) {
        x_pos = -x_pos;
    }
    if (invert_y) {
        y_pos = -y_pos;
    }

    const ShifterDetectConfig &detect_cfg = config.specific.shifter_detect;
    if (detect_cfg.gear_slots_count == 0) {
        _active_slot_index = -1;
        return;
    }
    bool has_gear_slots = false;

    float hysteresis = from_tenth_mm_unsigned(detect_cfg.hysteresis);
    auto is_inside = [&](const ShifterGearSlot &slot, float extra) {
        float center_x = from_tenth_mm(slot.center_x);
        float center_y = from_tenth_mm(slot.center_y);
        float half_width = from_tenth_mm_unsigned(slot.half_width) + extra;
        float half_height = from_tenth_mm_unsigned(slot.half_height) + extra;
        return (fabsf(x_pos - center_x) <= half_width) && (fabsf(y_pos - center_y) <= half_height);
    };

    if (_active_slot_index >= 0 && _active_slot_index < static_cast<int8_t>(detect_cfg.gear_slots_count)) {
        const ShifterGearSlot &slot = detect_cfg.gear_slots[_active_slot_index];
        if (!is_inside(slot, hysteresis)) {
            _active_slot_index = -1;
        }
    }

    if (_active_slot_index < 0) {
        float best_score = 0.0f;
        int8_t best_index = -1;
        for (uint32_t idx = 0; idx < detect_cfg.gear_slots_count; idx++) {
            const ShifterGearSlot &slot = detect_cfg.gear_slots[idx];
            if (is_gear_slot(slot.gear)) {
                has_gear_slots = true;
            }
            if (!is_inside(slot, 0.0f)) {
                continue;
            }
            float center_x = from_tenth_mm(slot.center_x);
            float center_y = from_tenth_mm(slot.center_y);
            float dx = x_pos - center_x;
            float dy = y_pos - center_y;
            float score = (dx * dx) + (dy * dy);
            if (best_index < 0 || score < best_score) {
                best_score = score;
                best_index = static_cast<int8_t>(idx);
            }
        }
        _active_slot_index = best_index;
    }

    if (_active_slot_index >= 0) {
        const ShifterGearSlot &slot = detect_cfg.gear_slots[_active_slot_index];
        uint8_t button_index = 0;
        if (map_gear_to_button(slot.gear, button_index) && button_index < CommManager::JOYSTICK_BUTTON_COUNT) {
            comm_manager.set_controller_button_value(button_index, true);
        }
    } else if (has_gear_slots) {
        uint8_t button_index = static_cast<uint8_t>(kGearNeutral);
        if (button_index < CommManager::JOYSTICK_BUTTON_COUNT) {
            comm_manager.set_controller_button_value(button_index, true);
        }
    }
}
