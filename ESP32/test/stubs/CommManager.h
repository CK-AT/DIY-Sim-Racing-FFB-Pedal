#pragma once

#include <array>
#include <cstdint>

#include "diy_ffb_protocol.pb.h"

class CommManager {
    public:
        static constexpr uint8_t JOYSTICK_BUTTON_COUNT = 32;

        CommManager(AxisID axis_id = AxisID_AXIS_UNDEFINED) : _axis_id(axis_id) {
            _positions_valid.fill(false);
            _button_values.fill(false);
            _positions.fill(0.0f);
            _controller_axis_values.fill(0.0f);
        }

        void set_axis_id(AxisID axis_id) {
            _axis_id = axis_id;
        }

        AxisID get_axis_id(void) {
            return _axis_id;
        }

        void set_position(AxisID axis_id, float value) {
            int idx = axis_index(axis_id);
            if (idx < 0) return;
            _positions[idx] = value;
            _positions_valid[idx] = true;
        }

        bool get_position(AxisID axis_id, float &value) {
            int idx = axis_index(axis_id);
            if (idx < 0 || !_positions_valid[idx]) return false;
            value = _positions[idx];
            return true;
        }

        bool set_controller_button_value(uint8_t button_index, bool pressed) {
            if (button_index >= JOYSTICK_BUTTON_COUNT) return false;
            _button_values[button_index] = pressed;
            return true;
        }

        float get_controller_output_value(ControllerAxis controller_axis) {
            return _controller_axis_values[controller_axis_index(controller_axis)];
        }

        void set_controller_output_value(ControllerAxis controller_axis, float &value) {
            _controller_axis_values[controller_axis_index(controller_axis)] = value;
        }

        bool get_controller_button_value(uint8_t button_index) const {
            if (button_index >= JOYSTICK_BUTTON_COUNT) return false;
            return _button_values[button_index];
        }

        void clear_buttons(void) {
            _button_values.fill(false);
        }

    private:
        static int axis_index(AxisID axis_id) {
            if (axis_id == AxisID_AXIS_UNDEFINED) return -1;
            int idx = int(axis_id) - 1;
            if (idx < 0 || idx >= kMaxAxes) return -1;
            return idx;
        }

        static int controller_axis_index(ControllerAxis controller_axis) {
            int idx = int(controller_axis) - 1;
            if (idx < 0) return 0;
            if (idx >= _ControllerAxis_MAX) return _ControllerAxis_MAX - 1;
            return idx;
        }

        static constexpr int kMaxAxes = 8;
        AxisID _axis_id;
        std::array<float, kMaxAxes> _positions;
        std::array<bool, kMaxAxes> _positions_valid;
        std::array<bool, JOYSTICK_BUTTON_COUNT> _button_values;
        std::array<float, _ControllerAxis_MAX> _controller_axis_values;
};
