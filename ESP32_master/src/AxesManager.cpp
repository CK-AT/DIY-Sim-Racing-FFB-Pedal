#include <AxesManager.h>

float NormalizeValue(float value, float minVal, float maxVal) {
    float valRange = (maxVal - minVal);
    if (abs(valRange) < 0.01) {
        return 0.0;  // avoid div-by-zero
    }
    if (value < minVal) {
        return 0.0;
    }
    if (value > maxVal) {
        return 1.0;
    }

    return (value - minVal) / valRange;
}

void AxesManager::on_config_update(uint8_t axis_id, const DAP_config_st *new_config) {
    memcpy(&(axis_configs[axis_id]), new_config, sizeof(DAP_config_st));
    axis_config_valid[axis_id] = true;
}

const DAP_config_st *AxesManager::get_config(uint8_t axis_id) {
    if (axis_config_valid[axis_id]) {
        return &(axis_configs[axis_id]);
    }
    return nullptr;
}

bool AxesManager::get_controller_value(uint8_t axis_id, float &value) {
    const DAP_config_st *config = get_config(axis_id);
    if (config) {
        if (config->payLoadPedalConfig_.travelAsJoystickOutput_u8) {
            float x_foot, x_foot_min, x_foot_max;
            if (!_can_manager.get_position(axis_id, x_foot)) return false;
            if (!_can_manager.get_position_limits(axis_id, x_foot_min, x_foot_max)) return false;
            value = NormalizeValue(x_foot, x_foot_min, x_foot_max);
            return true;
        } else {
            float f_foot;
            if (!_can_manager.get_force(axis_id, f_foot)) return false;
            value = NormalizeValue(f_foot, config->get_f_min(), config->get_f_max());
            return true;
        }
    }
    return false;
}

bool AxesManager::populate_basic_state(uint8_t axis_id, DAP_state_basic_st &state_struct) {
    const DAP_config_st *config = get_config(axis_id);
    if (config) {
        float x_foot, x_foot_min, x_foot_max, f_foot;
        if (_can_manager.get_position(axis_id, x_foot) && _can_manager.get_position_limits(axis_id, x_foot_min, x_foot_max)) {
            state_struct.payloadPedalState_Basic_.pedalPosition_u16 = uint16_t(NormalizeValue(x_foot, x_foot_min, x_foot_max) * 65535.0f);
        } else {
            state_struct.payloadPedalState_Basic_.pedalPosition_u16 = 0;
        }
        if (_can_manager.get_force(axis_id, f_foot)) {
            state_struct.payloadPedalState_Basic_.pedalForce_u16 =
                uint16_t(NormalizeValue(f_foot, config->get_f_min(), config->get_f_max()) * 65535.0f);
        } else {
            state_struct.payloadPedalState_Basic_.pedalForce_u16 = 0;
        }
        if (config->payLoadPedalConfig_.travelAsJoystickOutput_u8) {
            state_struct.payloadPedalState_Basic_.joystickOutput_u16 = state_struct.payloadPedalState_Basic_.pedalPosition_u16;
        } else {
            state_struct.payloadPedalState_Basic_.joystickOutput_u16 = state_struct.payloadPedalState_Basic_.pedalForce_u16;
        }
        state_struct.payloadPedalState_Basic_.error_code_u8 = 0;
        state_struct.payLoadHeader_.payloadType = FFBDataType::STATE_BASIC;
        state_struct.payLoadHeader_.PedalTag = axis_id;
        state_struct.payLoadHeader_.version = DAP_VERSION_CONFIG;
        state_struct.payloadFooter_.checkSum = 0;
        update_crc(state_struct.payloadFooter_.checkSum, &state_struct,
                   sizeof(DAP_state_basic_st::payLoadHeader_) + sizeof(DAP_state_basic_st::payloadPedalState_Basic_));
        return true;
    }
}