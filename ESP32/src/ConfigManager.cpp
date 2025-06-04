#include "ConfigManager.h"

#include <EEPROM.h>

#include "LogOutput.h"

const uint32_t EEPROM_OFFSET_AXIS_CONFIG = 0;
const uint32_t EEPROM_OFFSET_FUNCTION_CONFIG = EEPROM_OFFSET_AXIS_CONFIG + 1024;

void ConfigManager::set_axis_config_defaults(void) {
    _axis_config = AxisConfig_init_default;
    _axis_config.axis_id = _axis_id;
    _axis_config.coeffs_force_factor_over_contact_point_pos[0] = 6.20184902e-01;
    _axis_config.coeffs_force_factor_over_contact_point_pos[1] = -1.71372506e-03;
    _axis_config.coeffs_force_factor_over_contact_point_pos[2] = 1.07828479e-05;
    _axis_config.coeffs_force_factor_over_contact_point_pos[3] = 2.71382634e-09;
    _axis_config.coeffs_force_factor_over_contact_point_pos[4] = 7.34203389e-11;
    _axis_config.coeffs_sled_pos_over_contact_point_pos[0] = 5.50622588e+01;
    _axis_config.coeffs_sled_pos_over_contact_point_pos[1] = 5.55488175e-01;
    _axis_config.coeffs_sled_pos_over_contact_point_pos[2] = 7.59065219e-04;
    _axis_config.coeffs_sled_pos_over_contact_point_pos[3] = -2.81513616e-06;
    _axis_config.coeffs_sled_pos_over_contact_point_pos[4] = -1.11220289e-08;
    _axis_config.contact_point_pos_min_abs = -1200;
    _axis_config.contact_point_pos_max_abs = 900;
    _axis_config.f_max_loadcell = 200.0f * 9.81f;
    _axis_config.which_load_cell_filter_config = AxisConfig_kf_const_vel_tag;
    _axis_config.load_cell_filter_config.kf_const_vel.noise_scaling = 128;
    _axis_config.mm_per_rev = 5;
    _axis_config.steps_per_mm = 1000;
    _axis_config.store = false;
    _axis_config.which_kinematic_config = AxisConfig_diy_pedal_tag;
    _axis_config.kinematic_config.diy_pedal.l_pivot_foot = 180;
    _axis_config.kinematic_config.diy_pedal.l_pivot_link = 100;
    _axis_config.kinematic_config.diy_pedal.l_link = 153;
    _axis_config.kinematic_config.diy_pedal.l_pivot_sled_y = 32;
    _axis_config.kinematic_config.diy_pedal.l_pivot_sled_x_min = 82;
    _axis_config.kinematic_config.diy_pedal.l_sled_stroke = 114;
}

void ConfigManager::set_function_config_defaults(void) {
    _function_config = FunctionConfig_init_default;
    _function_config.base.function = Function_FUNCTION_BRAKE;
    _function_config.base.linked_axes[0] = _axis_id;
    _function_config.base.store = false;
    _function_config.base.controller_output_axis = ControllerAxis_CONTROLLER_AXIS_BRK;
    _function_config.base.output_mode = OutputMode_OUTPUT_MODE_FORCE;
    _function_config.base.output_min = 55.0f;
    _function_config.base.output_max = 145.0f;

    _function_config.which_specific = FunctionConfig_automotive_pedal_tag;
    _function_config.specific.automotive_pedal.has_force_curve_config = true;
    _function_config.specific.automotive_pedal.force_curve_config.pos_min = 10;
    _function_config.specific.automotive_pedal.force_curve_config.pos_max = 50;
    _function_config.specific.automotive_pedal.force_curve_config.f_min = 50.0f;
    _function_config.specific.automotive_pedal.force_curve_config.f_max = 150.0f;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[0] = 0;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[1] = 20;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[2] = 40;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[3] = 60;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[4] = 80;
    _function_config.specific.automotive_pedal.force_curve_config.f_rel_points[5] = 100;
    _function_config.specific.automotive_pedal.force_curve_config.force_direction = ForceDirection_DIRECTION_SUBTRACT;
    _function_config.specific.automotive_pedal.has_damper_config = true;
    _function_config.specific.automotive_pedal.damper_config.positive_factor = 0.1f;
    _function_config.specific.automotive_pedal.damper_config.negative_factor = 0.1f;
    _function_config.specific.automotive_pedal.pos_idle = 10;
    _function_config.specific.automotive_pedal.pos_end = 50;
}

void ConfigManager::init(AxisID axis_id, OnConfigUpdate config_update_callback) {
    _axis_id = axis_id;
    _on_config_update_callback = config_update_callback;
    _sem_cfg_update = xSemaphoreCreateMutex();
    if (!_sem_cfg_update) {
        LogOutput::printf("ConfigManager: failed to create config update semaphore!");
    }
}

void ConfigManager::load_configs(void) {
    EEPROM.begin(2048);
    LogOutput::printf("ConfigManager: trying to load axis config from EEPROM...");
    if (load_axis_config() == false) {
        set_axis_config_defaults();
        LogOutput::printf(" -> setting defaults");
        if (_axis_id == AxisID_AXIS_UNDEFINED) {
            LogOutput::printf(" -> WARNING: This axis has no ID yet. Upload a valid axis config via USB serial to fix this.", _axis_config.axis_id,
                              _axis_id);
        }
    } else {
        if (_axis_config.axis_id == _axis_id) {
            LogOutput::printf(" -> success");
        } else {
            if (_axis_id == AxisID_AXIS_UNDEFINED) {
                update_axis_id(_axis_config.axis_id);
                LogOutput::printf(" -> success (this is axis %d)", _axis_id);
            } else {
                LogOutput::printf(" -> success");
                LogOutput::printf(" -> WARNING: This axis' stored config references axis %d (this is axis %d).", _axis_config.axis_id, _axis_id);
                _axis_config.axis_id = _axis_id;
            }
        }
    }
    LogOutput::printf("ConfigManager: trying to load function config from EEPROM...");
    if (load_function_config() == false) {
        set_function_config_defaults();
        LogOutput::printf(" -> setting defaults");
    } else {
        LogOutput::printf(" -> success");
    }
    LogOutput::printf("ConfigManager: init done");
    on_config_update();
}

bool ConfigManager::load_axis_config(void) {
    EEPROMHeader header;
    EEPROM.get(EEPROM_OFFSET_AXIS_CONFIG, header);
    if (header.len < 500) {
        uint8_t buffer[header.len];
        EEPROM.readBytes(EEPROM_OFFSET_AXIS_CONFIG + sizeof(EEPROMHeader), buffer, header.len);
        if (MessageTools::check_and_decode_message(_temp_message, buffer, header.len, header.crc)) {
            if (_temp_message.which_payload == Message_axis_config_tag) {
                _axis_config = _temp_message.payload.axis_config;
                return true;
            } else {
                LogOutput::printf(" -> not an axis config");
            }
        }
    } else {
        LogOutput::printf(" -> invalid EEPROM header");
    }
    return false;
}

bool ConfigManager::load_function_config(void) {
    EEPROMHeader header;
    EEPROM.get(EEPROM_OFFSET_FUNCTION_CONFIG, header);
    if (header.len < 500) {
        uint8_t buffer[header.len];
        EEPROM.readBytes(EEPROM_OFFSET_FUNCTION_CONFIG + sizeof(EEPROMHeader), buffer, header.len);
        if (MessageTools::check_and_decode_message(_temp_message, buffer, header.len, header.crc)) {
            if (_temp_message.which_payload == Message_function_config_tag) {
                _function_config = _temp_message.payload.function_config;
                return true;
            } else {
                LogOutput::printf(" -> not a function config");
            }
        }
    } else {
        LogOutput::printf(" -> invalid EEPROM header");
    }
    return false;
}

void ConfigManager::update_axis_config(AxisConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg) {
    LogOutput::printf("ConfigManager: trying to update axis config...");
    if (try_take_config_semaphore()) {
        _axis_config = new_config;
        on_config_update();
        if (_axis_config.store) {
            LogOutput::printf(" -> storing to EEPROM...");
            EEPROMHeader header;
            header.crc = MessageTools::calc_crc(protobuf_msg, len_protobuf_msg);
            header.len = len_protobuf_msg;
            EEPROM.put(EEPROM_OFFSET_AXIS_CONFIG, header);
            EEPROM.writeBytes(EEPROM_OFFSET_AXIS_CONFIG + sizeof(EEPROMHeader), protobuf_msg, len_protobuf_msg);
            EEPROM.commit();
        }
        // TODO: add update code for calculation vars here
        release_config_semaphore();
        LogOutput::printf(" -> done");
    } else {
        LogOutput::printf(" -> failed to take sempahore");
    }
}

void ConfigManager::update_function_config(FunctionConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg) {
    LogOutput::printf("ConfigManager: trying to update function config...");
    if (try_take_config_semaphore()) {
        _function_config = new_config;
        on_config_update();
        if (_function_config.base.store) {
            LogOutput::printf(" -> storing to EEPROM...");
            EEPROMHeader header;
            header.crc = MessageTools::calc_crc(protobuf_msg, len_protobuf_msg);
            header.len = len_protobuf_msg;
            EEPROM.put(EEPROM_OFFSET_FUNCTION_CONFIG, header);
            EEPROM.writeBytes(EEPROM_OFFSET_FUNCTION_CONFIG + sizeof(EEPROMHeader), protobuf_msg, len_protobuf_msg);
            EEPROM.commit();
        }
        // TODO: add update code for calculation vars here
        release_config_semaphore();
        LogOutput::printf(" -> done");
    } else {
        LogOutput::printf(" -> failed to take sempahore");
    }
}

void ConfigManager::get_axis_config(Message &message) {
    message.which_payload = Message_axis_config_tag;
    message.payload.axis_config = _axis_config;
}

void ConfigManager::get_function_config(Message &message) {
    message.which_payload = Message_function_config_tag;
    message.payload.function_config = _function_config;
}

void ConfigManager::on_config_update(void) {
    calc_x_contact_point_limits();
    _on_config_update_callback();
}