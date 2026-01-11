#include "ConfigManager.h"

#include "LogOutput.h"

float calc_poly(const float &in, const double *coeffs, size_t num_coeffs) {
    double result = coeffs[0];
    double temp = in;
    for (uint8_t i = 1; i < num_coeffs; i++) {
        result += temp * coeffs[i];
        temp *= in;
    }
    return result;
}

void ConfigManager::set_axis_config_defaults(void) {
    _axis_config = AxisConfig_init_default;
    clear_axis_config_raw();
    if (_fixed_id) {
        _axis_config.axis_id = _axis_id;
    }
    _axis_config.has_kinematic_parameters = true;
    _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos[0] = 6.20184902e-01;
    _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos[1] = -1.71372506e-03;
    _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos[2] = 1.07828479e-05;
    _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos[3] = 2.71382634e-09;
    _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos[4] = 7.34203389e-11;
    _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos[0] = 5.50622588e+01;
    _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos[1] = 5.55488175e-01;
    _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos[2] = 7.59065219e-04;
    _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos[3] = -2.81513616e-06;
    _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos[4] = -1.11220289e-08;
    _axis_config.kinematic_parameters.contact_point_pos_min_abs = -1200;
    _axis_config.kinematic_parameters.contact_point_pos_max_abs = 900;
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
    _axis_config.physics_iterations_per_sample = 16;
    _axis_config.homing_direction = HomingDirection_HOMING_DIR_NEGATIVE;
    _axis_config.has_oscillation_guard = true;
    _axis_config.oscillation_guard.k_max = 0.5f;
    _axis_config.oscillation_guard.min_amplitude = 0.2f;
    _axis_config.oscillation_guard.min_velocity = 0.5f;
    _axis_config.oscillation_guard.min_frequency_hz = 2.0f;
    _axis_config.oscillation_guard.max_frequency_hz = 100.0f;
    _axis_config.oscillation_guard.hold_time_ms = 150;
    _axis_config.oscillation_guard.ramp_time_ms = 80;
    _axis_config.oscillation_guard.required_hits = 2;
}

void ConfigManager::set_function_config_defaults(void) {
    _function_config = FunctionConfig_init_default;
    _function_config.has_base = true;
    _function_config.base.function_id = FunctionID_FUNCTION_ID_UNDEFINED;
    _function_config.base.linked_axes[0] = _axis_id;
    _function_config.base.store = false;
    _function_config.base.controller_output_axis = ControllerAxis_CONTROLLER_AXIS_BRK;
    _function_config.base.output_mode = OutputMode_OUTPUT_MODE_FORCE;
    _function_config.base.output_min = 55.0f;
    _function_config.base.output_max = 145.0f;

    _function_config.simulated_mass = 0.2f;
    _function_config.friction = 2.0f;
}

void ConfigManager::store_axis_config_raw(const uint8_t *data, uint16_t len) {
    if (!data || len == 0) {
        clear_axis_config_raw();
        return;
    }
    _axis_config_raw.assign(data, data + len);
}

void ConfigManager::clear_axis_config_raw(void) {
    _axis_config_raw.clear();
}

void ConfigManager::init(AxisID axis_id, bool fixed_id, OnConfigUpdate config_update_callback, GetAuxFunction get_aux_function_callback) {
    LogOutput::printf("ConfigManager: init (axis only)");
    _axis_id = axis_id;
    _fixed_id = fixed_id;
    _on_config_update_callback = config_update_callback;
    _get_aux_function_callback = get_aux_function_callback;
    _sem_cfg_update = xSemaphoreCreateMutex();
    if (!_sem_cfg_update) {
        LogOutput::printf(" -> failed to create config update semaphore!");
        return;
    }
    load_configs();
    _mode = MODE_AXIS_ONLY;
    LogOutput::printf(" -> init done");
}

void ConfigManager::init(GatewayID gateway_id, GetAuxFunction get_aux_function_callback) {
    LogOutput::printf("ConfigManager: init (gateway only)");
    _gateway_id = gateway_id;
    _get_aux_function_callback = get_aux_function_callback;
    _mode = MODE_GATEWAY_ONLY;
    LogOutput::printf(" -> done");
}

void ConfigManager::load_configs(void) {
    if (_mode == MODE_GATEWAY_ONLY) {
        LogOutput::printf("ConfigManager: gateway only, no configs to load");
        return;
    }
    persistent_memory.begin("config", true);
    LogOutput::printf(" -> trying to load axis config from persistent memory...");
    if (!load_axis_config()) {
        set_axis_config_defaults();
        LogOutput::printf(" -> setting defaults");
        if (_axis_id == AxisID_AXIS_UNDEFINED) {
            LogOutput::printf(" -> WARNING: This axis has no ID yet. Upload a valid axis config via USB serial to fix this.");
        }
    } else if (_axis_config.axis_id == _axis_id) {
        LogOutput::printf(" -> success");
    } else {
        update_axis_id(_axis_config.axis_id);
        LogOutput::printf(" -> success (this is axis %d)", _axis_id);
    }

    LogOutput::printf(" -> trying to load function config from persistent memory...");
    if (!load_function_config()) {
        set_function_config_defaults();
        LogOutput::printf(" -> setting defaults");
    } else {
        LogOutput::printf(" -> success");
    }
    persistent_memory.end();
    on_config_update();
}

bool ConfigManager::load_axis_config(void) {
    if (persistent_memory.isKey("axis_config")) {
        size_t size = persistent_memory.getBytesLength("axis_config");
        uint8_t buffer[size];
        persistent_memory.getBytes("axis_config", buffer, size);
        uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
        if (MessageTools::check_and_decode_message(_temp_message, buffer, size - sizeof(uint16_t), crc)) {
            if (_temp_message.which_payload == Message_axis_config_tag) {
                _axis_config = _temp_message.payload.axis_config;
                bool axis_id_mismatch = _fixed_id && (_axis_config.axis_id != _axis_id);
                if (axis_id_mismatch) {
                    _axis_config.axis_id = _axis_id;
                    LogOutput::printf(" -> WARNING: This axis' stored config references axis %d (this is axis %d).", _axis_config.axis_id, _axis_id);
                    clear_axis_config_raw();
                } else {
                    store_axis_config_raw(buffer, size);
                }
                return true;
            } else {
                LogOutput::printf(" -> not an axis config");
            }
        }
    } else {
        LogOutput::printf(" -> not found");
    }
    return false;
}

bool ConfigManager::load_function_config(void) {
    if (persistent_memory.isKey("function_config")) {
        size_t size = persistent_memory.getBytesLength("function_config");
        uint8_t buffer[size];
        persistent_memory.getBytes("function_config", buffer, size);
        uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
        if (MessageTools::check_and_decode_message(_temp_message, buffer, size - sizeof(uint16_t), crc)) {
            if (_temp_message.which_payload == Message_function_config_tag) {
                _function_config = _temp_message.payload.function_config;
                update_lookup_tables(_function_config);
                return true;
            } else {
                LogOutput::printf(" -> not a function config");
            }
        }
    } else {
        LogOutput::printf(" -> not found");
    }
    return false;
}

ConfigManager::UpdateResult ConfigManager::update_axis_config(const AxisConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg,
                                                              bool force) {
    LogOutput::printf("ConfigManager: trying to update axis config...");
    if (new_config.axis_id != _axis_id) {
        if (!force || _fixed_id) {
            LogOutput::printf(" -> targeting a different axis (axis %d)", new_config.axis_id);
            return ConfigManager::UPDATE_OTHER_AXIS;
        }
    }
    if (try_take_config_semaphore(5)) {
        _axis_config = new_config;
        store_axis_config_raw(protobuf_msg, len_protobuf_msg);
        on_config_update();
        if (_axis_config.store) {
            LogOutput::printf(" -> storing to persistent memory...");
            persistent_memory.begin("config", false);
            if (persistent_memory.putBytes("axis_config", protobuf_msg, len_protobuf_msg) != len_protobuf_msg) {
                LogOutput::printf(" -> failed to store");
            }
            persistent_memory.end();
        }
        // TODO: add update code for calculation vars here
        release_config_semaphore();
        LogOutput::printf(" -> done");
        return ConfigManager::UPDATE_OK;
    } else {
        LogOutput::printf(" -> failed to take sempahore");
        return ConfigManager::UPDATE_FAILED;
    }
}

ConfigManager::UpdateResult ConfigManager::update_function_config(const FunctionConfig &new_config, const uint8_t *protobuf_msg,
                                                                  uint16_t len_protobuf_msg) {
    LogOutput::printf("ConfigManager: trying to update function config...");
    bool affecting_this_axis = false;
    for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
        auto linked_axis_id = AxisID(new_config.base.linked_axes[idx] & AxisID_AXIS_ID_MASK);
        if (linked_axis_id == _axis_id) {
            affecting_this_axis = true;
        }
    }
    update_lookup_tables(new_config);
    if (!affecting_this_axis) {
        LogOutput::printf(" -> not targeting this axis");
        return ConfigManager::UPDATE_OTHER_AXIS;
    } else if (try_take_config_semaphore(5)) {
        _function_config = new_config;
        on_config_update();
        if (_function_config.base.store) {
            LogOutput::printf(" -> storing to persistent memory...");
            persistent_memory.begin("config", false);
            if (persistent_memory.putBytes("function_config", protobuf_msg, len_protobuf_msg) != len_protobuf_msg) {
                LogOutput::printf(" -> failed to store");
            }
            persistent_memory.end();
        }
        release_config_semaphore();
        LogOutput::printf(" -> done");
        return ConfigManager::UPDATE_OK;
    } else {
        LogOutput::printf(" -> failed to take sempahore");
        return ConfigManager::UPDATE_FAILED;
    }
}

void ConfigManager::get_axis_config_as_message(Message &message) {
    message.which_payload = Message_axis_config_tag;
    message.payload.axis_config = _axis_config;
}

void ConfigManager::get_function_config_as_message(Message &message) {
    message.which_payload = Message_function_config_tag;
    message.payload.function_config = _function_config;
}

void ConfigManager::on_config_update(void) {
    if (_on_config_update_callback) {
        _active_funtion = _on_config_update_callback(_active_funtion, &_function_config);
    }
    update_x_contact_point_limits();
}

void ConfigManager::update_lookup_tables(const FunctionConfig &new_config) {
    _function_lut[new_config.base.function_id] = new_config.base;
    if (new_config.has_aux_function) {
        if (_get_aux_function_callback) {
            IAuxFunction *aux_function = _get_aux_function_callback(&new_config);
            if (aux_function) {
                _aux_function_lut[new_config.base.function_id] = {aux_function, new_config.aux_function};
                return;
            }
        }
    }
    _aux_function_lut[new_config.base.function_id] = {nullptr, {}};
}

float ConfigManager::calc_force_conversion_factor(float &x_contact_point) {
    return calc_poly(x_contact_point, _axis_config.kinematic_parameters.coeffs_force_factor_over_contact_point_pos, sizeof(KinematicParameters::coeffs_force_factor_over_contact_point_pos) / sizeof(KinematicParameters::coeffs_force_factor_over_contact_point_pos[0]));
}

float ConfigManager::calc_sled_position(float &x_contact_point) {
    return calc_poly(x_contact_point, _axis_config.kinematic_parameters.coeffs_sled_pos_over_contact_point_pos, sizeof(KinematicParameters::coeffs_sled_pos_over_contact_point_pos) / sizeof(KinematicParameters::coeffs_sled_pos_over_contact_point_pos[0]));
}
