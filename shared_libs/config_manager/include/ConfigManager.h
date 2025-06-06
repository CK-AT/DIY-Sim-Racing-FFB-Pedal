#pragma once
#include <Arduino.h>

#include <map>

#include "ConfigManager.fwd.h"
#include "MessageTools.h"
#include "LogOutput.h"

class ConfigManager {
    private:
        struct EEPROMHeader {
                uint16_t crc;
                uint16_t len;
        };

    public:
        enum UpdateResult {
            UPDATE_OK,
            UPDATE_OTHER_AXIS,
            UPDATE_FAILED
        };

        enum Mode {
            MODE_UNDEFINED = 0,
            MODE_AXIS_ONLY = 1,
            MODE_GATEWAY_ONLY = 2,
            MODE_DUAL_ROLE = 3
        };
        const uint8_t MODE_AXIS_MASK = MODE_AXIS_ONLY;
        const uint8_t MODE_GATEWAY_MASK = MODE_GATEWAY_ONLY;

        typedef std::function<void(void)> OnConfigUpdate;

        void init(AxisID axis_id, bool fixed_id, OnConfigUpdate config_update_callback);
        void init(GatewayID gateway_id);
        UpdateResult update_axis_config(const AxisConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, bool force = false);
        UpdateResult update_function_config(const FunctionConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg);
        void update_function_config_lut(const FunctionConfig &new_config) {
            function_lut[new_config.base.function] = new_config;
        }
        void get_axis_config(Message &message);
        void get_function_config(Message &message);
        AxisID get_axis_id(void) const {
            if (_mode & MODE_AXIS_MASK) return _axis_id;
            return AxisID_AXIS_UNDEFINED;
        }
        GatewayID get_gateway_id(void) const {
            if (_mode & MODE_GATEWAY_MASK) return _gateway_id;
            return GatewayID_GATEWAY_UNDEFINED;
        }
        bool set_gateway_id(GatewayID gateway_id) {
            if (_mode == MODE_AXIS_ONLY) return false;
            _gateway_id = gateway_id;
            return true;
        }
        bool is_axis_config_valid(void) {
            return _axis_config.axis_id == _axis_id;
        }
        Mode get_mode(void) {
            return _mode;
        }
        bool enable_dual_role_mode(void) {
            if (_fixed_id) {
                LogOutput::printf("ConfigManager: can't enable dual role mode, this axis has a HW defined ID");
                return false;
            }
            LogOutput::printf("ConfigManager: enabling dual role mode");
            _mode = MODE_DUAL_ROLE;
            return true;
        }
        float get_x_contact_point_min(void) {
            return _x_contact_point_min;
        }
        float get_x_contact_point_max(void) {
            return _x_contact_point_max;
        }
        float get_x_contact_point_center(void) {
            return _x_contact_point_center;
        }
        const FunctionConfig *get_function_config(void) {
            return &_function_config;
        }
        const AutomotivePedalConfig *get_automotive_pedal_config(void) {
            if (_function_config.which_specific == FunctionConfig_automotive_pedal_tag) {
                return &_function_config.specific.automotive_pedal;
            }
            return nullptr;
        }
        const FlightPedalConfig *get_flight_pedal_config(void) {
            if (_function_config.which_specific == FunctionConfig_flight_pedal_tag) {
                return &_function_config.specific.flight_pedal;
            }
            return nullptr;
        }
        const AxisConfig *get_axis_config(void) {
            return &_axis_config;
        }
        bool try_take_config_semaphore(uint32_t timeout = 1) {
            return (_sem_cfg_update && xSemaphoreTake(_sem_cfg_update, (TickType_t)timeout));
        }
        void release_config_semaphore(void) {
            xSemaphoreGive(_sem_cfg_update);
        }

    private:
        void load_configs(void);
        bool load_axis_config(void);
        void set_axis_config_defaults(void);
        bool load_function_config(void);
        void set_function_config_defaults(void);
        void update_axis_id(AxisID new_axis_id) {
            _axis_id = new_axis_id;
        }
        void calc_x_contact_point_limits(void) {
            switch (_function_config.which_specific) {
                case FunctionConfig_automotive_pedal_tag:
                    _x_contact_point_min = float(_function_config.specific.automotive_pedal.pos_idle);
                    _x_contact_point_max = float(_function_config.specific.automotive_pedal.pos_end);
                    break;
                case FunctionConfig_flight_pedal_tag:
                    _x_contact_point_min = float(_function_config.specific.flight_pedal.pos_near_lim);
                    _x_contact_point_max = float(_function_config.specific.flight_pedal.pos_far_lim);
                    break;
                default:
                    _x_contact_point_min = -1.0f;
                    _x_contact_point_max = 1.0f;
                    LogOutput::printf("ConfigManager: Unknown function config!");
                    break;
            }
            _x_contact_point_center = _x_contact_point_min + ((_x_contact_point_max - _x_contact_point_min) / 2.0f);
        }
        void on_config_update(void);
        bool _fixed_id = false;
        AxisID _axis_id = AxisID_AXIS_UNDEFINED;
        GatewayID _gateway_id = GatewayID_GATEWAY_UNDEFINED;
        Mode _mode = MODE_UNDEFINED;
        AxisConfig _axis_config;
        FunctionConfig _function_config;
        std::map<Function, FunctionConfig> function_lut = {};
        Message _temp_message;
        SemaphoreHandle_t _sem_cfg_update = xSemaphoreCreateMutex();
        OnConfigUpdate _on_config_update_callback = nullptr;
        float _x_contact_point_min = 0.0f;
        float _x_contact_point_max = 0.0f;
        float _x_contact_point_center = 0.0f;
};