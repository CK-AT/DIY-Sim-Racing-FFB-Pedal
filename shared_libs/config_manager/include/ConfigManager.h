#pragma once
#include <Arduino.h>
#include "ConfigManager.fwd.h"
#include "MessageTools.h"

class ConfigManager {
    private:
        struct EEPROMHeader {
                uint16_t crc;
                uint16_t len;
        };

    public:
        typedef std::function<void(void)> OnConfigUpdate;

        void init(AxisID axis_id, OnConfigUpdate config_update_callback);
        void load_configs(void);
        void update_axis_config(AxisConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg);
        void update_function_config(FunctionConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg);
        void get_axis_config(Message &message);
        void get_function_config(Message &message);
        AxisID get_axis_id(void) const {
            return _axis_id;
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
                    break;
            }
            _x_contact_point_center = _x_contact_point_min + ((_x_contact_point_max - _x_contact_point_min) / 2.0f);
        }
        void on_config_update(void);
        AxisID _axis_id;
        AxisConfig _axis_config;
        FunctionConfig _function_config;
        Message _temp_message;
        SemaphoreHandle_t _sem_cfg_update = xSemaphoreCreateMutex();
        OnConfigUpdate _on_config_update_callback;
        float _x_contact_point_min = 0.0f;
        float _x_contact_point_max = 0.0f; 
        float _x_contact_point_center = 0.0f;  
};