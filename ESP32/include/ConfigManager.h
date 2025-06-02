#pragma once
#include <Arduino.h>

#include "FFBDataTools.h"

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
        void get_axis_config(FFBData &message);
        void get_function_config(FFBData &message);
        AxisID get_axis_id(void) {
            return _axis_id;
        }
        float get_x_contact_point_min(void) {
            switch (_function_config.which_specific) {
                case FunctionConfig_automotive_pedal_tag:
                    return float(_function_config.specific.automotive_pedal.pos_idle);
                case FunctionConfig_flight_pedal_tag:
                    return float(_function_config.specific.flight_pedal.pos_near_lim);
                default:
                    return -1.0f;
            }
        }
        float get_x_contact_point_max(void) {
            switch (_function_config.which_specific) {
                case FunctionConfig_automotive_pedal_tag:
                    return float(_function_config.specific.automotive_pedal.pos_end);
                case FunctionConfig_flight_pedal_tag:
                    return float(_function_config.specific.flight_pedal.pos_far_lim);
                default:
                    return 1.0f;
            }
        }
        float get_x_contact_point_center(void) {
            float x_min = get_x_contact_point_min();
            float x_max = get_x_contact_point_max();
            return x_min + ((x_max - x_min) / 2.0f);
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
        AxisID _axis_id;
        AxisConfig _axis_config;
        FunctionConfig _function_config;
        FFBData _temp_ffb_data;
        SemaphoreHandle_t _sem_cfg_update = xSemaphoreCreateMutex();
        OnConfigUpdate _on_config_update;
};