#pragma once
#include <Arduino.h>

#include <map>

#include "ConfigManager.fwd.h"
#include "IFunction.h"
#include "LogOutput.h"
#include "MessageTools.h"
#include "IAuxFunction.h"

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

        typedef std::function<IFunction *(IFunction *active_function, const FunctionConfig *function_cfg)> OnConfigUpdate;
        typedef std::function<IAuxFunction *(const FunctionConfig *func_cfg)> GetAuxFunction;

        void init(AxisID axis_id, bool fixed_id, OnConfigUpdate config_update_callback, GetAuxFunction get_aux_function_callback);
        void init(GatewayID gateway_id, GetAuxFunction get_aux_function_callback);
        UpdateResult update_axis_config(const AxisConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, bool force = false);
        UpdateResult update_function_config(const FunctionConfig &new_config, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg);
        void update_function_config_base_lut(const FunctionConfig &new_config) {
            _function_lut[new_config.base.function_id] = new_config.base;
        }
        void update_aux_function_lut(const FunctionConfig &new_config) {
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
        void get_axis_config_as_message(Message &message);
        void get_function_config_as_message(Message &message);
        AxisID get_axis_id(void) const {
            if (_mode & MODE_AXIS_MASK) return _axis_id;
            return AxisID_AXIS_UNDEFINED;
        }
        bool is_axis(void) {
            return get_axis_id() != AxisID_AXIS_UNDEFINED;
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
        FunctionID get_function_id(void) {
            return _function_config.base.function_id;
        }
        FunctionBase *get_function_base(FunctionID function_id) {
            auto result = _function_lut.find(function_id);
            if (result != _function_lut.end()) return &result->second;
            return nullptr;
        }
        std::tuple<IAuxFunction*,AuxFunctionConfig> get_aux_function(FunctionID function_id) {
            auto result = _aux_function_lut.find(function_id);
            if (result != _aux_function_lut.end()) return result->second;
            return {nullptr, {}};
        }
        AxisID get_primary_axis_id(FunctionID function_id) {
            FunctionBase *function_base = get_function_base(function_id);
            if (function_base) {
                return AxisID(function_base->linked_axes[0] & AxisID_AXIS_ID_MASK);
            }
            return AxisID_AXIS_UNDEFINED;
        }
        const AutomotivePedalConfig *get_automotive_pedal_config(void) {
            if (_function_config.which_specific == FunctionConfig_automotive_pedal_tag) {
                return &_function_config.specific.automotive_pedal;
            }
            return nullptr;
        }
        const FlightPedalsConfig *get_flight_pedal_config(void) {
            if (_function_config.which_specific == FunctionConfig_flight_pedals_tag) {
                return &_function_config.specific.flight_pedals;
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
        IFunction *get_active_function(void) {
            return _active_funtion;
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
        void update_x_contact_point_limits(void) {
            if (_active_funtion) {
                _x_contact_point_min = _active_funtion->get_x_contact_point_min();
                _x_contact_point_max = _active_funtion->get_x_contact_point_max();
            } else {
                _x_contact_point_min = -1.0f;
                _x_contact_point_max = 1.0f;
                LogOutput::printf("ConfigManager: Unknown function config!");
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
        std::map<FunctionID, FunctionBase> _function_lut = {};
        std::map<FunctionID, std::tuple<IAuxFunction*, AuxFunctionConfig>> _aux_function_lut = {};
        Message _temp_message;
        SemaphoreHandle_t _sem_cfg_update = xSemaphoreCreateMutex();
        OnConfigUpdate _on_config_update_callback = nullptr;
        GetAuxFunction _get_aux_function_callback = nullptr;
        IFunction *_active_funtion = nullptr;
        float _x_contact_point_min = 0.0f;
        float _x_contact_point_max = 0.0f;
        float _x_contact_point_center = 0.0f;
};