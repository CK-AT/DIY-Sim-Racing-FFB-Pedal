#pragma once
#include <Arduino.h>
#include <Preferences.h>

#include <functional>
#include <map>
#include <vector>

#include "ConfigManager.fwd.h"
#include "IAuxFunction.h"
#include "IFunction.h"
#include "LogOutput.h"
#include "MessageTools.h"
#include "TopologyCache.h"

class ConfigManager {
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
        void update_lookup_tables(const FunctionConfig &new_config);
        void get_axis_config_as_message(Message &message);
        void get_function_config_as_message(Message &message);
        bool get_axis_config_raw(const uint8_t *&data, uint16_t &len) const {
            if (_axis_config_raw.empty()) {
                return false;
            }
            if (_axis_config_raw.size() > UINT16_MAX) {
                return false;
            }
            data = _axis_config_raw.data();
            len = static_cast<uint16_t>(_axis_config_raw.size());
            return true;
        }
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
        float get_x_contact_point_center_2x(void) const {
            return _x_contact_point_center_2x;
        }
        PositionMode get_position_mode(void) const {
            return _position_mode;
        }
        AxisID get_primary_axis_for_fetch(void) const {
            return _primary_axis_id;
        }
        bool is_subtractive_axis(void) const {
            return _position_mode == POSITION_MODE_FETCH_PRIMARY_MIRRORED;
        }
        uint8_t get_force_fetch_count(void) const {
            return _force_fetch_count;
        }
        const ForceFetchEntry &get_force_fetch_entry(uint8_t i) const {
            return _force_fetch[i];
        }
        float calc_force_conversion_factor(float &x_contact_point);
        float calc_sled_position(float &x_contact_point);
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
        std::tuple<IAuxFunction *, AuxFunctionConfig> get_aux_function(FunctionID function_id) {
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
        void store_axis_config_raw(const uint8_t *data, uint16_t len);
        void clear_axis_config_raw(void);
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
            _x_contact_point_center_2x = 2.0f * _x_contact_point_center;
        }
        void update_kinematic_poly_cache(void);
        void update_topology_cache(void) {
            // Replaces the per-tick walks of FunctionBase.linked_axes inside
            // CommManager::calc_input_force_sum / calc_final_position with a
            // precomputed view. Topology only changes on config update, so the
            // FFB hot path can read the cache directly. See TopologyCache.h.
            const auto &linked_axes = _function_config.base.linked_axes;
            const size_t n = sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0]);
            compute_topology(
                _axis_id,
                linked_axes,
                n,
                _position_mode,
                _primary_axis_id,
                _force_fetch,
                _force_fetch_count
            );
        }
        void on_config_update(void);
        bool _fixed_id = false;
        AxisID _axis_id = AxisID_AXIS_UNDEFINED;
        GatewayID _gateway_id = GatewayID_GATEWAY_UNDEFINED;
        Mode _mode = MODE_UNDEFINED;
        AxisConfig _axis_config;
        std::vector<uint8_t> _axis_config_raw = {};
        FunctionConfig _function_config;
        std::map<FunctionID, FunctionBase> _function_lut = {};
        std::map<FunctionID, std::tuple<IAuxFunction *, AuxFunctionConfig>> _aux_function_lut = {};
        Message _temp_message;
        SemaphoreHandle_t _sem_cfg_update = xSemaphoreCreateMutex();
        OnConfigUpdate _on_config_update_callback = nullptr;
        GetAuxFunction _get_aux_function_callback = nullptr;
        IFunction *_active_funtion = nullptr;
        float _x_contact_point_min = 0.0f;
        float _x_contact_point_max = 0.0f;
        float _x_contact_point_center = 0.0f;
        float _x_contact_point_center_2x = 0.0f;
        PositionMode _position_mode = POSITION_MODE_USE_OWN;
        AxisID _primary_axis_id = AxisID_AXIS_UNDEFINED;
        ForceFetchEntry _force_fetch[TOPOLOGY_MAX_FETCH] = {};
        uint8_t _force_fetch_count = 0;
        // Cached single-precision copies of the kinematic polynomial
        // coefficients. The wire format keeps double for offline-fit
        // fidelity, but on-device evaluation runs in float on the ESP32's
        // single-precision FPU. Populated by update_kinematic_poly_cache()
        // whenever _axis_config is committed.
        static constexpr uint8_t KINEMATIC_POLY_DEGREE = 5;  // proto: max_count:5 fixed_count:true
        float _coeffs_force_factor_f[KINEMATIC_POLY_DEGREE] = {};
        float _coeffs_sled_pos_f[KINEMATIC_POLY_DEGREE] = {};
        bool _kinematic_use_double_fallback = false;
        Preferences persistent_memory;
};
