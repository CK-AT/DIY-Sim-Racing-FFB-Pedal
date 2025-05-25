#include <Arduino.h>
#include <CANManager.h>
#include <DiyActivePedal_types.h>

class AxesManager {
    public:
        AxesManager(GatewayCANManager &can_manager) : _can_manager(can_manager) {};
        void on_config_update(uint8_t axis_id, const DAP_config_st *new_config);
        const DAP_config_st *get_config(uint8_t axis_id);
        bool get_controller_value(uint8_t axis_id, float &value);
        float get_controller_value(uint8_t axis_id) {
            float value = 0.0f;
            get_controller_value(axis_id, value);
            return value;
        }
        bool populate_basic_state(uint8_t axis_id, DAP_state_basic_st &state_struct);

    private:
        DAP_config_st axis_configs[MAX_AXES] = {};
        bool axis_config_valid[MAX_AXES] = {};
        GatewayCANManager &_can_manager;
};
