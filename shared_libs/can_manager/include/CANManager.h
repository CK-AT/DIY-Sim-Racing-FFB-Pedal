#pragma once
#include <Arduino.h>
#include <isotp.h>

#include <ESP32-TWAI-CAN.hpp>

#define MAX_AXES 4
#define ISOTP_BUFFER_SIZE 256

class CANManager {
    protected:
        enum BusState {
            PRE_ONLINE,
            ONLINE,
            BUS_OFF
        };

        enum AxisFrameTypesHS {
            FORCE_AND_POSITION = 0
        };

        enum AxisFrameTypesLS {
            POSITION_LIMITS = 0
        };

        enum FFBFrameTypes {
            ABS = 0
        };

        struct ForceAndPosition {
                float f_foot;
                float x_foot;
        };

        struct PositionLimits {
                float x_foot_min;
                float x_foot_max;
        };

        struct AxisState {
                ForceAndPosition force_and_position;
                PositionLimits position_limits;
                uint32_t ti_last_seen;
                uint32_t ti_last_limit_update;
                bool online;
                bool limits_valid;
        };

        struct IsotpState {
                IsoTpLink link;
                uint8_t isotp_link_rx_buff[ISOTP_BUFFER_SIZE];
                uint8_t isotp_link_tx_buff[ISOTP_BUFFER_SIZE];
        };

    public:
        struct FFBUpdate {
                bool trigger_abs : 1;
        };

    public:
        bool get_force(int8_t axis_id, float &f_foot);
        bool get_position(int8_t axis_id, float &x_foot);
        bool get_position_limits(int8_t axis_id, float &x_foot_min, float &x_foot_max);
        bool is_online(int8_t axis_id);

    protected:
        virtual void process(void);
        static void task_func(void *pvParameters) {
            CANManager *manager = (CANManager *)pvParameters;
            delay(1000);
            for (;;) {
                manager->process();
                delay(1);
            }
        }
        bool check_bus(uint32_t ti_now);
        void update_axis_timeouts(uint32_t now);
        bool try_process_high_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
        bool try_process_low_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
        void switch_bus_state(uint32_t ti_now, BusState new_state) {
            ti_state_change = ti_now;
            bus_state = new_state;
        }
        void switch_bus_state(BusState new_state) {
            switch_bus_state(micros(), new_state);
        }
        AxisState axis_states[MAX_AXES] = {};
        uint8_t isotp_rx_buff[ISOTP_BUFFER_SIZE];
        uint32_t isotp_rx_size;
        uint32_t tx_err_cnt = 0;
        uint32_t rx_err_cnt = 0;
        uint32_t ti_state_change = 0;
        BusState bus_state = BusState::PRE_ONLINE;
};

class AxisCANManager : public CANManager {
        typedef std::function<void(uint8_t *data, uint32_t len)> OnGatewayPayload;
        typedef std::function<void(FFBUpdate &update)> OnFFBUpdate;

    public:
        AxisCANManager(void) {};
        void setup(int8_t axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload on_gateway_payload, OnFFBUpdate on_ffb_update);
        void process(void);
        void broadcast_position_limits(uint32_t now);
        void send_force_and_position(float &f_foot, float &x_foot);
        bool send_payload_to_gateway(const uint8_t *data, uint32_t len);
        void send_position_limits(float &x_foot_min, float &x_foot_max);

    private:
        bool try_process_isotp_can_frame(CanFrame &rx_frame);
        bool try_process_ffb_update_frame(CanFrame &rx_frame);
        int8_t own_axis_id = -1;
        IsotpState isotp_state;
        OnGatewayPayload on_gateway_payload = nullptr;
        OnFFBUpdate on_ffb_update = nullptr;
};

class GatewayCANManager : public CANManager {
        typedef std::function<void(uint8_t axis_id, uint8_t *data, uint32_t len)> OnAxisPayload;

    public:
        GatewayCANManager(void) {};
        void setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb);
        bool send_payload_to_axis(uint8_t axis_id, const uint8_t *data, uint32_t len);
        void send_abs_trigger_to_axis(uint8_t axis_id);

    private:
        void process(void);
        bool try_process_isotp_can_frame(CanFrame &rx_frame);
        IsotpState isotp_state[MAX_AXES];
        OnAxisPayload on_axis_payload = nullptr;
};