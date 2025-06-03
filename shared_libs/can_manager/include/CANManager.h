#pragma once
#include <Arduino.h>
#include <FFBDataTools.h>
#include <isotp.h>

#include <ESP32-TWAI-CAN.hpp>

#include "IAxisCommChannel.h"
#include "IGatewayCommChannel.h"

#define ISOTP_BUFFER_SIZE 512

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
        bool get_force(AxisID axis_id, float &f_foot);
        bool get_position(AxisID axis_id, float &x_foot);
        bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max);
        bool is_online(AxisID axis_id);

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
        void update_timeouts(uint32_t now);
        bool try_process_high_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
        bool try_process_low_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
        void switch_bus_state(uint32_t ti_now, BusState new_state) {
            ti_state_change = ti_now;
            bus_state = new_state;
        }
        void switch_bus_state(BusState new_state) {
            switch_bus_state(micros(), new_state);
        }
        AxisState axis_states[FFBDataTools::MAX_AXES_COUNT] = {};
        uint8_t isotp_rx_buff[ISOTP_BUFFER_SIZE];
        uint32_t isotp_rx_size;
        uint32_t tx_err_cnt = 0;
        uint32_t rx_err_cnt = 0;
        uint32_t ti_state_change = 0;
        BusState bus_state = BusState::PRE_ONLINE;
        uint32_t ti_last_ping = 0;
};

class AxisCANManager : public CANManager, public IAxisCommChannel {
        typedef std::function<void(uint8_t *data, uint32_t len)> OnGatewayPayload;
        typedef std::function<void(FFBAction &action)> OnFFBAction;

    public:
        AxisCANManager(void) {};
        void setup(AxisID axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload on_gateway_payload, OnFFBAction on_ffb_update);
        void process(void) override;
        void send_force_and_position(float &f_foot, float &x_foot) override;
        bool send_payload_to_gateway(const uint8_t *data, uint32_t len) override;
        void send_position_limits(float x_foot_min, float x_foot_max) override;
        void update_force(float &f_foot) override {
            if (own_axis_index < 0) return;
            axis_states[own_axis_index].force_and_position.f_foot = f_foot;
            axis_states[own_axis_index].online = true;
        }
        bool get_force(AxisID axis_id, float &f_foot) override {
            return CANManager::get_force(axis_id, f_foot);
        }
        bool get_position(AxisID axis_id, float &x_foot) override {
            return CANManager::get_position(axis_id, x_foot);
        }
        bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) override {
            return CANManager::get_position_limits(axis_id, x_foot_min, x_foot_max);
        }
        bool is_online(AxisID axis_id) override {
            return CANManager::is_online(axis_id);
        }
        bool is_gateway_online(void) override {
            return _gateway_online;
        };

    private:
        bool try_process_isotp_can_frame(CanFrame &rx_frame);
        bool try_process_ffb_update_frame(CanFrame &rx_frame);
        bool try_process_ping_frame(CanFrame &rx_frame, uint32_t now);
        void broadcast_position_limits(void);
        void broadcast_position_limits(uint32_t now);
        void update_timeouts(uint32_t now);
        AxisID own_axis_id = AxisID_AXIS_UNDEFINED;
        int8_t own_axis_index = -1;
        IsotpState isotp_state;
        OnGatewayPayload on_gateway_payload = nullptr;
        OnFFBAction on_ffb_action = nullptr;
        float _x_foot_min = 0.0f;
        float _x_foot_max = 0.0f;
        bool _gateway_online = false;
};

class GatewayCANManager : public CANManager, public IGatewayCommChannel {
        typedef std::function<void(AxisID axis_id, uint8_t *data, uint32_t len)> OnAxisPayload;

    public:
        GatewayCANManager(void) {};
        void setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb);
        void send_ffb_data_to_axis(AxisID axis_id, FFBData &ffb_data, const uint8_t *raw_data, uint32_t len_raw_data) override;
        bool get_force(AxisID axis_id, float &f_foot) override {
            return CANManager::get_force(axis_id, f_foot);
        }
        bool get_position(AxisID axis_id, float &x_foot) override {
            return CANManager::get_position(axis_id, x_foot);
        }
        bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) override {
            return CANManager::get_position_limits(axis_id, x_foot_min, x_foot_max);
        }
        bool is_online(AxisID axis_id) override {
            return CANManager::is_online(axis_id);
        }

    private:
        void process(void);
        void ping(uint32_t now);
        bool try_process_isotp_can_frame(CanFrame &rx_frame);
        bool send_payload_to_axis(AxisID axis_id, const uint8_t *data, uint32_t len);
        void send_abs_trigger_to_axis(AxisID axis_id);
        IsotpState isotp_state[FFBDataTools::MAX_AXES_COUNT];
        OnAxisPayload on_axis_payload = nullptr;
};