#pragma once
#include <Arduino.h>
#include <MessageTools.h>
#include "CommManager.fwd.h"
#include "SerialManager.fwd.h"
#include <PacketSerial.h>
#include "ICommChannel.h"

class SerialManager {
    public:
        bool setup(Stream *serial, CommManager *comm_manager, ICommChannel::OnGatewayPayload on_gateway_payload);
        void update_force_and_position(float &f_contact_point, float &x_contact_point);
        bool send_message_to_host(const Message &message, const uint8_t *raw_data, uint32_t len_raw_data);

    protected:
        void process(void);
        static void task_func(void *pvParameters) {
            SerialManager *manager = (SerialManager *)pvParameters;
            delay(1000);
            for (;;) {
                manager->process();
                delay(1);
            }
        }
        uint32_t ti_last_state_frame = 0;
        /* Axis related */
        float _f_contact_point = 0.0f;
        float _x_contact_point = 0.0f;
        PacketSerial packet_serial;
        Message state_message = Message_init_default;
        CommManager *comm_manager;
        SemaphoreHandle_t _sem_write = xSemaphoreCreateMutex();
};

// class GatewaySerialManager {
//     public:
//         bool get_force(AxisID axis_id, float &f_foot) override {return false;}
//         bool get_position(AxisID axis_id, float &x_foot) override {return false;}
//         bool get_position_limits(AxisID axis_id, float &x_foot_min, float &x_foot_max) override {return false;}
//         bool is_online(AxisID axis_id) override {return false;}
//         void process(void) override {}
//         /* Axis related */
//         bool setup(AxisID axis_id, Stream *serial, OnGatewayPayload on_gateway_payload, OnFFBAction on_ffb_update);
//         bool send_force_and_position(float &f_foot, float &x_foot) override;
//         bool send_message_to_gateway(const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) override;
//         bool update_position_limits(float x_foot_min, float x_foot_max) override;
//         bool update_force(float &f_foot) override {
//             return false;
//         }
//         bool is_gateway_online(void) override {
//             return _gateway_online;
//         };
//         /* Gateway related */
//         bool setup(Stream *serial, OnGatewayPayload on_gateway_payload);
//         bool send_message_to_axis(AxisID axis_id, const Message &message, const uint8_t *raw_data, uint32_t len_raw_data) override {return false;};

//     protected:
//         static void task_func(void *pvParameters) {
//             SerialManager *manager = (SerialManager *)pvParameters;
//             delay(1000);
//             for (;;) {
//                 manager->process();
//                 delay(1);
//             }
//         }
//         void shared_setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin);
//         bool check_bus(uint32_t ti_now);
//         void update_timeouts(uint32_t now);
//         bool try_process_high_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
//         bool try_process_low_prio_axis_frame(CanFrame &rx_frame, uint32_t now);
//         void switch_bus_state(uint32_t ti_now, BusState new_state) {
//             ti_state_change = ti_now;
//             bus_state = new_state;
//         }
//         void switch_bus_state(BusState new_state) {
//             switch_bus_state(micros(), new_state);
//         }
//         AxisState axis_states[MessageTools::MAX_AXES_COUNT] = {};
//         uint8_t isotp_rx_buff[ISOTP_BUFFER_SIZE];
//         uint32_t isotp_rx_size;
//         uint32_t tx_err_cnt = 0;
//         uint32_t rx_err_cnt = 0;
//         uint32_t ti_state_change = 0;
//         BusState bus_state = BusState::PRE_ONLINE;
//         uint32_t ti_last_ping = 0;
//         /* Axis related */
//         bool try_process_gateway_isotp_can_frame(CanFrame &rx_frame);
//         bool try_process_ffb_update_frame(CanFrame &rx_frame);
//         bool try_process_ping_frame(CanFrame &rx_frame, uint32_t now);
//         bool send_payload_to_gateway(const uint8_t *data, uint32_t len);
//         void broadcast_position_limits(void);
//         void broadcast_position_limits(uint32_t now);
//         AxisID own_axis_id = AxisID_AXIS_UNDEFINED;
//         int8_t own_axis_index = -1;
//         IsotpState gateway_isotp_state;
//         OnGatewayPayload on_gateway_payload = nullptr;
//         OnFFBAction on_ffb_action = nullptr;
//         float _x_foot_min = 0.0f;
//         float _x_foot_max = 0.0f;
//         bool _gateway_online = false;
//         /* Gateway related */
//         bool _is_gateway = false;
//         void send_ping_frame(uint32_t now);
//         bool try_process_axis_isotp_can_frame(CanFrame &rx_frame);
//         bool send_payload_to_axis(AxisID axis_id, const uint8_t *data, uint32_t len);
//         bool send_abs_trigger_to_axis(AxisID axis_id);
//         IsotpState isotp_state[MessageTools::MAX_AXES_COUNT];
//         OnAxisPayload on_axis_payload = nullptr;
// };