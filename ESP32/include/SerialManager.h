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
