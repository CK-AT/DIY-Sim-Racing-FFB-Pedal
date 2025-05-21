#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <isotp.h>

#define MAX_AXES 4
#define ISOTP_BUFFER_SIZE 256

class CANManager {
    protected:
        enum AxisFrameTypesHS {
            FORCE_AND_POSITION = 0
        };

        struct ForceAndPosition {
            float f_foot;
            float x_foot;
        };

        struct AxisState {
            struct ForceAndPosition force_and_position;
            uint32_t ti_last_seen;
            bool online;
        };
    
        struct IsotpState {
            IsoTpLink link;
            uint8_t isotp_link_rx_buff[ISOTP_BUFFER_SIZE];
            uint8_t isotp_link_tx_buff[ISOTP_BUFFER_SIZE];
        };

    public:
        bool get_force(int8_t axis_id, float &f_foot);
        bool get_position(int8_t axis_id, float &x_foot);
        bool is_online(int8_t axis_id);

    protected:
        virtual void process(void);
        static void task_func(void* pvParameters) {
            CANManager* manager = (CANManager*) pvParameters;
            delay(1000);
            for (;;) {
                manager->process();
                delay(1);
            }
        }
        AxisState axis_states[MAX_AXES] = {};
        uint8_t isotp_rx_buff[ISOTP_BUFFER_SIZE];
        uint32_t isotp_rx_size;
        uint32_t tx_err_cnt = 0;
        uint32_t rx_err_cnt = 0;

};

class AxisCANManager : public CANManager {
    typedef std::function<void(uint8_t *data, uint32_t len)> OnGatewayPayload;

    public:
        AxisCANManager(void) {};
        void setup(int8_t axis_id, uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnGatewayPayload cb);
        void process(void);
        void send_force_and_position(float &f_foot, float & x_foot);
        bool send_payload_to_gateway(uint8_t *data, uint32_t len);

    private:
        int8_t own_axis_id = -1;
        IsotpState isotp_state;
        OnGatewayPayload on_gateway_payload = nullptr;

};

class GatewayCANManager : public CANManager {
    typedef std::function<void(uint8_t axis_id, uint8_t *data, uint32_t len)> OnAxisPayload;
   
    public:
        GatewayCANManager(void) {};
        void setup(uint16_t baud_rate, int8_t tx_pin, int8_t rx_pin, OnAxisPayload cb);
        bool send_payload_to_axis(uint8_t axis_id, uint8_t *data, uint32_t len);

    private:
        void process(void);
        IsotpState isotp_state[MAX_AXES];
        OnAxisPayload on_axis_payload = nullptr;

};