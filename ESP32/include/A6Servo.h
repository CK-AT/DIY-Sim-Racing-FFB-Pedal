#include <Arduino.h>
#include <Servo.h>
#include <FastNonAccelStepper.h>
#include "ModbusClientRTU.h"
#include <LogOutput.h>

#define MAXIMUM_SPEED 2000000

class A6Servo : public Servo {
    public:
        A6Servo(uint8_t pin_step, uint8_t pin_dir, bool dir_inverted, HardwareSerial &serial, unsigned long baud, uint32_t config, uint8_t pin_rx, uint8_t pin_tx, uint8_t pin_tx_ena, bool serial_inverted = false);
        bool setup(uint32_t steps_per_mm, uint32_t mm_per_rev, bool autohome=true);
        bool home(void);
        bool enable(void);
        bool disable(void);
        void write_trq_limit(float limit_percent);
        bool move_to(float position, bool blocking = false);
        void move_to_slow(float position);
        float get_min_pos(void) {
            return 0.0;
        }
        float get_max_pos(void) {
            return float(_pos_max) / float(_steps_per_mm * _mm_per_rev);
        }
        void periodic_task_func(void);
        void lock_onto_curr_pos(void);

    private:
        template <typename T>
        Modbus::Error write_hold_register(uint16_t addr, T value) {
            ModbusMessage request;
            if (sizeof(T) < 2) {
                request = ModbusMessage(1, WRITE_HOLD_REGISTER, addr, int16_t(value));
            } else if (sizeof(T) == 2) {
                request = ModbusMessage(1, WRITE_HOLD_REGISTER, addr, value);
            } else {
                uint16_t data[2] = {uint16_t((value >> 16) & 0xFFFF), uint16_t(value & 0xFFFF)};
                request = ModbusMessage(1, WRITE_MULT_REGISTERS, addr, 2, 4, data);
            }
            ModbusMessage response;
            uint8_t retries = 3;
            while (retries) {
                response = _modbus.syncRequest(request, 0);
                if (response.getError() == Modbus::Error::SUCCESS) return Modbus::Error::SUCCESS;
                retries--;
            }
            LogOutput::printf("write_hold_register(0x%04X) failed after 3 retries!\n", addr);
            return response.getError();
        }
        template <typename T>
        Modbus::Error read_hold_register(uint16_t addr, T &value) {
            ModbusMessage request;
            if (sizeof(T) <= 2) {
                request = ModbusMessage(1, READ_HOLD_REGISTER, addr, 1);
            } else {
                request = ModbusMessage(1, READ_HOLD_REGISTER, addr, 2);
            }
            ModbusMessage response;
            uint8_t retries = 3;
            while (retries) {
                response = _modbus.syncRequest(request, 0);
                if (response.getError() == Modbus::Error::SUCCESS) {
                    response.get(3, value);
                    return Modbus::Error::SUCCESS;
                }
                retries--;
            }
            LogOutput::printf("read_hold_register(0x%04X) failed after 3 retries!\n", addr);
            return response.getError();
        }
        void on_response(ModbusMessage msg, uint32_t token);
        bool move_to(int32_t position, bool blocking = false);
        void move_to_slow(int32_t position);
        void write_min_pos(int32_t counts);
        void write_max_pos(int32_t counts);
        void write_homing_trq_limit(float limit_percent);
        bool check_required_registers(void);
        int32_t read_min_pos(void);
        int32_t read_max_pos(void);
        float get_speed(void);
        void set_speed(float rpm);
        int32_t read_position(void);
        void do_homing(void);
        int32_t get_target_pos();
        FastNonAccelStepper* _stepper_engine;
        ModbusClientRTU _modbus;
        uint32_t _steps_per_mm;
        uint32_t _mm_per_rev;
        int32_t _pos_max = 0;
        float _trq_locked_in = 300.0;
        float _trq_open_loop = 10.0;
        static void task_func(void* pvParameters) {
            A6Servo* servo = (A6Servo*) pvParameters;
            delay(1000);
            for (;;) {
                servo->periodic_task_func();
                delay(100);
            }
        }
};