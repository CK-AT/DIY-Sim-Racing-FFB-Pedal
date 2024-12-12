#include <Arduino.h>
#include <Servo.h>
#include <FastNonAccelStepper.h>
#include "ModbusClientRTU.h"

#define MAXIMUM_SPEED 2000000

class A6Servo : public Servo {
    public:
        A6Servo(uint8_t pin_step, uint8_t pin_dir, bool dir_inverted, HardwareSerial &serial, unsigned long baud, uint32_t config, uint8_t pin_rx, uint8_t pin_tx, uint8_t pin_tx_ena, bool serial_inverted = false);
        void setup(uint32_t steps_per_mm, uint32_t mm_per_rev);
        uint8_t home(void);
        void enable(void);
        void disable(void);
        void write_trq_limit(float limit_percent);
        int8_t move_to(double position, bool blocking = false);
        void move_to_slow(double position);
        double get_min_pos(void) {
            return 0.0;
        }
        double get_max_pos(void) {
            return double(_pos_max) / double(_steps_per_mm * _mm_per_rev);
        }
    private:
        template <typename T>
        ModbusMessage write_hold_register(uint16_t addr, T value) {
            if (sizeof(T) < 2) {
                return ModbusMessage(1, WRITE_HOLD_REGISTER, addr, int16_t(value));
            } else if (sizeof(T) == 2) {
                return ModbusMessage(1, WRITE_HOLD_REGISTER, addr, value);
            } else {
                uint16_t data[2] = {(value >> 16) & 0xFFFF, value & 0xFFFF};
                return ModbusMessage(1, WRITE_MULT_REGISTERS, addr, 2, 4, data);
            }
        }
        template <typename T>
        ModbusMessage read_hold_register(uint16_t addr) {
            if (sizeof(T) <= 2) {
                return ModbusMessage(1, READ_HOLD_REGISTER, addr, 1);
            } else {
                return ModbusMessage(1, READ_HOLD_REGISTER, addr, 2);
            }
        }
        void on_response(ModbusMessage msg, uint32_t token);
        int8_t move_to(int32_t position, bool blocking = false);
        void move_to_slow(int32_t position);
        void write_min_pos(int32_t counts);
        void write_max_pos(int32_t counts);
        int32_t read_min_pos(void);
        int32_t read_max_pos(void);
        float get_speed(void);
        void set_speed(float rpm);
        int32_t read_position(void);
        FastNonAccelStepper* _stepper_engine;
        ModbusClientRTU _modbus;
        uint32_t _steps_per_mm;
        uint32_t _mm_per_rev;
        int32_t _pos_max = 0;
};