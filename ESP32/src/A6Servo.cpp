#include <A6Servo.h>
#include <LogOutput.h>

void A6Servo::periodic_task_func(void) {
    if (_homing_state == HomingState::Pending && _state == State::Enabled) {
        do_homing();
    } else if (_homing_state == HomingState::Homed && _state == State::Enabled) {
        if (_curr_pos_valid) {
            lock_onto_curr_pos();
        }
    }
}

A6Servo::A6Servo(uint8_t pin_step, uint8_t pin_dir, bool dir_inverted, HardwareSerial &serial, unsigned long baud, uint32_t config, uint8_t pin_rx, uint8_t pin_tx, uint8_t pin_tx_ena, bool serial_inverted) {
    serial.begin(baud, config, pin_rx, pin_tx, serial_inverted); // Modbus serial
    _modbus = new Modbus(serial);
    _modbus->init(pin_tx_ena, false);
    _stepper_engine = new FastNonAccelStepper(pin_step, pin_dir, dir_inverted); 
    _stepper_engine->setMaxSpeed(MAXIMUM_SPEED);
}

void A6Servo::setup(uint32_t steps_per_mm, uint32_t mm_per_rev) {
    _steps_per_mm = steps_per_mm;
    _mm_per_rev = mm_per_rev;
    disable();
    delay(100);
    _modbus->holdingRegisterWriteI32(1, 0x0122, 10); // 1.0ms LPF on position input

    _modbus->holdingRegisterWriteI32(1, 0x0304, steps_per_mm * mm_per_rev); // gear ratio denominator (steps_per_rev)
    _modbus->holdingRegisterWriteI32(1, 0x0306, 131072); // gear ratio numerator (encoder counts per rev)
    _modbus->holdingRegisterWrite(1, 0x0607, 2); // limit active after homing
    _modbus->holdingRegisterWriteI32(1, 0x0600, 1000000); // relax excessive local position deviation threshold
    write_trq_limit(5.0);
    set_speed(100.0);
    int32_t min_pos = read_min_pos();
    if (min_pos == 0) {
        _pos_max = read_max_pos();
        Serial.printf("Negative endstop @ %i, positive endstop @ %i, homed already.\n", min_pos, _pos_max);
        _stepper_engine->setCurrentPosition(read_position());
        _homing_state = HomingState::Homed;
    } else {
        Serial.printf("Negative endstop @ %i, not homed.\n", min_pos);
    }
    xTaskCreatePinnedToCore(this->task_func, "A6ServoTask", 5000, this, 1, NULL, 0);
}

void A6Servo::enable(void) {
    _modbus->holdingRegisterWrite(1, 0x0411, 1); // enable
    _state = State::Enabled;
}

void A6Servo::disable(void) {
    _state = State::Disabled;
    _modbus->holdingRegisterWrite(1, 0x0411, 0); // enable
}

uint8_t A6Servo::home(void) {
    if (_state != State::Enabled) return 0;
    _homing_state = HomingState::Pending;
    return 1;
}

void A6Servo::do_homing(void) {
    _state = State::Homing;
    _homing_state = HomingState::HomeUnknown;
    _stepper_engine->setCurrentPosition(0);
    write_min_pos(-20000000);
    write_max_pos(20000000);
    write_trq_limit(5.0);
    set_speed(100.0);
    _modbus->holdingRegisterWriteI32(1, 0x0600, 1000000); // excessive local position deviation threshold
    _modbus->holdingRegisterWrite(1, 0x1000, 0); // homing off
    delay(100);
    _modbus->holdingRegisterWrite(1, 0x1000, 1); // homing on
    LogOutput::printf("Waiting for negative endstop...\n");
    int num_zero_spd = 0;
    float speed;
    while (num_zero_spd < 20)
    {
        delay(100);
        speed = get_speed();
        if (abs(speed) < 2) {
            num_zero_spd++;
        } else {
            num_zero_spd = 0;
        }
    }
    LogOutput::printf("Negative endstop found, moving to positive endstop...\n");
    _stepper_engine->keepRunningForward((_steps_per_mm * _mm_per_rev) / 0.6);
    num_zero_spd = 0;
    while (num_zero_spd < 2)
    {
        delay(100);
        speed = get_speed();
        LogOutput::printf("%.3f mm @ %.0f rpm\n", double(read_position()) / double(_steps_per_mm), speed);
        if (abs(speed) < 2) {
            num_zero_spd++;
        } else {
            num_zero_spd = 0;
        }
    }
    _stepper_engine->forceStop();
    int32_t pos_endstop = read_position();
    LogOutput::printf("Positive endstop found @ %.3f mm\n", double(pos_endstop) / double(_steps_per_mm));
    _pos_max = pos_endstop - 2500;
    _stepper_engine->moveTo(_pos_max, true);
    write_min_pos(0);
    write_max_pos(_pos_max);
    _modbus->holdingRegisterWrite(1, 0x1000, 0); // reset homing command
    _homing_state = HomingState::Homed;
    _state = State::Enabled;
    LogOutput::printf("Homing done.\n");
}

void A6Servo::lock_onto_curr_pos(void) {
    LogOutput::printf("Locking onto last commanded position...\n");
    uint8_t max_tries = 10;
    bool is_locked = false;
    while (!is_locked && max_tries)
    {
        // FastNonAccelStepper does a maximum of 32767 steps at once for now
        move_to(get_target_pos(), true);
        is_locked = abs(get_target_pos() - _stepper_engine->getCurrentPosition()) < 5;
        max_tries--;
        delay(2);
    }
    if (is_locked) {
        LogOutput::printf("Locked in.\n");
        _modbus->holdingRegisterWriteI32(1, 0x0600, 30000); // tighten excessive local position deviation threshold
        set_speed(6000.0);
        write_trq_limit(300.0);
        _homing_state = HomingState::LockedIn;
    } else {
        LogOutput::printf("Failed to lock in, commanded position probably out of bounds.\n");
        _homing_state = HomingState::LockingError;
    }
}

int32_t A6Servo::get_target_pos()
{
    return int32_t(_curr_pos * double(_steps_per_mm));
}

void A6Servo::write_trq_limit(float limit_percent) {
    _modbus->holdingRegisterWrite(1, 0x0343, uint16_t(limit_percent * 10.0)); // negative limit (0.1%/LSB)
    _modbus->holdingRegisterWrite(1, 0x0344, uint16_t(limit_percent * 10.0)); // positive limit (0.1%/LSB)
}

void A6Servo::write_min_pos(int32_t counts) {
    _modbus->holdingRegisterWriteI32(1, 0x060A, counts); // negative position limit
}

void A6Servo::write_max_pos(int32_t counts) {
    _modbus->holdingRegisterWriteI32(1, 0x0608, counts); // positive position limit
}

float A6Servo::get_speed(void) {
    for (int8_t i = 0; i < 3; i++) {
        int16_t val;
        if (!_modbus->holdingRegisterRead<int16_t>(1, 0x4001, val)) {
            return float(val);
        }
    }
    LogOutput::printf("A6Servo::get_speed() failed after 3 retries!\n");
    return NAN;
}

int32_t A6Servo::read_min_pos(void) {
    for (int8_t i = 0; i < 3; i++) {
        int32_t val;
        if (!_modbus->holdingRegisterRead<int32_t>(1, 0x060A, val)) {
            return val;
        }
    }
    LogOutput::printf("A6Servo::read_min_pos() failed after 3 retries!\n");
    return 0;
}

int32_t A6Servo::read_max_pos(void) {
    for (int8_t i = 0; i < 3; i++) {
        int32_t val;
        if (!_modbus->holdingRegisterRead<int32_t>(1, 0x0608, val)) {
            return val;
        }
    }
    LogOutput::printf("A6Servo::read_max_pos() failed after 3 retries!\n");
    return 0;
}

void A6Servo::move_to_slow(int32_t position) {
    set_speed(10.0);
    _stepper_engine->moveTo(constrain(position, 0, _pos_max), true);
    set_speed(6000.0);
}

int8_t A6Servo::move_to(int32_t position, bool blocking) {
    _stepper_engine->moveTo(constrain(position, 0, _pos_max), blocking);
    return 1;
}

void A6Servo::move_to_slow(double position) {
    _curr_pos = position;
    _curr_pos_valid = true;
    if (_state == State::Enabled && _homing_state == HomingState::LockedIn) {
        move_to_slow(int32_t(position * double(_steps_per_mm)));
    }
}

int8_t A6Servo::move_to(double position, bool blocking) {
    _curr_pos = position;
    _curr_pos_valid = true;
    if (_state == State::Enabled && _homing_state == HomingState::LockedIn) {
        return move_to(int32_t(position * double(_steps_per_mm)), blocking);
    }
    return 0;
}

void A6Servo::set_speed(float rpm) {
    _stepper_engine->setMaxSpeed(uint32_t((_steps_per_mm * _mm_per_rev) * rpm / 60.0));    
}

int32_t A6Servo::read_position(void) {
    for (int8_t i = 0; i < 3; i++) {
        int32_t val;
        if (!_modbus->holdingRegisterRead<int32_t>(1, 0x4016, val)) {
            return val;
        }
    }
    LogOutput::printf("A6Servo::read_position() failed after 3 retries!\n");
    return 0;
}

