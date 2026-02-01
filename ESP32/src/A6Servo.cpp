#include <A6Servo.h>
#include <LogOutput.h>

// Fault code lookup table: code -> {description, resettable}
struct FaultInfo {
    uint16_t code;
    const char* description;
    bool resettable;
};

static const FaultInfo fault_table[] = {
    // Class 1: Non-resettable hardware faults
    {0x0600, "Er06.0 Runaway protection", false},
    {0x1400, "Er20.0 Encoder disconnected", false},
    {0x1401, "Er20.1 Encoder internal fault", false},
    {0x1500, "Er21.0 Encoder count error", false},
    {0x1600, "Er22.0 Encoder multi-turn overflow", false},
    {0x1700, "Er23.0 Encoder battery low", false},
    {0x1701, "Er23.1 Encoder battery disconnected", false},
    // Class 2: Resettable faults
    {0x2800, "Er40.0 Drive overload", true},
    {0x2900, "Er41.0 Motor overload", true},
    {0x2901, "Er41.1 Motor over-temp (locked rotor)", true},
    {0x2A00, "Er42.0 IGBT temp too high", true},
    {0x2A02, "Er42.2 Heatsink temp too high", true},
    {0x2B00, "Er43.0 Main circuit overvoltage", true},
    {0x2B01, "Er43.1 Main circuit undervoltage", true},
    {0x2C00, "Er44.0 Brake resistor overload", true},
    {0x2D00, "Er45.0 Motor wire break", true},
    {0x2E00, "Er46.0 Motor overspeed", true},
    {0x2F00, "Er47.0 Position deviation overflow (static)", true},
    {0x2F01, "Er47.1 Position deviation overflow (running)", true},
    {0x3000, "Er48.0 Full-closed loop deviation overflow", true},
    {0x3100, "Er49.0 Output phase loss", true},
    {0x3200, "Er50.0 IPM protection", true},
    {0x3400, "Er52.0 Output short-circuit", true},
    {0x3500, "Er53.0 Brake circuit fault", true},
    {0x3800, "Er56.0 Inrush resistor fault", true},
    // Class 3: Alarms (resettable)
    {0xF200, "ALF2.0 Forward overtravel", true},
    {0xF201, "ALF2.1 Reverse overtravel", true},
    {0xF400, "ALF4.0 Homing timeout", true},
    {0xF401, "ALF4.1 Homing interrupted", true},
    {0xF600, "ALF6.0 Input phase loss", true},
    {0xFA00, "ALFA.0 Drive high temp warning", true},
    {0, nullptr, false}  // sentinel
};

static const char* get_fault_description(uint16_t code, bool* resettable) {
    for (const FaultInfo* f = fault_table; f->description != nullptr; f++) {
        if (f->code == code) {
            if (resettable) *resettable = f->resettable;
            return f->description;
        }
    }
    if (resettable) *resettable = (code >= 0x2800);  // Class 2+ are generally resettable
    return nullptr;
}

void A6Servo::periodic_task_func(void) {
    if (ti_pause_end && (esp_timer_get_time() > ti_pause_end)) {
        ti_pause_end = 0;
        resume();
    }
    if (_homing_state == HomingState::Pending && _state == State::Enabled) {
        do_homing();
    } else if (!_locking_blocked && _homing_state == HomingState::Homed && _state == State::Enabled) {
        if (_curr_pos_valid) {
            lock_onto_curr_pos();
        }
    }

    // Poll fault register (U41.00 = 0x4100)
    uint16_t fault_code = 0;
    if (read_hold_register<uint16_t>(0x4100, fault_code) == Modbus::Error::SUCCESS) {
        if (fault_code != _last_fault_code) {
            if (fault_code != 0) {
                bool resettable = false;
                const char* desc = get_fault_description(fault_code, &resettable);
                if (desc) {
                    LogOutput::printf("A6Servo: FAULT %s [%s]", desc, resettable ? "resettable" : "non-resettable");
                } else {
                    LogOutput::printf("A6Servo: FAULT 0x%04X (unknown)", fault_code);
                }
            } else if (_last_fault_code != 0) {
                LogOutput::printf("A6Servo: Fault cleared");
            }
            _last_fault_code = fault_code;
        }
    }
}

void A6Servo::on_response(ModbusMessage msg, uint32_t token) {
}

A6Servo::A6Servo(uint8_t pin_step, uint8_t pin_dir, bool dir_inverted, HardwareSerial &serial, unsigned long baud, uint32_t config, int8_t pin_rx,
                 int8_t pin_tx, int8_t pin_tx_ena, bool serial_inverted) {
    RTUutils::prepareHardwareSerial(serial);
    serial.begin(baud, config, pin_rx, pin_tx, serial_inverted);  // Modbus serial
    _modbus = new ModbusClientRTU(pin_tx_ena);
    _modbus->onResponseHandler(std::bind(&A6Servo::on_response, this, std::placeholders::_1, std::placeholders::_2));
    _modbus->setTimeout(10);
    _modbus->begin(serial, 0);
    _stepper_engine = new FastNonAccelStepper(pin_step, pin_dir, !dir_inverted);
    _stepper_engine->set_max_speed(MAXIMUM_SPEED);
}

bool A6Servo::check_required_registers(void) {
    uint16_t value;
    bool success = true;
    // read RS485 EEPROM storage flag
    if (read_hold_register<uint16_t>(0x0A05, value) != Modbus::Error::SUCCESS) {
        return false;
    }
    if (value != 0) {
        LogOutput::printf(" -> Register 0x0A05 (store RS485 register writes to EEPROM) is not 0!");
        success = false;
    }
    if (read_hold_register<uint16_t>(0x0A06, value) != Modbus::Error::SUCCESS) {
        return false;
    }
    if (value != 1) {
        LogOutput::printf(" -> Register 0x0A06 (register order) is not 1!");
        success = false;
    }
    if (read_hold_register<uint16_t>(0x0022, value) != Modbus::Error::SUCCESS) {
        return false;
    }
    if (value != 1) {
        LogOutput::printf(" -> A6Servo: Register 0x0022 (pulse channel selection) is not 1!");
        success = false;
    }
    return success;
}

bool A6Servo::setup(uint32_t steps_per_mm, uint32_t mm_per_rev, bool autohome) {
    return setup(steps_per_mm, mm_per_rev, autohome, _homing_direction);
}

bool A6Servo::setup(uint32_t steps_per_mm, uint32_t mm_per_rev, bool autohome, HomingDirection homing_dir) {
    LogOutput::printf("A6Servo: Performing setup...");
    _steps_per_mm = steps_per_mm;
    _mm_per_rev = mm_per_rev;
    _homing_direction = homing_dir;
    if (!check_required_registers()) {
        LogOutput::printf(" -> failed");
        return false;
    }
    disable();
    delay(100);
    write_hold_register<uint32_t>(0x0122, 50);                         // 5.0ms LPF on position input
    write_hold_register<uint32_t>(0x0304, steps_per_mm * mm_per_rev);  // gear ratio denominator (steps_per_rev)
    write_hold_register<uint32_t>(0x0306, 131072);                     // gear ratio numerator (encoder counts per rev)
    write_hold_register<uint16_t>(0x0607, 2);                          // limit active after homing
    write_hold_register<uint32_t>(0x0600, 1000000);                    // relax excessive local position deviation threshold
    write_trq_limit(_trq_open_loop);
    set_speed(_spd_open_loop);
    int32_t min_pos = read_min_pos();
    int32_t max_pos = read_max_pos();
    bool homed_already = (min_pos > -2000000);
    if (homed_already) {
        float travel_mm = (max_pos - min_pos) / float(_steps_per_mm);
        LogOutput::printf(" -> Endstop min @ %i, max @ %i (travel = %.3f mm), homed already.", min_pos, max_pos, travel_mm);
        _pos_min = min_pos;
        _pos_max = max_pos;
        _stepper_engine->set_current_position(read_position());
        _homing_state = HomingState::Homed;
    } else {
        LogOutput::printf(" -> Endstop min @ %i, max @ %i, not homed.", min_pos, max_pos);
        if (autohome) {
            _homing_state = HomingState::Pending;
        }
    }
    xTaskCreatePinnedToCore(this->task_func, "A6ServoTask", 5000, this, 1, nullptr, 0);
    LogOutput::printf(" -> done");
    return true;
}

bool A6Servo::enable(void) {
    auto resp = write_hold_register<int16_t>(0x0411, 1);  // enable
    if (resp == Modbus::Error::SUCCESS) {
        _state = State::Enabled;
        return true;
    }
    return false;
}

bool A6Servo::disable(void) {
    auto resp = write_hold_register<int16_t>(0x0411, 0);  // disable
    if (resp == Modbus::Error::SUCCESS) {
        _state = State::Disabled;
        return true;
    }
    return false;
}

bool A6Servo::home(void) {
    if (_state != State::Enabled) return false;
    _homing_state = HomingState::Pending;
    return true;
}

void A6Servo::do_homing(void) {
    _state = State::Homing;
    _homing_state = HomingState::HomeUnknown;
    _stepper_engine->set_current_position(0);
    write_min_pos(-20000000);
    write_max_pos(20000000);
    write_trq_limit(_trq_open_loop);
    write_homing_trq_limit(_trq_open_loop);
    set_speed(_spd_open_loop);
    write_hold_register<uint32_t>(0x0600, 1000000);  // relax excessive local position deviation threshold
    const bool homing_negative = _homing_direction == HomingDirection::Negative;
    const char *first_endstop = homing_negative ? "negative" : "positive";
    const char *second_endstop = homing_negative ? "positive" : "negative";
    // C10.01 (0x1001) homing modes (A6-RS manual Table 4-14):
    // | Mode | Meaning                                                            |
    // | -2   | Forward to mech limit, then Z pulse                                |
    // | -1   | Reverse to mech limit, then Z pulse                                |
    // | 1    | Reverse to NL, slow back to Z                                      |
    // | 2    | Forward to PL, slow back to Z                                      |
    // | 3-8  | Use HSW transitions (ON/OFF) to find limit, then Z (dir per mode)  |
    // | 9-14 | Always fwd/rev; use HSW transition to limit, then Z (dir per mode) |
    // | 17-30| Same as 1-14 but stop at limit (no Z search)                       |
    // | 33   | Reverse; nearest Z pulse                                            |
    // | 34   | Forward; nearest Z pulse                                            |
    // | 35   | Use current position as home                                        |
    // | 15,16,31,32 | Reserved                                                     |
    auto wait_for_stop = [this](uint8_t required_consecutive, uint16_t poll_ms) {
        uint8_t num_zero_spd = 0;
        float speed;
        while (num_zero_spd < required_consecutive) {
            delay(poll_ms);
            speed = get_speed();
            if (abs(speed) < 2) {
                num_zero_spd++;
            } else {
                num_zero_spd = 0;
            }
        }
    };
    // Start drive-controlled homing first
    // Mode 35 = current position as home (skip mechanical limit + Z pulse search)
    // This avoids Er47.1 position deviation overflow during Z pulse reversal on some axes
    // The stepper verification phase below handles actual endstop detection
    write_hold_register<int16_t>(0x1001, 35);
    write_hold_register<uint16_t>(0x1000, 0);        // homing off
    delay(100);
    write_hold_register<uint16_t>(0x1000, 1);  // homing on
    LogOutput::printf("A6Servo: Setting current position as home, starting stepper verification...");
    delay(200);  // brief delay for mode 35 to complete
    write_hold_register<uint16_t>(0x1000, 0);  // reset homing command
    int32_t pos_start = read_position();
    LogOutput::printf("A6Servo: Home set @ %i counts, searching for %s endstop...", pos_start, first_endstop);

    // Find first endstop using stepper
    uint32_t homing_speed_counts = (_steps_per_mm * _mm_per_rev) * (_spd_open_loop / 60.0);  // unified homing speed
    if (homing_negative) {
        _stepper_engine->keep_running_backward(homing_speed_counts);
    } else {
        _stepper_engine->keep_running_forward(homing_speed_counts);
    }
    wait_for_stop(10, 100);
    _stepper_engine->force_stop();
    const int32_t margin = 2500;
    int32_t pos_endstop_first = read_position();
    LogOutput::printf("A6Servo: %s endstop found @ %i counts", first_endstop, pos_endstop_first);
    _stepper_engine->move_to(pos_endstop_first, true);
    // Move to second endstop
    LogOutput::printf("A6Servo: Moving to %s endstop...", second_endstop);
    if (homing_negative) {
        _stepper_engine->keep_running_forward(homing_speed_counts);
    } else {
        _stepper_engine->keep_running_backward(homing_speed_counts);
    }
    wait_for_stop(5, 100);
    _stepper_engine->force_stop();
    int32_t pos_endstop_second = read_position();
    float travel_mm = (abs(float(pos_endstop_second - pos_endstop_first)) - (2 * margin)) / float(_steps_per_mm);
    if (travel_mm > 20.0f) {
        LogOutput::printf("A6Servo: %s endstop found @ %i counts, total travel within margins = %.3f mm", second_endstop, pos_endstop_second, travel_mm);
        _pos_min = homing_negative ? (pos_endstop_first + margin) : (pos_endstop_second + margin);
        _pos_max = homing_negative ? (pos_endstop_second - margin) : (pos_endstop_first - margin);
        _stepper_engine->move_to(_pos_max, true);
        write_min_pos(_pos_min);
        write_max_pos(_pos_max);
        write_hold_register<uint16_t>(0x1000, 0);  // reset homing command
        _homing_state = HomingState::Homed;
        _state = State::Enabled;
        LogOutput::printf("A6Servo: Homing done.");
    } else {
        LogOutput::printf("A6Servo: Homing failed, sled did not move far enough from %s endstop (only %.3f mm).", first_endstop, travel_mm);
        _homing_state = HomingState::HomeUnknown;
        _state = State::Enabled;
        write_hold_register<uint16_t>(0x1000, 0);  // reset homing command
    }
}

void A6Servo::lock_onto_curr_pos(void) {
    write_trq_limit(_trq_open_loop);
    set_speed(_spd_open_loop);
    LogOutput::printf("A6Servo: Locking onto last commanded position...");
    uint16_t max_tries = 10000;
    bool is_locked = false;
    while (!is_locked && max_tries) {
        // FastNonAccelStepper does a maximum of 32767 steps at once for now
        move_to(get_target_pos(), true);
        is_locked = abs(get_target_pos() - read_position()) < 10;
        max_tries--;
        delay(2);
    }
    if (is_locked) {
        LogOutput::printf("A6Servo: Locked in.");
        write_hold_register<uint32_t>(0x0600, 30000);  // tighten excessive local position deviation threshold
        set_speed(_spd_locked_in);
        write_trq_limit(_trq_locked_in);
        _homing_state = HomingState::LockedIn;
    } else {
        LogOutput::printf("A6Servo: Failed to lock in, commanded position probably out of bounds.");
        _homing_state = HomingState::LockingError;
    }
}

int32_t A6Servo::get_target_pos() {
    return logical_to_counts(_curr_pos);
}

int32_t A6Servo::logical_to_counts(float logical_mm) const {
    int32_t span_counts = abs(_pos_max - _pos_min);
    float clamped = constrain(logical_mm, 0.0f, float(span_counts) / float(_steps_per_mm));
    int32_t counts = int32_t(clamped * float(_steps_per_mm));
    counts = constrain(counts, 0, span_counts);
    return _reverse_motion ? (_pos_max - counts) : (_pos_min + counts);
}

void A6Servo::write_trq_limit(float limit_percent) {
    write_hold_register<uint16_t>(0x0343, uint16_t(limit_percent * 10.0));  // negative limit (0.1%/LSB)
    write_hold_register<uint16_t>(0x0344, uint16_t(limit_percent * 10.0));  // positive limit (0.1%/LSB)
}

void A6Servo::write_homing_trq_limit(float limit_percent) {
    write_hold_register<uint16_t>(0x1030, uint16_t(limit_percent * 10.0));  // 0.1%/LSB
}

void A6Servo::write_min_pos(int32_t counts) {
    write_hold_register<int32_t>(0x060A, counts);  // negative position limit
}

void A6Servo::write_max_pos(int32_t counts) {
    write_hold_register<int32_t>(0x0608, counts);  // positive position limit
}

float A6Servo::get_speed(void) {
    int16_t value = 0;
    auto resp = read_hold_register<int16_t>(0x4001, value);
    if (resp != Modbus::Error::SUCCESS) {
        return NAN;
    }
    return float(value);
}

int32_t A6Servo::read_min_pos(void) {
    int32_t value;
    auto resp = read_hold_register<int32_t>(0x060A, value);
    return value;
}

int32_t A6Servo::read_max_pos(void) {
    int32_t value;
    auto resp = read_hold_register<int32_t>(0x0608, value);
    return value;
}

void A6Servo::move_to_slow(int32_t position) {
    set_speed(_spd_open_loop);
    _stepper_engine->move_to(constrain(position, _pos_min, _pos_max), true);
    set_speed(_spd_locked_in);
}

bool A6Servo::move_to(int32_t position, bool blocking) {
    _stepper_engine->move_to(constrain(position, _pos_min, _pos_max), blocking);
    return true;
}

void A6Servo::move_to_slow(float position) {
    float max_pos_mm = float(abs(_pos_max - _pos_min)) / float(_steps_per_mm);
    _curr_pos = constrain(position, 0.0f, max_pos_mm);
    _curr_pos_valid = true;
    if (_state == State::Enabled && _homing_state == HomingState::LockedIn) {
        move_to_slow(logical_to_counts(_curr_pos));
    }
}

bool A6Servo::move_to(float position, bool blocking) {
    float max_pos_mm = float(abs(_pos_max - _pos_min)) / float(_steps_per_mm);
    _curr_pos = constrain(position, 0.0f, max_pos_mm);
    _curr_pos_valid = true;
    if (_state == State::Enabled && _homing_state == HomingState::LockedIn) {
        return move_to(logical_to_counts(_curr_pos), blocking);
    }
    return false;
}

void A6Servo::set_speed(float rpm) {
    _stepper_engine->set_max_speed(uint32_t((_steps_per_mm * _mm_per_rev) * rpm / 60.0));
}

int32_t A6Servo::read_position(void) {
    int32_t value;
    auto resp = read_hold_register<int32_t>(0x4016, value);
    return value;
}
