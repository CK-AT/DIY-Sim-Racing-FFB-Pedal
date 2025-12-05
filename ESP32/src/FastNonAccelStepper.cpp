#include "FastNonAccelStepper.h"

#include <driver/mcpwm.h>
#include <driver/pcnt.h>

#include "LogOutput.h"

/************************************************************************/
/*								Defines */
/************************************************************************/
#define MAX_SPEED_IN_HZ (int32_t)500000
#define MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE (int32_t)32767
#define PWM_DUTY_CYCLE 50.0f
#define PCNT_MIN_MAX_THRESHOLD 32767  // INT16_MAX = (2^15)-1 = 32767
#define POSITION_TRIGGER_THRESHOLD 1
#define PCNT_FILTER_VALUE 1
#define MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP 50

/************************************************************************/
/*								Implementation */
/************************************************************************/
FastNonAccelStepper::FastNonAccelStepper(uint8_t step_pin, uint8_t dir_pin, bool invert_motor_dir)
    : _step_pin(step_pin),
      _dir_pin(dir_pin),
      _target_position(0),
      _max_speed(MAX_SPEED_IN_HZ),
      _overflow_count(0),
      _pcnt_queue(nullptr),
      _invert_motor_direction(invert_motor_dir),
      _zero_position_i32(0) {
    begin(_step_pin, _dir_pin, _invert_motor_direction);
}

void FastNonAccelStepper::begin(uint8_t step_pin, uint8_t dir_pin, bool invert_motor_dir) {
    // set dir logic for counting
    // 1) If invert_motor_dir == false, the DIR_PIN == HIGH, when motor moving forward and DIR_PIN == LOW, when motor moving backwards
    // 2) If invert_motor_dir == false, the PCNT counter should go up, when DIR_PIN == HIGH
    if (false == invert_motor_dir) {
        _dir_level_forward_b = LOW;
        _dir_level_backward_b = HIGH;
        _dir_pcnt_lctrl_mode_b = PCNT_MODE_KEEP;
        _dir_pcnt_hctrl_mode_b = PCNT_MODE_REVERSE;
    } else {
        _dir_level_forward_b = HIGH;
        _dir_level_backward_b = LOW;
        _dir_pcnt_lctrl_mode_b = PCNT_MODE_REVERSE;
        _dir_pcnt_hctrl_mode_b = PCNT_MODE_KEEP;
    }

    // init MCPWM and PCNTs
    init_mcpwm();
    init_pcnt_multiturn();
    init_pcnt_control();

    pinMode(_step_pin, OUTPUT);
    pinMode(_dir_pin, OUTPUT);
    digitalWrite(_step_pin, LOW);
    digitalWrite(_dir_pin, LOW);

    // configure pin modes
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, _step_pin);

    // connect PCNT to GPIO pin
    gpio_iomux_in(_step_pin, PCNT_SIG_CH0_IN0_IDX);
    gpio_iomux_in(_step_pin, PCNT_SIG_CH0_IN1_IDX);

    // reset pcnt counters
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    // make sure mcpwm is stopped
    force_stop();
}

void FastNonAccelStepper::set_max_speed(uint32_t speed) {
    // Constrain the speed to valid limits
    _max_speed = constrain(speed, 1, MAX_SPEED_IN_HZ);

    // Update the MCPWM timer with the new frequency
    if (_max_speed > 0) {
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, _max_speed);
        force_stop();
    } else {
        force_stop();
    }
}

void FastNonAccelStepper::move(long steps_to_move, bool blocking) {
    // stop previous move
    force_stop();

    long abs_steps_to_move = abs(steps_to_move);

    if (abs_steps_to_move > POSITION_TRIGGER_THRESHOLD) {
        // calculate number of required pcnt wraps
        long numb_wraps = 0;  // abs_steps_to_move / PCNT_MIN_MAX_THRESHOLD;

        long abs_steps_to_move_helper = abs_steps_to_move;
        for (uint8_t idx = 0; idx < 100; idx++) {
            if (abs_steps_to_move_helper > PCNT_MIN_MAX_THRESHOLD) {
                abs_steps_to_move_helper -= PCNT_MIN_MAX_THRESHOLD;
                numb_wraps++;
            } else {
                break;
            }
        }

        long limit;
        if (numb_wraps > 0) {
            limit = abs_steps_to_move / numb_wraps;
        } else {
            limit = abs_steps_to_move;
        }

        int16_t limit_i16 = constrain(limit, 0, PCNT_MIN_MAX_THRESHOLD);

        int16_t high_limit;
        int16_t low_limit;
        // 1) set DIR pin
        // 2) define upper limit for control pcnt
        // 3) define lower limit for control pcnt
        if (steps_to_move > 0) {
            digitalWrite(_dir_pin, _dir_level_forward_b);
            high_limit = limit_i16;  // absPositionChange - 1;
            low_limit = -MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            _overflow_count_control = numb_wraps;
        } else {
            digitalWrite(_dir_pin, _dir_level_backward_b);
            high_limit = MCPWM_PCNT_MAX_ALLOWED_MOVEMENT_IN_OPPOSITE_DIR_TILL_STOP;
            low_limit = -limit_i16;  //-(absPositionChange - 1);
            _overflow_count_control = numb_wraps;
        }

        // parameterize control pcnt
        pcnt_counter_pause(PCNT_UNIT_1);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_H_LIM, high_limit);
        pcnt_set_event_value(PCNT_UNIT_1, PCNT_EVT_L_LIM, low_limit);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
        pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);
        pcnt_counter_clear(PCNT_UNIT_1);
        pcnt_counter_resume(PCNT_UNIT_1);

        // Serial.printf("Hlim: %d,    LLim: %d,    wraps: %d\n", high_limit, low_limit, numb_wraps);

        // start mcpwm
        delayMicroseconds(5);
        _is_running = true;
        mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

        if (blocking) {
            while (is_running()) {
                delay(1);

                /*int16_t pulse_count_local = 0;
                pcnt_get_counter_value(PCNT_UNIT_1, &pulse_count_local);
                Serial.printf( "CurPos: %d,    CtrlPos:%d,    overfl: %d\n", get_current_position(), pulse_count_local, _overflow_count_control);
                delay(30);*/
            }
        }
    }
}

void FastNonAccelStepper::move_to(long target_pos, bool blocking) {
    long current_pos = get_current_position();
    long position_change = constrain(target_pos - current_pos, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);
    _target_position = current_pos + position_change;
    // long position_change = target_pos - current_pos;
    // _target_position = target_pos;
    move(position_change, blocking);
}

void FastNonAccelStepper::move_to_verbose(long target_pos, bool blocking) {
    long current_pos = get_current_position();
    long position_change = constrain(target_pos - current_pos, -MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE, MAX_ALLOWED_POSITION_CHANGE_PER_CYCLE);
    _target_position = current_pos + position_change;
    // long position_change = target_pos - current_pos;
    // _target_position = target_pos;
    LogOutput::printf("%i -> %i (%i)\n", current_pos, _target_position, position_change);
    move(position_change, blocking);
}

long FastNonAccelStepper::get_current_position() const {
    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);
    return ((long)_overflow_count * (long)PCNT_MIN_MAX_THRESHOLD) + (long)pulse_count - _zero_position_i32;
}

void FastNonAccelStepper::init_mcpwm() {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = _max_speed;
    pwm_config.cmpr_a = PWM_DUTY_CYCLE;
    pwm_config.cmpr_b = 0.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void FastNonAccelStepper::init_pcnt_multiturn() {
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = _step_pin;
    pcnt_config.ctrl_gpio_num = _dir_pin;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_0;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;
    pcnt_config.lctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_lctrl_mode_b;
    pcnt_config.hctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_hctrl_mode_b;
    pcnt_config.counter_h_lim = PCNT_MIN_MAX_THRESHOLD;
    pcnt_config.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD;

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(PCNT_UNIT_0, PCNT_FILTER_VALUE);
    pcnt_filter_enable(PCNT_UNIT_0);

    // Activate pcnt
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // PCNT event
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, multiturn_pcnt_isr, this);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
}

void FastNonAccelStepper::init_pcnt_control() {
    pcnt_config_t pcnt_config;
    pcnt_config.pulse_gpio_num = _step_pin;
    pcnt_config.ctrl_gpio_num = _dir_pin;
    pcnt_config.channel = PCNT_CHANNEL_0;
    pcnt_config.unit = PCNT_UNIT_1;
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    pcnt_config.neg_mode = PCNT_COUNT_DIS;
    pcnt_config.lctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_lctrl_mode_b;
    pcnt_config.hctrl_mode = (pcnt_ctrl_mode_t)_dir_pcnt_hctrl_mode_b;
    pcnt_config.counter_h_lim = PCNT_MIN_MAX_THRESHOLD;
    pcnt_config.counter_l_lim = -PCNT_MIN_MAX_THRESHOLD;

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_1, PCNT_FILTER_VALUE);
    pcnt_filter_enable(PCNT_UNIT_1);

    // Activate pcnt
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    // PCNT event
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_isr_handler_add(PCNT_UNIT_1, control_pcnt_isr, this);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

void IRAM_ATTR FastNonAccelStepper::multiturn_pcnt_isr(void* arg) {
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(arg);
    uint32_t status;
    pcnt_get_event_status(PCNT_UNIT_0, &status);

    if (status & PCNT_EVT_H_LIM) {
        instance->_overflow_count++;
    }
    if (status & PCNT_EVT_L_LIM) {
        instance->_overflow_count--;
    }
}

void IRAM_ATTR FastNonAccelStepper::control_pcnt_isr(void* arg) {
    FastNonAccelStepper* instance = static_cast<FastNonAccelStepper*>(arg);
    uint32_t status;
    pcnt_get_event_status(PCNT_UNIT_1, &status);

    // Serial.println("X\n");
    if (status & PCNT_EVT_H_LIM || status & PCNT_EVT_L_LIM) {
        if (instance->_overflow_count_control < 1) {
            instance->force_stop();
        }
        instance->_overflow_count_control--;
    }
}

void FastNonAccelStepper::force_stop() {
    // stop mcpwm
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    _is_running = false;
}

void FastNonAccelStepper::set_current_position(int32_t new_position_i32) {
    // set new position
    // new_position_i32 = (get_current_position + old_zero_pos) - (new_zero_pos)
    // new_zero_pos = (get_current_position + old_zero_pos) - new_position_i32
    int32_t new_zero_pos_i32 = (get_current_position() + _zero_position_i32) - new_position_i32;
    _zero_position_i32 = new_zero_pos_i32;
}

void FastNonAccelStepper::force_stop_and_new_position(int32_t new_position_i32) {
    // stop mcpwm
    force_stop();

    // set new position
    set_current_position(new_position_i32);
}

bool FastNonAccelStepper::is_running() {
    return _is_running;
}

void FastNonAccelStepper::keep_running_in_dir(bool forward_dir, uint32_t speed) {
    force_stop();

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);

    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT_1, PCNT_EVT_L_LIM);

    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);

    set_max_speed(speed);

    if (forward_dir) {
        digitalWrite(_dir_pin, _dir_level_forward_b);
    } else {
        digitalWrite(_dir_pin, _dir_level_backward_b);
    }

    delayMicroseconds(5);
    _is_running = true;
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
}

void FastNonAccelStepper::keep_running_forward(uint32_t speed) {
    keep_running_in_dir(true, speed);
}

void FastNonAccelStepper::keep_running_backward(uint32_t speed) {
    keep_running_in_dir(false, speed);
}

int32_t FastNonAccelStepper::get_position_after_commands_completed() {
    return _target_position;
}
