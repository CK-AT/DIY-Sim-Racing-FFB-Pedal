
/* Todo*/
// https://github.com/espressif/arduino-esp32/issues/7779

#define DEBUG_INFO_0_CYCLE_TIMER 1
#define DEBUG_INFO_0_STEPPER_POS 2
#define DEBUG_INFO_0_LOADCELL_READING 4
#define DEBUG_INFO_0_SERVO_READINGS 8
#define DEBUG_INFO_0_PRINT_ALL_SERVO_REGISTERS 16
#define DEBUG_INFO_0_STATE_BASIC_INFO_STRUCT 32
#define DEBUG_INFO_0_STATE_EXTENDED_INFO_STRUCT 64
#define DEBUG_INFO_0_CONTROL_LOOP_ALGO 128

#include "Main.h"

#include "Arduino.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"
#include "OscillationGuard.h"
#include "StaticBalancer.h"
#include "Version.h"
#include "Version_Board.h"

/**********************************************************************************************/
/*                                                                                            */
/*                         function declarations                                              */
/*                                                                                            */
/**********************************************************************************************/
void physics_task_func(void *pv_parameters);

#include "AutomotivePedalFunction.h"
#include "FlightPedalsFunction.h"
#include "FlightStickFunction.h"
#include "RudderBrake.h"
#include "ShifterDetect.h"
#include "ShifterFunction.h"

AutomotivePedalFunction automotive_pedal_function = {};
FlightPedalsFunction flight_pedals_function = {};
FlightStickFunction flight_stick_pitch_function = {};
FlightStickFunction flight_stick_roll_function = {};
RudderBrake rudder_brake = {};
ShifterDetect shifter_detect = {};
ShifterFunction shifter_function = {};

#include "CycleTimer.h"
#include "LogOutput.h"
#include "RTDebugOutput.h"

uint8_t debug_flags = 0;

/**********************************************************************************************/
/*                                                                                            */
/*                         multitasking  definitions                                          */
/*                                                                                            */
/**********************************************************************************************/
#ifndef CONFIG_IDF_TARGET_ESP32S3
    #include "soc/rtc_wdt.h"
#endif

TaskHandle_t physics_task_handle;

/**********************************************************************************************/
/*                                                                                            */
/*                         target-specific  definitions                                       */
/*                                                                                            */
/**********************************************************************************************/

/**********************************************************************************************/
/*                                                                                            */
/*                         Kalman filter definitions                                          */
/*                                                                                            */
/**********************************************************************************************/

#include "SignalFilter.h"
KalmanFilter *kalman_filter = nullptr;

#include "SignalFilter_2nd_order.h"
KalmanFilterSecondOrder *kalman_second_order = nullptr;

/**********************************************************************************************/
/*                                                                                            */
/*                         load_cell definitions                                               */
/*                                                                                            */
/**********************************************************************************************/

#include "LoadCell.h"
LoadCellAds1256 *load_cell = nullptr;

/**********************************************************************************************/
/*                                                                                            */
/*                         servo motor definitions                                          */
/*                                                                                            */
/**********************************************************************************************/

#ifdef A6SERVO
    #include "A6Servo.h"
#endif
Servo *servo = nullptr;

static Servo::HomingDirection to_homing_direction(const AxisConfig *axis_cfg) {
    if (!axis_cfg) return A6Servo::HomingDirection::Negative;
    if (axis_cfg->homing_direction == HomingDirection_HOMING_DIR_POSITIVE) {
        return Servo::HomingDirection::Positive;
    }
    return Servo::HomingDirection::Negative;
}

/**********************************************************************************************/
/*                                                                                            */
/*                         RGB LED                                                            */
/*                                                                                            */
/**********************************************************************************************/
#ifdef RGB_LED
    #include <NeoPixelBus.h>
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> pixels(NUM_LEDS, RGB_LED);
const RgbColor k_yellow = RgbColor(46, 34, 0);
const RgbColor k_green = RgbColor(0, 46, 0);
const RgbColor k_red = RgbColor(46, 0, 0);
const RgbColor k_purple = RgbColor(36, 0, 46);
#endif

Sim sim = Sim(1.0, 0.0, 0.0);
StaticBalancer static_balancer = StaticBalancer();
CompoundElement function_elements = CompoundElement();
Friction friction = Friction(2.0);
OscillationGuard oscillation_guard = OscillationGuard();

#include "CommManager.h"
#include "ConfigManager.h"
#include "MessageTools.h"

CommManager comm_manager;

ConfigManager config_manager;

void IRAM_ATTR adc_isr(void) {
    if (physics_task_handle) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(physics_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void on_ffb_action(const FFBAction &ffb_action);
void on_axis_action(const AxisAction &axis_action, CommChannel comm_channel);
void on_ota_state_change(bool ota_active);

static void apply_oscillation_guard_config(const AxisConfig *axis_cfg) {
    constexpr float k_default_k_max = 0.5f;
    constexpr float k_default_min_amplitude = 0.2f;
    constexpr float k_default_min_velocity = 0.5f;
    constexpr float k_default_min_frequency_hz = 2.0f;
    constexpr float k_default_max_frequency_hz = 100.0f;
    constexpr uint32_t k_default_hold_time_ms = 150;
    constexpr uint32_t k_default_ramp_time_ms = 80;
    constexpr uint8_t k_default_required_hits = 2;

    float k_max = k_default_k_max;
    float min_amplitude = k_default_min_amplitude;
    float min_velocity = k_default_min_velocity;
    float min_frequency_hz = k_default_min_frequency_hz;
    float max_frequency_hz = k_default_max_frequency_hz;
    uint32_t hold_time_ms = k_default_hold_time_ms;
    uint32_t ramp_time_ms = k_default_ramp_time_ms;
    uint8_t required_hits = k_default_required_hits;

    if (axis_cfg && axis_cfg->has_oscillation_guard) {
        const AxisConfig_OscillationGuard &guard_cfg = axis_cfg->oscillation_guard;
        k_max = guard_cfg.k_max;
        min_amplitude = guard_cfg.min_amplitude;
        min_velocity = guard_cfg.min_velocity;
        min_frequency_hz = guard_cfg.min_frequency_hz;
        max_frequency_hz = guard_cfg.max_frequency_hz;
        hold_time_ms = guard_cfg.hold_time_ms;
        ramp_time_ms = guard_cfg.ramp_time_ms;
        required_hits = static_cast<uint8_t>(guard_cfg.required_hits);
    }

    if (min_frequency_hz <= 0.0f) {
        min_frequency_hz = k_default_min_frequency_hz;
    }
    if (max_frequency_hz <= 0.0f) {
        max_frequency_hz = k_default_max_frequency_hz;
    }

    float freq_low = min(min_frequency_hz, max_frequency_hz);
    float freq_high = max(min_frequency_hz, max_frequency_hz);

    uint32_t min_half_period_us = static_cast<uint32_t>(1000000.0f / (2.0f * freq_high));
    uint32_t max_half_period_us = static_cast<uint32_t>(1000000.0f / (2.0f * freq_low));
    uint64_t hold_time_us = static_cast<uint64_t>(hold_time_ms) * 1000ULL;
    uint64_t ramp_time_us = static_cast<uint64_t>(ramp_time_ms) * 1000ULL;
    if (hold_time_us > UINT32_MAX) {
        hold_time_us = UINT32_MAX;
    }
    if (ramp_time_us > UINT32_MAX) {
        ramp_time_us = UINT32_MAX;
    }

    oscillation_guard.set_damping_gain(k_max);
    oscillation_guard.set_detection(max(min_amplitude, 0.0f), max(min_velocity, 0.0f),
                                    min_half_period_us, max_half_period_us);
    oscillation_guard.set_timing(static_cast<uint32_t>(hold_time_us), static_cast<uint32_t>(ramp_time_us));
    oscillation_guard.set_required_hits(required_hits < 1 ? 1 : required_hits);
}

IAuxFunction *get_aux_function(const FunctionConfig *func_cfg) {
    if (!func_cfg->has_aux_function) return nullptr;
    switch (func_cfg->aux_function.which_specific) {
        case AuxFunctionConfig_rudder_brake_tag:
            return &rudder_brake;
            break;
        case AuxFunctionConfig_shifter_detect_tag:
            return &shifter_detect;
            break;
        default:
            break;
    }
    return nullptr;
}

IFunction *on_config_update(IFunction *active_function, const FunctionConfig *function_cfg) {
    const AxisConfig *axis_cfg = config_manager.get_axis_config();
    if (servo) {
        servo->pause(1000);
        servo->set_reversed(axis_cfg->b_motor_inverted);
        servo->set_homing_direction(to_homing_direction(axis_cfg));
    }
    apply_oscillation_guard_config(axis_cfg);

    if (active_function) {
        active_function->disable();
    }
    active_function = nullptr;
    if (function_cfg->base.function_id != FunctionID_FUNCTION_ID_UNDEFINED) {
        switch (function_cfg->which_specific) {
            case FunctionConfig_automotive_pedal_tag:
                automotive_pedal_function.update_config(function_cfg->specific.automotive_pedal);
                active_function = &automotive_pedal_function;
                break;
            case FunctionConfig_flight_pedals_tag:
                flight_pedals_function.update_config(function_cfg->specific.flight_pedals);
                active_function = &flight_pedals_function;
                break;
            case FunctionConfig_flight_stick_pitch_tag:
                flight_stick_pitch_function.update_config(function_cfg->specific.flight_stick_pitch);
                active_function = &flight_stick_pitch_function;
                break;
            case FunctionConfig_flight_stick_roll_tag:
                flight_stick_roll_function.update_config(function_cfg->specific.flight_stick_roll);
                active_function = &flight_stick_roll_function;
                break;
            case FunctionConfig_shifter_tag:
                shifter_function.update_config(function_cfg->specific.shifter, function_cfg->aux_function.specific.shifter_detect, comm_manager, function_cfg->base.linked_axes);
                active_function = &shifter_function;
                break;
            default:
                break;
        }
    }
    const AxisConfig_StaticBalanceConfig *static_balance_cfg =
        (axis_cfg && axis_cfg->has_static_balance_config) ? &axis_cfg->static_balance_config : nullptr;
    const FunctionConfig_StaticBalanceTuning *static_balance_tuning =
        (function_cfg && function_cfg->has_static_balance_tuning) ? &function_cfg->static_balance_tuning : nullptr;
    static_balancer.update_config(static_balance_cfg, static_balance_tuning);
    if (active_function) {
        float x_curr;
        comm_manager.get_position(comm_manager.get_axis_id(), x_curr);
        friction.set_f(max(function_cfg->friction, 0.0f));
        sim.set_m(max(function_cfg->simulated_mass, 0.05f));
        sim.set_x_min(x_curr, true);
        sim.set_x_max(x_curr, true);
        sim.set_x_min(active_function->get_x_contact_point_min());
        sim.set_x_max(active_function->get_x_contact_point_max());
        comm_manager.update_position_limits(sim.get_x_min(), sim.get_x_max());
        comm_manager.update_function_id(function_cfg->base.function_id);
        active_function->enable();
    }
    return active_function;
}

/**********************************************************************************************/
/*                                                                                            */
/*                         setup function                                                     */
/*                                                                                            */
/**********************************************************************************************/
void setup() {
#ifdef RGB_LED
    pixels.Begin();
    pixels.SetPixelColor(0, k_purple);
    pixels.Show();
#endif

#if PCB_VERSION == 6
    Serial.setTxTimeoutMs(0);
#else
    Serial.setRxBufferSize(1024);
    Serial.setTimeout(5);
    Serial.begin(3000000);
#endif

    CommManager::CANConfig can_config = {.baud_rate = 1000, .tx_pin = CAN_TX, .rx_pin = CAN_RX};

    comm_manager.setup(&Serial, can_config, &config_manager, on_ffb_action, on_axis_action);
    comm_manager.set_ota_state_callback(on_ota_state_change);

    LogOutput::printf("**************************************************************************************************************");
    LogOutput::printf("This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.");
    LogOutput::printf("Please check github repo for more detail: https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal");
    LogOutput::printf("Board: %s", CONTROL_BOARD);
    LogOutput::printf("FW Version: %s (%s)", VERSION, BUILD_TIMESTAMP);
    // TODO: printout the github releasing version

#ifdef PEDAL_ASSIGNMENT
    uint8_t own_axis_index = 0;
    #ifdef CFG1
    pinMode(CFG1, INPUT_PULLUP);
    own_axis_index |= (~digitalRead(CFG1)) & 0x01;
    #endif
    #ifdef CFG2
    pinMode(CFG2, INPUT_PULLUP);
    own_axis_index |= (~digitalRead(CFG2) << 1) & 0x02;
    #endif
    #ifdef CFG3
    pinMode(CFG3, INPUT_PULLUP);
    own_axis_index |= (~digitalRead(CFG3) << 2) & 0x04;
    #endif
    #ifdef CFG4
    pinMode(CFG4, INPUT_PULLUP);
    own_axis_index |= (~digitalRead(CFG4) << 3) & 0x08;
    #endif
    if ((own_axis_index & 0x0C) == 0x0C) {
        // bits 2 and 3 are set, this is a Gateway
        GatewayID gateway_id = GatewayID((own_axis_index & 0x03) + 1);
        config_manager.init(gateway_id, get_aux_function);
    } else if (own_axis_index < MessageTools::MAX_AXES_COUNT) {
        LogOutput::printf("Setup: Identified as axis %d", own_axis_index + 1);
        config_manager.init(AxisID(own_axis_index + 1), true, on_config_update, get_aux_function);
    } else {
        LogOutput::printf("Setup: Assignment error, axis id = %d (max. %d)", own_axis_index + 1, MessageTools::MAX_AXES_COUNT);
    }
#else
    config_manager.init(AxisID_AXIS_UNDEFINED, false, on_config_update);
#endif

    if (config_manager.is_axis()) {
        // wait 200ms for a CAN gateway to be detected (ping interval is 100ms)
        // log messages will be forwarded once a gateway has been detected
        delay(200);

        // we are an axis right now, seeting up ADC, servo and physics task
        const AxisConfig *axis_cfg = config_manager.get_axis_config();

#ifdef A6SERVO
        servo = new A6Servo(stepPinStepper, dirPinStepper, false, Serial1, 115200, SERIAL_8N1, ISV57_RXPIN, ISV57_TXPIN,
                            ISV57_DEPIN, false);
        servo->set_reversed(axis_cfg->b_motor_inverted);
        servo->set_homing_direction(to_homing_direction(axis_cfg));
        // disable servo to reduce noise floor for load cell calibration (might be enabled after a restart)
        servo->disable();
        delay(100);
#endif
        load_cell = new LoadCellAds1256();

        load_cell->set_loadcell_rating(axis_cfg->f_max_loadcell / 9.81f);  // from N to kg

        load_cell->set_zero_point();
        load_cell->estimate_variance();  // automatically identify sensor noise for KF parameterization

        // setup Kalman filter
        float var_est = load_cell->get_variance_estimate();
        kalman_filter = new KalmanFilter(var_est);
        kalman_second_order = new KalmanFilterSecondOrder(var_est);

        if (!servo->setup(axis_cfg->steps_per_mm, axis_cfg->mm_per_rev)) {
            LogOutput::printf("Setup: Failed to initialize the servo (check power and connections)");
        } else {
            servo->enable();
            delay(100);
        }

        function_elements.add_element(&automotive_pedal_function);
        function_elements.add_element(&flight_pedals_function);
        function_elements.add_element(&flight_stick_pitch_function);
        function_elements.add_element(&flight_stick_roll_function);
        function_elements.add_element(&shifter_function);
        sim.add_element(&static_balancer);
        sim.add_element(&function_elements);
        sim.add_element(&friction);
        sim.add_element(&oscillation_guard);

        xTaskCreatePinnedToCore(physics_task_func,    /* Task function. */
                                "PhysicsTask",        /* name of task. */
                                10000,                /* Stack size of task */
                                nullptr,              /* parameter of the task */
                                10,                   /* priority of the task */
                                &physics_task_handle, /* Task handle to keep track of created task */
                                1);                   /* pin task to core 1 */

        enableCore1WDT();

        attachInterrupt(PIN_DRDY, &adc_isr, FALLING);
    } else {
        // we are NOT an axis right now but either a (dedicated) gateway or an axis w/o ID DIP switches missing a proper
        // AxisConfig, there is no point in setting up things like servo or ADC
        // Wait a bit to allow CommManager to initialize CANManager properly before ending setup()
        delay(500);
    }

    LogOutput::printf("Setup: done");
}

/**********************************************************************************************/
/*                                                                                            */
/*                         Main function                                                      */
/*                                                                                            */
/**********************************************************************************************/
unsigned long joystick_state_last_update = millis();
void loop() {
    delay(1000);
#ifdef RGB_LED
    if (config_manager.get_mode() == ConfigManager::MODE_GATEWAY_ONLY) {
        pixels.SetPixelColor(0, k_green);
    } else if (config_manager.is_axis_config_valid()) {
        if (servo) {
            if (servo->get_state() == Servo::State::Disabled) {
                pixels.SetPixelColor(0, k_red);
            } else if (servo->is_locked_in()) {
                pixels.SetPixelColor(0, k_green);
            } else {
                pixels.SetPixelColor(0, k_yellow);
            }
        } else {
            pixels.SetPixelColor(0, k_red);
        }
    } else {
        pixels.SetPixelColor(0, k_red);
    }
    pixels.Show();
#endif

    StaticBalanceResultData result = {};
    if (static_balancer.calibration_done(result)) {
        Message msg = Message_init_zero;
        msg.which_payload = Message_static_balance_result_tag;
        msg.payload.static_balance_result.axis_id = config_manager.get_axis_id();
        msg.payload.static_balance_result.x_min = result.x_min;
        msg.payload.static_balance_result.x_max = result.x_max;
        msg.payload.static_balance_result.sample_step = result.step;
        msg.payload.static_balance_result.f_offset_count = static_cast<pb_size_t>(result.count);
        for (uint16_t idx = 0; idx < result.count; idx++) {
            msg.payload.static_balance_result.f_offset[idx] = result.samples[idx];
        }
        comm_manager.send_message_to_host(msg);
        if (servo) {
            servo->pause(1000);
        }
        function_elements.enable();
    }
}

/**********************************************************************************************/
/*                                                                                            */
/*                         pedal update task                                                  */
/*                                                                                            */
/**********************************************************************************************/

// long lastCallTime = micros();
void physics_task_func(void *pv_parameters) {
    (void)pv_parameters;
    uint32_t ti_prev = micros();
    float dt = 1000.0;
    const AxisConfig *axis_cfg = config_manager.get_axis_config();
    float x_contact_point = 0.0;
    float f_contact_point = 0.0;
    float x_sled = 0.0;
    float f_in = 0.0;

    comm_manager.on_physics_task_start();

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, 10) == 0) {
            continue;
        }

        if (config_manager.try_take_config_semaphore() == false) {
            continue;
        }

        // print the execution time averaged over multiple cycles
        static CycleTimer timer_pu("PU cycle time");
        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timer_pu.bump_start();
        }

        // Get the load_cell reading
        float load_cell_reading = load_cell->get_reading_kg();

        unsigned long now = micros();
        dt = (now - ti_prev) / 1000.0;
        ti_prev = now;

        // Invert the load_cell reading digitally if desired
        if (axis_cfg->b_loadcell_inverted) {
            load_cell_reading *= -1.0;
        }

        // Do the load_cell signal filtering
        float filtered_reading = 0;

        // const velocity model denoising filter
        switch (axis_cfg->which_load_cell_filter_config) {
            case AxisConfig_kf_const_vel_tag:
                filtered_reading = kalman_filter->filtered_value(load_cell_reading, 0, axis_cfg->load_cell_filter_config.kf_const_vel.noise_scaling);
                break;
            case AxisConfig_kf_const_accel_tag:
                filtered_reading =
                    kalman_second_order->filtered_value(load_cell_reading, 0, axis_cfg->load_cell_filter_config.kf_const_accel.noise_scaling);
                break;
            case AxisConfig_filter_none_tag:
                filtered_reading = load_cell_reading;
                break;
            default:
                break;
        }

        float f_load_cell = filtered_reading * 9.81;

        float r_conv = config_manager.calc_force_conversion_factor(x_contact_point);
        float f_raw_contact = f_load_cell * r_conv;
        f_contact_point = static_balancer.get_force(x_contact_point, f_raw_contact);

#ifdef HAS_CAN
        /* CommManager is designed to run CANManager's main processing from within the pedal task to ensure minimum latency on other axes' position
         * and force values */
        comm_manager.process();
#endif

        if (config_manager.get_function_id() != FunctionID_FUNCTION_ID_UNDEFINED || static_balancer.is_calibrating()) {
            if (comm_manager.calc_input_force_sum(f_contact_point, f_in)) {
                // calc_input_force_sum returns true if this is a subtractive axis -> invert result
                f_in *= -1.0f;
            }

            uint16_t num_sub_iterations = max(axis_cfg->physics_iterations_per_sample, uint16_t(1));
            float dt_sub = dt / num_sub_iterations;

            for (uint8_t i = 0; i < num_sub_iterations; i++) {
                sim.update(dt_sub, f_in);
            }

            comm_manager.calc_final_position(sim.get_x(), x_contact_point);
        }

        x_sled = config_manager.calc_sled_position(x_contact_point);

        config_manager.release_config_semaphore();

        servo->move_to(x_sled);

        // #define DEBUG_FILTER
        if (debug_flags & DEBUG_INFO_0_LOADCELL_READING) {
            static uint16_t loop_cnt = 0;
            static RTDebugOutput<9> rt_debug_filter({"raw", "flt", "f_in", "f_contact_point", "f_sum", "a", "v", "x", "x_sled"});
            loop_cnt++;
            if (loop_cnt >= 20) {
                loop_cnt = 0;
                rt_debug_filter.offer_data(
                    {load_cell_reading, filtered_reading, f_in, f_contact_point, sim.get_f_sum(), sim.get_a(), sim.get_v(), x_contact_point, x_sled});
            }
        }

        comm_manager.send_force_and_position(f_contact_point, x_contact_point);

        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timer_pu.bump_end();
        }
    }
}

void on_ffb_action(const FFBAction &ffb_action) {
    IFunction *active_function = config_manager.get_active_function();
    if (active_function) {
        active_function->on_ffb_action(ffb_action);
    }
}

void on_axis_action(const AxisAction &axis_action, CommChannel comm_channel) {
    switch (axis_action.which_action) {
        case AxisAction_restart_tag:
            if (servo) servo->pause();
            ESP.restart();
            break;
        case AxisAction_debug_flags_tag:
            debug_flags = axis_action.action.debug_flags;
            break;
        case AxisAction_start_static_balance_calibration_tag:
            function_elements.disable();
            static_balancer.start_calibration(sim.get_x_min(), sim.get_x_max());
            break;
        default:
            break;
    }
}

void on_ota_state_change(bool ota_active) {
    if (!servo) return;
    if (ota_active) {
        servo->pause();
    } else {
        servo->resume();
    }
}
