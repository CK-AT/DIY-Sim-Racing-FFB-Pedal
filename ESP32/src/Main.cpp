
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
#include "Version.h"
#include "Version_Board.h"

/**********************************************************************************************/
/*                                                                                            */
/*                         function declarations                                              */
/*                                                                                            */
/**********************************************************************************************/
void physics_task_func(void *pvParameters);

#include "AutomotivePedalFunction.h"
#include "FlightPedalsFunction.h"
#include "RudderBrake.h"

AutomotivePedalFunction automotive_pedal_function = {};
FlightPedalsFunction flight_pedals_function = {};
RudderBrake rudder_brake = {};

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
KalmanFilter *kalman = NULL;

#include "SignalFilter_2nd_order.h"
KalmanFilter_2nd_order *kalman_2nd_order = NULL;

/**********************************************************************************************/
/*                                                                                            */
/*                         loadcell definitions                                               */
/*                                                                                            */
/**********************************************************************************************/

#include "LoadCell.h"
LoadCell_ADS1256 *loadcell = NULL;

/**********************************************************************************************/
/*                                                                                            */
/*                         servo motor definitions                                          */
/*                                                                                            */
/**********************************************************************************************/

#ifdef A6SERVO
    #include "A6Servo.h"
#endif
Servo *servo = NULL;

/**********************************************************************************************/
/*                                                                                            */
/*                         RGB LED                                                            */
/*                                                                                            */
/**********************************************************************************************/
#ifdef RGB_LED
    #include <NeoPixelBus.h>
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> pixels(NUM_LEDS, RGB_LED);
const RgbColor yellow = RgbColor(46, 34, 0);
const RgbColor green = RgbColor(0, 46, 0);
const RgbColor red = RgbColor(46, 0, 0);
const RgbColor purple = RgbColor(36, 0, 46);
#endif

float m = 0.1;
float x_min = 0.0;
float x_max = 100.0;
float v_min = -1000.0;
float v_max = 1000.0;
float a_min = -100000.0;
float a_max = 100000.0;
Sim sim = Sim(m, x_min, x_max, v_min, v_max, a_min, a_max);
// Spring spring1 = Spring(50.0, 2.0);
// Damper damper1 = Damper(0.1);
// Friction friction1 = Friction(0.0);
// ForceMap force_map1 = ForceMap({0.0, 100.0}, {-100.0, 100.0});
// CompoundElement endstops = CompoundElement();
// ForceMap force_map2 = ForceMap({0.0, 10.0, 90.0, 100.0}, {-100.0, 0.0, 0.0, 100.0});
// DampingMap damping_map1 = DampingMap({0.0, 15.0, 85.0, 100.0}, {3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 3.0});

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

IAuxFunction *get_aux_function(const FunctionConfig *func_cfg) {
    if (!func_cfg->has_aux_function) return nullptr;
    switch (func_cfg->aux_function.which_specific) {
        case AuxFunctionConfig_rudder_brake_tag:
            return &rudder_brake;
            break;
        default:
            break;
    }
    return nullptr;
}

IFunction *on_config_update(IFunction *active_function, const FunctionConfig *function_cfg) {
    if (servo) servo->pause(1000);
    if (active_function) {
        active_function->disable();
    }
    switch (function_cfg->which_specific) {
        case FunctionConfig_automotive_pedal_tag:
            automotive_pedal_function.update_config(function_cfg->specific.automotive_pedal);
            active_function = &automotive_pedal_function;
            break;
        case FunctionConfig_flight_pedals_tag:
            flight_pedals_function.update_config(function_cfg->specific.flight_pedals);
            active_function = &flight_pedals_function;
            break;
    }
    if (active_function) {
        float x_curr;
        comm_manager.get_position(comm_manager.get_axis_id(), x_curr);
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
    pixels.SetPixelColor(0, purple);
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
    if (own_axis_index & 0x0C) {
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
        servo = new A6Servo(stepPinStepper, dirPinStepper, !axis_cfg->b_motor_inverted, Serial1, 115200, SERIAL_8N1, ISV57_RXPIN, ISV57_TXPIN,
                            ISV57_DEPIN, false);
        // disable servo to reduce noise floor for loadcell calibration (might be enabled after a restart)
        servo->disable();
        delay(100);
#endif
        loadcell = new LoadCell_ADS1256();

        loadcell->setLoadcellRating(axis_cfg->f_max_loadcell / 9.81f);  // from N to kg

        loadcell->setZeroPoint();
        loadcell->estimateVariance();  // automatically identify sensor noise for KF parameterization

        // setup Kalman filter
        float var_est = loadcell->getVarianceEstimate();
        kalman = new KalmanFilter(var_est);
        kalman_2nd_order = new KalmanFilter_2nd_order(var_est);

        if (!servo->setup(axis_cfg->steps_per_mm, axis_cfg->mm_per_rev)) {
            LogOutput::printf("Setup: Failed to initialize the servo (check power and connections)");
        } else {
            servo->enable();
            delay(100);
        }

        sim.add_element(&automotive_pedal_function);
        sim.add_element(&flight_pedals_function);

        xTaskCreatePinnedToCore(physics_task_func,    /* Task function. */
                                "PhysicsTask",        /* name of task. */
                                10000,                /* Stack size of task */
                                NULL,                 /* parameter of the task */
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
        pixels.SetPixelColor(0, green);
    } else if (config_manager.is_axis_config_valid()) {
        if (servo) {
            if (servo->get_state() == Servo::State::Disabled) {
                pixels.SetPixelColor(0, red);
            } else if (servo->is_locked_in()) {
                pixels.SetPixelColor(0, green);
            } else {
                pixels.SetPixelColor(0, yellow);
            }
        } else {
            pixels.SetPixelColor(0, red);
        }
    } else {
        pixels.SetPixelColor(0, red);
    }
    pixels.Show();
#endif
}

/**********************************************************************************************/
/*                                                                                            */
/*                         pedal update task                                                  */
/*                                                                                            */
/**********************************************************************************************/

// long lastCallTime = micros();
void physics_task_func(void *pvParameters) {
    uint32_t ti_prev = micros();
    float dt = 1000.0;
    const AxisConfig *axis_cfg = config_manager.get_axis_config();
    float x_contact_point = 0.0;
    float f_contact_point = 0.0;

    comm_manager.on_physics_task_start();

    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, 10) == 0) {
            continue;
        }

        if (config_manager.try_take_config_semaphore() == false) {
            continue;
        }

        // print the execution time averaged over multiple cycles
        static CycleTimer timerPU("PU cycle time");
        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timerPU.BumpStart();
        }

        // Get the loadcell reading
        float loadcellReading = loadcell->getReadingKg();

        unsigned long now = micros();
        dt = (now - ti_prev) / 1000.0;
        ti_prev = now;

        // Invert the loadcell reading digitally if desired
        if (axis_cfg->b_loadcell_inverted) {
            loadcellReading *= -1.0;
        }

        // Do the loadcell signal filtering
        float filteredReading = 0;

        // const velocity model denoising filter
        switch (axis_cfg->which_load_cell_filter_config) {
            case AxisConfig_kf_const_vel_tag:
                filteredReading = kalman->filteredValue(loadcellReading, 0, axis_cfg->load_cell_filter_config.kf_const_vel.noise_scaling);
                break;
            case AxisConfig_kf_const_accel_tag:
                filteredReading = kalman_2nd_order->filteredValue(loadcellReading, 0, axis_cfg->load_cell_filter_config.kf_const_accel.noise_scaling);
                break;
            case AxisConfig_filter_none_tag:
                filteredReading = loadcellReading;
                break;
            default:
                break;
        }

        float f_loadcell = filteredReading * 9.81;

        float r_conv = config_manager.calc_force_conversion_factor(x_contact_point);

        f_contact_point = f_loadcell * r_conv;

#ifdef HAS_CAN
        /* CommManager is designed to run CANManager's main processing from within the pedal task to ensure minimum latency on other axes' position
         * and force values */
        comm_manager.process();
#endif

        float f_in;
        if (comm_manager.calc_input_force_sum(f_contact_point, f_in)) {
            // calc_input_force_sum returns true if this is a subractive axis -> invert result
            f_in *= -1.0f;
        }

        sim.update(dt, f_in);

        comm_manager.calc_final_position(sim.get_x(), x_contact_point);

        float x_sled = config_manager.calc_sled_position(x_contact_point);

        config_manager.release_config_semaphore();

        servo->move_to(x_sled);

        // #define DEBUG_FILTER
        if (debug_flags & DEBUG_INFO_0_LOADCELL_READING) {
            static uint16_t loop_cnt = 0;
            static RTDebugOutput<9> rtDebugFilter({"raw", "flt", "f_in", "f_contact_point", "f_sum", "a", "v", "x", "x_sled"});
            loop_cnt++;
            if (loop_cnt >= 20) {
                loop_cnt = 0;
                rtDebugFilter.offerData(
                    {loadcellReading, filteredReading, f_in, f_contact_point, sim.get_f_sum(), sim.get_a(), sim.get_v(), x_contact_point, x_sled});
            }
        }

        comm_manager.send_force_and_position(f_contact_point, x_contact_point);

        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timerPU.BumpEnd();
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
        default:
            break;
    }
}
