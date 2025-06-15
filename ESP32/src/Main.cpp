
/* Todo*/
// https://github.com/espressif/arduino-esp32/issues/7779

#define ESTIMATE_LOADCELL_VARIANCE
// #define ISV_COMMUNICATION
// #define PRINT_SERVO_STATES

#define DEBUG_INFO_0_CYCLE_TIMER 1
#define DEBUG_INFO_0_STEPPER_POS 2
#define DEBUG_INFO_0_LOADCELL_READING 4
#define DEBUG_INFO_0_SERVO_READINGS 8
#define DEBUG_INFO_0_PRINT_ALL_SERVO_REGISTERS 16
#define DEBUG_INFO_0_STATE_BASIC_INFO_STRUCT 32
#define DEBUG_INFO_0_STATE_EXTENDED_INFO_STRUCT 64
#define DEBUG_INFO_0_CONTROL_LOOP_ALGO 128

// #define OTA_update

#include "Main.h"

#include "Arduino.h"
#include "ConfigManager.h"
#include "IFunction.h"
#include "Physics.h"
#include "Version_Board.h"

#ifdef Using_analog_output_ESP32_S3
    #include <Adafruit_MCP4725.h>
    #include <Wire.h>
TwoWire MCP4725_I2C = TwoWire(1);
// MCP4725 MCP(0x60, &MCP4725_I2C);
Adafruit_MCP4725 dac;
int current_use_mcp_index;
bool MCP_status = false;
#endif

// #define ALLOW_SYSTEM_IDENTIFICATION

/**********************************************************************************************/
/*                                                                                            */
/*                         function declarations                                              */
/*                                                                                            */
/**********************************************************************************************/
void physics_task_func(void *pvParameters);
void OTATask(void *pvParameters);
void ESPNOW_SyncTask(void *pvParameters);

#include "AutomotivePedalFunction.h"
#include "FlightPedalsFunction.h"

AutomotivePedalFunction automotive_pedal_function = {};
FlightPedalsFunction flight_pedals_function = {};

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
/*                         controller  definitions                                            */
/*                                                                                            */
/**********************************************************************************************/

#include "Controller.h"

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
/*                         OTA                                                                */
/*                                                                                            */
/**********************************************************************************************/
// OTA update
#ifdef OTA_update
    #include "ota.h"
TaskHandle_t Task4;
char *APhost;
#endif

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

// ESPNOW
#ifdef ESPNOW_Enable
    #include "ESPNOW_lib.h"
TaskHandle_t ESPNowTask;
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

#ifdef USB_JOYSTICK
    SetupController();
    delay(100);
#endif

#if PCB_VERSION == 6
    Serial.setTxTimeoutMs(0);
#else
    Serial.setRxBufferSize(1024);
    Serial.setTimeout(5);
    Serial.begin(3000000);
#endif

    CommManager::CANConfig can_config = {
        .baud_rate = 1000,
        .tx_pin = CAN_TX,
        .rx_pin = CAN_RX
    };

    comm_manager.setup(&Serial, can_config, &config_manager, on_ffb_action, on_axis_action);

    LogOutput::printf("**************************************************************************************************************");
    LogOutput::printf("This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.");
    LogOutput::printf("Please check github repo for more detail: https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal");
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
        config_manager.init(gateway_id);
    } else if (own_axis_index < MessageTools::MAX_AXES_COUNT) {
        LogOutput::printf("Setup: Identified as axis %d", own_axis_index + 1);
        config_manager.init(AxisID(own_axis_index + 1), true, on_config_update);
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
#ifdef ESTIMATE_LOADCELL_VARIANCE
        loadcell->estimateVariance();  // automatically identify sensor noise for KF parameterization
#endif

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

#ifdef OTA_update

    switch (dap_config_st.payLoadPedalConfig_.pedal_type) {
        case 0:
            APhost = "FFBPedalClutch";
            break;
        case 1:
            APhost = "FFBPedalBrake";
            break;
        case 2:
            APhost = "FFBPedalGas";
            break;
        default:
            APhost = "FFBPedal";
            break;
    }

    xTaskCreatePinnedToCore(OTATask, "OTATask", 16000,
                            // STACK_SIZE_FOR_TASK_2,
                            NULL, 1, &Task4, 0);
    delay(500);

#endif

// MCP setup
#ifdef Using_analog_output_ESP32_S3
    // Wire.begin(MCP_SDA,MCP_SCL,400000);
    MCP4725_I2C.begin(MCP_SDA, MCP_SCL, 400000);
    uint8_t i2c_address[8] = {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67};
    int index_address = 0;
    int found_address = 0;
    int error;
    for (index_address = 0; index_address < 8; index_address++) {
        MCP4725_I2C.beginTransmission(i2c_address[index_address]);
        error = MCP4725_I2C.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at address");
            Serial.print(i2c_address[index_address]);
            Serial.println("  !");
            found_address = index_address;
            break;

        } else {
            Serial.print("try address");
            Serial.println(i2c_address[index_address]);
        }
    }

    if (dac.begin(i2c_address[found_address], &MCP4725_I2C) == false) {
        Serial.println("Couldn't find MCP, will not have analog output");
        MCP_status = false;
    } else {
        Serial.println("MCP founded");
        MCP_status = true;
        // MCP.begin();
    }
#endif

// enable ESP-NOW
#ifdef ESPNOW_Enable
    dap_calculationVariables_st.rudder_brake_status = false;

    dap_state_basic_st.payLoadHeader_.PedalTag = dap_config_st.payLoadPedalConfig_.pedal_type;

    if (dap_config_st.payLoadPedalConfig_.pedal_type == 0 || dap_config_st.payLoadPedalConfig_.pedal_type == 1 ||
        dap_config_st.payLoadPedalConfig_.pedal_type == 2) {
        ESPNow_initialize();
        xTaskCreatePinnedToCore(ESPNOW_SyncTask, "ESPNOW_update_Task", 5000,
                                // STACK_SIZE_FOR_TASK_2,
                                NULL, 1, &ESPNowTask, 0);
        delay(500);
    }

#endif

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
    /*
  #ifdef OTA_update
  server.handleClient();
  //delay(1);
  #endif
  */
}

void calc_poly(const float &in, float &out, const double *coeffs) {
    double result = coeffs[0];
    double temp = in;
    for (uint8_t i = 1; i < 5; i++) {
        result += temp * coeffs[i];
        temp *= in;
    }
    out = result;
}

float get_input_force_sum(float own_force) {
    comm_manager.update_force(own_force);
    const FunctionBase &func_base = config_manager.get_function_config()->base;
    float f_sum = 0.0f;
    float temp;
    bool is_subtractive_axis = false;
    AxisID own_axis_id = config_manager.get_axis_id();
    for (uint8_t idx = 0; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
        AxisID axis_id = AxisID(func_base.linked_axes[idx] & AxisID_AXIS_ID_MASK);
        if (axis_id == AxisID_AXIS_UNDEFINED) break;
        temp = 0.0f;
        comm_manager.get_force(axis_id, temp);  // get_force won't touch temp if the associated axis is not online, no need to check the return value
        if (func_base.linked_axes[idx] & AxisID_AXIS_SUBTRACTIVE) {
            if (axis_id == own_axis_id) is_subtractive_axis = true;
            f_sum -= temp;
        } else {
            f_sum += temp;
        }
    }
    if (is_subtractive_axis) {
        f_sum *= -1.0f;
    }
    return f_sum;
}

bool get_final_position(float own_position, float &final_position) {
    const FunctionBase &func_base = config_manager.get_function_config()->base;
    float other_position;
    AxisID primary_axis_id = AxisID(func_base.linked_axes[0] & AxisID_AXIS_ID_MASK);
    AxisID own_axis_id = config_manager.get_axis_id();
    if (primary_axis_id == own_axis_id) {
        // we are the primary axis -> own_position is the final position
        final_position = own_position;
        return true;
    } else if (comm_manager.get_position(primary_axis_id, other_position)) {
        // we are NOT the primary axis, start at idx 1
        for (uint8_t idx = 1; idx < (sizeof(FunctionBase::linked_axes) / sizeof(FunctionBase::linked_axes[0])); idx++) {
            AxisID axis_id = AxisID(func_base.linked_axes[idx] & AxisID_AXIS_ID_MASK);
            if (axis_id == own_axis_id) {
                if (func_base.linked_axes[idx] & AxisID_AXIS_SUBTRACTIVE) {
                    final_position = (config_manager.get_x_contact_point_center() * 2.0f) - other_position;
                    return true;
                } else {
                    final_position = other_position;
                    return true;
                }
            } else if (axis_id == AxisID_AXIS_UNDEFINED) {
                break;
            }
        }
    }
    return false;
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
    const FunctionConfig *function_cfg = config_manager.get_function_config();
    float x_foot = 0.0;
    float f_foot = 0.0;

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
            default:
                break;
        }

        float f_loadcell = filteredReading * 9.81;

        float r_conv;
        calc_poly(x_foot, r_conv, axis_cfg->kinematic_parameters.coeffs_force_factor_over_contact_point_pos);

        f_foot = f_loadcell * r_conv;

#ifdef HAS_CAN
        /* AxisCANManager is designed to run its main processing from within the pedal task to ensure minimum latency on other axes' position and
         * force values */
        comm_manager.process();
#endif

        float f_in = get_input_force_sum(f_foot);

        sim.update(dt, f_in);

        get_final_position(sim.get_x(), x_foot);

        float x_sled;
        calc_poly(x_foot, x_sled, axis_cfg->kinematic_parameters.coeffs_sled_pos_over_contact_point_pos);

        config_manager.release_config_semaphore();

        servo->move_to(x_sled);

        // #define DEBUG_FILTER
        if (debug_flags & DEBUG_INFO_0_LOADCELL_READING) {
            static uint16_t loop_cnt = 0;
            static RTDebugOutput<9> rtDebugFilter({"raw", "flt", "f_in", "f_foot", "f_sum", "a", "v", "x", "x_sled"});
            loop_cnt++;
            if (loop_cnt >= 20) {
                loop_cnt = 0;
                rtDebugFilter.offerData({loadcellReading, filteredReading, f_in, f_foot, sim.get_f_sum(), sim.get_a(), sim.get_v(), x_foot, x_sled});
            }
        }

        comm_manager.send_force_and_position(f_foot, x_foot);

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

void send_axis_config(CommChannel comm_channel) {
    Message msg;
    config_manager.get_axis_config_as_message(msg);
    comm_manager.send_message_to_gateway(msg, comm_channel);
}

void send_function_config(CommChannel comm_channel) {
    Message msg;
    config_manager.get_function_config_as_message(msg);
    comm_manager.send_message_to_gateway(msg, comm_channel);
}

void on_axis_action(const AxisAction &axis_action, CommChannel comm_channel) {
    switch (axis_action.which_action) {
        case AxisAction_restart_tag:
            if (servo) servo->pause();
            ESP.restart();
            break;
        case AxisAction_return_axis_config_tag:
            send_axis_config(comm_channel);
            break;
        case AxisAction_return_function_config_tag:
            send_function_config(comm_channel);
            break;
        case AxisAction_debug_flags_tag:
            debug_flags = axis_action.action.debug_flags;
        default:
            break;
    }
}

// OTA multitask

uint16_t OTA_count = 0;
bool message_out_b = false;
bool OTA_enable_start = false;
void OTATask(void *pvParameters) {
    for (;;) {
#ifdef OTA_update
        if (OTA_count > 200) {
            message_out_b = true;
            OTA_count = 0;
        } else {
            OTA_count++;
        }

        if (OTA_enable_b) {
            if (message_out_b) {
                message_out_b = false;
                Serial1.println("OTA enable flag on");
            }
            if (OTA_status) {
                server.handleClient();
            } else {
                Serial.println("de-initialize espnow");
                Serial.println("wait...");
                esp_err_t result = esp_now_deinit();
                ESPNow_initial_status = false;
                ESPNOW_status = false;
                delay(200);
                if (result == ESP_OK) {
                    OTA_status = true;
                    delay(1000);
                    ota_wifi_initialize(APhost);
                }
            }
        }

        delay(1);
#endif
    }
}

#ifdef ESPNOW_Enable
int ESPNOW_count = 0;
int error_count = 0;
int print_count = 0;
int ESPNow_no_device_count = 0;
bool basic_state_send_b = false;
bool extend_state_send_b = false;
uint8_t error_out;

int64_t timeNow_espNowTask_l = 0;
int64_t timePrevious_espNowTask_l = 0;
    #define REPETITION_INTERVAL_ESPNOW_TASK (int64_t)2

uint Pairing_timeout = 20000;
bool Pairing_timeout_status = false;
bool building_dap_esppairing_lcl = false;
unsigned long Pairing_state_start;
unsigned long Pairing_state_last_sending;
unsigned long Debug_rudder_last = 0;

uint32_t espNowTask_stackSizeIdx_u32 = 0;
void ESPNOW_SyncTask(void *pvParameters) {
    for (;;) {
        // if(ESPNOW_status)

        delay(1);

        // restart from espnow
        if (ESPNow_restart) {
            Serial.println("ESP restart by ESP now request");
            ESP.restart();
        }

        // basic state sendout interval
        if (ESPNOW_count % 18 == 0) {
            basic_state_send_b = true;
        }
        // entend state send out interval
        if (ESPNOW_count % 26 == 0 && dap_config_st.payLoadPedalConfig_.debug_flags_0 == DEBUG_INFO_0_STATE_EXTENDED_INFO_STRUCT) {
            extend_state_send_b = true;
        }

        ESPNOW_count++;
        if (ESPNOW_count > 10000) {
            ESPNOW_count = 0;
        }

        if (ESPNow_initial_status == false) {
            if (OTA_enable_b == false) {
                ESPNow_initialize();
            }

        } else {
    #ifdef ESPNow_Pairing_function
        #ifdef Hardware_Pairing_button
            if (digitalRead(Pairing_GPIO) == LOW) {
                hardware_pairing_action_b = true;
            }
        #endif
            if (hardware_pairing_action_b || software_pairing_action_b) {
                Serial.println("Pedal Pairing.....");
                delay(1000);
                Pairing_state_start = millis();
                Pairing_state_last_sending = millis();
                ESPNow_pairing_action_b = true;
                building_dap_esppairing_lcl = true;
                software_pairing_action_b = false;
                hardware_pairing_action_b = false;
            }
            if (ESPNow_pairing_action_b) {
                unsigned long now = millis();
                // sending package
                if (building_dap_esppairing_lcl) {
                    uint16_t crc = 0;
                    building_dap_esppairing_lcl = false;
                    dap_esppairing_lcl.payloadESPNowInfo_._deviceID = dap_config_st.payLoadPedalConfig_.pedal_type;
                    dap_esppairing_lcl.payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_ESPNOW_PAIRING;
                    dap_esppairing_lcl.payLoadHeader_.PedalTag = dap_config_st.payLoadPedalConfig_.pedal_type;
                    dap_esppairing_lcl.payLoadHeader_.version = DAP_VERSION_CONFIG;
                    crc = checksumCalculator((uint8_t *)(&(dap_esppairing_lcl.payLoadHeader_)),
                                             sizeof(dap_esppairing_lcl.payLoadHeader_) + sizeof(dap_esppairing_lcl.payloadESPNowInfo_));
                    dap_esppairing_lcl.payloadFooter_.checkSum = crc;
                }
                if (now - Pairing_state_last_sending > 400) {
                    Pairing_state_last_sending = now;
                    ESPNow.send_message(broadcast_mac, (uint8_t *)&dap_esppairing_lcl, sizeof(dap_esppairing_lcl));
                }

                // timeout check
                if (now - Pairing_state_start > Pairing_timeout) {
                    ESPNow_pairing_action_b = false;
                    Serial.print("Pedal: ");
                    Serial.print(dap_config_st.payLoadPedalConfig_.pedal_type);
                    Serial.println(" timeout.");
        #ifdef USING_BUZZER
                    Buzzer.single_beep_tone(700, 100);
        #endif
                    if (UpdatePairingToEeprom) {
                        EEPROM.put(EEPROM_offset, _ESP_pairing_reg);
                        EEPROM.commit();
                        UpdatePairingToEeprom = false;
                        // list eeprom
                        ESP_pairing_reg ESP_pairing_reg_local;
                        EEPROM.get(EEPROM_offset, ESP_pairing_reg_local);
                        for (int i = 0; i < 4; i++) {
                            if (ESP_pairing_reg_local.Pair_status[i] == 1) {
                                Serial.print("#");
                                Serial.print(i);
                                Serial.print("Pair: ");
                                Serial.print(ESP_pairing_reg_local.Pair_status[i]);
                                Serial.printf(" Mac: %02X:%02X:%02X:%02X:%02X:%02X\n", ESP_pairing_reg_local.Pair_mac[i][0],
                                              ESP_pairing_reg_local.Pair_mac[i][1], ESP_pairing_reg_local.Pair_mac[i][2],
                                              ESP_pairing_reg_local.Pair_mac[i][3], ESP_pairing_reg_local.Pair_mac[i][4],
                                              ESP_pairing_reg_local.Pair_mac[i][5]);
                            }
                        }
                        // adding peer

                        for (int i = 0; i < 4; i++) {
                            if (_ESP_pairing_reg.Pair_status[i] == 1) {
                                if (i == 0) {
                                    ESPNow.remove_peer(Clu_mac);
                                    memcpy(&Clu_mac, &_ESP_pairing_reg.Pair_mac[i], 6);
                                    delay(100);
                                    ESPNow.add_peer(Clu_mac);
                                }
                                if (i == 1) {
                                    ESPNow.remove_peer(Brk_mac);
                                    memcpy(&Brk_mac, &_ESP_pairing_reg.Pair_mac[i], 6);
                                    delay(100);
                                    ESPNow.add_peer(Brk_mac);
                                }
                                if (i == 2) {
                                    ESPNow.remove_peer(Gas_mac);
                                    memcpy(&Gas_mac, &_ESP_pairing_reg.Pair_mac[i], 6);
                                    delay(100);
                                    ESPNow.add_peer(Gas_mac);
                                }
                                if (i == 3) {
                                    ESPNow.remove_peer(esp_Host);
                                    memcpy(&esp_Host, &_ESP_pairing_reg.Pair_mac[i], 6);
                                    delay(100);
                                    ESPNow.add_peer(esp_Host);
                                }
                                if (dap_config_st.payLoadPedalConfig_.pedal_type == 1) {
                                    Recv_mac = Gas_mac;
                                }
                                if (dap_config_st.payLoadPedalConfig_.pedal_type == 2) {
                                    Recv_mac = Brk_mac;
                                }
                            }
                        }
                    }
                }
            }
    #endif
            // joystick sync
            float controller_val;
            if (dap_config_st.payLoadPedalConfig_.travelAsJoystickOutput_u8 || dap_calculationVariables_st.Rudder_status) {
                controller_val = normalize_value(x_foot, dap_calculationVariables_st.x_foot_min_curr, dap_calculationVariables_st.x_foot_max_curr);
            } else {
                controller_val = normalize_value(f_foot, dap_calculationVariables_st.Force_Min, dap_calculationVariables_st.Force_Max);
            }

            sendMessageToMaster(f_foot, x_foot, controller_val);

            if (basic_state_send_b) {
                if (semaphore_updatePedalStates != NULL) {
                    if (xSemaphoreTake(semaphore_updatePedalStates, (TickType_t)0) == pdTRUE) {
                        ESPNow.send_message(broadcast_mac, (uint8_t *)&dap_state_basic_st, sizeof(dap_state_basic_st));
                        basic_state_send_b = false;
                        xSemaphoreGive(semaphore_updatePedalStates);
                    }
                }
            }
            if (extend_state_send_b) {
                if (semaphore_updatePedalStates != NULL) {
                    if (xSemaphoreTake(semaphore_updatePedalStates, (TickType_t)0) == pdTRUE) {
                        ESPNow.send_message(broadcast_mac, (uint8_t *)&dap_state_extended_st, sizeof(dap_state_extended_st));
                        extend_state_send_b = false;
                        xSemaphoreGive(semaphore_updatePedalStates);
                    }
                }
            }
            if (ESPNow_config_request) {
                ESPNow.send_message(broadcast_mac, (uint8_t *)&dap_config_st, sizeof(dap_config_st));
                ESPNow_config_request = false;
                LogOutput::printf("ESPNow: Config sent");
            }
            if (ESPNow_OTA_enable) {
                LogOutput::printf("Get OTA command");
                OTA_enable_b = true;
                OTA_enable_start = true;
                ESPNow_OTA_enable = false;
            }
            if (OTA_update_action_b) {
                LogOutput::printf("Get OTA command");
                OTA_enable_b = true;
                OTA_enable_start = true;
                ESPNow_OTA_enable = false;
                Serial.println("get basic wifi info");
                Serial.readBytes((char *)&_basic_wifi_info, sizeof(Basic_WIfi_info));
    #ifdef OTA_update
                if (_basic_wifi_info.device_ID == dap_config_st.payLoadPedalConfig_.pedal_type) {
                    SSID = new char[_basic_wifi_info.SSID_Length + 1];
                    PASS = new char[_basic_wifi_info.PASS_Length + 1];
                    memcpy(SSID, _basic_wifi_info.WIFI_SSID, _basic_wifi_info.SSID_Length);
                    memcpy(PASS, _basic_wifi_info.WIFI_PASS, _basic_wifi_info.PASS_Length);
                    SSID[_basic_wifi_info.SSID_Length] = 0;
                    PASS[_basic_wifi_info.PASS_Length] = 0;
                    OTA_enable_b = true;
                }
    #endif
            }
    // rudder sync
    #ifndef HAS_CAN
            if (dap_calculationVariables_st.Rudder_status) {
                if (ESPNow_update) {
                    // dap_calculationVariables_st.sync_pedal_position=ESPNow_recieve;
                    dap_calculationVariables_st.f_foot_other_pedal = other_data.force_dbl;
                    dap_calculationVariables_st.x_foot_other_pedal = other_data.position_dbl;
                    ESPNow_update = false;
                }
            }
    #endif
        }

    #ifdef ESPNow_debug_rudder
        if (print_count > 1000) {
            if (dap_calculationVariables_st.Rudder_status) {
                Serial.print("Pedal:");
                Serial.print(dap_config_st.payLoadPedalConfig_.pedal_type);
                Serial.print(", Send %: ");
                Serial.print(_ESPNow_Send.pedal_position_ratio);
                Serial.print(", Recieve %:");
                Serial.print(_ESPNow_Recv.pedal_position_ratio);
                Serial.print(", Send Position: ");
                Serial.print(dap_calculationVariables_st.current_pedal_position);
                Serial.print(", % in cal: ");
                Serial.print(dap_calculationVariables_st.current_pedal_position_ratio);
                Serial.print(", min cal: ");
                Serial.print(dap_calculationVariables_st.stepperPosMin_default);
                Serial.print(", max cal: ");
                Serial.print(dap_calculationVariables_st.stepperPosMax_default);
                Serial.print(", range in cal: ");
                Serial.println(dap_calculationVariables_st.stepperPosRange_default);
            }

            // Debug_rudder_last=now_rudder;
            // Serial.println(dap_calculationVariables_st.current_pedal_position);

            print_count = 0;
        } else {
            print_count++;
        }

    #endif

    #ifdef PRINT_TASK_FREE_STACKSIZE_IN_WORDS
        if (espNowTask_stackSizeIdx_u32 == 1000) {
            UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            Serial.print("StackSize (ESP-Now): ");
            Serial.println(stackHighWaterMark);
            espNowTask_stackSizeIdx_u32 = 0;
        }
        espNowTask_stackSizeIdx_u32++;
    #endif
    }
}
#endif
