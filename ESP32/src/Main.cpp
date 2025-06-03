
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

bool resetServoEncoder = true;
bool isv57LifeSignal_b = false;
bool isv57_not_live_b = false;

// #define OTA_update

#include "Main.h"

#include "Arduino.h"
#include "ConfigManager.h"
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

enum CommChannel {
    USB_SERIAL,
    ISOTP,
    ESP_NOW
};

// #define ALLOW_SYSTEM_IDENTIFICATION

/**********************************************************************************************/
/*                                                                                            */
/*                         function declarations                                              */
/*                                                                                            */
/**********************************************************************************************/
void updatePedalCalcParameters();
void pedalUpdateTask(void *pvParameters);
void serialCommunicationTask(void *pvParameters);
void servoCommunicationTask(void *pvParameters);
void OTATask(void *pvParameters);
void ESPNOW_SyncTask(void *pvParameters);
// https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
uint16_t checksumCalculator(uint8_t *data, uint16_t length) {
    uint16_t curr_crc = 0x0000;
    uint8_t sum1 = (uint8_t)curr_crc;
    uint8_t sum2 = (uint8_t)(curr_crc >> 8);
    int index;
    for (index = 0; index < length; index = index + 1) {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

bool systemIdentificationMode_b = false;

int16_t servoPos_i16 = 0;

bool splineDebug_b = false;

#include <EEPROM.h>
#define EEPROM_offset 15

#include "AutomotivePedalFunction.h"

AutomotivePedalFunction automotive_pedal_function = {};

// #include "ABSOscillation.h"
// ABSOscillation absOscillation;
// RPMOscillation _RPMOscillation;
// BitePointOscillation _BitePointOscillation;
// G_force_effect _G_force_effect;
// WSOscillation _WSOscillation;
// Road_impact_effect _Road_impact_effect;
// Custom_vibration CV1;
// Custom_vibration CV2;
// Rudder _rudder;
// Rudder_G_Force _rudder_g_force;
// #define ABS_OSCILLATION

// #include "DiyActivePedal_types.h"
// DAP_config_st dap_config_st;
// DAP_mech_config_st dap_mech_config_st;
// DAP_calculationVariables_st dap_calculationVariables_st;
// DAP_state_basic_st dap_state_basic_st;
// DAP_state_extended_st dap_state_extended_st;
// DAP_ESPPairing_st dap_esppairing_st;   // saving
// DAP_ESPPairing_st dap_esppairing_lcl;  // sending

#include "CycleTimer.h"
#include "LogOutput.h"
#include "RTDebugOutput.h"

uint8_t debug_flags = 0;

/**********************************************************************************************/
/*                                                                                            */
/*                         iterpolation  definitions                                          */
/*                                                                                            */
/**********************************************************************************************/

// #include "ForceCurve.h"
// ForceCurve_Interpolated forceCurve;

/**********************************************************************************************/
/*                                                                                            */
/*                         multitasking  definitions                                          */
/*                                                                                            */
/**********************************************************************************************/
#ifndef CONFIG_IDF_TARGET_ESP32S3
    #include "soc/rtc_wdt.h"
#endif

#ifdef HAS_CAN
    #include <CANManager.h>
AxisCANManager can_manager;
#endif

// #define PRINT_USED_STACK_SIZE
//  https://stackoverflow.com/questions/55998078/freertos-task-priority-and-stack-size
#define STACK_SIZE_FOR_TASK_1 0.2 * (configTOTAL_HEAP_SIZE / 4)
#define STACK_SIZE_FOR_TASK_2 0.2 * (configTOTAL_HEAP_SIZE / 4)

TaskHandle_t PedalTask;
TaskHandle_t SerialCommTask;

// static SemaphoreHandle_t semaphore_updateConfig = NULL;
// DAP_config_st dap_config_st_local;
// DAP_mech_config_st dap_mech_config_st_local;

static SemaphoreHandle_t semaphore_updateJoystick = NULL;
int32_t joystickNormalizedToInt32 = 0;  // semaphore protected data

static SemaphoreHandle_t semaphore_resetServoPos = NULL;
bool resetPedalPosition = false;

static SemaphoreHandle_t semaphore_readServoValues = NULL;

static SemaphoreHandle_t semaphore_updatePedalStates = NULL;

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
/*                         pedal mechanics definitions                                        */
/*                                                                                            */
/**********************************************************************************************/

#include "PedalGeometry.h"

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
/*                         stepper motor definitions                                          */
/*                                                                                            */
/**********************************************************************************************/

#ifdef A6SERVO
    #include "A6Servo.h"
#endif
Servo *stepper = NULL;
// static const int32_t MIN_STEPS = 5;

// #include "StepperMovementStrategy.h"

bool moveSlowlyToPosition_b = false;
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

#include "FFBDataTools.h"
#include "PacketSerial.h"

PacketSerial myPacketSerial;

ConfigManager config_manager;

void IRAM_ATTR adc_isr(void) {
    if (PedalTask) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(PedalTask, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void on_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel);
void on_ffb_action(FFBAction &ffb_action);

void on_config_update(void) {
    if (stepper) stepper->pause(1000);
    sim.set_x_min(config_manager.get_x_contact_point_min(), true);
    sim.set_x_max(config_manager.get_x_contact_point_max(), true);
    can_manager.send_position_limits(sim.get_x_min(), sim.get_x_max());
    const FunctionConfig *function_cfg = config_manager.get_function_config();
    switch (function_cfg->which_specific) {
        case FunctionConfig_automotive_pedal_tag:
            automotive_pedal_function.update_config(function_cfg->specific.automotive_pedal);
            automotive_pedal_function.enable();
            break;
    }
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

// Serial.begin(115200);
// Serial.begin(921600);
// Serial.begin(512000);
//
#ifdef USB_JOYSTICK
    SetupController();
    delay(100);
#endif

#if PCB_VERSION == 6
    Serial.setTxTimeoutMs(0);
#else
    Serial.begin(921600);
    Serial.setTimeout(5);
#endif
    myPacketSerial.setStream(&Serial);
    myPacketSerial.setPacketHandler([](const uint8_t *buffer, size_t size) { on_packet_received(buffer, size, CommChannel::USB_SERIAL); });
    // send some zero bytes to ensure proper COBS sync on the first message
    Serial.print("\x00\x00\x00");

    // start serialCommunicationTask early to support LogOutput
    xTaskCreatePinnedToCore(serialCommunicationTask, "serialCommunicationTask", 10000,
                            // STACK_SIZE_FOR_TASK_2,
                            NULL, 1, &SerialCommTask, 0);

    delay(100);

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
    config_manager.init(FFBDataTools::axis_id_from_index(own_axis_index), on_config_update);
#else
    config_manager.init(AxisID_AXIS_UNDEFINED, on_config_update);
#endif


    // init controller
    LogOutput::printf("**************************************************************************************************************");
    LogOutput::printf("This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.");
    LogOutput::printf("Please check github repo for more detail: https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal");
    // printout the github releasing version

#ifdef PEDAL_ASSIGNMENT
    if (own_axis_index < FFBDataTools::MAX_AXES_COUNT) {
        LogOutput::printf("Identified as axis %d", own_axis_index + 1);
    } else {
        LogOutput::printf("Assignment error, axis id = %d (max. %d)", own_axis_index + 1, FFBDataTools::MAX_AXES_COUNT);
    }
#endif

    config_manager.load_configs();

#ifdef HAS_CAN
    can_manager.setup(config_manager.get_axis_id(), 1000, CAN_TX, CAN_RX, [](const uint8_t *buffer, size_t size) { on_packet_received(buffer, size, CommChannel::ISOTP); }, on_ffb_action);
#endif

// check whether iSV57 communication can be established
// and in case, (a) send tuned servo parameters and (b) prepare the servo for signal read
#ifdef ESPNow_Pairing_function
    pinMode(Pairing_GPIO, INPUT_PULLUP);
#endif

//     // initialize configuration and update local variables
//     dap_config_st.initialiseDefaults();
//     dap_mech_config_st.initialiseDefaults();

//     // Load config from EEPROM, if valid, overwrite initial config
//     EEPROM.begin(2048);
//     dap_mech_config_st.loadConfigFromEprom(dap_mech_config_st_local);
//     dap_config_st.loadConfigFromEprom(dap_config_st_local);

//     // mechanical config
//     // if checks are successfull, overwrite global configuration struct
//     if (verify_ffb_data_struct(&dap_mech_config_st_local, sizeof(dap_mech_config_st_local)) == FFBDataType::MECH_CONFIG) {
//         LogOutput::printf("Updating mechanical pedal config from EEPROM");
//         dap_mech_config_st = dap_mech_config_st_local;
//     } else {
//         LogOutput::printf("Can't load mechanical config from EEPROM:");
//         if (DAP_PAYLOAD_TYPE_MECH_CONFIG != dap_mech_config_st_local.payLoadHeader_.payloadType) {
//             LogOutput::printf("  Payload type expected: %d, Payload type received: %d", DAP_PAYLOAD_TYPE_MECH_CONFIG,
//                               dap_mech_config_st_local.payLoadHeader_.payloadType);
//         }
//         if (DAP_VERSION_CONFIG, dap_mech_config_st_local.payLoadHeader_.version) {
//             LogOutput::printf("  Target version: %d,    Source version: %d", DAP_VERSION_CONFIG, dap_mech_config_st_local.payLoadHeader_.version);
//         }
//     }

//     // general config
//     // if checks are successfull, overwrite global configuration struct
//     if (verify_ffb_data_struct(&dap_config_st_local, sizeof(dap_config_st_local)) == FFBDataType::CONFIG) {
//         LogOutput::printf("Updating general pedal config from EEPROM");
//         dap_config_st = dap_config_st_local;
//     } else {
//         LogOutput::printf("Can't load general config from EEPROM:");
//         if (DAP_PAYLOAD_TYPE_CONFIG != dap_config_st_local.payLoadHeader_.payloadType) {
//             LogOutput::printf("  Payload type expected: %d,   Payload type received: %d", DAP_VERSION_CONFIG,
//                               dap_config_st_local.payLoadHeader_.version);
//         }
//         if (DAP_VERSION_CONFIG != dap_config_st_local.payLoadHeader_.version) {
//             LogOutput::printf("  Target version: %d,    Source version: %d", DAP_VERSION_CONFIG, dap_config_st_local.payLoadHeader_.version);
//         }
//     }

// #ifdef PEDAL_ASSIGNMENT
//     if (own_axis_id < FFBDataTools::MAX_AXES_COUNT) {
//         dap_config_st.payLoadPedalConfig_.pedal_type = own_axis_id;
//     }
// #endif

//     // interprete config values
//     dap_calculationVariables_st.updateFromMechConfig(dap_mech_config_st);
//     dap_calculationVariables_st.updateFromConfig(dap_config_st);

    const AxisConfig *axis_cfg = config_manager.get_axis_config();

#ifdef A6SERVO
    stepper = new A6Servo(stepPinStepper, dirPinStepper, !axis_cfg->b_motor_inverted, Serial1, 115200, SERIAL_8N1, ISV57_RXPIN, ISV57_TXPIN, ISV57_DEPIN, false);
#endif
    loadcell = new LoadCell_ADS1256();

    loadcell->setLoadcellRating(axis_cfg->f_max_loadcell / 9.81f); // from N to kg

    loadcell->setZeroPoint();
#ifdef ESTIMATE_LOADCELL_VARIANCE
    loadcell->estimateVariance();  // automatically identify sensor noise for KF parameterization
#endif

    // Serial.print("Min Position is "); Serial.println(stepper->getLimitMin());
    // Serial.print("Max Position is "); Serial.println(stepper->getLimitMax());

    // setup Kalman filter
    float var_est = loadcell->getVarianceEstimate();
    LogOutput::printf("Given loadcell variance: %f", var_est);
    kalman = new KalmanFilter(var_est);
    kalman_2nd_order = new KalmanFilter_2nd_order(var_est);

    // setup multi tasking
    semaphore_updateJoystick = xSemaphoreCreateMutex();
    semaphore_resetServoPos = xSemaphoreCreateMutex();
    semaphore_updatePedalStates = xSemaphoreCreateMutex();
    delay(10);

    // activate parameter update in first cycle
    // update_config();
    // equalize pedal config for both tasks

    if (semaphore_updateJoystick == NULL) {
        LogOutput::printf("Could not create semaphore");
        ESP.restart();
    }

    if (!stepper->setup(axis_cfg->steps_per_mm, axis_cfg->mm_per_rev)) {
        LogOutput::printf("Failed to initialize the servo (check power and connections).");
    } else {
        stepper->enable();
        delay(100);
    }

    sim.add_element(&automotive_pedal_function);
    // sim.add_element(&spring1);
    // spring1.disable();
    // sim.add_element(&damper1);
    // sim.add_element(&force_map1);
    // force_map1.disable();
    // endstops.add_element(&force_map2);
    // endstops.add_element(&damping_map1);
    // endstops.disable();
    // sim.add_element(&endstops);
    // sim.add_element(&absOscillation);
    // sim.add_element(&friction1);

// Serial.begin(115200);
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

    xTaskCreatePinnedToCore(pedalUpdateTask,   /* Task function. */
                            "pedalUpdateTask", /* name of task. */
                            10000,             /* Stack size of task */
                            // STACK_SIZE_FOR_TASK_1,
                            NULL,       /* parameter of the task */
                            10,         /* priority of the task */
                            &PedalTask, /* Task handle to keep track of created task */
                            1);         /* pin task to core 1 */

    LogOutput::printf("pedalUpdateTask created");

    enableCore1WDT();

    attachInterrupt(PIN_DRDY, &adc_isr, FALLING);
    LogOutput::printf("ADC DRDY ISR attached");

    LogOutput::printf("Setup end");
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
    if (stepper->get_state() == Servo::State::Disabled) {
        pixels.SetPixelColor(0, red);
    } else if (stepper->is_locked_in()) {
        pixels.SetPixelColor(0, green);
    } else {
        pixels.SetPixelColor(0, yellow);
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
    float f_sum = own_force;
    float temp;
    const FunctionBase &func_base = config_manager.get_function_config()->base;
    if (func_base.additive_axis_id_1 != config_manager.get_axis_id()) return 0.0f;
    if (func_base.subtractive_axis_id_1 != AxisID_AXIS_UNDEFINED) {
        if (can_manager.get_force(func_base.subtractive_axis_id_1, temp)) {
            f_sum -= temp;
        }
    }
    if (func_base.additive_axis_id_2 != AxisID_AXIS_UNDEFINED) {
        if (can_manager.get_force(func_base.additive_axis_id_2, temp)) {
            f_sum += temp;
        }
    }
    if (func_base.subtractive_axis_id_2 != AxisID_AXIS_UNDEFINED) {
        if (can_manager.get_force(func_base.subtractive_axis_id_2, temp)) {
            f_sum -= temp;
        }
    }
    return f_sum;
}

float get_final_position(float own_position) {
    float other_position;
    const FunctionBase &func_base = config_manager.get_function_config()->base;
    AxisID own_axis_id = config_manager.get_axis_id();
    if (func_base.additive_axis_id_1 == own_axis_id) return own_position;
    if (func_base.subtractive_axis_id_1 == own_axis_id) {
        if (can_manager.get_position(func_base.additive_axis_id_1, other_position)) {
            return config_manager.get_x_contact_point_center() - other_position;
        }
    }
    if (func_base.additive_axis_id_2 == own_axis_id) {
        if (can_manager.get_position(func_base.additive_axis_id_1, other_position)) {
            return other_position;
        }
    }
    if (func_base.subtractive_axis_id_2 == own_axis_id) {
        if (can_manager.get_position(func_base.additive_axis_id_1, other_position)) {
            return config_manager.get_x_contact_point_center() - other_position;
        }
    }
    return 0.0f;
}

/**********************************************************************************************/
/*                                                                                            */
/*                         pedal update task                                                  */
/*                                                                                            */
/**********************************************************************************************/

// long lastCallTime = micros();
unsigned long cycleTimeLastCall = micros();
unsigned long minCyclesForFirToInit = 1000;
unsigned long firCycleIncrementer = 0;

float filteredReading_exp_filter = 0;
unsigned long printCycleCounter = 0;

uint printCntr = 0;
float x_foot = 0.0;
float f_foot = 0.0;

int64_t timeNow_pedalUpdateTask_l = 0;
int64_t timePrevious_pedalUpdateTask_l = 0;
#define REPETITION_INTERVAL_PEDALUPDATE_TASK (int64_t)1

uint8_t can_output_loop_cnt = 0;
uint8_t can_output_prescaler = 2;

// void loop()
void pedalUpdateTask(void *pvParameters) {
    float dt = 1000.0;
    const AxisConfig *axis_cfg = config_manager.get_axis_config();
    const FunctionConfig *function_cfg = config_manager.get_function_config();

    for (;;) {
// system identification mode
#ifdef ALLOW_SYSTEM_IDENTIFICATION
        if (systemIdentificationMode_b == true) {
            measureStepResponse(stepper, &dap_calculationVariables_st, &dap_config_st, loadcell);
            systemIdentificationMode_b = false;
        }
#endif

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

        /*
            //#define RECALIBRATE_POSITION
            #ifdef RECALIBRATE_POSITION
              stepper->checkLimitsAndResetIfNecessary();
            #endif

            // compute pedal oscillation, when ABS is active
            float absForceOffset_fl32 = 0.0f;
        */
        float absForceOffset = 0;
        float absPosOffset = 0;
#ifdef ABS_OSCILLATION
        absOscillation.forceOffset(config_manager.get_automotive_pedal_config()->abs_effect_config);
        // _RPMOscillation.trigger();
        // _RPMOscillation.forceOffset(&dap_calculationVariables_st);
        // _BitePointOscillation.forceOffset(&dap_calculationVariables_st);
        // _G_force_effect.forceOffset(&dap_calculationVariables_st, dap_config_st.payLoadPedalConfig_.G_multi);
        // _WSOscillation.forceOffset(&dap_calculationVariables_st);
        // _Road_impact_effect.forceOffset(&dap_calculationVariables_st, dap_config_st.payLoadPedalConfig_.Road_multi);
        // CV1.forceOffset(dap_config_st.payLoadPedalConfig_.CV_freq_1,dap_config_st.payLoadPedalConfig_.CV_amp_1);
        // CV2.forceOffset(dap_config_st.payLoadPedalConfig_.CV_freq_2,dap_config_st.payLoadPedalConfig_.CV_amp_2);
        // _rudder.offset_calculate(&dap_calculationVariables_st);
        //_rudder.force_offset_calculate(&dap_calculationVariables_st);
#endif
        /*
            //update max force with G force effect
            movingAverageFilter.dataPointsCount = dap_config_st.payLoadPedalConfig_.G_window;
            movingAverageFilter_roadimpact.dataPointsCount = dap_config_st.payLoadPedalConfig_.Road_window;
            dap_calculationVariables_st.reset_maxforce();
            dap_calculationVariables_st.Force_Max += _G_force_effect.G_force;
            dap_calculationVariables_st.Force_Max += _Road_impact_effect.Road_Impact_force;
            dap_calculationVariables_st.dynamic_update();
            dap_calculationVariables_st.updateStiffness();
            dap_calculationVariables_st.update_stepperpos(_rudder.offset_filter);

        */

        // Get the loadcell reading
        float loadcellReading = loadcell->getReadingKg();

        unsigned long now = micros();
        dt = (now - cycleTimeLastCall) / 1000.0;
        cycleTimeLastCall = now;

        // Invert the loadcell reading digitally if desired
        if (axis_cfg->b_loadcell_inverted) {
            loadcellReading *= -1.0;
        }

        /*
            // Convert loadcell reading to pedal force
            float sledPosition = 0.0; // sledPositionInMM(stepper, dap_config_st);
            float pedalInclineAngleInDeg_fl32 = pedalInclineAngleDeg(sledPosition, dap_config_st);
            float pedalForce_fl32 = convertToPedalForce(loadcellReading, sledPosition, dap_config_st);
            float d_phi_d_x = convertToPedalForceGain(sledPosition, dap_config_st);

            // compute gain for horizontal foot model
            float b = dap_config_st.payLoadPedalConfig_.lengthPedal_b;
            float d = dap_config_st.payLoadPedalConfig_.lengthPedal_d;
            float d_x_hor_d_phi = -(b+d) * sinf(pedalInclineAngleInDeg_fl32 * DEG_TO_RAD);

        */

        // Do the loadcell signal filtering
        float filteredReading = 0;
        float changeVelocity = 0;

        // const velocity model denoising filter
        switch (axis_cfg->kf_model_order) {
            case KFModelOrder_KF_CONST_VEL:
                filteredReading = kalman->filteredValue(loadcellReading, 0, axis_cfg->kf_model_noise_scaling);
                changeVelocity = kalman->changeVelocity();
                break;
            case KFModelOrder_KF_CONST_ACC:
                filteredReading = kalman_2nd_order->filteredValue(loadcellReading, 0, axis_cfg->kf_model_noise_scaling);
                changeVelocity = kalman->changeVelocity();
                break;
            default:
                break;
        }

        /*
            // exponential denoising filter
            if (dap_config_st.payLoadPedalConfig_.kf_modelOrder == 2)
            {
              float alpha_exp_filter = 1.0f - ( (float)dap_config_st.payLoadPedalConfig_.kf_modelNoise) / 5000.0f;
              float filteredReading_exp_filter = filteredReading_exp_filter * alpha_exp_filter + pedalForce_fl32 * (1.0-alpha_exp_filter);
              filteredReading = filteredReading_exp_filter;
            }
        */
        // if (stepper->get_homing_state() == Servo::HomingState::Homed) {
        //   sim.set_x_min(0.0);
        //   sim.set_x_max(100.0);
        // } else {
        //   sim.set_x_min(50.0, true);
        //   sim.set_x_max(50.0, true);
        // }

        float f_loadcell = filteredReading * 9.81;

        float r_conv;
        calc_poly(x_foot, r_conv, axis_cfg->coeffs_force_factor_over_contact_point_pos);

        f_foot = f_loadcell * r_conv;

#ifdef HAS_CAN
        /* AxisCANManager is designed to run its main processing from within the pedal task to ensure minimum latency on other axes' position and
         * force values */
        can_manager.process();
#endif

        float f_in = get_input_force_sum(f_foot);

        sim.update(dt, f_in);

        x_foot = get_final_position(sim.get_x());

        float x_sled;
        calc_poly(x_foot, x_sled, axis_cfg->coeffs_sled_pos_over_contact_point_pos);

        config_manager.release_config_semaphore();

        stepper->move_to(x_sled);

        // #define DEBUG_FILTER
        if (debug_flags & DEBUG_INFO_0_LOADCELL_READING) {
            static uint16_t loop_cnt = 0;
            static RTDebugOutput<10> rtDebugFilter({"raw", "flt", "f_in", "f_foot", "f_sum", "a", "v", "x", "x_sled"});
            loop_cnt++;
            if (loop_cnt >= 20) {
                loop_cnt = 0;
                rtDebugFilter.offerData(
                    {loadcellReading, filteredReading, f_in, f_foot, sim.get_f_sum(), sim.get_a(), sim.get_v(), x_foot, x_sled});
            }
        }

#ifdef HAS_CAN
        can_output_loop_cnt++;
        if ((can_output_loop_cnt % can_output_prescaler) == 0) {
            can_output_loop_cnt = 0;
            can_manager.send_force_and_position(f_foot, x_foot);
        }
#endif
        /*

            //Add effect by force
            float effect_force = absForceOffset + _BitePointOscillation.BitePoint_Force_offset + _WSOscillation.WS_Force_offset + CV1.CV_Force_offset
           + CV2.CV_Force_offset;
            // float stepperPosFraction = stepper->getCurrentPositionFraction();
            int32_t Position_Next = 0;






            // select control loop algo
            if (dap_config_st.payLoadPedalConfig_.control_strategy_b <= 1)
            {
              // Position_Next = MoveByPidStrategy(filteredReading, stepperPosFraction, stepper, &forceCurve, &dap_calculationVariables_st,
           &dap_config_st, 0, changeVelocity);
            }

            if (dap_config_st.payLoadPedalConfig_.control_strategy_b == 2)
            {
              // Position_Next = MoveByForceTargetingStrategy(filteredReading, stepper, &forceCurve, &dap_calculationVariables_st, &dap_config_st, 0,
           changeVelocity, d_phi_d_x, d_x_hor_d_phi);
            }




            // add dampening
            if (dap_calculationVariables_st.dampingPress  > 0.0001)
            {
              // dampening is proportional to velocity --> D-gain for stability
              Position_Next -= dap_calculationVariables_st.dampingPress * changeVelocity * dap_calculationVariables_st.springStiffnesssInv;
            }



            // clip target position to configured target interval with RPM effect movement in the endstop
            Position_Next = (int32_t)constrain(Position_Next, dap_calculationVariables_st.stepperPosMin, dap_calculationVariables_st.stepperPosMax);


            //Adding effects
            int32_t Position_effect= effect_force/dap_calculationVariables_st.Force_Range*dap_calculationVariables_st.stepperPosRange;
            Position_Next -=_RPMOscillation.RPM_position_offset;
            Position_Next -= absPosOffset;
            Position_Next -= Position_effect;
            Position_Next = (int32_t)constrain(Position_Next, dap_calculationVariables_st.stepperPosMinEndstop,
           dap_calculationVariables_st.stepperPosMaxEndstop);

            //bitepoint trigger
            int32_t BP_trigger_value = dap_config_st.payLoadPedalConfig_.BP_trigger_value;
            int32_t BP_trigger_min = (BP_trigger_value-4);
            int32_t BP_trigger_max = (BP_trigger_value+4);
            int32_t Position_check = 100*((Position_Next-dap_calculationVariables_st.stepperPosMin) / dap_calculationVariables_st.stepperPosRange);


            dap_calculationVariables_st.current_pedal_position = Position_Next;


            //Serial.println(Position_check);
            if(dap_config_st.payLoadPedalConfig_.BP_trigger==1)
            {
              if(Position_check > BP_trigger_min)
              {
                if(Position_check < BP_trigger_max)
                {
                  _BitePointOscillation.trigger();
                }
              }
            }

            // if pedal in min position, recalibrate position

            // Move to new position
            if (!moveSlowlyToPosition_b)
            {
              // stepper->moveTo(Position_Next, false);
            }
            else
            {
              moveSlowlyToPosition_b = false;
              // stepper->moveSlowlyToPos(Position_Next);
            }


            // compute controller output
            dap_calculationVariables_st.StepperPos_setback();
            dap_calculationVariables_st.reset_maxforce();
            dap_calculationVariables_st.dynamic_update();
            dap_calculationVariables_st.updateStiffness();


            // set joystick value
            // if(semaphore_updateJoystick!=NULL)
            // {
            //   if(xSemaphoreTake(semaphore_updateJoystick, (TickType_t)1)==pdTRUE) {


            //     if(dap_calculationVariables_st.Rudder_status&&dap_calculationVariables_st.rudder_brake_status)
            //     {
            //       if (1 == dap_config_st.payLoadPedalConfig_.travelAsJoystickOutput_u8)
            //       {
            //         //joystickNormalizedToInt32 = NormalizeControllerOutputValue((Position_Next-dap_calculationVariables_st.stepperPosRange/2),
           dap_calculationVariables_st.stepperPosMin, dap_calculationVariables_st.stepperPosMin+dap_calculationVariables_st.stepperPosRange/2,
           dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //         joystickNormalizedToInt32 = NormalizeControllerOutputValue((Position_Next-dap_calculationVariables_st.stepperPosRange/2),
           dap_calculationVariables_st.stepperPosMin, dap_calculationVariables_st.stepperPosMin+dap_calculationVariables_st.stepperPosRange/2,
           dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //         joystickNormalizedToInt32 = constrain(joystickNormalizedToInt32,0,JOYSTICK_MAX_VALUE);
            //       }
            //       else
            //       {
            //         //joystickNormalizedToInt32 = NormalizeControllerOutputValue(loadcellReading, dap_calculationVariables_st.Force_Min,
           dap_calculationVariables_st.Force_Max, dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //         joystickNormalizedToInt32 = NormalizeControllerOutputValue((filteredReading), dap_calculationVariables_st.Force_Min,
           dap_calculationVariables_st.Force_Max, dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //       }
            //     }
            //     else
            //     {
            //       if (1 == dap_config_st.payLoadPedalConfig_.travelAsJoystickOutput_u8)
            //       {
            //         joystickNormalizedToInt32 = NormalizeControllerOutputValue(Position_Next, dap_calculationVariables_st.stepperPosMin,
           dap_calculationVariables_st.stepperPosMax, dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //       }
            //       else
            //       {
            //         joystickNormalizedToInt32 = NormalizeControllerOutputValue(filteredReading, dap_calculationVariables_st.Force_Min,
           dap_calculationVariables_st.Force_Max, dap_config_st.payLoadPedalConfig_.maxGameOutput);
            //       }
            //     }

            //     xSemaphoreGive(semaphore_updateJoystick);
            //   }
            // }
            // else
            // {
            //   semaphore_updateJoystick = xSemaphoreCreateMutex();
            // }

            // provide joystick output on PIN
            #ifdef Using_analog_output
              int dac_value=(int)(joystickNormalizedToInt32*255/10000);
              dacWrite(D_O,dac_value);
            #endif

            #ifdef Using_analog_output_ESP32_S3
              if(MCP_status)
              {
                int dac_value=(int)(joystickNormalizedToInt32*4096*0.9/10000);//limit the max to 5V*0.9=4.5V to prevent the overvolatage
                dac.setVoltage(dac_value, false);
              }
            #endif


            float normalizedPedalReading_fl32 = 0;
            if ( fabs(dap_calculationVariables_st.Force_Range) > 0.01)
            {
                normalizedPedalReading_fl32 = constrain((filteredReading - dap_calculationVariables_st.Force_Min) /
           dap_calculationVariables_st.Force_Range, 0, 1);
            }

            // simulate ABS trigger
            if(dap_config_st.payLoadPedalConfig_.Simulate_ABS_trigger==1)
            {
              int32_t ABS_trigger_value=dap_config_st.payLoadPedalConfig_.Simulate_ABS_value;
              if( (normalizedPedalReading_fl32*100) > ABS_trigger_value)
              {
                absOscillation.trigger();
              }
            }

          */

        // update pedal states
        // if (semaphore_updatePedalStates != NULL) {
        //     if (xSemaphoreTake(semaphore_updatePedalStates, (TickType_t)0) == pdTRUE) {
        //         // update basic pedal state struct
        //         dap_state_basic_st.payloadPedalState_Basic_.pedalForce_u16 =
        //             uint16_t(normalize_value(f_foot, dap_calculationVariables_st.Force_Min, dap_calculationVariables_st.Force_Max) * 65535.0f);
        //         dap_state_basic_st.payloadPedalState_Basic_.pedalPosition_u16 = uint16_t(x_foot_norm * 65535.0f);
        //         dap_state_basic_st.payloadPedalState_Basic_.joystickOutput_u16 = 1000;  // 65535;

        //         dap_state_basic_st.payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_STATE_BASIC;
        //         dap_state_basic_st.payLoadHeader_.version = DAP_VERSION_CONFIG;
        //         dap_state_basic_st.payLoadHeader_.PedalTag = dap_config_st.payLoadPedalConfig_.pedal_type;

        //         // error code
        //         dap_state_basic_st.payloadPedalState_Basic_.error_code_u8 = 0;
        //         // if(ESPNow_error_code!=0)
        //         // {
        //         //   dap_state_basic_st.payloadPedalState_Basic_.erroe_code_u8=ESPNow_error_code;
        //         //   ESPNow_error_code=0;
        //         // }
        //         // dap_state_basic_st.payloadPedalState_Basic_.erroe_code_u8=200;
        //         // if(isv57_not_live_b)
        //         // {
        //         //   dap_state_basic_st.payloadPedalState_Basic_.erroe_code_u8=12;
        //         //   isv57_not_live_b=false;
        //         // }
        //         dap_state_basic_st.payloadFooter_.checkSum =
        //             checksumCalculator((uint8_t *)(&(dap_state_basic_st.payLoadHeader_)),
        //                                sizeof(dap_state_basic_st.payLoadHeader_) + sizeof(dap_state_basic_st.payloadPedalState_Basic_));

        //         // update extended struct
        //         dap_state_extended_st.payloadPedalState_Extended_.timeInMs_u32 = millis();
        //         dap_state_extended_st.payloadPedalState_Extended_.pedalForce_raw_fl32 = loadcellReading;
        //         dap_state_extended_st.payloadPedalState_Extended_.pedalForce_filtered_fl32 = filteredReading;
        //         dap_state_extended_st.payloadPedalState_Extended_.forceVel_est_fl32 = changeVelocity;

        //         // dap_state_extended_st.payloadPedalState_Extended_.servoPositionTarget_i16 = stepper->getCurrentPositionFromMin();
        //         dap_state_extended_st.payLoadHeader_.PedalTag = dap_config_st.payLoadPedalConfig_.pedal_type;
        //         dap_state_extended_st.payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_STATE_EXTENDED;
        //         dap_state_extended_st.payLoadHeader_.version = DAP_VERSION_CONFIG;
        //         dap_state_extended_st.payloadFooter_.checkSum =
        //             checksumCalculator((uint8_t *)(&(dap_state_extended_st.payLoadHeader_)),
        //                                sizeof(dap_state_extended_st.payLoadHeader_) + sizeof(dap_state_extended_st.payloadPedalState_Extended_));

        //         // release semaphore
        //         xSemaphoreGive(semaphore_updatePedalStates);
        //     }
        // } else {
        //     semaphore_updatePedalStates = xSemaphoreCreateMutex();
        // }

        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timerPU.BumpEnd();
        }
    }
}

bool send_ffb_data_msg(const FFBData &msg, CommChannel comm_channel) {
    uint8_t tx_buffer[FFBDataTools::MAX_ENCODED_SIZE + 2];
    uint16_t crc;
    uint16_t num_bytes_encoded = FFBDataTools::encode_ffb_data_and_calc_crc(msg, tx_buffer, FFBDataTools::MAX_ENCODED_SIZE, crc);
    if (num_bytes_encoded) {
        memcpy(tx_buffer + num_bytes_encoded, &crc, sizeof(uint16_t));
        switch (comm_channel) {
            case CommChannel::USB_SERIAL:
                myPacketSerial.send(tx_buffer, num_bytes_encoded + sizeof(uint16_t));
                break;
            case CommChannel::ISOTP:
                can_manager.send_payload_to_gateway(tx_buffer, num_bytes_encoded + sizeof(uint16_t));
                break;
            default:
                break;
        }
        return true;
    }
    return false;
}

void on_ffb_action(FFBAction &ffb_action) {
    if (ffb_action.trigger_abs) {
        automotive_pedal_function.trigger_abs();
    }
}

void send_axis_config(CommChannel comm_channel) {
    FFBData msg;
    config_manager.get_axis_config(msg);
    send_ffb_data_msg(msg, comm_channel);
}

void send_function_config(CommChannel comm_channel) {
    FFBData msg;
    config_manager.get_function_config(msg);
    send_ffb_data_msg(msg, comm_channel);
}

void on_axis_action(AxisAction &axis_action, CommChannel comm_channel) {
    switch (axis_action.which_action) {
        case AxisAction_restart_tag:
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

void send_log_msg(const char *buff) {
    FFBData log_msg = FFBData_init_zero;
    log_msg.payload.log_message.axis_id = config_manager.get_axis_id();
    log_msg.which_payload = FFBData_log_message_tag;
    strncpy(log_msg.payload.log_message.msg, buff, sizeof(log_msg.payload.log_message.msg) - 1);
    send_ffb_data_msg(log_msg, CommChannel::USB_SERIAL); // TODO: dispatch to appropriate comm channel
}

void on_ffb_data_message(FFBData *msg, const uint8_t *protobuf_msg, uint16_t len_protobuf_msg, CommChannel comm_channel) {
    switch (msg->which_payload) {
        case FFBData_axis_config_tag:
            config_manager.update_axis_config(msg->payload.axis_config, protobuf_msg, len_protobuf_msg);
            break;
        case FFBData_function_config_tag:
            config_manager.update_function_config(msg->payload.function_config, protobuf_msg, len_protobuf_msg);
            break;
        case FFBData_ffb_action_tag:
            on_ffb_action(msg->payload.ffb_action);
            break;
        case FFBData_axis_action_tag:
            on_axis_action(msg->payload.axis_action, comm_channel);
            break;
        default:
            LogOutput::printf("Unknown FFBData received");
            break;
    }
}

void on_packet_received(const uint8_t *buffer, size_t size, CommChannel comm_channel) {
    FFBData msg = FFBData_init_zero;
    uint16_t crc = *reinterpret_cast<const uint16_t *>(buffer + size - sizeof(uint16_t));
    if (FFBDataTools::check_and_decode_ffb_data(msg, buffer, size - sizeof(uint16_t), crc)) {
        on_ffb_data_message(&msg, buffer, size - sizeof(uint16_t), comm_channel);
    }
    // else if (buffer[0] == '>') {
    //     char *param = strtok((char *)buffer + 1, "=");
    //     if (param) {
    //         // Serial.printf("Param: %s\n", param);
    //         char *val = strtok(NULL, "=");
    //         if (val) {
    //             // Serial.printf("Val: %s\n", val);
    //             if (strcmp(param, "m") == 0) {
    //                 float val_num = atof(val);
    //                 sim.set_m(val_num);
    //                 Serial.printf("Simulation mass set to %.3f kg\n", val_num);
    //             } else if (strcmp(param, "debug") == 0) {
    //                 int flags = atoi(val);
    //                 dap_config_st.payLoadPedalConfig_.debug_flags_0 = flags;
    //                 Serial.printf("Debug flags set to %04X\n", flags);
    //             } else if (strcmp(param, "can_output_prescaler") == 0) {
    //                 can_output_prescaler = max(atoi(val), 1);
    //                 Serial.printf("can_output_prescaler set to %02X\n", can_output_prescaler);
    //             } else if (strcmp(param, "endstops") == 0) {
    //                 if (atoi(val)) {
    //                     endstops.enable();
    //                     Serial.printf("Endstops enabled\n");
    //                 } else {
    //                     endstops.disable();
    //                     Serial.printf("Endstops disabled\n");
    //                 }
    //             } else if (strcmp(param, "fric") == 0) {
    //                 float val_num = atof(val);
    //                 friction1.set_f(val_num);
    //                 Serial.printf("Friction set to %.3f N\n", val_num);
    //             } else if (strcmp(param, "spr") == 0) {
    //                 float val_num = atof(val);
    //                 spring1.set_k(val_num);
    //                 Serial.printf("Spring set to %.3f N/mm\n", val_num);
    //             } else if (strcmp(param, "damp") == 0) {
    //                 float val_num = atof(val);
    //                 damper1.set_k(val_num);
    //                 Serial.printf("Damper set to %.3f N/(mm/s)\n", val_num);
    //             } else if (strcmp(param, "damp_pos") == 0) {
    //                 float val_num = atof(val);
    //                 damper1.set_k_pos(val_num);
    //                 Serial.printf("Positive damper set to %.3f N/(mm/s)\n", val_num);
    //             } else if (strcmp(param, "damp_neg") == 0) {
    //                 float val_num = atof(val);
    //                 damper1.set_k_neg(val_num);
    //                 Serial.printf("Negative damper set to %.3f N/(mm/s)\n", val_num);
    //             } else {
    //                 Serial.printf("Unknown param \"%s\"\n", param);
    //             }
    //         } else {
    //             if (strncmp(param, "home", sizeof("home") - 1) == 0) {
    //                 Serial.printf("Homing command received\n");
    //                 stepper->home();
    //                 // } else if (strncmp(param, "lock", sizeof("lock") - 1) == 0) {
    //                 //   Serial.printf("Locking command received\n");
    //                 //   stepper->lock_onto_curr_pos();
    //             } else if (strncmp(param, "restart", sizeof("restart") - 1) == 0) {
    //                 Serial.printf("Restarting...\n");
    //                 ESP.restart();
    //             } else {
    //                 Serial.printf("Unknown param \"%s\"\n", param);
    //             }
    //         }
    //     }
    // }
}

/**********************************************************************************************/
/*                                                                                            */
/*                         communication task                                                 */
/*                                                                                            */
/**********************************************************************************************/

int64_t timeNow_serialCommunicationTask_l = 0;
int64_t timePrevious_serialCommunicationTask_l = 0;
#define REPETITION_INTERVAL_SERIALCOMMUNICATION_TASK (int64_t)10
RTDebugOutputService debugOutput = RTDebugOutputService();
LogOutputService logOutput = LogOutputService(send_log_msg);

int32_t joystickNormalizedToInt32_local = 0;
void serialCommunicationTask(void *pvParameters) {
    for (;;) {
        static CycleTimer timerSC("SC cycle time");
        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timerSC.BumpStart();
        }

        delay(SERIAL_COOMUNICATION_TASK_DELAY_IN_MS);

        myPacketSerial.update();

        // // read serial input
        // uint8_t n = Serial.available();

        // if (n) {
        //     char buffer[n];
        //     Serial.readBytes(buffer, n);
        // }

        // // send pedal state structs
        // // update pedal states
        // printCycleCounter++;
        // DAP_state_basic_st dap_state_basic_st_lcl;
        // DAP_state_extended_st dap_state_extended_st_lcl;

        // if(semaphore_updatePedalStates!=NULL)
        // {

        //   if(xSemaphoreTake(semaphore_updatePedalStates, (TickType_t)1)==pdTRUE)
        //   {

        //     // UPDATE basic pedal state struct
        //     dap_state_basic_st_lcl = dap_state_basic_st;

        //     // UPDATE extended pedal state struct
        //     dap_state_extended_st_lcl = dap_state_extended_st;

        //     // release semaphore
        //     xSemaphoreGive(semaphore_updatePedalStates);

        //   }
        // }
        // else
        // {
        //   semaphore_updatePedalStates = xSemaphoreCreateMutex();
        // }

        // send the pedal state structs
        // send basic pedal state struct
        // if ( !(dap_config_st.payLoadPedalConfig_.debug_flags_0 & DEBUG_INFO_0_STATE_BASIC_INFO_STRUCT) )
        // {
        //   if (printCycleCounter >= 2)
        //   {
        //     printCycleCounter = 0;
        //     Serial.write((char*)&dap_state_basic_st_lcl, sizeof(DAP_state_basic_st));
        //     Serial.print("\r\n");
        //   }
        // }

        // if ( (dap_config_st.payLoadPedalConfig_.debug_flags_0 & DEBUG_INFO_0_STATE_EXTENDED_INFO_STRUCT) )
        // {
        //   Serial.write((char*)&dap_state_extended_st_lcl, sizeof(DAP_state_extended_st));
        //   Serial.print("\r\n");
        // }

        // delay( SERIAL_COOMUNICATION_TASK_DELAY_IN_MS );
        // if(semaphore_updateJoystick!=NULL)
        // {
        //   if(xSemaphoreTake(semaphore_updateJoystick, (TickType_t)1)==pdTRUE)
        //   {
        //      //Serial.print(" 3");
        //     joystickNormalizedToInt32_local = joystickNormalizedToInt32;
        //     xSemaphoreGive(semaphore_updateJoystick);
        //   }
        // }
        // if (IsControllerReady())
        // {
        //   if(dap_calculationVariables_st.Rudder_status==false)
        //   {
        //     //general output
        //     SetControllerOutputValue(joystickNormalizedToInt32_local);
        //   }
        // }

#ifdef USB_JOYSTICK
        SetControllerOutputValue_rudder(int32_t(x_foot * 100.0), int32_t(f_in * 10.0));
#endif

        debugOutput.pump(2);
        logOutput.pump(5);

        if (debug_flags & DEBUG_INFO_0_CYCLE_TIMER) {
            timerSC.BumpEnd();
        }
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
