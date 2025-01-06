#include "DiyActivePedal_types.h"
#include "Arduino.h"

#include "PedalGeometry.h"
#include "StepperWithLimits.h"

#include <EEPROM.h>

static const float ABS_SCALING = 50;

const uint32_t EEPROM_OFFSET_MECH_CONFIG = 0;
const uint32_t EEPROM_OFFSET_GENERAL_CONFIG = 0 + sizeof(DAP_mech_config_st);

void DAP_config_st::initialiseDefaults() {
  payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_CONFIG;
  payLoadHeader_.version = DAP_VERSION_CONFIG;
  payLoadHeader_.storeToEeprom = false;

  payLoadPedalConfig_.pedalStartPosition = 10;
  payLoadPedalConfig_.pedalEndPosition = 85;

  payLoadPedalConfig_.maxForce = 60;
  payLoadPedalConfig_.preloadForce = 2;

  payLoadPedalConfig_.relativeForce_p000 = 0;
  payLoadPedalConfig_.relativeForce_p020 = 20;
  payLoadPedalConfig_.relativeForce_p040 = 40;
  payLoadPedalConfig_.relativeForce_p060 = 60;
  payLoadPedalConfig_.relativeForce_p080 = 80;
  payLoadPedalConfig_.relativeForce_p100 = 100;

  payLoadPedalConfig_.dampingPress = 0;
  payLoadPedalConfig_.dampingPull = 0;

  payLoadPedalConfig_.absFrequency = 15;
  payLoadPedalConfig_.absAmplitude = 0;
  payLoadPedalConfig_.absPattern = 0;
  payLoadPedalConfig_.absForceOrTarvelBit = 0;

  payLoadPedalConfig_.lengthPedal_a = 205;
  payLoadPedalConfig_.lengthPedal_b = 220; 
  payLoadPedalConfig_.lengthPedal_d = 60; 
  payLoadPedalConfig_.lengthPedal_c_horizontal = 215;
  payLoadPedalConfig_.lengthPedal_c_vertical = 60;
  payLoadPedalConfig_.lengthPedal_travel = 100;
  

  payLoadPedalConfig_.Simulate_ABS_trigger = 0;// add for abs trigger
  payLoadPedalConfig_.Simulate_ABS_value = 80;// add for abs trigger
  payLoadPedalConfig_.RPM_max_freq = 40;
  payLoadPedalConfig_.RPM_min_freq = 10;
  payLoadPedalConfig_.RPM_AMP = 5;
  payLoadPedalConfig_.BP_trigger_value =50;
  payLoadPedalConfig_.BP_amp=1;
  payLoadPedalConfig_.BP_freq=15;
  payLoadPedalConfig_.BP_trigger=0;
  payLoadPedalConfig_.G_multi = 50;
  payLoadPedalConfig_.G_window=60;
  payLoadPedalConfig_.WS_amp=1;
  payLoadPedalConfig_.WS_freq=15;
  payLoadPedalConfig_.Road_multi = 50;
  payLoadPedalConfig_.Road_window=60;
  payLoadPedalConfig_.cubic_spline_param_a_array[0] = 0;
  payLoadPedalConfig_.cubic_spline_param_a_array[1] = 0;
  payLoadPedalConfig_.cubic_spline_param_a_array[2] = 0;
  payLoadPedalConfig_.cubic_spline_param_a_array[3] = 0;
  payLoadPedalConfig_.cubic_spline_param_a_array[4] = 0;

  payLoadPedalConfig_.cubic_spline_param_b_array[0] = 0;
  payLoadPedalConfig_.cubic_spline_param_b_array[1] = 0;
  payLoadPedalConfig_.cubic_spline_param_b_array[2] = 0;
  payLoadPedalConfig_.cubic_spline_param_b_array[3] = 0;
  payLoadPedalConfig_.cubic_spline_param_b_array[4] = 0;

  payLoadPedalConfig_.PID_p_gain = 0.3f;
  payLoadPedalConfig_.PID_i_gain = 50.0f;
  payLoadPedalConfig_.PID_d_gain = 0.0f;
  payLoadPedalConfig_.PID_velocity_feedforward_gain = 0.0f;


  payLoadPedalConfig_.MPC_0th_order_gain = 10.0f;
  payLoadPedalConfig_.MPC_1st_order_gain = 0.0f;
  payLoadPedalConfig_.MPC_2nd_order_gain = 0.0f;

  payLoadPedalConfig_.control_strategy_b = 0;

  payLoadPedalConfig_.maxGameOutput = 100;

  payLoadPedalConfig_.kf_modelNoise = 128;
  payLoadPedalConfig_.kf_modelOrder = 0;

  payLoadPedalConfig_.debug_flags_0 = 0;

  payLoadPedalConfig_.loadcell_rating = 100;

  payLoadPedalConfig_.travelAsJoystickOutput_u8 = 0;

  payLoadPedalConfig_.invertLoadcellReading_u8 = 0;

  payLoadPedalConfig_.invertMotorDirection_u8 = 0;
  payLoadPedalConfig_.pedal_type=1;
  //payLoadPedalConfig_.OTA_flag=0;
  payLoadPedalConfig_.stepLossFunctionFlags_u8=0b11;
  //payLoadPedalConfig_.Joystick_ESPsync_to_ESP=0;
}




void DAP_config_st::storeConfigToEprom(DAP_config_st& config_st)
{

  EEPROM.put(EEPROM_OFFSET_GENERAL_CONFIG, config_st); 
  EEPROM.commit();
  Serial.println("Successfully stored general config to EEPROM");
  
  /*if (true == config_st.payLoadHeader_.storeToEeprom)
  {
    config_st.payLoadHeader_.storeToEeprom = false; // set to false, thus at restart existing EEPROM config isn't restored to EEPROM
    EEPROM.put(0, config_st); 
    EEPROM.commit();
    Serial.println("Successfully stored config in EPROM");
  }*/
}

void DAP_config_st::loadConfigFromEprom(DAP_config_st& config_st)
{
  DAP_config_st local_config_st;

  EEPROM.get(EEPROM_OFFSET_GENERAL_CONFIG, local_config_st);
  //EEPROM.commit();

  config_st = local_config_st;

  // check if version matches revision, in case, update the default config
  /*if (local_config_st.payLoadHeader_.version == DAP_VERSION_CONFIG)
  {
    config_st = local_config_st;
    Serial.println("Successfully loaded config from EPROM");
  }
  else
  { 
    Serial.println("Couldn't load config from EPROM due to version mismatch");
    Serial.print("Target version: ");
    Serial.println(DAP_VERSION_CONFIG);
    Serial.print("Source version: ");
    Serial.println(local_config_st.payLoadHeader_.version);

  }*/

}

void DAP_mech_config_st::initialiseDefaults() {
  payLoadHeader_.payloadType = DAP_PAYLOAD_TYPE_MECH_CONFIG;
  payLoadHeader_.version = DAP_VERSION_CONFIG;
  payLoadHeader_.storeToEeprom = false;

  payLoadMechConfig_.coeffs_force_factor_over_pedal_pos[0] = 6.20184902e-01;
  payLoadMechConfig_.coeffs_force_factor_over_pedal_pos[1] = -1.71372506e-03;
  payLoadMechConfig_.coeffs_force_factor_over_pedal_pos[2] = 1.07828479e-05;
  payLoadMechConfig_.coeffs_force_factor_over_pedal_pos[3] = 2.71382634e-09;
  payLoadMechConfig_.coeffs_force_factor_over_pedal_pos[4] = 7.34203389e-11;

  payLoadMechConfig_.coeffs_sled_pos_over_pedal_pos[0] = 5.50622588e+01;
  payLoadMechConfig_.coeffs_sled_pos_over_pedal_pos[1] = 5.55488175e-01;
  payLoadMechConfig_.coeffs_sled_pos_over_pedal_pos[2] = 7.59065219e-04;
  payLoadMechConfig_.coeffs_sled_pos_over_pedal_pos[3] = -2.81513616e-06;
  payLoadMechConfig_.coeffs_sled_pos_over_pedal_pos[4] = -1.11220289e-08;

  payLoadMechConfig_.x_foot_min_abs = -1259;
  payLoadMechConfig_.x_foot_max_abs = 927;

  payLoadMechConfig_.loadcell_rating = 100;
  payLoadMechConfig_.spindlePitch_mmPerRev_u8 = 5;

  payLoadMechConfig_.invertLoadcellReading_u8 = 0;

  payLoadMechConfig_.invertMotorDirection_u8 = 0;
  payLoadMechConfig_.pedal_type=1;
}

void DAP_mech_config_st::storeConfigToEprom(DAP_mech_config_st& config_st)
{

  EEPROM.put(EEPROM_OFFSET_MECH_CONFIG, config_st); 
  EEPROM.commit();
  Serial.println("Successfully stored mech config to EEPROM");
  
  /*if (true == config_st.payLoadHeader_.storeToEeprom)
  {
    config_st.payLoadHeader_.storeToEeprom = false; // set to false, thus at restart existing EEPROM config isn't restored to EEPROM
    EEPROM.put(0, config_st); 
    EEPROM.commit();
    Serial.println("Successfully stored config in EPROM");
  }*/
}

void DAP_mech_config_st::loadConfigFromEprom(DAP_mech_config_st& config_st)
{
  DAP_mech_config_st local_config_st;

  EEPROM.get(EEPROM_OFFSET_MECH_CONFIG, local_config_st);
  //EEPROM.commit();

  config_st = local_config_st;

  // check if version matches revision, in case, update the default config
  /*if (local_config_st.payLoadHeader_.version == DAP_VERSION_CONFIG)
  {
    config_st = local_config_st;
    Serial.println("Successfully loaded config from EPROM");
  }
  else
  { 
    Serial.println("Couldn't load config from EPROM due to version mismatch");
    Serial.print("Target version: ");
    Serial.println(DAP_VERSION_CONFIG);
    Serial.print("Source version: ");
    Serial.println(local_config_st.payLoadHeader_.version);

  }*/

}





void DAP_calculationVariables_st::updateFromConfig(DAP_config_st& config_st) {
  startPosRel = ((float)config_st.payLoadPedalConfig_.pedalStartPosition) / 100.0f;
  endPosRel = ((float)config_st.payLoadPedalConfig_.pedalEndPosition) / 100.0f;

  x_foot_min_curr = x_foot_rel_to_abs(startPosRel);
  x_foot_max_curr = x_foot_rel_to_abs(endPosRel);

  x_foot_center_curr = x_foot_min_curr + ((x_foot_max_curr - x_foot_min_curr) / 2.0f);

  if (startPosRel  ==  endPosRel)
  {
    endPosRel =   startPosRel + 1 / 100;
  }
  
  absFrequency = ((float)config_st.payLoadPedalConfig_.absFrequency);
  absAmplitude = ((float)config_st.payLoadPedalConfig_.absAmplitude) / 20.0f * 9.81; // from kg to N

  dampingPress = ((float)config_st.payLoadPedalConfig_.dampingPress) * 0.00015f;
  RPM_max_freq = ((float)config_st.payLoadPedalConfig_.RPM_max_freq);
  RPM_min_freq = ((float)config_st.payLoadPedalConfig_.RPM_min_freq);
  RPM_AMP = ((float)config_st.payLoadPedalConfig_.RPM_AMP) / 100.0f;
  //Bite point effect;
  
  BP_trigger_value=(float)config_st.payLoadPedalConfig_.BP_trigger_value;
  BP_amp=((float)config_st.payLoadPedalConfig_.BP_amp) / 100.0f;
  BP_freq=(float)config_st.payLoadPedalConfig_.BP_freq;
  WS_amp=((float)config_st.payLoadPedalConfig_.WS_amp) / 20.0f;
  WS_freq=(float)config_st.payLoadPedalConfig_.WS_freq;
  // update force variables
  Force_Min = ((float)config_st.payLoadPedalConfig_.preloadForce) * 9.81; // to N
  Force_Max = ((float)config_st.payLoadPedalConfig_.maxForce) * 9.81; // to N
  Force_Range = Force_Max - Force_Min;
  Force_Max_default=((float)config_st.payLoadPedalConfig_.maxForce); 
  pedal_type=config_st.payLoadPedalConfig_.pedal_type;

  // calculate steps per motor revolution
  // float helper = MAXIMUM_STEPPER_SPEED / (MAXIMUM_STEPPER_RPM / SECONDS_PER_MINUTE);
  // helper = floor(helper / 10) * 10;
  // helper = constrain(helper, 2000, 10000);
  // stepsPerMotorRevolution = helper;

  // when spindle pitch is smaller than 8, choose coarse microstepping
  if ( 8 > config_st.payLoadPedalConfig_.spindlePitch_mmPerRev_u8)
  {stepsPerMotorRevolution = 3200;}
  else{stepsPerMotorRevolution = 6400;}
  
}

void DAP_calculationVariables_st::updateFromMechConfig(DAP_mech_config_st& mech_config_st) {
  x_foot_min_abs = float(mech_config_st.payLoadMechConfig_.x_foot_min_abs) / 10.0f;
  x_foot_max_abs = float(mech_config_st.payLoadMechConfig_.x_foot_max_abs) / 10.0f;
  x_foot_range_abs = x_foot_max_abs - x_foot_min_abs;
}

float DAP_calculationVariables_st::x_foot_rel_to_abs(float x_foot_rel) {
  return x_foot_min_abs + (x_foot_range_abs * x_foot_rel);
}

void DAP_calculationVariables_st::dynamic_update()
{
  Force_Range = Force_Max - Force_Min;
}

void DAP_calculationVariables_st::reset_maxforce()
{
  Force_Max = Force_Max_default;
}

void DAP_calculationVariables_st::updateEndstops(long newMinEndstop, long newMaxEndstop) {
 
  if ( newMinEndstop == newMaxEndstop )
  {
    newMaxEndstop = newMinEndstop  + 10;
  }
  
  stepperPosMinEndstop = newMinEndstop;
  stepperPosMaxEndstop = newMaxEndstop;
  stepperPosEndstopRange = stepperPosMaxEndstop - stepperPosMinEndstop;
  
  stepperPosMin = stepperPosEndstopRange * startPosRel;
  stepperPosMax = stepperPosEndstopRange * endPosRel;
  stepperPosMin_default = stepperPosMin;
  stepperPosRange = stepperPosMax - stepperPosMin;
  //current_pedal_position_ratio=((float)(current_pedal_position-stepperPosMin_default))/((float)stepperPosRange_default);
}

void DAP_calculationVariables_st::updateStiffness() {
  springStiffnesss = Force_Range / stepperPosRange;
  if ( fabs(springStiffnesss) > 0.0001 )
  {
      springStiffnesssInv = 1.0 / springStiffnesss;
  }
  else
  {
    springStiffnesssInv = 1000000;
  }
  
  }

void DAP_calculationVariables_st::StepperPos_setback()
{
  stepperPosMin=stepperPosMin_default;
  stepperPosMax=stepperPosMax_default;
  stepperPosRange = stepperPosRange_default;
}

void DAP_calculationVariables_st::update_stepperMinpos(long newMinstop)
{
  stepperPosMin=newMinstop;
  
  stepperPosRange = stepperPosMax - stepperPosMin;
}
void DAP_calculationVariables_st::update_stepperMaxpos( long newMaxstop)
{
  
  stepperPosMax=newMaxstop;
  stepperPosRange = stepperPosMax - stepperPosMin;
}

void DAP_calculationVariables_st::Default_pos()
{
  stepperPosMin_default = stepperPosMin;
  stepperPosMax_default = stepperPosMax;
  stepperPosRange_default=stepperPosRange;
}


