#include "DiyActivePedal_types.h"
#include "Arduino.h"
#include "LogOutput.h"

#include "PedalGeometry.h"
//#include "StepperWithLimits.h"

#include <EEPROM.h>

static const float ABS_SCALING = 50;

const uint32_t EEPROM_OFFSET = (DAP_VERSION_CONFIG-128) * sizeof(DAP_config_st) % (2048-sizeof(DAP_config_st));

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

  payLoadPedalConfig_.PID_p_gain = 0.3;
  payLoadPedalConfig_.PID_i_gain = 50.0;
  payLoadPedalConfig_.PID_d_gain = 0.0;
  payLoadPedalConfig_.PID_velocity_feedforward_gain = 0.0;


  payLoadPedalConfig_.MPC_0th_order_gain = 1.0;
  payLoadPedalConfig_.MPC_1st_order_gain = 0.0;
  payLoadPedalConfig_.MPC_2nd_order_gain = 0.0;

  payLoadPedalConfig_.control_strategy_b = 0;

  payLoadPedalConfig_.maxGameOutput = 100;

  payLoadPedalConfig_.kf_modelNoise = 128;
  payLoadPedalConfig_.kf_modelOrder = 1;

  payLoadPedalConfig_.debug_flags_0 = 0;

  payLoadPedalConfig_.loadcell_rating = 150;

  payLoadPedalConfig_.travelAsJoystickOutput_u8 = 0;

  payLoadPedalConfig_.invertLoadcellReading_u8 = 0;

  payLoadPedalConfig_.invertMotorDirection_u8 = 0;
  payLoadPedalConfig_.pedal_type=4;
  //payLoadPedalConfig_.OTA_flag=0;
  payLoadPedalConfig_.stepLossFunctionFlags_u8=0b11;
  //payLoadPedalConfig_.Joystick_ESPsync_to_ESP=0;
}




void DAP_config_st::storeConfigToEprom(DAP_config_st& config_st)
{

  EEPROM.put(EEPROM_OFFSET, config_st); 
  EEPROM.commit();
  LogOutput::printf("Successfully stored config in EPROM");
}

void DAP_config_st::loadConfigFromEprom(DAP_config_st& config_st)
{
  DAP_config_st local_config_st;

  EEPROM.get(EEPROM_OFFSET, local_config_st);

  config_st = local_config_st;


}





void DAP_calculationVariables_st::updateFromConfig(DAP_config_st& config_st) {
  /*
  startPosRel = ((float)config_st.payLoadPedalConfig_.pedalStartPosition) / 100.0f;
  endPosRel = ((float)config_st.payLoadPedalConfig_.pedalEndPosition) / 100.0f;


  if (startPosRel  ==  endPosRel)
  {
    endPosRel =   startPosRel + 1 / 100;
  }
  
  absFrequency = ((float)config_st.payLoadPedalConfig_.absFrequency);
  absAmplitude = ((float)config_st.payLoadPedalConfig_.absAmplitude) / 20.0f; // in kg or percent

  dampingPress = ((float)config_st.payLoadPedalConfig_.dampingPress) / 400.0f;
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
  Force_Min = ((float)config_st.payLoadPedalConfig_.preloadForce);
  Force_Max = ((float)config_st.payLoadPedalConfig_.maxForce); 
  Force_Range = Force_Max - Force_Min;
  Force_Max_default=((float)config_st.payLoadPedalConfig_.maxForce); 
  pedal_type=config_st.payLoadPedalConfig_.pedal_type;
  */
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
  stepperPosRange = stepperPosRange_default;
}

void DAP_calculationVariables_st::update_stepperpos(long newMinstop)
{
  stepperPosMin=newMinstop;
  stepperPosRange = stepperPosMax - stepperPosMin;
}

void DAP_calculationVariables_st::Default_pos()
{
  stepperPosMin_default = stepperPosMin;
  stepperPosRange_default=stepperPosRange;
}


