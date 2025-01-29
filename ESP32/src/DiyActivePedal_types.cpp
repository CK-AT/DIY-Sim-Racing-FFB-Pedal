#include "DiyActivePedal_types.h"
#include "Arduino.h"

#include "PedalGeometry.h"
#include "StepperWithLimits.h"

#include <EEPROM.h>

static const float ABS_SCALING = 50;

const uint32_t EEPROM_OFFSET_BASE_CONFIG = 0;
const uint32_t EEPROM_OFFSET_ROAD_PEDAL_CONFIG = EEPROM_OFFSET_BASE_CONFIG + sizeof(DAP_base_config_st);
const uint32_t EEPROM_OFFSET_FLIGHT_PEDAL_CONFIG = EEPROM_OFFSET_ROAD_PEDAL_CONFIG + sizeof(DAP_road_pedal_config_st);

void DAP_road_pedal_config_st::initialiseDefaults() {
  header.payloadType = DAP_PAYLOAD_TYPE_ROAD_PEDAL_CONFIG;
  header.version = DAP_VERSION_CONFIG;
  header.storeToEeprom = false;

  data.pedalStartPosition = 10;
  data.pedalEndPosition = 85;

  data.maxForce = 60;
  data.preloadForce = 2;

  data.relativeForce_p000 = 0;
  data.relativeForce_p020 = 20;
  data.relativeForce_p040 = 40;
  data.relativeForce_p060 = 60;
  data.relativeForce_p080 = 80;
  data.relativeForce_p100 = 100;

  data.dampingPress = 0.01f;
  data.dampingPull = 0.01f;

  data.absFrequency = 15;
  data.absAmplitude = 0;
  data.absPattern = 0;
  data.absForceOrTarvelBit = 0;

  data.Simulate_ABS_trigger = 0;// add for abs trigger
  data.Simulate_ABS_value = 80;// add for abs trigger
  data.RPM_max_freq = 40;
  data.RPM_min_freq = 10;
  data.RPM_AMP = 5;
  data.BP_trigger_value =50;
  data.BP_amp=1;
  data.BP_freq=15;
  data.BP_trigger=0;
  data.G_multi = 50;
  data.G_window=60;
  data.WS_amp=1;
  data.WS_freq=15;
  data.Road_multi = 50;
  data.Road_window=60;
  data.cubic_spline_param_a_array[0] = 0;
  data.cubic_spline_param_a_array[1] = 0;
  data.cubic_spline_param_a_array[2] = 0;
  data.cubic_spline_param_a_array[3] = 0;
  data.cubic_spline_param_a_array[4] = 0;

  data.cubic_spline_param_b_array[0] = 0;
  data.cubic_spline_param_b_array[1] = 0;
  data.cubic_spline_param_b_array[2] = 0;
  data.cubic_spline_param_b_array[3] = 0;
  data.cubic_spline_param_b_array[4] = 0;

  data.travelAsJoystickOutput_u8 = 0;
}




void DAP_road_pedal_config_st::storeConfigToEprom(DAP_road_pedal_config_st& config_st)
{

  EEPROM.put(EEPROM_OFFSET_ROAD_PEDAL_CONFIG, config_st); 
  EEPROM.commit();
  Serial.println("Successfully stored general config to EEPROM");
  
  /*if (true == config_st.header.storeToEeprom)
  {
    config_st.header.storeToEeprom = false; // set to false, thus at restart existing EEPROM config isn't restored to EEPROM
    EEPROM.put(0, config_st); 
    EEPROM.commit();
    Serial.println("Successfully stored config in EPROM");
  }*/
}

void DAP_road_pedal_config_st::loadConfigFromEprom(DAP_road_pedal_config_st& config_st)
{
  DAP_road_pedal_config_st local_config_st;

  EEPROM.get(EEPROM_OFFSET_ROAD_PEDAL_CONFIG, local_config_st);
  //EEPROM.commit();

  config_st = local_config_st;

  // check if version matches revision, in case, update the default config
  /*if (local_config_st.header.version == DAP_VERSION_CONFIG)
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
    Serial.println(local_config_st.header.version);

  }*/

}

void DAP_base_config_st::initialiseDefaults() {
  header.payloadType = DAP_PAYLOAD_TYPE_BASE_CONFIG;
  header.version = DAP_VERSION_CONFIG;
  header.storeToEeprom = false;

  data.lengthPedal_a = 100;
  data.lengthPedal_b = 153; 
  data.lengthPedal_d = 80; 
  data.lengthPedal_c_horizontal = 82;
  data.lengthPedal_c_vertical = 32;
  data.lengthPedal_travel = 110;
  
  data.coeffs_force_factor_over_pedal_pos[0] = 6.20184902e-01;
  data.coeffs_force_factor_over_pedal_pos[1] = -1.71372506e-03;
  data.coeffs_force_factor_over_pedal_pos[2] = 1.07828479e-05;
  data.coeffs_force_factor_over_pedal_pos[3] = 2.71382634e-09;
  data.coeffs_force_factor_over_pedal_pos[4] = 7.34203389e-11;

  data.coeffs_sled_pos_over_pedal_pos[0] = 5.50622588e+01;
  data.coeffs_sled_pos_over_pedal_pos[1] = 5.55488175e-01;
  data.coeffs_sled_pos_over_pedal_pos[2] = 7.59065219e-04;
  data.coeffs_sled_pos_over_pedal_pos[3] = -2.81513616e-06;
  data.coeffs_sled_pos_over_pedal_pos[4] = -1.11220289e-08;

  data.x_foot_min_abs = -1259;
  data.x_foot_max_abs = 927;

  data.loadcell_rating = 100;
  data.spindlePitch_mmPerRev_u8 = 5;
  data.steps_per_mm_u8 = 1000;

  data.invertLoadcellReading_u8 = 0;

  data.invertMotorDirection_u8 = 0;

  data.kf_modelNoise = 128;
  data.kf_modelOrder = 0;

  data.debug_flags_0 = 0;

  data.pedal_type=1;
}

void DAP_base_config_st::storeConfigToEprom(DAP_base_config_st& config_st)
{

  EEPROM.put(EEPROM_OFFSET_BASE_CONFIG, config_st); 
  EEPROM.commit();
  Serial.println("Successfully stored mech config to EEPROM");
  
  /*if (true == config_st.header.storeToEeprom)
  {
    config_st.header.storeToEeprom = false; // set to false, thus at restart existing EEPROM config isn't restored to EEPROM
    EEPROM.put(0, config_st); 
    EEPROM.commit();
    Serial.println("Successfully stored config in EPROM");
  }*/
}

void DAP_base_config_st::loadConfigFromEprom(DAP_base_config_st& config_st)
{
  DAP_base_config_st local_config_st;

  EEPROM.get(EEPROM_OFFSET_BASE_CONFIG, local_config_st);
  //EEPROM.commit();

  config_st = local_config_st;

  // check if version matches revision, in case, update the default config
  /*if (local_config_st.header.version == DAP_VERSION_CONFIG)
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
    Serial.println(local_config_st.header.version);

  }*/

}





void DAP_calculationVariables_st::updateFromRoadPedalConfig(DAP_road_pedal_config_st& config_st) {
  startPosRel = ((float)config_st.data.pedalStartPosition) / 100.0f;
  endPosRel = ((float)config_st.data.pedalEndPosition) / 100.0f;

  x_foot_min_curr = x_foot_rel_to_abs(startPosRel);
  x_foot_max_curr = x_foot_rel_to_abs(endPosRel);

  x_foot_center_curr = x_foot_min_curr + ((x_foot_max_curr - x_foot_min_curr) / 2.0f);

  if (startPosRel  ==  endPosRel)
  {
    endPosRel =   startPosRel + 1 / 100;
  }
  
  absFrequency = ((float)config_st.data.absFrequency);
  absAmplitude = ((float)config_st.data.absAmplitude) / 20.0f * 9.81; // from kg to N

  dampingPress = config_st.data.dampingPress;
  dampingPull = config_st.data.dampingPull;
  RPM_max_freq = ((float)config_st.data.RPM_max_freq);
  RPM_min_freq = ((float)config_st.data.RPM_min_freq);
  RPM_AMP = ((float)config_st.data.RPM_AMP) / 100.0f;
  //Bite point effect;
  
  BP_trigger_value=(float)config_st.data.BP_trigger_value;
  BP_amp=((float)config_st.data.BP_amp) / 100.0f;
  BP_freq=(float)config_st.data.BP_freq;
  WS_amp=((float)config_st.data.WS_amp) / 20.0f;
  WS_freq=(float)config_st.data.WS_freq;
  // update force variables
  Force_Min = ((float)config_st.data.preloadForce) * 9.81; // to N
  Force_Max = ((float)config_st.data.maxForce) * 9.81; // to N
  Force_Range = Force_Max - Force_Min;
  Force_Max_default=((float)config_st.data.maxForce); 

  // calculate steps per motor revolution
  // float helper = MAXIMUM_STEPPER_SPEED / (MAXIMUM_STEPPER_RPM / SECONDS_PER_MINUTE);
  // helper = floor(helper / 10) * 10;
  // helper = constrain(helper, 2000, 10000);
  // stepsPerMotorRevolution = helper;
}

void DAP_calculationVariables_st::updateFromBaseConfig(DAP_base_config_st& mech_config_st) {
  pedal_type=mech_config_st.data.pedal_type;
  x_foot_min_abs = float(mech_config_st.data.x_foot_min_abs) / 10.0f;
  x_foot_max_abs = float(mech_config_st.data.x_foot_max_abs) / 10.0f;
  x_foot_range_abs = x_foot_max_abs - x_foot_min_abs;
}

void DAP_calculationVariables_st::updatePedalType(uint8_t new_pedal_type) {
  pedal_type = new_pedal_type;
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


