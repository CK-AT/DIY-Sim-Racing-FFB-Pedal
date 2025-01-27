#include <FastAccelStepper.h>
#include "isv57communication.h"
#include "DiyActivePedal_types.h"
#include "Main.h"

// these are physical properties of the stepper
static const uint32_t MAXIMUM_STEPPER_RPM = 4000;     
static const uint32_t SECONDS_PER_MINUTE = 60;
//static const uint32_t MAXIMUM_STEPPER_SPEED = (  (float)MAXIMUM_STEPPER_RPM * (float)STEPS_PER_MOTOR_REVOLUTION) / (float)SECONDS_PER_MINUTE;   // steps/s
#define MAXIMUM_STEPPER_SPEED (uint32_t)300000//  max steps per second, see https://github.com/gin66/FastAccelStepper
static const int32_t MAXIMUM_STEPPER_ACCELERATION = 1e10;                                                 // steps/s²


class StepperWithLimits {
private:
  FastAccelStepper* _stepper;
  uint8_t _pinMin,   _pinMax;      // pins that limit switches attach to
  int32_t _limitMin, _limitMax;    // stepper position at limit switches
  int32_t _posMin,   _posMax;      // stepper position at min/max of travel

public:
  StepperWithLimits(uint8_t pinStep, uint8_t pinDirection, uint8_t pinMin, uint8_t pinMax, bool invertMotorDir_b);
  bool hasValidStepper() const { return NULL != _stepper; }

public:
  void findMinMaxEndstops();
  void refindMinLimit();
  void checkLimitsAndResetIfNecessary();
  void updatePedalMinMaxPos(uint8_t pedalStartPosPct, uint8_t pedalEndPosPct);
  bool isAtMinPos();
  bool correctPos(int32_t posOffset);
  void findMinMaxSensorless(isv57communication * isv57, DAP_base_config_st& mech_config_st);
  void refindMinLimitSensorless(isv57communication * isv57);
public:
  int8_t moveTo(int32_t position, bool blocking = false);
  void moveSlowlyToPos(int32_t targetPos_ui32);
  void printStates();

public:
  int32_t getCurrentPositionFromMin() const;
  int32_t getCurrentPosition() const;
  double getCurrentPositionFraction() const;
  double getCurrentPositionFractionFromExternalPos(int32_t extPos_i32) const;
  int32_t getTargetPositionSteps() const;

public:
  int32_t getLimitMin() const { return _limitMin; }
  int32_t getLimitMax() const { return _limitMax; }
  int32_t getTravelSteps() const { return _posMax - _posMin; }
  void setSpeed(uint32_t speedInStepsPerSecond);
};
