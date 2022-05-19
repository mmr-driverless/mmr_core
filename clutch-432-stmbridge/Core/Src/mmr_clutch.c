#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {
  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
    _adcValues: adcValues,
	mode: MANUAL,
  };

  return clutch;
}

void setDrivingMode(Clutch *clutch, DrivingMode mode) {
	clutch->mode = mode;
}

void setTargetAngle(Clutch *clutch, float angle) {
	clutch->targetAngle = angle;
}

float getMotorDutyCycle(Clutch *clutch) {
  clutch->measuredAngle = getPotMotAngle(clutch);

  if(clutch->mode == MANUAL)
    clutch->targetAngle =  getLeverAngle(clutch);

  return PIDCompute(clutch->clutchPID.pid1, clutch->targetAngle, clutch->measuredAngle);
}


float getPotMotAngle(Clutch *clutch) {
  const float potValue = _getMotorPotentiomerValue(clutch);
  float measuredPosition = (potValue / MAX_ADC_VALUE) * 
    MAX_VOLTAGE * 
    VOLTAGE_RATIO;

	return (( measuredPosition - 0.25f ) / 4.5f) * 1.74f;
}

float getLeverAngle(Clutch *clutch) {
  const float leverValue = _getLeverValue(clutch);
  float targetPosition = (leverValue / MAX_ADC_VALUE) * 
    MAX_VOLTAGE * 
    VOLTAGE_RATIO;

  targetPosition = (targetPosition * -0.545f) + 1.69f;

  if(targetPosition < ENGAGED_CLUTCH_ANGLE)
    return ENGAGED_CLUTCH_ANGLE;

  if(targetPosition > OPEN_CLUTCH_ANGLE)
    return OPEN_CLUTCH_ANGLE;

  return targetPosition;
}

float _getMotorPotentiomerValue(Clutch *clutch) {
	  return clutch->_adcValues[clutch->indexes.motorPotentiometer];
}

float _getLeverValue(Clutch *clutch) {
	  return clutch->_adcValues[clutch->indexes.lever];
}
