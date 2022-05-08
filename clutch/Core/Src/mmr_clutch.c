#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, uint16_t *adcValues, uint8_t bufferLength) {
  Clutch clutch = {
    indexes: indexes,
    bufferLength: bufferLength,
    measuredAngle: 0,
    targetAngle: 0,
    _adcValues: adcValues,
  };

  return clutch;
}

float getMotorDutyCycle(Clutch *clutch, PID *pid) {
  const float measuredAngle = getMeasuredAngle(clutch); 
  const float targetAngle = getTargetAngle(clutch);

  clutch->measuredAngle = measuredAngle;
  clutch->targetAngle = targetAngle;

  return PIDCompute(pid, targetAngle, measuredAngle);
}

float getMeasuredAngle(Clutch *clutch) {
  const float potValue = _getMotorPotentiomerValue(clutch);
  float measuredPosition = (potValue / MAX_ADC_VALUE) * 
    MAX_VOLTAGE * 
    VOLTAGE_RATIO;

	return (( measuredPosition - 0.25f ) / 4.5f) * 1.74f;
}

float getTargetAngle(Clutch *clutch) {
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

float _getCurrentValue(Clutch *clutch) {
  return clutch->_adcValues[clutch->indexes.current];
}
