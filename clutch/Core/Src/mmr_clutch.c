#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {
  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
    _adcValues: adcValues,
  };

  return clutch;
}

float getMotorDutyCycle(Clutch *clutch) {
  const float measuredAngle = getMeasuredAngle(clutch);
  const float targetAngle = getTargetAngle(clutch);
  const float current = getCurrent(clutch);

  clutch->measuredAngle = measuredAngle;
  clutch->targetAngle = targetAngle;
  clutch->current = current;

  #ifdef POSITION_ONLY
  	  return PIDCompute(clutch->clutchPID.pid1, targetAngle, measuredAngle);
  #else
	  return PIDCascade(clutch->clutchPID.pid1, clutch->clutchPID.pid2, targetAngle, measuredAngle, current);
  #endif
}


float getMeasuredAngle(Clutch *clutch) {
  const float potValue = _getMotorPotentiomerValue(clutch);
  float measuredPosition = (potValue / MAX_ADC_VALUE) * 
    MAX_VOLTAGE * 
    VOLTAGE_RATIO;

	return (( measuredPosition - 0.25f ) / 4.5f) * 1.74f;
}

float _getMotorPotentiomerValue(Clutch *clutch) {
	#ifdef ADC_AVERAGE
	  return _getAvg(clutch, clutch->indexes.motorPotentiometer);
	#else
	  return clutch->_adcValues[clutch->indexes.motorPotentiometer];
	#endif
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

float _getLeverValue(Clutch *clutch) {
	#ifdef ADC_AVERAGE
	  return _getAvg(clutch, clutch->indexes.lever);
	#else
	  return clutch->_adcValues[clutch->indexes.lever];
	#endif
  return clutch->_adcValues[clutch->indexes.lever];
}

float getCurrent(Clutch *clutch) {
	return _getCurrentValue(clutch);
}

float _getCurrentValue(Clutch *clutch) {
	#ifdef ADC_AVERAGE
	  return _getAvg(clutch, clutch->indexes.current);
	#else
	  return clutch->_adcValues[clutch->indexes.current];
	#endif
}

float _getAvg(Clutch *clutch, AdcIndex index) {
	float averageValue = 0;

	for(AdcIndex i = index; i < BUFFER_LENGTH; i += ADC_CHANNELS) {
		averageValue += clutch->_adcValues[index];
	}

	return averageValue / (BUFFER_LENGTH / ADC_CHANNELS);
}
