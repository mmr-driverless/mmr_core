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
  return _getPotentiometerAngle(potValue);
}

float getLeverAngle(Clutch *clutch) {
  const float leverValue = _getLeverValue(clutch);
  float targetPosition = _getPotentiometerAngle(leverValue);

  const float m = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) /
		    (ENGAGED_LEVER_ANGLE - OPEN_LEVER_ANGLE);

  const float q = ENGAGED_CLUTCH_ANGLE - (m * ENGAGED_LEVER_ANGLE);

  if(targetPosition >= ENGAGED_LEVER_ANGLE)
	  targetPosition = ENGAGED_LEVER_ANGLE;

  if(targetPosition <= OPEN_LEVER_ANGLE)
	  targetPosition = OPEN_LEVER_ANGLE;

  targetPosition = (targetPosition * m) + q;
  return targetPosition;
}

float _getPotentiometerAngle(AdcValue value) {
	const float voltage = ((float)value / MAX_ADC_VALUE) *
		MAX_VOLTAGE *
		VOLTAGE_RATIO;

	return (( voltage - 0.25f ) / 4.5f) * 1.74f;
}

AdcValue _getMotorPotentiomerValue(Clutch *clutch) {
	  return clutch->_adcValues[clutch->indexes.motorPotentiometer];
}

AdcValue _getLeverValue(Clutch *clutch) {
	  return clutch->_adcValues[clutch->indexes.lever];
}

void openClutch(Clutch *clutch) {
	clutch->_inProgress = clutch->measuredAngle < (OPEN_CLUTCH_ANGLE - 0.15f);
	setTargetAngle(clutch, OPEN_CLUTCH_ANGLE);
}

void engagedClutch(Clutch *clutch) {
	static uint16_t count = 0;
	const uint16_t countLimit = 16000;
	const float clutchDelta = ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE;
	clutch->_inProgress = count <= countLimit + 16000;

	if(clutch->_inProgress) {
		count++;
		float c = count > countLimit ? countLimit : count;
		float pos = (float)(countLimit - c) / countLimit;
		float t = ENGAGED_CLUTCH_ANGLE - (clutchDelta * pos);
		setTargetAngle(clutch, t);
	}
	else {
		count = 0;
	}
}
