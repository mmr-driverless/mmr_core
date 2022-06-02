#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {
  LowpassData lpData = {
		input: 500,
		output: 500,
		cutoffFrequency: 20.0f,
		dt: 0.0125f
  };

  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
	mode: MANUAL,
    _adcValues: adcValues,
	_lpData: lpData,
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
  return _getPotentiometerAngle(clutch, potValue);
}

float getLeverAngle(Clutch *clutch) {
  const float leverValue = _getLeverValue(clutch);
  float targetPosition = _getPotentiometerAngle(clutch, leverValue);

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

float _getPotentiometerAngle(Clutch *clutch, AdcValue value) {
	const uint16_t filteredValue = lowpassFilter(value, &clutch->_lpData);

	const float voltage = ((float)filteredValue / MAX_ADC_VALUE) *
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
	clutch->inProgress = clutch->measuredAngle < (OPEN_CLUTCH_ANGLE - 0.15f);
	setTargetAngle(clutch, OPEN_CLUTCH_ANGLE);
}

void engagedClutch(Clutch *clutch) {
	static uint8_t step = 0;
	static uint16_t count = 0;
	const uint16_t countLimit = 16000;
	const float clutchDelta = ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE;


	switch(step) {
		case 0:
			if(clutch->measuredAngle <= clutch->targetAngle + 0.1f) {
				clutch->inProgress = true;
				count = 0;
				step = 1;
			}

			break;

		case 1:
			if(count <= countLimit + 16000) {
				count++;
				float c = count > countLimit
						? countLimit
						: count;
				float pos = (float)(countLimit - c) / countLimit;
				float t = ENGAGED_CLUTCH_ANGLE - (clutchDelta * pos);
				setTargetAngle(clutch, t);
			}
			else {
				step = 2;
			}
			break;

		case 2:
			count = 0;
			clutch->inProgress = false;
			step = 0;
			break;
	}
}
