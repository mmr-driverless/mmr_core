#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {
  LowpassData lpDataMeasured = {
		input: 500,
		output: 500,
		cutoffFrequency: 10.0f,
  };

  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
	mode: MANUAL,
	_adcValues: adcValues,
	_lpDataMeasured: lpDataMeasured,
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
  clutch->measuredAngle = getPotMotAngle(clutch);//lowpassFilter(getPotMotAngle(clutch), &clutch->_lpDataMeasured);
  clutch->current = getCurrent(clutch);
  //clutch->targetAngle = 0.8f;

  if(clutch->mode == MANUAL)
    clutch->targetAngle =  getLeverAngle(clutch);

  //return PIDCompute(clutch->clutchPID.pid1, clutch->targetAngle, clutch->measuredAngle);
  return PIDCascade(
		  clutch->clutchPID.pid1,
		  clutch->clutchPID.pid2,
		  clutch->targetAngle,
		  clutch->measuredAngle,
		  clutch->current
  );
}

float getCurrent(Clutch *clutch) {
  const float value = clutch->_adcValues[clutch->indexes.current];
  float current = ((value / MAX_ADC_VALUE) * MAX_VOLTAGE) *
		VOLTAGE_RATIO *
		SENSITIVITY;

  return current;
}

float getPotMotAngle(Clutch *clutch) {
  const float potValue = _getMotorPotentiomerValue(clutch);
  return _getPotentiometerAngle(potValue, clutch);
}

float getLeverAngle(Clutch *clutch) {
  const float leverValue = _getLeverValue(clutch);
  float targetPosition = _getPotentiometerAngle(leverValue, clutch);

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

float _getPotentiometerAngle(AdcValue value, Clutch *clutch) {
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
	setTargetAngle(clutch, OPEN_CLUTCH_ANGLE);
}

void engagedClutch(Clutch *clutch) {
	static uint8_t step = 0;
	static int countRamp = 0;

	const int countLimit1 = 8000;
	const int countLimit2 = 32000;

	const float clutchDelta1 = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) * 0.7;
	const float clutchDelta2 = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) - clutchDelta1;

	switch(step) {
		case 0:
			countRamp = 0;

			if(clutch->measuredAngle < ENGAGED_CLUTCH_ANGLE - 0.2f)
				step = 1;

			break;
		case 1:
			if(countRamp < countLimit1) {
				countRamp = countRamp + 1;
				float t = ((clutchDelta1 / countLimit1) * countRamp) + OPEN_CLUTCH_ANGLE;
				setTargetAngle(clutch, t);
			}
			else {
				step = 2;
			}

			break;
		case 2:
			if(countRamp < countLimit2) {
				countRamp = countRamp + 1;
				float t = (clutchDelta1 + (clutchDelta2 / countLimit2) * (countRamp - countLimit1)) + OPEN_CLUTCH_ANGLE;
				setTargetAngle(clutch, t);
			}
			else {
				step = 0;
			}

			break;

	}
}
