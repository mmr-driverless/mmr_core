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
	state: OK,
	_adcValues: adcValues,
	_lpDataMeasured: lpDataMeasured,
  };

  return clutch;
}

void setDrivingMode(Clutch *clutch, DrivingMode mode) {
	clutch->mode = mode;
}

void setErrorState(Clutch *clutch, ErrorState state){
	clutch->state = state;
}

void setTargetAngle(Clutch *clutch, float angle) {
	clutch->targetAngle = angle;
}

float getMotorDutyCycle(Clutch *clutch) {
  clutch->measuredAngle = getPotMotAngle(clutch);//lowpassFilter(getPotMotAngle(clutch), &clutch->_lpDataMeasured);
  //clutch->targetAngle = 0.8f;
  //clutch->current = getCurrent(clutch);

  if(clutch->mode == MANUAL)
    clutch->targetAngle =  getLeverAngle(clutch);

  if (clutch->measuredAngle>ENGAGED_CLUTCH_ANGLE || clutch->measuredAngle<OPEN_CLUTCH_ANGLE){
	  setErrorState(clutch, POTENTIOMETER_OUT_OF_RANGE);
	  return 0.0f;
  }

  setErrorState(clutch, OK);
  return PIDCompute(clutch->clutchPID.pid1, clutch->targetAngle, clutch->measuredAngle);
  /*return PIDCascade(
		  clutch->clutchPID.pid1,
		  clutch->clutchPID.pid2,
		  clutch->targetAngle,
		  clutch->measuredAngle,
		  clutch->current
  );*/
}

float getCurrent(Clutch *clutch) {
  const float value = clutch->_adcValues[clutch->indexes.current];
  float current = (((value / MAX_ADC_VALUE) *
	5.0f) - 2.57f) *
	0.1f;//COSTANTE K

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

	//Ramp parameters
	const int countLimit1 = 6000; //First slope (higher=slower)
	const int countLimit2 = 18000; //Second slope (higher=slower)

	const float clutchDelta1 = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) * 0.35; //Engagement point
	const float clutchDelta2 = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) - clutchDelta1; // 0.3

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
				float t = (clutchDelta1 + (clutchDelta2 / (countLimit2 - countLimit1)) * (countRamp - countLimit1)) + OPEN_CLUTCH_ANGLE;
				setTargetAngle(clutch, t);
			}
			else {
				step = 0;
			}

			break;

	}
}
