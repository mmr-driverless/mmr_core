#include "mmr_clutch.h"

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {

   LowpassData lpDataCurrent = {
		input: 500,
		output: 500,
		cutoffFrequency: 100.0f,
  };
  LowpassData lpDataAngle = {
 		input: 500,
 		output: 500,
 		cutoffFrequency: 100.0f,
   };

  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
	mode: MANUAL,
	_adcValues: adcValues,
	_lpDataAngle: lpDataAngle,
	_lpDataCurrent: lpDataCurrent,
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
  clutch->targetAngle = getLeverAngle(clutch);
  clutch->current = getCurrent(clutch);

  /*if(clutch->mode == MANUAL){
	    //clutch->targetAngle =  getLeverAngle(clutch);
	  clutch->targetAngle = 2.0f;
  }*/



// return PIDCompute(clutch->clutchPID.pid2, /*clutch->targetAngle*/voltageTarget, /*clutch->measuredAngle*/ clutch->current);
  return PIDCascade(
		  clutch->clutchPID.pid1,
		  clutch->clutchPID.pid2,
		  clutch->targetAngle,
		  clutch->measuredAngle,
		  clutch->current
  );
}

float getCurrent(Clutch *clutch) {
  float current = CURRENT_CORRECTIVE_FACTOR *
		  (clutch->_adcValues[clutch->indexes.current]/MAX_ADC_VALUE) *
		  3.6f;

  return lowpassFilter(current, &clutch->_lpDataCurrent);
}

float getPotMotAngle(Clutch *clutch) {
	float angle = _getPotentiometerAngle(
			  _getMotorPotentiomerValue(clutch),
			  clutch);
  return  lowpassFilter(angle, &clutch->_lpDataAngle);
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
