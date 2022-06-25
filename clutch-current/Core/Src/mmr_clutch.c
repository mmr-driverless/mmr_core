#include "mmr_clutch.h"

#define BUFFER_DIM 40

float bufferCurr[BUFFER_DIM];

extern float voltageTarget;



Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues) {
  LowpassData lpDataMeasured = {
		input: 500,
		output: 500,
		cutoffFrequency: 10.0f,
  };

  LowpassData lpDataCurrent = {
		input: 500,
		output: 500,
		cutoffFrequency: 10.0f,
  };


  float m = (ENGAGED_CLUTCH_ANGLE - OPEN_CLUTCH_ANGLE) / (ENGAGED_LEVER_ANGLE - OPEN_LEVER_ANGLE);
  float q = ENGAGED_CLUTCH_ANGLE - (m * ENGAGED_LEVER_ANGLE);

  Clutch clutch = {
	clutchPID: clutchPID,
    indexes: indexes,
	mode: MANUAL,
	_adcValues: adcValues,
	_lpDataMeasured: lpDataMeasured,
	_lpDataMeasured: lpDataCurrent,

	_leverM: m,
	_leverQ: q,

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
  clutch->targetAngle = 0.8f;
  clutch->current = getCurrent(clutch);

  /*if(clutch->mode == MANUAL){
	    //clutch->targetAngle =  getLeverAngle(clutch);
	  clutch->targetAngle = 2.0f;
  }*/


  //return PIDCompute(clutch->clutchPID.pid2, /*clutch->targetAngle*/voltageTarget, /*clutch->measuredAngle*/ clutch->current);
  return PIDCascade(
		  clutch->clutchPID.pid1,
		  clutch->clutchPID.pid2,
		  0.7f,
		  //clutch->targetAngle,
		  clutch->measuredAngle,
		  clutch->current
  );
}

float getCurrent(Clutch *clutch) {
  //float avg = 0;
  /*for(int i = 2; i < 21; i += 3) {
	  avg += clutch->_adcValues[i];//lowpassFilter(clutch->_adcValues[index], &clutch->_lpDataMeasured);
  }

  for(int i = 0; i < BUFFER_DIM - 1; i++) {
	  bufferCurr[i] = bufferCurr[i + 1];
  }

  bufferCurr[BUFFER_DIM - 1] = avg / 7.0f;

  float min = bufferCurr[0];
  float max = bufferCurr[0];

  for(int i = 1; i < BUFFER_DIM; i ++) {
      if(min > bufferCurr[i]) {
		  min = bufferCurr[i];
      }

      if(max < bufferCurr[i]) {
		  max = bufferCurr[i];
      }
  }

  avg = (max + min) / 2.0f;*/
 //  float current = FattoreCorrettivo*10.0f * ((value / MAX_ADC_VALUE) * 3.6f) - 16.8f;
  float current = CURRENT_CORRECTIVE_FACTOR *
		  (clutch->_adcValues[clutch->indexes.current]/MAX_ADC_VALUE) *
		  3.6f;

  return lowpassFilter(current, &clutch->_lpDataCurrent);
}

float getPotMotAngle(Clutch *clutch) {
  return _getPotentiometerAngle(
	  _getMotorPotentiomerValue(clutch),
	  clutch
  );
}

float getLeverAngle(Clutch *clutch) {
  float targetPosition = _getPotentiometerAngle(
		  _getLeverValue(clutch),
		  clutch
  );

  if(targetPosition >= ENGAGED_LEVER_ANGLE)
	  targetPosition = ENGAGED_LEVER_ANGLE;

  if(targetPosition <= OPEN_LEVER_ANGLE)
	  targetPosition = OPEN_LEVER_ANGLE;

  return  (targetPosition * clutch->_leverM) + clutch->_leverQ;;
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
