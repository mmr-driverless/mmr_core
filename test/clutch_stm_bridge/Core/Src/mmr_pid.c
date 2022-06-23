#include "mmr_pid.h"
#include "stm32l4xx_hal.h"

extern float currentTarget;
extern float errorPos;
extern float voltageTarget;
extern float current;
extern float currentError;


bool _isLastOutputSaturated(PID* pid) {
  return 
    pid->_lastOutputs.outputPresaturation != 
    pid->_lastOutputs.output;
}

void _updateTerms(PID* pid, float error) {
  pid->_terms.p = getProportionalTerm(pid, error);
  pid->_terms.i = getIntegralTerm(pid, error);
  pid->_terms.d = getDerivativeTerm(pid, error);
}

float getOutput(PID* pid) {
	return
		pid->_terms.p + 
		pid->_terms.i + 
		pid->_terms.d;
}

float getOutputInSaturationRange(PID* pid, float output) {
	if (output >= pid->saturation.max * MAGIC_K)
		return pid->saturation.max * MAGIC_K;

	if (output <= pid->saturation.min * MAGIC_K)
		return pid->saturation.min * MAGIC_K;

	return output;
}

float getError(float reference, float measured) {
	#ifdef INVERT_ERROR
		return measured - reference;
	#else
		return reference - measured;
	#endif
}

float getProportionalTerm(PID* pid, float error) {
  return pid->parameters.p * error;
}

float getIntegralTerm(PID* pid, float error) {
	float i = pid->_terms.i +
		pid->parameters.i *
		0.5f *
		pid->sampleTime *
		(error + pid->_lastError);

	float maxInt = pid->saturation.max > pid->_terms.p
			? pid->saturation.max - pid->_terms.p
			: 0.0f;

	float minInt = pid->saturation.min < pid->_terms.p
			? pid->saturation.min - pid->_terms.p
			: 0.0f;

	if(i > maxInt)
		return maxInt;

	if(i < minInt)
		return minInt;

	return i;
}

float getDerivativeTerm(PID* pid, float error) {
	return 
		(2.0f * pid->parameters.d * (error - pid->_lastError)) + 
		((2.0f * pid->tau - pid->sampleTime) / (2.0f * pid->tau + pid->sampleTime)) *
		pid->_terms.d;
}
 
int c = 0;
float PIDCompute(PID* pid, float reference, float measured) {
	float error = getError(reference, measured);
	c++;

	if(c > 4000) {
		resetDir(error);

		if(c == 4050) {
			c = 0;
		}
	}

	setDirection(error);

	error = fabs(error);
	errorPos = error;

	_updateTerms(pid, error);
<<<<<<< Updated upstream:test/clutch_stm_bridge/Core/Src/mmr_pid.c
=======

	/*if(fabs(error) < 0.2f) {
		return 0.5f;
	}*/

>>>>>>> Stashed changes:clutch-432/Core/Src/mmr_pid.c
	const float outputPresaturation = getOutput(pid);
	const float output = getOutputInSaturationRange(pid, outputPresaturation);

	pid->_lastError = error;
	pid->_lastOutputs.outputPresaturation = outputPresaturation;
	pid->_lastOutputs.output = output;

	return output / MAGIC_K;
}

float PIDCascade(PID* pid1, PID* pid2, float reference, float measured1, float measured2) {
	const float pid1Result = PIDCompute(pid1, reference, measured1);
	//currentTarget = pid1Result;
	return PIDCompute(pid2, pid1Result, measured2);
}

PID PIDInit(PIDSaturation saturation, PIDParameters parameters, float sampleTime, float tau) {
	PID pid = {
		saturation: saturation,
		parameters: parameters,

		.sampleTime = sampleTime,
		.tau = tau,

		_terms: {
			p: 0,
			i: 0,
			d: 0,
		},
		_lastOutputs: {
			output: 0,
			outputPresaturation: 0,
		},
		_lastError: 0,
	};

	return pid;
}


void setDirection(float error){
	if(error > 0)
		setClockwise();
	else
		setCounterClockwise();
}

void setClockwise() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //DIR A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //DIR B
}

void setCounterClockwise() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //DIR A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); //DIR B
}

void resetDir() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //DIR A
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); //DIR B
}
