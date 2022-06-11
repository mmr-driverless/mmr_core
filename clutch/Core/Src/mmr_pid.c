#include "mmr_pid.h"

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
	if (output >= pid->saturation.max)
		return pid->saturation.max;

	if (output <= pid->saturation.min)
		return pid->saturation.min;

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
	#ifdef INTEGRAL_ANTI_WINDUP
		//Integral term + Anti-windup clamping
		return _isLastOutputSaturated(pid)
			? 0.0f
			: pid->_terms.i +
				pid->parameters.i *
				0.5f *
				pid->sampleTime *
				(error + pid->_lastError);
	#else
		return pid->_terms.i +
			pid->parameters.i * 
			0.5f * 
			pid->sampleTime * 
			(error + pid->_lastError);
	#endif
}

float getDerivativeTerm(PID* pid, float error) {
	return 
		(2.0f * pid->parameters.d * (error - pid->_lastError)) + 
		((2.0f * pid->tau - pid->sampleTime) / (2.0f * pid->tau + pid->sampleTime)) *
		pid->_terms.d;
}
 
float PIDCompute(PID* pid, float reference, float measured) {
	float error = getError(reference, measured);
	
	_updateTerms(pid, getError(reference, measured));
	const float outputPresaturation = getOutput(pid);
	const float output = getOutputInSaturationRange(pid, outputPresaturation);

	pid->_lastError = error;
	pid->_lastOutputs.outputPresaturation = outputPresaturation;
	pid->_lastOutputs.output = output;

	return output;
}

float PIDCascade(PID* pid1, PID* pid2, float reference, float measured1, float measured2) {
	const float pid1Result = PIDCompute(pid1, reference, measured1);
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