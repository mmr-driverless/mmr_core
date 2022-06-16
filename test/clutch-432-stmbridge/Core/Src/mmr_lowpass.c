#include "mmr_lowpass.h"

// Input must be the latest ADC reading.
float lowpassFilter(float input, LowpassData *lpData){
	lpData->input = input;
	float deltaT = 1.0f / 16000.0f;
	float tau = 1.0f / (2.0f * 3.14f * lpData->cutoffFrequency);
	/*lpData->output +=
			(lpData->input - lpData->output) *
			(1-exp(deltaT * lpData->cutoffFrequency));*/


	lpData->output = lpData->input * (deltaT / (tau + deltaT)) + lpData->output * (tau / (tau + deltaT));

	return lpData->output;
}
