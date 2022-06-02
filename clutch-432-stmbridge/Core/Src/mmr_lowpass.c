#include "mmr_lowpass.h"

// Input must be the latest ADC reading.
uint16_t lowpassFilter(uint16_t input, LowpassData *lpData){
	lpData->input = input;
	lpData->output +=
			(lpData->input - lpData->output) *
			(1-exp(-lpData->dt * lpData->cutoffFrequency));

	return lpData->output;
}
