
#include "stm32l4xx_hal.h"

typedef struct LowpassData {
	float input;
	float output;
	float cutoffFrequency;
} LowpassData;


float lowpassFilter(float input, LowpassData *lpData);
