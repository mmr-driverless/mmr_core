#include "stm32l4xx_hal.h"

typedef struct LowpassData {
	uint16_t input;
	uint16_t output;
	float cutoffFrequency;
	float dt;
} LowpassData;


uint16_t lowpassFilter(uint16_t input, LowpassData *lpData);
