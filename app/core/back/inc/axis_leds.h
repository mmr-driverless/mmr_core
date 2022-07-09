#ifndef APP_CORE_BACK_AXIS_LEDS_H_
#define APP_CORE_BACK_AXIS_LEDS_H_

#include "as.h"
#include <pin.h>
#include <delay.h>

typedef enum MmrAxisLedState {
	MMR_AXIS_LED_OFF,
	MMR_AXIS_LED_ON,
} MmrAxisLedState;


void MMR_AXIS_LEDS_Init(MmrPin *AssiBlue, MmrPin *AssiYellow, MmrDelay *assi_delay);
void MMR_AXIS_LEDS_Run(MmrAsState state);

#endif // !APP_CORE_BACK_AXIS_LEDS_H_
