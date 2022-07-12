#ifndef APP_LIB_GPIO_INC_LED_H_
#define APP_LIB_GPIO_INC_LED_H_

#include "delay.h"
#include "pin.h"

typedef enum MmrLedState {
  MMR_LED_ON,
  MMR_LED_OFF,
} MmrLedState;

typedef struct MmrLed {
  MmrPin *pin;
} MmrLed;


MmrLed MMR_Led(MmrPin *pin);

void MMR_LED_Set(MmrLed *led, MmrLedState state);
void MMR_LED_Toggle(MmrLed *led);

MmrLedState MMR_LED_Get(MmrLed *led);
bool MMR_LED_IsOn(MmrLed *led);
void MMR_LED_BlinkAsync(MmrLed *led, MmrDelay *delay);

#endif // !APP_LIB_GPIO_INC_LED_H_
