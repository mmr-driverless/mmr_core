#ifndef APP_LIB_LED_LED_H_
#define APP_LIB_LED_LED_H_

#include <pin.h>

typedef enum MmrLedState {

} MmrLedState;

typedef struct MmrLed {
  MmrPin *pin;
  MmrLedState state;
} MmrLed;


void MMR_LED_On(MmrLed *led);
void MMR_LED_Off(MmrLed *led);
void MMR_LED_Toggle(MmrLed *led);
void MMR_LED_BlinkAsync(MmrLed *led);

#endif // !APP_LIB_LED_LED_H_
