#include "inc/led.h"


void MMR_LED_Set(MmrLed *led, MmrLedState state) {
  MmrPinState out = state == MMR_LED_ON
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;

  MMR_PIN_Write(led->pin, out);
}

MmrLedState MMR_LED_Get(MmrLed *led) {
  return MMR_PIN_Read(led->pin) == MMR_PIN_HIGH
    ? MMR_LED_ON
    : MMR_LED_OFF;
}


void MMR_LED_Toggle(MmrLed *led) {
  MmrLedState out = MMR_LED_IsOn(led)
    ? MMR_LED_OFF
    : MMR_LED_ON;

  MMR_LED_Set(led, out);
}

bool MMR_LED_IsOn(MmrLed *led) {
  return MMR_LED_Get(led) == MMR_LED_ON;
}

bool MMR_LED_Blink(MmrLed *led, MmrDelay *delay) {
  if (MMR_DELAY_WaitAsync(delay)) {
    MMR_LED_Toggle(led);
  }
}
