#include "inc/axis_leds.h"
#include "inc/as.h"
#include "inc/peripherals.h"
#include <pin.h>
#include <delay.h>

static void toggleBlueLed(MmrAxisLedState state);
static void toggleYellowLed(MmrAxisLedState state);


void MMR_AXIS_LEDS_Run(MmrAsState state) {
  static MmrDelay delay = { .ms = 20 };

  switch (state) {
  case MMR_AS_OFF:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_OFF);
    break;

  case MMR_AS_READY:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_ON);
    break;

  case MMR_AS_DRIVING:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_ON);
    
    if (MMR_DELAY_WaitAsync(&delay)) {
      toggleYellowLed(MMR_AXIS_LED_OFF);
    }
    
    break;

  case MMR_AS_EMERGENCY:
    toggleBlueLed(MMR_AXIS_LED_ON);
    toggleYellowLed(MMR_AXIS_LED_OFF);

    if(MMR_DELAY_WaitAsync(&delay)) {
      toggleBlueLed(MMR_AXIS_LED_ON);
    }

    break;

  case MMR_AS_FINISHED:
    toggleBlueLed(MMR_AXIS_LED_ON);
    toggleYellowLed(MMR_AXIS_LED_OFF);
    break;
  }
}


void toggleBlueLed(MmrAxisLedState state) {
  MmrPinState out = state == MMR_AXIS_LED_ON
    ? MMR_PIN_LOW
    : MMR_PIN_HIGH;

  MMR_PIN_Write(asp.blueLed, out);
}


void toggleYellowLed(MmrAxisLedState state) {
  MmrPinState out = state == MMR_AXIS_LED_ON
    ? MMR_PIN_LOW
    : MMR_PIN_HIGH;

  MMR_PIN_Write(asp.yellowLed, out);
}
