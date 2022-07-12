#include "inc/axis_leds.h"
#include "inc/as.h"
#include "inc/peripherals.h"
#include <pin.h>
#include <delay.h>
#include <led.h>


void MMR_AXIS_LEDS_Run(MmrAsState state) {
  static MmrDelay delay = { .ms = 500 };

  switch (state) {
  case MMR_AS_OFF:
    MMR_LED_Set(asp.blueAxisLed, MMR_LED_OFF);
    MMR_LED_Set(asp.yellowAxisLed, MMR_LED_OFF);
    break;

  case MMR_AS_READY:
    MMR_LED_Set(asp.blueAxisLed, MMR_LED_OFF);
    MMR_LED_Set(asp.yellowAxisLed, MMR_LED_ON);
    break;

  case MMR_AS_DRIVING:
    MMR_LED_Set(asp.blueAxisLed, MMR_LED_OFF);
    MMR_LED_BlinkAsync(asp.yellowAxisLed, &delay);
    break;

  case MMR_AS_EMERGENCY:
    MMR_LED_BlinkAsync(asp.blueAxisLed, &delay);
    MMR_LED_Set(asp.yellowAxisLed, MMR_LED_OFF);
    break;

  case MMR_AS_FINISHED:
    MMR_LED_Set(asp.blueAxisLed, MMR_LED_ON);
    MMR_LED_Set(asp.yellowAxisLed, MMR_LED_OFF);
    break;
  }
}
