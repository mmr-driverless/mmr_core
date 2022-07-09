#include "inc/axis_leds.h"
#include "inc/as.h"
#include <pin.h>
#include <delay.h>

static MmrPin *__AssiBlue;
static MmrPin *__AssiYellow;
static MmrDelay *__assi_delay;

static void toggleBlueLed(MmrAxisLedState state);
static void toggleYellowLed(MmrAxisLedState state);


void MMR_AXIS_LEDS_Init(MmrPin *AssiBlue, MmrPin *AssiYellow, MmrDelay *assi_delay) {
  __AssiBlue = AssiBlue;
  __AssiYellow = AssiYellow;
  __assi_delay = assi_delay;

  *__assi_delay = MMR_Delay(20);
}

void MMR_AXIS_LEDS_Run(MmrAsState state) {
  switch (state) {
  case AS_OFF:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_OFF);
    break;

  case AS_READY:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_ON);
    break;

  case AS_DRIVING:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_ON);
    MMR_DELAY_Reset(__assi_delay);
    
    if (MMR_DELAY_WaitAsync(__assi_delay)) {
      toggleYellowLed(MMR_AXIS_LED_OFF);
    }
    if (MMR_DELAY_WaitAsync(__assi_delay)) {
      break;
    }

  case AS_EMERGENCY:
    toggleBlueLed(MMR_AXIS_LED_ON);
    toggleYellowLed(MMR_AXIS_LED_OFF);
    MMR_DELAY_Reset(__assi_delay);

    if(MMR_DELAY_WaitAsync(__assi_delay)) {
      toggleBlueLed(MMR_AXIS_LED_ON);
    }

    if(MMR_DELAY_WaitAsync(__assi_delay)) {
      break;
    }

  case AS_FINISHED:
    toggleBlueLed(MMR_AXIS_LED_ON);
    toggleYellowLed(MMR_AXIS_LED_OFF);

  case AS_IDLE:
    toggleBlueLed(MMR_AXIS_LED_OFF);
    toggleYellowLed(MMR_AXIS_LED_OFF);
    break;
  }
}


void toggleBlueLed(MmrAxisLedState state) {
  MmrPinState out = state == MMR_AXIS_LED_ON
    ? MMR_PIN_LOW
    : MMR_PIN_HIGH;

  MMR_PIN_Write(__AssiBlue, out);
}


void toggleYellowLed(MmrAxisLedState state) {
  MmrPinState out = state == MMR_AXIS_LED_ON
    ? MMR_PIN_LOW
    : MMR_PIN_HIGH;

  MMR_PIN_Write(__AssiYellow, out);
}
