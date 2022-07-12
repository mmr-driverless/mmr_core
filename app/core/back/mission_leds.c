#include "inc/mission_leds.h"
#include "inc/peripherals.h"
#include <delay.h>
#include <led.h>


void MMR_MISSION_LEDS_Run(MmrMission mission) {
  static MmrDelay blinkDelay = { .ms = 500 };

  if (mission == MMR_MISSION_FINISHED) {
    MMR_LED_BlinkAsync(asp.ctrLed1, &blinkDelay);
    MMR_LED_BlinkAsync(asp.ctrLed2, &blinkDelay);
    MMR_LED_BlinkAsync(asp.ctrLed3, &blinkDelay);
    return;
  }

  bool isLed1High = mission & B_(0001) ? MMR_LED_ON : MMR_LED_OFF;
  bool isLed2High = mission & B_(0010) ? MMR_LED_ON : MMR_LED_OFF;
  bool isLed3High = mission & B_(0100) ? MMR_LED_ON : MMR_LED_OFF;

  MMR_LED_Set(asp.ctrLed1, isLed1High);
  MMR_LED_Set(asp.ctrLed2, isLed2High);
  MMR_LED_Set(asp.ctrLed3, isLed3High);
}
