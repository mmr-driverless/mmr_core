#include "inc/mission_leds.h"
#include "inc/peripherals.h"
#include <delay.h>


void MMR_MISSION_LEDS_Run(MmrMission mission) {
  static MmrDelay blinkDelay = { .ms = 500 };

  if (mission == MMR_MISSION_FINISHED) {

  }

  bool isLed1High = mission & B_(0001);
  bool isLed2High = mission & B_(0010);
  bool isLed3High = mission & B_(0100);

  MMR_PIN_Write(asp.ctrLed1, isLed1High);
  MMR_PIN_Write(asp.ctrLed2, isLed2High);
  MMR_PIN_Write(asp.ctrLed3, isLed3High);
}
