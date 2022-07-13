#include "inc/back.h"

MmrAsPeripherals asp;

void MMR_BACK_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,

  MmrButton *blueButton,

  uint32_t *appsOut,
  uint32_t *appsIn,

  BuzzerPlay buzzerPlay,
  BuzzerStop buzzerStop,

  MmrPin *ebs1,
  MmrPin *ebs2,

  MmrLed *blueAxisLed,
  MmrLed *yellowAxisLed,

  MmrLed *ebsErrorLed,
  MmrLed *asmsErrorLed,

  MmrLed *ctrLed1,
  MmrLed *ctrLed2,
  MmrLed *ctrLed3,

  WatchdogStart watchdogStart,
  WatchdogStop watchdogStop,

  MmrPin *asms
) {
  asp = (MmrAsPeripherals) {
    .can = can,
    .gearN = gearN,
    .gearUp = gearUp,
    .gearDown = gearDown,

    .blueButton = blueButton,

    .appsOut = appsOut,
    .appsIn = appsIn,

    .buzzerPlay = buzzerPlay,
    .buzzerStop = buzzerStop,

    .ebs1 = ebs1,
    .ebs2 = ebs2,

    .blueAxisLed = blueAxisLed,
    .yellowAxisLed = yellowAxisLed,

    .ebsErrorLed = ebsErrorLed,
    .asmsErrorLed = asmsErrorLed,

    .ctrLed1 = ctrLed1,
    .ctrLed2 = ctrLed2,
    .ctrLed3 = ctrLed3,

    .watchdogStart = watchdogStart,
    .watchdogStop = watchdogStop,

    .asms = asms,
  };
}
