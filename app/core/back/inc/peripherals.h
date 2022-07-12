#ifndef APP_CORE_BACK_INC_PERIPHERALS_H_
#define APP_CORE_BACK_INC_PERIPHERALS_H_

#include <can.h>
#include <led.h>
#include <pin.h>
#include <button.h>
#include <delay.h>
#include <stdint.h>

typedef bool (*WatchdogStart)(void);
typedef bool (*WatchdogStop)(void);

typedef struct MmrAsPeripherals {
  MmrCan *can;

  MmrPin *gearN;
  MmrPin *gearUp;
  MmrPin *gearDown;

  MmrButton *blueButton;

  uint32_t *appsOut;
  uint32_t *appsIn;

  MmrPin *ebsAsDrivingMode;
  MmrPin *ebsAsCloseSdc;
  MmrPin *ebsAsSdcIsReady;

  MmrPin *ebs1;
  MmrPin *ebs2;

  MmrLed *blueAxisLed;
  MmrLed *yellowAxisLed;

  MmrLed *ctrLed1;
  MmrLed *ctrLed2;
  MmrLed *ctrLed3;

  WatchdogStart watchdogStart;
  WatchdogStop watchdogStop;
} MmrAsPeripherals;

extern MmrAsPeripherals asp;

#endif // !APP_CORE_BACK_INC_PERIPHERALS_H_
