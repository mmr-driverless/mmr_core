#include "inc/hwtest.h"
#include "inc/peripherals.h"
#include "inc/global_state.h"
#include <pin.h>

void MMR_BACK_HWTEST_ResGoButton() {
  while (1) {
    if (gs.resGoButton == MMR_BUTTON_PRESSED)
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
    else
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_OFF);
  }
}


void MMR_BACK_HWTEST_MissionReady() {}


void MMR_BACK_HWTEST_AsmsOn() {
  while (1) {
    if (MMR_PIN_Read(asp.asms) == MMR_PIN_HIGH)
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
    else
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_OFF);
  }
}

void MMR_BACK_HWTEST_TsOk() {
  while (1) {
    if (gs.gear == 0 && gs.rpm >= 1000)
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
    else
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_OFF);
  }
}


void MMR_BACK_HWTEST_BrakeEngaged() {
  static const int BRAKE_MIN_PRESSURE = 4;
  static bool isBrakeEngaged;
  while (1) {
    isBrakeEngaged =
      gs.brakePressureFront >= BRAKE_MIN_PRESSURE &&
      gs.brakePressureRear >= BRAKE_MIN_PRESSURE;
    
    if (isBrakeEngaged)
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
    else
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_OFF);
  }
    
}

void MMR_BACK_HWTEST_MissionSelected() {
  static bool isAutonomous;
  while (1) {
    isAutonomous =
      gs.currentMission != MMR_MISSION_IDLE &&
      gs.currentMission != MMR_MISSION_MANUAL;
    
    if (isAutonomous)
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
    else
      MMR_LED_Set(asp.ebsErrorLed, MMR_LED_OFF);
  }
}