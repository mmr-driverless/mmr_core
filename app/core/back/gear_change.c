#include "inc/gear_change.h"
#include "inc/as.h"
#include <pin.h>
#include <delay.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

const int N_UP_MIN = 4500;   // Limite minimo di cambiata Up
const int N_UP_MAX = 10000;  // Limite massimo di cambiata Up
const int N_DN_MIN = 3500;   // Limite minimo di cambiata Down
const int N_DN_MAX = 7000;   // Limite massimo di cambiata Down


static MmrPin *__gearUp;
static MmrPin *__gearDn;


void MMR_GEAR_CHANGE_Init(MmrPin *gearUp, MmrPin *gearDn) {
  __gearUp = gearUp;
  __gearDn = gearDn;
}


void MMR_GEAR_CHANGE_Run() {
  static MmrDelay gearChangeDelay = { .ms = 50 };
  static int previousSpeed = 0;

  static bool enableGearUp = false;
  static bool enableGearDn = false;

  // TODO ask info
  uint8_t lap = MMR_GS_GetLap();
  if (lap <= 1)
    return;

  uint16_t currentSpeed = MMR_GS_GetSpeed();
  int16_t steeringAngle = MMR_GS_GetSteeringAngle();
  uint16_t ath = MMR_GS_GetAth();
  uint16_t gear = MMR_GS_GetGear();
  uint16_t rpm = MMR_GS_GetRpm();

  currentSpeed = round(currentSpeed / 2) * 2;
  int deltaSpeed = currentSpeed - previousSpeed;


  float rpmUp = N_UP_MIN + ((N_UP_MAX - N_UP_MIN) / 100) * (ath / 100);
  float rpmDn = gear != 2
    ? N_DN_MAX - ((N_DN_MAX - N_DN_MIN) / 100) * (ath / 100)
    : N_DN_MIN;


  if (!enableGearUp && !enableGearDn) {
    if (deltaSpeed > 0 && rpm >= rpmUp && gear != 6) { //Sto guardando la Retta Up-Shifting
      enableGearUp = true;
      enableGearDn = false;
      MMR_DELAY_Reset(&gearChangeDelay);
      MMR_PIN_Write(__gearUp, MMR_PIN_HIGH);
    }
    else if (deltaSpeed < 0 && rpm <= rpmDn && gear != 1 && abs(steeringAngle) < 15 / 6.25) {
      enableGearUp = false;
      enableGearDn = true;
      MMR_DELAY_Reset(&gearChangeDelay);
      MMR_PIN_Write(__gearDn, MMR_PIN_HIGH);
    }
  }


  if (enableGearUp) {
    if (MMR_DELAY_WaitAsync(&gearChangeDelay)) {
      MMR_PIN_Write(__gearUp, MMR_PIN_LOW);
      enableGearUp = false;
      enableGearDn = false;
    }
  }

  if(enableGearDn) {
    if (MMR_DELAY_WaitAsync(&gearChangeDelay)) {
      MMR_PIN_Write(__gearDn, MMR_PIN_LOW);
      enableGearUp = false;
      enableGearDn = false;
    }
  }

  previousSpeed = currentSpeed;
}
