#include "inc/ebs.h"
#include "inc/peripherals.h"


void MMR_EBS_Arm() {
  MMR_PIN_Write(asp.ebsAsCloseSdc, MMR_PIN_HIGH);
}

void MMR_EBS_Disarm() {
  MMR_PIN_Write(asp.ebsAsCloseSdc, MMR_PIN_LOW);
}

void MMR_EBS_SetDrivingMode(MmrEbsDrivingMode mode) {
  MmrPinState out = mode == MMR_EBS_DRIVING_MODE_MANUAL
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;

  MMR_PIN_Write(asp.ebsAsDrivingMode, out);
}

bool MMR_EBS_IsReady() {
  return MMR_PIN_Read(asp.ebsAsSdcIsReady) == MMR_PIN_HIGH;
}
