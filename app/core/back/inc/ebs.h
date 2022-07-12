#ifndef APP_CORE_BACK_INC_EBS_H_
#define APP_CORE_BACK_INC_EBS_H_

#include <stdbool.h>


typedef enum MmrEbsCheck {
  EBS_IDLE,
  EBS_SDC_IS_READY,
  EBS_SDC_IS_NOT_READY,
  EBS_CHECK_NOT_ENDED,
  EBS_PRESSURE_CHECK,
  EBS_TS_CHECK,
  EBS1_CONTROL,
  EBS2_CONTROL,
  EBS_ERROR,
  EBS_FINAL_CHECK,
  EBS_OK,
} MmrEbsCheck;

typedef enum MmrEbsState {
  EBS_STATE_UNAVAILABLE,
  EBS_STATE_ARMED,
  EBS_STATE_ACTIVATED,
  EBS_STATE_DISACTIVATED,
} MmrEbsState;


typedef enum MmrEbsDrivingMode {
  MMR_EBS_DRIVING_MODE_MANUAL,
  MMR_EBS_DRIVING_MODE_AUTONOMOUS,
} MmrEbsDrivingMode;

void MMR_EBS_Arm();
void MMR_EBS_Disarm();
void MMR_EBS_SetDrivingMode(MmrEbsDrivingMode mode);
bool MMR_EBS_IsReady();

#endif // !APP_CORE_BACK_INC_EBS_H_
