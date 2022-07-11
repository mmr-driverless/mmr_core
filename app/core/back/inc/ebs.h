#ifndef APP_CORE_BACK_INC_EBS_H_
#define APP_CORE_BACK_INC_EBS_H_

#include <stdbool.h>

typedef enum MmrEbsDrivingMode {
  MMR_EBS_DRIVING_MODE_MANUAL,
  MMR_EBS_DRIVING_MODE_AUTONOMOUS,
} MmrEbsDrivingMode;

void MMR_EBS_Arm();
void MMR_EBS_Disarm();
void MMR_EBS_SetDrivingMode(MmrEbsDrivingMode mode);
bool MMR_EBS_IsReady();

#endif // !APP_CORE_BACK_INC_EBS_H_
