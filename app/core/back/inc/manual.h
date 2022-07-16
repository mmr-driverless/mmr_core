#ifndef APP_CORE_BACK_INC_MANUAL_H_
#define APP_CORE_BACK_INC_MANUAL_H_

typedef enum {
  MMR_MANUAL_ENTRY_POINT,
  MMR_MANUAL_WAIT_TS,
  MMR_MANUAL_WAIT_SDC,
  MMR_MANUAL_EBS_OFF,
  MMR_MANUAL_RUNNING,
} MmrManualState;

MmrManualState MMR_MANUAL_Run(MmrManualState state);

#endif

