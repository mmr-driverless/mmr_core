#ifndef APP_CORE_BACK_INC_MANUAL_H_
#define APP_CORE_BACK_INC_MANUAL_H_

typedef enum {
  MMR_MANUAL_INIT,
  MMR_MANUAL_RUNNING,
} MmrManualState;

MmrManualState MMR_MANUAL_Run(MmrManualState state);

#endif
