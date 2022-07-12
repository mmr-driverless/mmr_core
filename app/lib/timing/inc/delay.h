#ifndef LIB_TIMING_INC_DELAY_H_
#define LIB_TIMING_INC_DELAY_H_

#include <stdint.h>
#include <stdbool.h>
#include "timing.h"

typedef struct MmrDelay {
  uint32_t ms;
  Tick start;
} MmrDelay;

MmrDelay MMR_Delay(uint32_t delayMs);
void MMR_DELAY_Reset(MmrDelay *delay);
void MMR_DELAY_Change(MmrDelay *delay, uint32_t delayMs);
bool MMR_DELAY_WaitAsync(MmrDelay *delay);
bool MMR_DELAY_WaitOnceAsync(MmrDelay *delay);

#endif // !LIB_TIMING_INC_DELAY_H_
