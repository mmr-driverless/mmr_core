#ifndef LIB_TIMING_INC_DELAY_H_
#define LIB_TIMING_INC_DELAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <timing.h>

typedef struct MmrDelay {
  uint32_t ms;
  Tick start;
} MmrDelay;

void MMR_DELAY_Change(MmrDelay *delay, uint32_t delayMs);
bool MMR_DELAY_WaitAsync(MmrDelay *delay);

#endif // !LIB_TIMING_INC_DELAY_H_
