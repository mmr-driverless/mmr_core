#ifndef LIB_TIMING_INC_DELAY_H_
#define LIB_TIMING_INC_DELAY_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct MmrDelay MmrDelay;

MmrDelay MMR_Delay(uint32_t delayMs);
void MMR_DELAY_Change(MmrDelay *delay, uint32_t delayMs);
bool MMR_DELAY_WaitAsync(MmrDelay *delay);

#endif // !LIB_TIMING_INC_DELAY_H_
