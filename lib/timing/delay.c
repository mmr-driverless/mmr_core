#include "inc/delay.h"
#include "inc/timing.h"


struct MmrDelay {
  uint32_t ms;
  Tick start;
};


MmrDelay MMR_Delay(uint32_t ms) {
  return (MmrDelay) {
    .ms = ms,
  };
}


void MMR_DelayChange(MmrDelay *delay, uint32_t delayMs) {
  delay->ms = delayMs;
}


bool MMR_WaitAsync(MmrDelay *delay) {
  Tick tick = MMR_GetTick();
  if (delay->start == 0) {
    delay->start = tick;
  }

  if (tick - delay->start >= delay->ms) {
    delay->start = tick;
    return true;
  }

  return false;
}
