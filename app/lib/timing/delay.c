#include "inc/delay.h"
#include "inc/timing.h"


MmrDelay MMR_Delay(uint32_t delayMs) {
  return (MmrDelay) {
    .ms = delayMs,
  };
}


void MMR_DELAY_Reset(MmrDelay *delay) {
  delay->start = MMR_GetTick();
}


void MMR_DELAY_Change(MmrDelay *delay, uint32_t delayMs) {
  delay->ms = delayMs;
}


bool MMR_DELAY_WaitAsync(MmrDelay *delay) {
  Tick tick = MMR_GetTick();
  if (delay->start == 0) {
    delay->start = tick;
  }

  if (tick - delay->start >= delay->ms) {
    MMR_DELAY_Reset(delay);
    return true;
  }

  return false;
}
