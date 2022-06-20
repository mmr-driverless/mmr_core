#ifndef APP_LIB_FILTER_INC_LOWPASS_H_
#define APP_LIB_FILTER_INC_LOWPASS_H_

#include <stdint.h>
#include "filter.h"

typedef struct MmrLowPass {
  float cutoffFreq;
  float timePeriod;
  MmrFilterBits bitNumber;
  uint32_t lastOutput;
  float expCache;
} MmrLowPass;

/**
 * @brief build a new low-pass filter object
 * 
 * @param cutoffFreq [rad/s]
 * @param timePeriod [s]
 * @return MmrLowPass freshly built object
 */
MmrLowPass MMR_LowPass(float cutoffFreq, float timePeriod, MmrFilterBits bitNumber);

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoffFreq);
void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float time_period);

/**
 * @brief filter input signal and return filtered value
 * 
 * @param obj MmrLowPass object reference
 * @param input input signal
 * @return uint32_t filtered signal
 */
uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input);

#endif // !APP_LIB_FILTER_INC_LOWPASS_H_
