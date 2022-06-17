#ifndef LIB_FILTER_INC_LOWPASS_H_
#define LIB_FILTER_INC_LOWPASS_H_

#include <stdint.h>

typedef struct MmrLowPass MmrLowPass;

/**
 * @brief build a new low-pass filter object
 * 
 * @param cutoff_freq rad/s
 * @param time_period time period
 * @return MmrLowPass freshly built object
 */
MmrLowPass MMR_LowPass(float cutoff_freq, float time_period);

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoff_freq);
void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float time_period);

/**
 * @brief filter input signal and return filtered value
 * 
 * @param obj MmrLowPass object reference
 * @param input input signal
 * @return uint32_t filtered signal
 */
uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input);

#endif  // !LIB_FILTER_INC_LOWPASS_H_