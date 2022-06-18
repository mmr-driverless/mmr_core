#include "inc/lowpass.h"

static void updateLastOutput(MmrLowPass *obj, uint32_t output);
static float computeExpPortion(MmrLowPass *obj);

struct MmrLowPass {
  float cutoff_freq;
  float time_period;
  uint32_t last_output;
};

MmrLowPass MMR_LowPass(float cutoff_freq, float time_period) {
  return (MmrLowPass) {
    .cutoff_freq = cutoff_freq,
    .time_period = time_period,
    .last_output = 0U
  };
}

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoff_freq) {
  obj->cutoff_freq = cutoff_freq;
}

void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float time_period) {
  obj->time_period = time_period;
}

uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input) {
  // TODO: implement, check diffs between lowpass16 and lowpass32
  uint32_t output =
    obj->last_output +
    computeExpPortion(obj) * (input - obj->last_output);
  updateLastOutput(obj, output);
  return output;
}

static void updateLastOutput(MmrLowPass *obj, uint32_t output) {
  obj->last_output = output;
}

static float computeExpPortion(MmrLowPass *obj) {
  // TODO
  return 0;
}

