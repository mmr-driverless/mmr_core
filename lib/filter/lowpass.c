#include "inc/lowpass.h"

static void updateLastOutput(MmrLowPass *obj, uint32_t output);
static float computeExpPortion(MmrLowPass *obj);

struct MmrLowPass {
  float cutoffFreq;
  float timePeriod;
  uint32_t lastOutput;
};

MmrLowPass MMR_LowPass(float cutoffFreq, float timePeriod) {
  return (MmrLowPass) {
    .cutoffFreq = cutoffFreq,
    .timePeriod = timePeriod,
    .lastOutput = 0U
  };
}

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoffFreq) {
  obj->cutoffFreq = cutoffFreq;
}

void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float timePeriod) {
  obj->timePeriod = timePeriod;
}

uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input) {
  // TODO: implement, check diffs between lowpass16 and lowpass32
  uint32_t output =
    obj->lastOutput +
    computeExpPortion(obj) * (input - obj->lastOutput);
  updateLastOutput(obj, output);
  return output;
}

static void updateLastOutput(MmrLowPass *obj, uint32_t output) {
  obj->lastOutput = output;
}

static float computeExpPortion(MmrLowPass *obj) {
  // TODO
  return 0;
}

