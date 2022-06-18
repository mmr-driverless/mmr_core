#include "inc/lowpass.h"
#include <math.h>

static void updateLastOutput(MmrLowPass *obj, uint32_t output);
static float computeExpPortion(MmrLowPass *obj);

struct MmrLowPass {
  float cutoffFreq;
  float timePeriod;
  MmrFilterBits bitNumber;
  union {
    uint16_t lastOutput16;
    uint32_t lastOutput32;
  };
};

MmrLowPass MMR_LowPass(float cutoffFreq, float timePeriod, MmrFilterBits bitNumber) {
  return (MmrLowPass) {
    .cutoffFreq = cutoffFreq,
    .timePeriod = timePeriod,
    .bitNumber = bitNumber,
    .lastOutput32 = 0U
  };
}

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoffFreq) {
  obj->cutoffFreq = cutoffFreq;
}

void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float timePeriod) {
  obj->timePeriod = timePeriod;
}

uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input) {
  uint32_t output =
    (float) obj->lastOutput32 +
    computeExpPortion(obj) * (float) (input - obj->lastOutput32);

  updateLastOutput(obj, output);
  return output;
}

static void updateLastOutput(MmrLowPass *obj, uint32_t output) {
  if (obj->bitNumber == MMR_FILTER_16)
    obj->lastOutput16 = (uint16_t) output;
  if (obj->bitNumber == MMR_FILTER_32)
    obj->lastOutput32 = output;
}

static float computeExpPortion(MmrLowPass *obj) {
  return (1.0 - exp(-(obj->timePeriod) * obj->cutoffFreq));
}