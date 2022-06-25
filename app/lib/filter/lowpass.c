#include "inc/lowpass.h"
#include <math.h>

static uint32_t getLastOutput(MmrLowPass *obj);
static void updateLastOutput(MmrLowPass *obj, uint32_t output);
static void updateExpCache(MmrLowPass *obj);
static float computeExpPortion(MmrLowPass *obj);

struct MmrLowPass {
  float cutoffFreq;
  float timePeriod;
  MmrFilterBits bitNumber;
  union {
    uint16_t lastOutput16;
    uint32_t lastOutput32;
  };
  float expCache;
};

MmrLowPass MMR_LowPass(float cutoffFreq, float timePeriod, MmrFilterBits bitNumber) {
  return (MmrLowPass) {
    .cutoffFreq = cutoffFreq,
    .timePeriod = timePeriod,
    .bitNumber = bitNumber,
    .lastOutput32 = 0U,
    .expCache = exp(-timePeriod * cutoffFreq)
  };
}

void MMR_LOWPASS_SetCutoffFreq(MmrLowPass *obj, float cutoffFreq) {
  obj->cutoffFreq = cutoffFreq;
  updateExpCache(obj);
}

void MMR_LOWPASS_SetTimePeriod(MmrLowPass *obj, float timePeriod) {
  obj->timePeriod = timePeriod;
  updateExpCache(obj);
}

uint32_t MMR_LOWPASS_Filter(MmrLowPass *obj, uint32_t input) {
  uint32_t output =
    (float) getLastOutput(obj) +
    computeExpPortion(obj) * (float) (input - obj->lastOutput32);

  updateLastOutput(obj, output);
  return output;
}

static uint32_t getLastOutput(MmrLowPass *obj) {
  if (obj->bitNumber == MMR_FILTER_16)
    return obj->lastOutput16;
  else  // MMR_FILTER_32
    return obj->lastOutput32;
}

static void updateLastOutput(MmrLowPass *obj, uint32_t output) {
  if (obj->bitNumber == MMR_FILTER_16)
    obj->lastOutput16 = (uint16_t) output;
  else  // MMR_FILTER_32
    obj->lastOutput32 = output;
}

static void updateExpCache(MmrLowPass *obj) {
  obj->expCache = exp(-(obj->timePeriod) * obj->cutoffFreq);
}

static float computeExpPortion(MmrLowPass *obj) {
  return (1.0F - obj->expCache);
}