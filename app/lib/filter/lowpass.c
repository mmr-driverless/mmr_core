#include "inc/lowpass.h"
#include <math.h>

static uint32_t getLastOutput(MmrLowPass *obj);
static void updateLastOutput(MmrLowPass *obj, uint32_t output);
static void updateExpCache(MmrLowPass *obj);
static float computeExpPortion(MmrLowPass *obj);


MmrLowPass MMR_LowPass(float cutoffFreq, float timePeriod, MmrFilterBits bitNumber) {
  return (MmrLowPass) {
    .cutoffFreq = cutoffFreq,
    .timePeriod = timePeriod,
    .bitNumber = bitNumber,
    .lastOutput = 0U,
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
    computeExpPortion(obj) *
    (float) (input - getLastOutput(obj));

  updateLastOutput(obj, output);
  return output;
}

static uint32_t getLastOutput(MmrLowPass *obj) {
  return obj->lastOutput;
}

static void updateLastOutput(MmrLowPass *obj, uint32_t output) {
  obj->lastOutput = obj->bitNumber == MMR_FILTER_16
    ? (uint16_t) output
    : output;
}

static void updateExpCache(MmrLowPass *obj) {
  obj->expCache = exp(-(obj->timePeriod) * obj->cutoffFreq);
}

static float computeExpPortion(MmrLowPass *obj) {
  return (1.0F - obj->expCache);
}