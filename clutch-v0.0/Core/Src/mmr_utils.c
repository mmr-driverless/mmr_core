#include "mmr_utils.h"

float adcAverageValue(
  uint16_t *adcValues, 
  uint8_t bufferLength, 
  uint8_t index, 
  uint8_t step
) {
  float averageValue = 0;

  for(uint8_t i = index; i < bufferLength; i += step) {
    averageValue += adcValues[index];
  }

  return averageValue / (bufferLength / step);
}