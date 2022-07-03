#ifndef APP_STM_INC_ADC2_H_
#define APP_STM_INC_ADC2_H_

#include "stm_hal_adc_defs.h"
#include <adc.h>


typedef enum {
  MMR_ADC_RESOLUTION_6B,
  MMR_ADC_RESOLUTION_8B,
  MMR_ADC_RESOLUTION_10B,
  MMR_ADC_RESOLUTION_12B
} MmrAdcResolution;

typedef enum {
  MMR_ADC_READING_SIZE_8,
  MMR_ADC_READING_SIZE_16,
  MMR_ADC_READING_SIZE_32
} MmrAdcReadingSize;

struct MmrAdc {
  union {
    uint8_t *readings8,
    uint16_t *readings16,
    uint32_t *readings32
  },
  size_t bufferLength,
  ADC_HandleTypeDef adcHandle,
  MmrAdcResolution resolution,
  MmrAdcReadingSize readingSize
};

MmrAdc MMR_Adc(size_t channels, MmrAdcResolution res);

#endif  // !APP_STM_INC_ADC2_H_