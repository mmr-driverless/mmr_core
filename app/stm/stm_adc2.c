#include "inc/stm_adc.h"

#include <memory.h>

MmrAdc MMR_Adc(size_t channels, MmrAdcResolution res, MmrAdcReadingSize size) {
  MmrAdc adc = {.resolution = res, .readingSize = size, .bufferLength = channels};
  switch (size) {
    case MMR_ADC_READING_SIZE_8:
      adc.readings8 = malloc(channels * sizeof(uint8_t));
      break;
    case MMR_ADC_READING_SIZE_16:
      adc.readings16 = malloc(channels * sizeof(uint16_t));
      break;
    case MMR_ADC_READING_SIZE_32:
      adc.readings32 = malloc(channels * sizeof(uint32_t));
      break;
  }
  return adc;
}

bool MMR_ADC_Init(MmrAdc *adc) {
  ADC_HandleTypeDef handle;

  handle.Instance = ADC2;
  handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  handle.Init.Resolution = ADC_RESOLUTION_10B;
  handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  handle.Init.ContinuousConvMode = ENABLE;
  handle.Init.DiscontinuousConvMode = DISABLE;
  handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  handle.Init.NbrOfConversion = 2;
  handle.Init.DMAContinuousRequests = ENABLE;
  handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  handle.Init.LowPowerAutoWait = DISABLE;
  handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&handle) != HAL_OK) {
    return false;
  }

  adc->adcHandle = handle;
  return true;
}

bool MMR_ADC_SelectChannel(MmrAdc *adc, uint32_t channel, uint32_t rank, uint32_t sampleTime) {
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = rank;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&(adc->adcHandle), &sConfig) != HAL_OK) {
    return false;
  }
  
  switch (adc->readingSize) {
    case MMR_ADC_READING_SIZE_8:
      HAL_ADC_Start_DMA(&(adc->adcHandle), (uint16_t*) adc->readings8, adc->bufferLength);
      break;
    case MMR_ADC_READING_SIZE_16:
      HAL_ADC_Start_DMA(&(adc->adcHandle), (uint16_t*) adc->readings16, adc->bufferLength);
      break;
    case MMR_ADC_READING_SIZE_32:
      HAL_ADC_Start_DMA(&(adc->adcHandle), (uint16_t*) adc->readings32, adc->bufferLength);
      break;
  }

  return true;
}