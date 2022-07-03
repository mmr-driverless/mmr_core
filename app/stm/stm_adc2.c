#include "inc/stm_adc.h"

#include <memory.h>

MmrAdc MMR_Adc(size_t channels, MmrAdcResolution res) {
  MmrAdc adc = {};
  return adc;
}

bool MMR_ADC_Init(MmrAdc *adc) {
  adc->handle.Instance = ADC2;
  adc->handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  adc->handle.Init.Resolution = ADC_RESOLUTION_10B;
  adc->handle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  adc->handle.Init.ContinuousConvMode = ENABLE;
  adc->handle.Init.DiscontinuousConvMode = DISABLE;
  adc->handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc->handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adc->handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adc->handle.Init.NbrOfConversion = 2;
  adc->handle.Init.DMAContinuousRequests = ENABLE;
  adc->handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adc->handle.Init.LowPowerAutoWait = DISABLE;
  adc->handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&adc->handle) != HAL_OK) {
    Error_Handler();
  }
}

bool MMR_ADC_SelectChannel(MmrAdc *adc, uint32_t channel, uint32_t rank, uint32_t sampleTime) {
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  
  HAL_ADC_Start_DMA(&(adc->handle), (uint16_t*) adc->readings, adc->size);
}