#ifndef APP_LIB_ADC_INC_ADC_H_
#define APP_LIB_ADC_INC_ADC_H_

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct MmrAdc MmrAdc;

bool MMR_ADC_Init(MmrAdc *adc);
bool MMR_ADC_SelectChannel(MmrAdc *adc, uint32_t channel, uint32_t rank, uint32_t sampleTime);

uint8_t* MMR_ADC_Read8(MmrAdc *adc);
uint16_t* MMR_ADC_Read16(MmrAdc *adc);
uint32_t* MMR_ADC_Read32(MmrAdc *adc);

#endif // !APP_LIB_ADC_INC_ADC_H_
