/*
 * APPS.c
 *
 *  Created on: Jun 9, 2022
 *      Author: simon
 */
#include "apps.h"
#include "main.h"
#include "stm32f3xx_it.h"

#include <stdbool.h>



extern DMA_HandleTypeDef hdma_dac_ch1;
extern DAC_HandleTypeDef hdac;

extern uint32_t ADC_value[2];

_Bool APPS_check(uint32_t*ADC_value)
{
	if (ADC_value[0] <= APPS1_TH_LOW || ADC_value[0] >= APPS1_TH_HIGH) {
		if( (ADC_value[0] <= (1 - THRESHOLD)*APPS1_TH_LOW) || (ADC_value[0] >= (1 + THRESHOLD)*APPS1_TH_HIGH) ){

			return true;
		}
	}

	if (ADC_value[1] <= APPS2_TH_LOW || ADC_value[1] >= APPS2_TH_HIGH) {
		if( (ADC_value[1] <= (1 - THRESHOLD)*APPS2_TH_LOW) || (ADC_value[1] >= (1 + THRESHOLD)*APPS2_TH_HIGH) ) {

			return true;
		}
	}

	return false;

}

//_Bool APPS_Handling(uint32_t APPS1, uint32_t APPS2)
//{
////	apps_FLAG = APPS_check(APPS1,APPS2);
//
//	if( APPS_Flag2==true)
//	{
//		   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	  // da inserire eventuale messaggio can da inviare tipo al res per dirgli "oh zi accendì sto cazz di SDC"
////	   HAL_DMA_Abort(&hdma_dac_ch1);  // here i stop the dac dma so no more signal will go out from dac of stm
//
//	   return true;
//	}
//	else{
//
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//
//	 return false;
//	}
//}
