/*
 * APPS.c
 *
 *  Created on: Jun 9, 2022
 *      Author: simon
 */
#include "APPS.h"
#include "main.h"
#include "stm32f3xx_it.h"
#include <stdbool.h>



extern DMA_HandleTypeDef hdma_dac_ch1;
extern DAC_HandleTypeDef hdac;

_Bool APPS_check(uint32_t APPS1, uint32_t APPS2)
{
	if (APPS1 <= APPS1_TH_LOW || APPS1 >= APPS1_TH_HIGH) {
		if( (APPS1 <= (1 - THRESHOLD)*APPS1_TH_LOW) || (APPS1 >= (1 + THRESHOLD)*APPS1_TH_HIGH) ){

			return 1;
		}
	}

	if (APPS2 <= APPS2_TH_LOW || APPS2 >= APPS2_TH_HIGH) {
		if( (APPS2 <= (1 - THRESHOLD)*APPS2_TH_LOW) || (APPS2 >= (1 + THRESHOLD)*APPS2_TH_HIGH) ) {

			return 1;
		}
	}

	return 0;

}

_Bool APPS_Handling(uint32_t APPS1, uint32_t APPS2)
{
	_Bool apps_FLAG = 0;

	apps_FLAG = APPS_check(APPS1,APPS2);

	if(apps_FLAG == 1)
	{
	  // da inserire eventuale messaggio can da inviare tipo al res per dirgli "oh zi accendì sto cazz di SDC"
	   HAL_DMA_Abort(&hdma_dac_ch1);  // here i stop the dac dma so no more signal will go out from dac of stm
	   return 1;
	}
	else{


	 return 0;
	}
}
