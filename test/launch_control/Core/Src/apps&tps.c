/*
 * APPS.c
 *
 *  Created on: Jun 9, 2022
 *      Author: simon
 */
#include <apps&tps.h>
#include "main.h"
#include "stm32f3xx_it.h"

#include <stdbool.h>





_Bool APPS_check(uint32_t APPS1, uint32_t APPS2)
{
	if (APPS1 <= APPS1_TH_LOW ||APPS1 >= APPS1_TH_HIGH) {
		if( (APPS1 <= (1 - THRESHOLD)*APPS1_TH_LOW) || (APPS1 >= (1 + THRESHOLD)*APPS1_TH_HIGH) ){

			return true;
		}
	}

	if (APPS2 <= APPS2_TH_LOW ||APPS2 >= APPS2_TH_HIGH) {
		if( (APPS2 <= (1 - THRESHOLD)*APPS2_TH_LOW) || (APPS2 >= (1 + THRESHOLD)*APPS2_TH_HIGH) ) {

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


_Bool TPS_check(uint32_t TPS1, uint32_t TPS2)
{
	return 1; // modficiare
}
