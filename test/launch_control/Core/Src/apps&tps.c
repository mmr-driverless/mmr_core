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
#include "math.h"




_Bool APPS_check(uint32_t APPS1, uint32_t APPS2)
{
	if(fabs((APPS1 - 2*APPS2)) <= THRESHOLD) return false;
	else return true;

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
	 uint32_t tps_buff = TPS1 + TPS2;
	if (fabs(tps_buff - 5) <= THRESHOLD) return false;
	else return true;
}
