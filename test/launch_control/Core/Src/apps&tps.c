/*
 * APPS.c
 *
 *  Created on: Jun 9, 2022
 *      Author: simon
 */
#include <apps&tps.h>
#include <global_state.h>
#include <stdbool.h>
#include <stdint.h>
#include "math.h"




_Bool APPS_check(uint32_t APPS1, uint32_t APPS2)
{
	double appsBuff= 3.3f*(APPS2/4096)-2*3.3f*APPS1;
	if(fabs(appsBuff) <= 0.5f) return false;
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


_Bool TPS_check()
{
	uint32_t difference = gs.uThrottleA + gs.uThrottleB;
	return fabs(difference - 5) <= 0.5;
}
