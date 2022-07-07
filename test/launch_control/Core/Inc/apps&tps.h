/*
 * APPS.h
 *
 *  Created on: Jun 9, 2022
 *      Author: simon
 */

#ifndef INC_APPS_TPS_H_
#define INC_APPS_TPS_H_

#include "main.h"
#include <stdbool.h>


#define APPS1_TH_LOW 1000  // low threshold for APPS1
#define APPS1_TH_HIGH 4000 // upper threshold for APPS2
#define APPS2_TH_LOW 500  // low threshold for APPS1
#define APPS2_TH_HIGH 2000 // upper threshold for APPS2
#define THRESHOLD 0.50f



_Bool APPS_check(uint32_t APPS1, uint32_t APPS2);
_Bool TPS_check(uint32_t TPS1, uint32_t TPS2);
//_Bool APPS_Handling(uint32_t *ADC_value);

//typedef struct APPS_Struct{
//	uint32_t APPS1;
//	uint32_t APPS2;
//	bool Flag1;
//	bool Flag2;
//
//}APPS_Struct;
//


#endif /* INC_APPS_TPS_H_ */
