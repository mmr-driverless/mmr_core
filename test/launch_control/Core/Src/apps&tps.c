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




bool APPS_check(uint32_t APPS1, uint32_t APPS2) {
	double appsBuff= 3.3f*(APPS2/4096)-2*3.3f*APPS1;
	if(fabs(appsBuff) <= 0.5f) return false;
	else return true;

}

bool TPS_check() {
	return fabs(gs.uThrottleA + gs.uThrottleB - 5) <= 0.5;
}
