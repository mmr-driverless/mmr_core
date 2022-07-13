#ifndef MMR_APP_BRAKE_INC_BRAKE_H_
#define MMR_APP_BRAKE_INC_BRAKE_H_

#include "lib/can/inc/can.h"


void MMR_BRAKE_Init(Can *can);
bool MMR_NET_EngageBrakeAsync();

#ifndef  // !MMR_APP_BRAKE_INC_BRAKE_H_
