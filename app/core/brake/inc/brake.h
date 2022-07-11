#ifndef MMR_APP_BRAKE_INC_BRAKE_H_
#define MMR_APP_BRAKE_INC_BRAKE_H_

#include "lib/can/inc/can.h"


void MMR_BRAKE_Init(Can *can);
void MMR_BRAKE_Start();
bool MMR_BRAKE_Check();
bool MMR_BRAKE_IsEngaged();

#ifndef
