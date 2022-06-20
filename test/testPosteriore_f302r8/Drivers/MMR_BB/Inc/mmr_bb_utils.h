#ifndef INC_MMR_BB_UTILS_H_
#define INC_MMR_BB_UTILS_H_

#include "main.h"
#include "mmr_can.h"
#include <stdbool.h>

#define WAIT_PERIOD 1000

void MMR_BB_utilsSetTick();
bool MMR_BB_utilsTimer();
void MMR_BB_createDefaultPacket(MmrCanPacket *packet);

#endif /* INC_MMR_BB_UTILS_H_ */
