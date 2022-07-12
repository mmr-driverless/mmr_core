#ifndef APP_CORE_NET_INC_NET_H_
#define APP_CORE_NET_INC_NET_H_

#include <stdbool.h>
#include <can.h>
#include <button.h>

bool MMR_NET_BRAKE_Init(MmrCan *can);
bool MMR_NET_IsBrakeEngagedRequest();
bool MMR_NET_BrakeCheckRequest();

bool MMR_NET_PullClutchAsync(MmrCan *can);
bool MMR_NET_ReleaseClutchAsync(MmrCan *can);

void MMR_NET_ResParse(uint8_t resPayload, MmrButtonState *emergencyButtonState, MmrButtonState *goButtonState, MmrButtonState *bagButtonState);

#endif  // !APP_CORE_NET_INC_NET_H_
