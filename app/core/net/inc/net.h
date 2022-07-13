#ifndef APP_CORE_NET_INC_NET_H_
#define APP_CORE_NET_INC_NET_H_

#include <stdbool.h>
#include <can.h>
#include <button.h>

void MMR_NET_BRAKE_Init(MmrCan *can);
bool MMR_NET_EngageBrakeAsync();

bool MMR_NET_ClutchPullAsync(MmrCan *can);
bool MMR_NET_ClutchReleaseAsync(MmrCan *can);

bool MMR_NET_LaunchControlSetAsync(MmrCan *can);
bool MMR_NET_LaunchControlUnsetAsync(MmrCan *can);

void MMR_NET_ResParse(uint8_t resPayload, MmrButtonState *emergencyButtonState, MmrButtonState *goButtonState, MmrButtonState *bagButtonState);

#endif  // !APP_CORE_NET_INC_NET_H_
