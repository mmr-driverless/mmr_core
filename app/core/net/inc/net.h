#ifndef APP_CORE_NET_INC_NET_H_
#define APP_CORE_NET_INC_NET_H_

#include <stdbool.h>
#include <can.h>

bool MMR_NET_BRAKE_Init(MmrCan *can)
bool MMR_NET_IsBrakeEngagedRequest();
bool MMR_NET_BrakeCheckRequest();

bool MMR_NET_ClutchPullAsync(MmrCan *can);
bool MMR_NET_ClutchReleaseAsync(MmrCan *can);

bool MMR_NET_LaunchControlSetAsync(MmrCan *can);
bool MMR_NET_LaunchControlUnsetAsync(MmrCan *can);

#endif  // !APP_CORE_NET_INC_NET_H_
