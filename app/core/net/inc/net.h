#ifndef APP_CORE_NET_INC_NET_H_
#define APP_CORE_NET_INC_NET_H_

#include <stdbool.h>
#include <can.h>

bool MMR_NET_IsBrakeEngaged(MmrCanMessage *message);
bool MMR_NET_BrakeCheck(MmrCan *can);

bool MMR_NET_PullClutchAsync(MmrCan *can);
bool MMR_NET_ReleaseClutchAsync(MmrCan *can);

#endif  // !APP_CORE_NET_INC_NET_H_
