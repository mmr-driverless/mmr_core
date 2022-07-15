#include "inc/net.h"
#include <can.h>
#include <delay.h>


bool MMR_NET_EngageBrakeAsync(MmrCan *can, float target) {
  static MmrDelay sendDelay = { .ms = 200 };

  if (MMR_DELAY_WaitAsync(&sendDelay)) {
    MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_BRK_TARGET_PRESSURE);
    MmrCanMessage message = MMR_CAN_OutMessage(header);
    MMR_CAN_MESSAGE_SetPayload(&message, (uint8_t*)&target, sizeof(target));
    return MMR_CAN_Send(can, &message);
  }

  return false;
}
