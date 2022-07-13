#include "inc/net.h"
#include <can.h>
#include <delay.h>

static MmrCan *localCan;

void MMR_NET_BRAKE_Init(MmrCan *can) {
  localCan = can;
}

bool MMR_NET_EngageBrakeAsync() {
  static MmrDelay sendDelay = { .ms = 10};
  if (MMR_DELAY_WaitAsync(&sendDelay)) {
    MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_BRK_TARGET_PRESSURE);
    MmrCanMessage message = MMR_CAN_OutMessage(header);
    float brakeTargetPressure = 0.9;
    MMR_CAN_MESSAGE_SetPayload(&message, (uint8_t*) &brakeTargetPressure, 4);
    return MMR_CAN_Send(localCan, &message);
  }
  return false;
}