#include "inc/net.h"
#include <can.h>

static MmrCan *localCan;

void MMR_NET_BRAKE_Init(MmrCan *can) {
  localCan = can;
}

bool MMR_NET_IsBrakeEngagedRequest() {
  // TODO: send request to brake board and get response in global_state.c (?)
}

bool MMR_NET_BrakeCheckRequest() {
  MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_BRK_CHECK_ASB_STATE);
  MmrCanMessage message = MMR_CAN_OutMessage(header);
  MMR_CAN_Send(localCan, &message);
}
