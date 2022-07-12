#include "inc/net.h"
#include <message_id.h>
#include <can.h>

bool MMR_NET_IsBrakeEngaged(MmrCanMessage *message) {
  // TODO: send request to brake board and get response in global_state.c (?)
}

bool MMR_NET_BrakeCheck(MmrCan *can) {
  MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_BRK_CHECK_ASB_STATE);
  MmrCanMessage message = MMR_CAN_OutMessage(header);
  MMR_CAN_Send(can, &message);
}
