#include "inc/can.h"
#include <util.h>


bool MMR_CAN_SetFilter(MmrCan *can, MmrCanFilter *filter) {
  return can->__trySetFilter(filter);
}

bool MMR_CAN_Send(MmrCan *can, MmrCanMessage *message) {
  return can->__trySend(message);
}

bool MMR_CAN_Receive(MmrCan *can, MmrCanMessage *result) {
  return can->__tryReceive(result);
}

uint8_t MMR_CAN_GetPendingMessages(MmrCan *can) {
  return can->__getPendingMessages();
}

