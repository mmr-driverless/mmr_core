#include <can.h>
#include <util.h>

struct MmrCan {
  MmrCanTrySendFn __trySend;
  MmrCanTryReceiveFn __tryReceive;
  MmrCanGetPendingMessagesFn __getPendingMessages;
};


MmrCan MMR_Can(
  MmrCanTrySendFn trySend,
  MmrCanTryReceiveFn tryReceive,
  MmrCanGetPendingMessagesFn getPendingMessages
) {
  return (MmrCan) {
    .__trySend = trySend,
    .__tryReceive = tryReceive,
    .__getPendingMessages = getPendingMessages,
  };
}


bool MMR_CAN_Send(MmrCan *can, MmrCanPacket packet) {
  return can->__trySend(packet);
}

bool MMR_CAN_Receive(MmrCan *can, MmrCanMessage *result) {
  return can->__tryReceive(result);
}

uint8_t MMR_CAN_GetPendingMessages(MmrCan *can) {
  return can->__getPendingMessages();
}

