#ifndef INC_MMR_CAN_H_
#define INC_MMR_CAN_H_

#include "message.h"
#include "header.h"
#include "packet.h"

#include "../../util/inc/util.h"
#include <stdint.h>
#include <stdbool.h>

typedef bool (*MmrCanTrySendFn)(MmrCanMessage *message);
typedef bool (*MmrCanTryReceiveFn)(MmrCanMessage *message);
typedef bool (*MmrCanGetPendingMessagesFn)();


typedef struct MmrCan {
  MmrCanTrySendFn __trySend;
  MmrCanTryReceiveFn __tryReceive;
  MmrCanGetPendingMessagesFn __getPendingMessages;
} MmrCan;


MmrCan MMR_Can(
  MmrCanTrySendFn trySend,
  MmrCanTryReceiveFn tryReceive,
  MmrCanGetPendingMessagesFn getPendingMessages
);

bool MMR_CAN_Send(MmrCan *can, MmrCanMessage *message);

uint8_t MMR_CAN_GetPendingMessages(MmrCan *can);
bool MMR_CAN_Receive(MmrCan *can, MmrCanMessage *result);
MmrTaskResult MMR_CAN_ReceiveAsync(MmrCan *can, MmrCanMessage *result);

#endif /* INC_MMR_CAN_H_ */
