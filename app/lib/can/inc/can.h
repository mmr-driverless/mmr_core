#ifndef INC_MMR_CAN_H_
#define INC_MMR_CAN_H_

#include "message.h"
#include "header.h"
#include "filter.h"

#include "../../util/inc/util.h"
#include <stdint.h>
#include <stdbool.h>

typedef bool (*MmrCanTrySendFn)(MmrCanMessage *message);
typedef bool (*MmrCanTryReceiveFn)(MmrCanMessage *message);
typedef bool (*MmrCanTrySetFilterFn)(MmrCanFilter *filter);
typedef uint8_t (*MmrCanGetPendingMessagesFn)();


typedef struct MmrCan {
  MmrCanTrySendFn __trySend;
  MmrCanTryReceiveFn __tryReceive;
  MmrCanTrySetFilterFn __trySetFilter;
  MmrCanGetPendingMessagesFn __getPendingMessages;
} MmrCan;


bool MMR_CAN_SetFilter(MmrCan *can, MmrCanFilter *filter);
bool MMR_CAN_Send(MmrCan *can, MmrCanMessage *message);

uint8_t MMR_CAN_GetPendingMessages(MmrCan *can);
bool MMR_CAN_Receive(MmrCan *can, MmrCanMessage *result);
MmrTaskResult MMR_CAN_ReceiveAsync(MmrCan *can, MmrCanMessage *result);

#endif /* INC_MMR_CAN_H_ */
