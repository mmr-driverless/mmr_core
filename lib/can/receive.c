#include "inc/can.h"
#include "../util/inc/util.h"
#include <stdbool.h>


MmrTaskResult MMR_CAN_ReceiveAsync(MmrCan *can, MmrCanMessage *result) {
  uint8_t pendingMessages = MMR_CAN_GetPendingMessages(can);
  if (pendingMessages <= 0) {
    return MMR_TASK_PENDING;
  }

  return MMR_CAN_Receive(can, result)
    ? MMR_TASK_COMPLETED
    : MMR_TASK_ERROR;
}
