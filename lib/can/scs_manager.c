#include "mmr_can_scs.h"

static void maybeIncrementRtr(MmrCanScsEntry *entry);
static HalStatus sendScs(CanHandle *hcan, MmrCanHeader header);
static MmrCanScsCheckResult checkScs(MmrCanScsEntry *entry);


bool MMR_CAN_MaybeHandleAck(MmrCanHeader header) {
  if (header.messageType != MMR_CAN_MESSAGE_TYPE_ACK) {
    return false;
  }

  header.messageType = MMR_CAN_MESSAGE_TYPE_SCS;
  MmrCanScsEntry *entry = MMR_CAN_FindScsEntry(header);
  if (entry == NULL) {
    return false;
  }

  MMR_CAN_ClearScsEntry(header);
  return true;
}


HalStatus MMR_CAN_SendAck(
  CanHandle *hcan,
  MmrCanHeader originalHeader
) {
  originalHeader.messageType = MMR_CAN_MESSAGE_TYPE_ACK;
  return sendScs(hcan, originalHeader);
}


HalStatus MMR_CAN_SendScs(
  CanHandle *hcan,
  MmrCanMessageId scsId,
  CanId senderId
) {
  MmrCanHeader header = {
    .senderId = senderId,
    .messageId = scsId,
    .priority = MMR_CAN_MESSAGE_PRIORITY_HIGH,
    .messageType = MMR_CAN_MESSAGE_TYPE_SCS,
  };

  if (MMR_CAN_PutScsEntry(header) == NULL) {
    return HAL_ERROR;
  }

  return sendScs(hcan, header);
}


HalStatus MMR_CAN_HandleNextScs(CanHandle *hcan) {
  MmrCanScsEntry *entry = MMR_CAN_GetNextScsEntry();

  if (MMR_CAN_HeaderToBits(entry->header) == 0) {
    return HAL_OK;
  }

  switch (checkScs(entry)) {
    case MMR_CAN_SCS_CHECK_ERROR: return HAL_ERROR;
    case MMR_CAN_SCS_CHECK_OK: return HAL_OK;
    default: break;
  }

  entry->counter = MMR_CAN_GetCurrentTick();
  return sendScs(hcan, entry->header);
}


static HalStatus sendScs(CanHandle *hcan, MmrCanHeader header) {
  MmrCanPacket packet = {
    .header = header,
    .length = 0,
  };

  return MMR_CAN_SendNoTamper(hcan, packet);
}


static MmrCanScsCheckResult checkScs(MmrCanScsEntry *entry) {
  maybeIncrementRtr(entry);
  return
    entry->rtr == 1 ? MMR_CAN_SCS_CHECK_RTR :
    entry->rtr > 1 ? MMR_CAN_SCS_CHECK_ERROR :
    MMR_CAN_SCS_CHECK_OK;
}

static void maybeIncrementRtr(MmrCanScsEntry *entry) {
  TimerRange now = MMR_CAN_GetCurrentTick();
  TimerRange delay = now - entry->counter;

  if (delay >= MMR_CAN_MAX_TIMEOUT) {
    entry->rtr++;
  }
}
