#include <stdbool.h>
#include "inc/can.h"


typedef struct {
  CanHandle *handle;
  uint8_t *result;

  struct {
    CanRxHeader rx;
    MmrCanHeader mmr;
  } headers;

  uint8_t fifo;
} ReceptionParams;


static HalStatus receiveOne(ReceptionParams *rp);
static HalStatus receiveAll(ReceptionParams *rp);
static bool headerIsMultiFrame(MmrCanHeader header, CanId targetId);


MmrResult MMR_CAN_TryReceive(CanHandle *hcan, MmrCanMessage *result) {
  size_t pendingMessages =
    HAL_CAN_GetRxFifoFillLevel(hcan, MMR_CAN_RX_FIFO);

  if (pendingMessages > 0) {
    return MMR_CAN_Receive(hcan, result) != HAL_OK
      ? MMR_RESULT_COMPLETED
      : MMR_RESULT_ERROR;
  }

  return MMR_RESULT_PENDING;
}


HalStatus MMR_CAN_Receive(CanHandle *hcan, MmrCanMessage *result) {
  ReceptionParams rp = {
    .handle = hcan,
    .result = (uint8_t*)result->store,
    .fifo = MMR_CAN_RX_FIFO,
  };

  HalStatus status = receiveOne(&rp);
  if (status != HAL_OK) {
    return status;
  }

  result->header = rp.headers.mmr;
  if (MMR_CAN_IsMultiFrame(rp.headers.mmr)) {
    status |= receiveAll(&rp);
  }

  return status;
}


static HalStatus receiveAll(ReceptionParams *rp) {
  CanId targetId = rp->headers.mmr.senderId;
  HalStatus status = HAL_OK;
  do {
    rp->result += MMR_CAN_MAX_DATA_LENGTH;
    status |= receiveOne(rp);
  }
  while (
    headerIsMultiFrame(rp->headers.mmr, targetId) && status == HAL_OK
  );

  return status;
}


static HalStatus receiveOne(ReceptionParams *rp) {
  HalStatus status = HAL_CAN_GetRxMessage(
    rp->handle,
    rp->fifo,
    &rp->headers.rx,
    rp->result
  );

  rp->headers.mmr = MMR_CAN_HeaderFromBits(rp->headers.rx.ExtId);
  return status;
}


static bool headerIsMultiFrame(MmrCanHeader header, CanId targetId) {
  return
    MMR_CAN_IsMultiFrame(header) &&
    !MMR_CAN_IsMultiFrameEnd(header) &&
    header.senderId == targetId;
}
