#include "can0.h"
#include "message.h"

#define MAILBOXES_COUNT 3

static uint32_t __mailboxes[MAILBOXES_COUNT] = {};
static uint8_t __currentMailbox = 0;

extern CAN_HandleTypeDef hcan1;

static uint32_t *getNextMailbox() {
  __currentMailbox++;
  __currentMailbox %= MAILBOXES_COUNT;

  return __mailboxes + __currentMailbox;
}


static bool can0Send(MmrCanMessage *message) {
  CAN_TxHeaderTypeDef tx = {
    .IDE = message->isStandardId ? CAN_ID_STD : CAN_ID_EXT,
    .RTR = CAN_RTR_DATA,
    .DLC = message->length,
    .TransmitGlobalTime = DISABLE,
  };

  if (message->isStandardId) {
    tx.StdId = message->id;
  }
  else {
    tx.ExtId = message->id;
  }

  HAL_StatusTypeDef status =
    HAL_CAN_AddTxMessage(&hcan1, &tx, message->payload, getNextMailbox());

  return status == HAL_OK;
}

static bool can0Receive(MmrCanMessage *message) {
  CAN_RxHeaderTypeDef rx = {};
  HAL_StatusTypeDef status =
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, message->payload);

  message->length = rx.DLC;
  MMR_CAN_MESSAGE_SetId(message, rx.ExtId);
  return status == HAL_OK;
}

static uint8_t can0PendingMessages() {
  return HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
}


MmrCan can0 = {
  .__trySend = can0Send,
  .__tryReceive = can0Receive,
  .__getPendingMessages = can0PendingMessages,
};
