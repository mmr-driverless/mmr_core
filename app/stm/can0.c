#include "inc/can0.h"

#define MMR_CAN0_MAILBOXES_COUNT 3

extern CAN_HandleTypeDef hcan1;


static uint32_t __mmr_can0_mailboxes[MMR_CAN0_MAILBOXES_COUNT] = {};
static uint8_t __mmr_can0_currentMailbox = 0;

static uint32_t *__mmr_can0_getNextMailbox() {
  __mmr_can0_currentMailbox++;
  __mmr_can0_currentMailbox %= MMR_CAN0_MAILBOXES_COUNT;

  return __mmr_can0_mailboxes + __mmr_can0_currentMailbox;
}


static bool __mmr_can0_send(MmrCanMessage *message) {
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
    HAL_CAN_AddTxMessage(&hcan1, &tx, message->payload, __mmr_can0_getNextMailbox());

  return status == HAL_OK;
}


static bool __mmr_can0_receive(MmrCanMessage *message) {
  CAN_RxHeaderTypeDef rx = {};
  HAL_StatusTypeDef status =
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, message->payload);

  message->length = rx.DLC;
  MMR_CAN_MESSAGE_SetId(message, rx.ExtId);
  return status == HAL_OK;
}


static uint8_t __mmr_can0_pendingMessages() {
  return HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
}


MmrCan can0 = {
  .__trySend = __mmr_can0_send,
  .__tryReceive = __mmr_can0_receive,
  .__getPendingMessages = __mmr_can0_pendingMessages,
};
