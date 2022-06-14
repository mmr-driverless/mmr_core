#include <stdbool.h>
#include "mmr_can.h"
#include "mmr_can_util.h"


typedef struct {
  CanHandle *handle;
  MmrCanPacket *packet;

  struct {
    CanTxHeader tx;
    MmrCanHeader mmr;
  } headers;
} TransmissionParams;


static TransmissionParams buildParams(CanHandle *hcan, MmrCanPacket *packet);
static HalStatus send(TransmissionParams *tp);
static HalStatus sendNormal(TransmissionParams *tp);
static HalStatus sendMulti(TransmissionParams *tp);
static HalStatus sendSingleMultiFrame(TransmissionParams *tp);
static uint8_t computeFramesToSend(MmrCanPacket *packet);
static uint8_t computeNextMessageLength(MmrCanPacket *packet);
static void setMessageType(TransmissionParams *header, MmrCanMessageType type);
static void syncHeaders(TransmissionParams *tp);
static CanMailbox *getNextMailbox();


#define MAILBOXES_COUNT 3

static CanMailbox __mailboxes[MAILBOXES_COUNT] = {};
static uint8_t __currentMailbox = 0;


HalStatus MMR_CAN_Send(CanHandle *hcan, MmrCanPacket packet) {
  TransmissionParams tp =
    buildParams(hcan, &packet);

  syncHeaders(&tp);
  return packet.length <= MMR_CAN_MAX_DATA_LENGTH
    ? sendNormal(&tp)
    : sendMulti(&tp);
}


HalStatus MMR_CAN_SendNoTamper(CanHandle *hcan, MmrCanPacket packet) {
  TransmissionParams tp =
    buildParams(hcan, &packet);

  return send(&tp);
}


static TransmissionParams buildParams(CanHandle *hcan, MmrCanPacket *packet) {
  return (TransmissionParams) {
    .handle = hcan,
    .packet = packet,
    .headers.mmr = packet->header,
    .headers.tx = {
      .IDE = CAN_ID_EXT,
      .RTR = CAN_RTR_DATA,
      .DLC = packet->length,
      .TransmitGlobalTime = DISABLE,
    },
  };
}


static HalStatus sendNormal(TransmissionParams *tp) {
  setMessageType(tp, MMR_CAN_MESSAGE_TYPE_NORMAL);
  return send(tp);
}


static HalStatus sendMulti(TransmissionParams *tp) {
  HalStatus status = HAL_OK;
  uint8_t framesToSend = computeFramesToSend(tp->packet);

  setMessageType(tp, MMR_CAN_MESSAGE_TYPE_MULTI_FRAME);
  do {
    bool isLastFrame = framesToSend <= 1;
    if (isLastFrame) {
      setMessageType(tp, MMR_CAN_MESSAGE_TYPE_MULTI_FRAME_END);
    }

    status |= sendSingleMultiFrame(tp);
  }
  while (framesToSend-- > 1 && status == HAL_OK);

  return status;
}


static HalStatus sendSingleMultiFrame(TransmissionParams *tp) {
  uint8_t length =
    computeNextMessageLength(tp->packet);

  tp->headers.tx.DLC = length;
  tp->packet->length -= length;

  HalStatus result = send(tp);

  tp->packet->data += length;
  return result;
}


static HalStatus send(TransmissionParams *tp) {
  syncHeaders(tp);

  return HAL_CAN_AddTxMessage(
    tp->handle,
    &tp->headers.tx,
    tp->packet->data,
    getNextMailbox()
  );
}


static CanMailbox *getNextMailbox() {
  __currentMailbox++;
  __currentMailbox %= MAILBOXES_COUNT;

  return __mailboxes + __currentMailbox;
}


static void syncHeaders(TransmissionParams *tp) {
  tp->headers.tx.ExtId = MMR_CAN_HeaderToBits(tp->headers.mmr);
}


/**
 * @brief 
 * Computes the frames that will need to be sent for a packet.
 * 
 * For example, if a packet has 15 bytes, the function will return
 * 2, as one packet is needed for the first 8 bytes and another is
 * needed for the remaining 7.
 * 
 * @example
 * The computation is pretty simple
 * Given
 *               length = 17 (bytes)
 * We'll have
 *         framesToSend = 17 / 8 = 2 (frames)
 *            remainder = 17 % 8 = 1 (bytes)
 * maybeOneForRemainder = 1 > 0  = 1 (frame)
 * -------------------------------------------
 *               result = 2 + 1  = 3 (frames) 
 */
static uint8_t computeFramesToSend(MmrCanPacket *packet) {
  uint8_t length = packet->length;
  uint8_t framesToSend = length / MMR_CAN_MAX_DATA_LENGTH;
  uint8_t remainder = length % MMR_CAN_MAX_DATA_LENGTH;
  uint8_t maybeOneForRemainder = remainder > 0;

  return framesToSend + maybeOneForRemainder;
}


/**
 * @brief 
 * Returns the length for the next message, either
 * 8 bytes or a lower value.
 */
static uint8_t computeNextMessageLength(MmrCanPacket *packet) {
  return min(
    packet->length, MMR_CAN_MAX_DATA_LENGTH
  );
}

static always_inline void setMessageType(TransmissionParams *tp, MmrCanMessageType type) {
  tp->headers.mmr.messageType = type;
}
