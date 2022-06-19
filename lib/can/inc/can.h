/**
 * @file mmr_can.h
 * @brief
 * Main header for the mmr_can library.
 */

#ifndef INC_MMR_CAN_H_
#define INC_MMR_CAN_H_

#include "message.h"
#include "header.h"
#include "scs.h"
#include "packet.h"

#include <binary_literals.h>
#include <util.h>
#include <stdint.h>
#include <stdbool.h>


typedef bool (*MmrCanTrySendFn)(MmrCanPacket packet);
typedef bool (*MmrCanTryReceiveFn)(MmrCanMessage *message);
typedef bool (*MmrCanGetPendingMessagesFn)();


typedef struct MmrCan MmrCan;


MmrCan MMR_Can(
  MmrCanTrySendFn trySend,
  MmrCanTryReceiveFn tryReceive,
  MmrCanGetPendingMessagesFn getPendingMessages
);

bool MMR_CAN_Send(MmrCan *can, MmrCanPacket packet);
bool MMR_CAN_SendId(MmrCan *can, uint32_t id);
bool MMR_CAN_SendBool(MmrCan *can, bool value);
bool MMR_CAN_SendByte(MmrCan *can, uint8_t value);
bool MMR_CAN_SendString(MmrCan *can, const char *value);
bool MMR_CAN_SendInt32(MmrCan *can, int32_t value);
bool MMR_CAN_SendUInt32(MmrCan *can, uint32_t value);
bool MMR_CAN_SendFloat(MmrCan *can, float value);

uint8_t MMR_CAN_GetPendingMessages(MmrCan *can);
bool MMR_CAN_Receive(MmrCan *can, MmrCanMessage *result);
MmrTaskResult MMR_CAN_ReceiveAsync(MmrCan *can, MmrCanMessage *result);

#endif /* INC_MMR_CAN_H_ */
