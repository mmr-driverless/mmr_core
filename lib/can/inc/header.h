/**
 * @file mmr_can_header.h
 * @brief
 * This file defines the header used for the can message,
 * along with its utilities.
 * 
 * With header is intended the ExtendedId portion
 * of the can message.
 */

#ifndef INC_MMR_CAN_HEADER_H_
#define INC_MMR_CAN_HEADER_H_

#include "message_id.h"

#include <stdbool.h>
#include <util.h>
#include <binary_literals.h>


typedef enum MmrCanMessageType {
  MMR_CAN_MESSAGE_TYPE_SCS = B_(0000),
  MMR_CAN_MESSAGE_TYPE_ACK = B_(0001),
  MMR_CAN_MESSAGE_TYPE_MULTI_FRAME = B_(0010),
  MMR_CAN_MESSAGE_TYPE_MULTI_FRAME_END = B_(0011),
  MMR_CAN_MESSAGE_TYPE_NORMAL = B_(0100),
} MmrCanMessageType;


typedef enum MmrCanMessagePriority {
  MMR_CAN_MESSAGE_PRIORITY_LOW = B_(0010),
  MMR_CAN_MESSAGE_PRIORITY_NORMAL = B_(0001),
  MMR_CAN_MESSAGE_PRIORITY_HIGH = B_(0000),
} MmrCanMessagePriority;


typedef struct MmrCanHeader {
  MmrCanMessagePriority priority : 3;
  uint16_t messageId : 10;
  uint16_t senderId : 10;
  uint8_t seqNumber : 3;
  MmrCanMessageType messageType : 3;
} MmrCanHeader;


uint32_t MMR_CAN_HEADER_ToBits(MmrCanHeader header);
MmrCanHeader MMR_CAN_HEADER_FromBits(uint32_t bits);

#endif /* INC_MMR_CAN_HEADER_H_ */
