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

#include <stdbool.h>
#include "mmr_can_util.h"
#include "mmr_can_binary_literals.h"
#include "mmr_can_message_id.h"

/**
 * @brief
 * These values can be appended to the extended-id
 * portion of the CAN bus message (that is, the lower 5 bits
 * of the standard id)
 * 
 * They are used to check if a message is either standalone, an
 * acknowledgement or split into multiple frames
 * 
 * Constants with lower values have an higher priority.
 */
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


/**
 * @brief
 * This struct encodes the values stored inside the
 * extended id field of a CAN packet.
 */
typedef struct MmrCanHeader {
  MmrCanMessagePriority priority : 3;
  uint16_t messageId : 10;
  uint16_t senderId : 10;
  uint8_t seqNumber : 3;
  MmrCanMessageType messageType : 3;
} MmrCanHeader;


/**
 * @brief
 * Serializes an MmrCanHeader to bits.
 * That is, a 32bits integer with the first
 * 3 bits set to zero and the remaining 29 containing the
 * extended id
 */
uint32_t MMR_CAN_HeaderToBits(MmrCanHeader header);

/**
 * @brief
 * Deserializes a 32bits integer to an MmrCanHeader.
 * The 3 left-most bits must be of padding.
 */
MmrCanHeader MMR_CAN_HeaderFromBits(uint32_t bits);


/**
 * @brief
 * Tells wether the given header represents an SCS.
 */
bool MMR_CAN_IsHeaderScs(MmrCanHeader header);
bool MMR_CAN_IsMultiFrame(MmrCanHeader header);
bool MMR_CAN_IsMultiFrameEnd(MmrCanHeader header);

#endif /* INC_MMR_CAN_HEADER_H_ */
