#include "mmr_can_header.h"
#include "mmr_can_util.h"
#include "mmr_can_optimize.h"

uint32_t MMR_CAN_HeaderToBits(MmrCanHeader header) {
  return 0
    | ((uint32_t)header.priority << 26)
    | ((uint32_t)header.seqNumber << 23)
    | ((uint32_t)header.messageType << 20)
    | ((uint32_t)header.senderId << 10)
    | ((uint32_t)header.messageId);
}

MmrCanHeader MMR_CAN_HeaderFromBits(uint32_t bits) {
  return (MmrCanHeader) {
    .priority = bits >> 26,
    .seqNumber = bits >> 23,
    .messageType = bits >> 20,
    .senderId = bits >> 10,
    .messageId = bits,
  };
}


bool MMR_CAN_IsHeaderScs(MmrCanHeader header) {
  return header.messageType == MMR_CAN_MESSAGE_TYPE_SCS;
}

bool MMR_CAN_IsMultiFrame(MmrCanHeader header) {
  return header.messageType == MMR_CAN_MESSAGE_TYPE_MULTI_FRAME;
}

bool MMR_CAN_IsMultiFrameEnd(MmrCanHeader header) {
  return header.messageType == MMR_CAN_MESSAGE_TYPE_MULTI_FRAME_END;
}