#include "mmr_can_header.h"
#include "mmr_can_util.h"
#include "mmr_can_optimize.h"

uint32_t MMR_CAN_HeaderToBits(MmrCanHeader header) {
  return 0
    | ((uint32_t)header.priority << 26)
    | ((uint32_t)header.messageId << 16)
    | ((uint32_t)header.senderId << 6)
    | ((uint32_t)header.seqNumber << 3)
    | ((uint32_t)header.messageType);
}

MmrCanHeader MMR_CAN_HeaderFromBits(uint32_t bits) {
  return (MmrCanHeader) {
    .priority = bits >> 26,
    .messageId = bits >> 16,
    .senderId = bits >> 6,
    .seqNumber = bits >> 3,
    .messageType = bits,
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
