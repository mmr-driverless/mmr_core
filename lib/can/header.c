#include "inc/header.h"

uint32_t MMR_CAN_HEADER_ToBits(MmrCanHeader header) {
  return 0
    | ((uint32_t)header.messageType << 26)
    | ((uint32_t)header.priority << 23)
    | ((uint32_t)header.seqNumber << 20)
    | ((uint32_t)header.senderId << 10)
    | ((uint32_t)header.messageId);
}

MmrCanHeader MMR_CAN_HEADER_FromBits(uint32_t bits) {
  return (MmrCanHeader) {
    .messageType = bits >> 26,
    .priority = bits >> 23,
    .seqNumber = bits >> 20,
    .senderId = bits >> 10,
    .messageId = bits,
  };
}
