#include <message.h>
#include <stdint.h>

struct MmrCanMessage {
  uint32_t id;
  uint8_t *payload;
  uint8_t length;
};


MmrCanMessage MMR_CAN_Message() {
  return (MmrCanMessage){};
}

uint32_t MMR_CAN_MESSAGE_GetId(MmrCanMessage *message) {
  return message->id;
}

void MMR_CAN_MESSAGE_SetId(MmrCanMessage *message, uint32_t id) {
  message->id = id;
}

void MMR_CAN_MESSAGE_SetPayload(MmrCanMessage *message, uint8_t *payload, uint8_t length) {
  message->payload = payload;
  message->length = length;
}

uint8_t* MMR_CAN_MESSAGE_GetPayload(MmrCanMessage *message, uint8_t *length) {
  *length = message->length;
  return message->payload;
}
