#include <stdbool.h>
#include <can.h>
#include <util.h>

#define sendAny(pcan, id, pvalue) \
  MMR_CAN_Send(pcan, MMR_CAN_Packet(id, pvalue, sizeof(*pvalue)))


bool MMR_CAN_SendId(MmrCan *can, uint32_t id) {
  return MMR_CAN_Send(can, MMR_CAN_Packet(id, NULL, 0));
}

bool MMR_CAN_SendBool(MmrCan *can, uint32_t id, bool value) {
  return sendAny(can, id, &value);
}

bool MMR_CAN_SendByte(MmrCan *can, uint8_t value) {
  return sendAny(can, id, &value);
}

bool MMR_CAN_SendInt32(MmrCan *can, int32_t value) {
  return sendAny(can, id, &value);
}

bool MMR_CAN_SendUInt32(MmrCan *can, uint32_t value) {
  return sendAny(can, id, &value);
}

bool MMR_CAN_SendFloat(MmrCan *can, float value) {
  return sendAny(can, id, &value);
}

bool MMR_CAN_SendString(MmrCan *can, const char *value) {
  uint8_t length = strnlen(value, MMR_CAN_MAX_DATA_LENGTH);
  MmrCanPacket packet = MMR_CAN_Packet(id, (unt8_t*)value, length);

  return MMR_CAN_Send(can, packet);
}
