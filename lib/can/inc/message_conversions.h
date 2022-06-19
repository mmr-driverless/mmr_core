#ifndef INC_MMR_CAN_MSG_CONVERSIONS_H_
#define INC_MMR_CAN_MSG_CONVERSIONS_H_

#include <stdint.h>

#define MMR_CAN_MESSAGE_ReadByte(msg, offset) (*(uint8_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadInt16(msg, offset) (*(int16_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadUint16(msg, offset) (*(uint16_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadInt32(msg, offset) (*(int32_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadUint32(msg, offset) (*(uint32_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadInt64(msg, offset) (*(int64_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadUint64(msg, offset) (*(uint64_t*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadFloat(msg, offset) (*(float*)(msg.store + offset))
#define MMR_CAN_MESSAGE_ReadDouble(msg, offset) (*(double*)(msg.store + offset))

#endif
