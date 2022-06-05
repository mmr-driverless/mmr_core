#ifndef INC_MMR_CAN_MSG_CONVERSIONS_H_
#define INC_MMR_CAN_MSG_CONVERSIONS_H_

#include <stdint.h>

#define MMR_CAN_MsgToByte(msg) (*(uint8_t*)(msg.store))
#define MMR_CAN_MsgToInt16(msg) (*(int16_t*)(msg.store))
#define MMR_CAN_MsgToUint16(msg) (*(uint16_t*)(msg.store))
#define MMR_CAN_MsgToInt32(msg) (*(int32_t*)(msg.store))
#define MMR_CAN_MsgToUint32(msg) (*(uint32_t*)(msg.store))
#define MMR_CAN_MsgToInt64(msg) (*(int64_t*)(msg.store))
#define MMR_CAN_MsgToUint64(msg) (*(uint64_t*)(msg.store))
#define MMR_CAN_MsgToFloat(msg) (*(float*)(msg.store))
#define MMR_CAN_MsgToDouble(msg) (*(double*)(msg.store))

#endif
