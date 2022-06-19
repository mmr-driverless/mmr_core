#ifndef CAN_INC_MESSAGE_H_
#define CAN_INC_MESSAGE_H_

#include "message_id.h"
#include "message_conversions.h"

const uint8_t MMR_CAN_MAX_DATA_LENGTH = 8;
typedef uint8_t MmrCanRxBuffer[MMR_CAN_MAX_DATA_LENGTH];

typedef struct MmrCanMessage MmrCanMessage;


MmrCanMessage MMR_CAN_Message();
uint32_t MMR_CAN_MESSAGE_GetId(MmrCanMessage *message);
void MMR_CAN_MESSAGE_SetId(MmrCanMessage *message, uint32_t id);
void MMR_CAN_MESSAGE_SetPayload(MmrCanMessage *message, uint8_t *payload, uint8_t length);
int8_t* MMR_CAN_MESSAGE_GetPayload(MmrCanMessage *message, uint8_t *length);

#endif /* CAN_INC_MESSAGE_H_ */
