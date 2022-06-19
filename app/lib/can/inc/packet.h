#ifndef CAN_INC_PACKET_H_
#define CAN_INC_PACKET_H_

#include <stdint.h>

typedef struct MmrCanPacket MmrCanPacket;

MmrCanPacket MMR_CAN_Packet(uint32_t id, uint8_t *data, uint8_t length);

#endif /* CAN_INC_PACKET_H_ */
