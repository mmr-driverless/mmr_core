#include "mmr_bb_utils.h"

uint8_t tick = 0;

void MMR_BB_utilsSetTick() {
	tick = uwTick;
}

bool MMR_BB_utilsTimer() {
	int delay = uwTick - tick;

	if (delay > WAIT_PERIOD)
		return true;

	return false;
}

void MMR_BB_createDefaultPacket(MmrCanPacket *packet) {
	switch (packet->header.messageId) {
		case MMR_CAN_MESSAGE_ID_S_CLUTCH :
			packet->data = (uint8_t*) 10;
			packet->length = sizeof(uint16_t);
			break;
		case MMR_CAN_MESSAGE_ID_S_LV12 :
			packet->data = (uint8_t*) 1;
			packet->length = sizeof(int);
			break;
		case MMR_CAN_MESSAGE_ID_TYPE_MANUAL_MISSION :
			;
			break;

		default :
			packet->data = (uint8_t*)"sample";
			packet->length = 6;
			break;
	}
}
