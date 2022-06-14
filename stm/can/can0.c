#include "can0.h"

static bool can0Send(uint32_t id, uint8_t *buffer, int len) {
  HAL_CAN_AddTxMessage();
}


extern Can can0 = {
  .trySend = can0Send,
};