#include "net.h"

static bool maskBit(uint8_t val, unsigned bitIdx) {
  uint8_t mask = 0x01 << bitIdx;
  uint8_t masked = val & mask;
  if (masked == mask)
    return true;
  return false;
}

void MMR_NET_ResParse(uint8_t resPayload, MmrButtonState *emergencyButtonState, MmrButtonState *goButtonState, MmrButtonState *bagButtonState) {
  *emergencyButtonState = maskBit(resPayload, 0) ? MMR_BUTTON_PRESSED : MMR_BUTTON_RELEASED;
  *goButtonState = maskBit(resPayload, 1) ? MMR_BUTTON_PRESSED : MMR_BUTTON_RELEASED;
  *bagButtonState = maskBit(resPayload, 2) ? MMR_BUTTON_PRESSED : MMR_BUTTON_RELEASED;
}