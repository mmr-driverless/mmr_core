#include "lib/can/inc/can.h"

bool sendOk(uint8_t *buffer, int length) {
  return true;
}

static Can can = {
  .trySend = sendOk,
};


int main() {
  
}