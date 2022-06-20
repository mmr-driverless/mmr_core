#include "inc/timing.h"


static TickProvider __tickProvider;

void MMR_SetTickProvider(TickProvider tickProvider) {
  __tickProvider = tickProvider;
}

Tick MMR_GetTick() {
  return __tickProvider();
}
