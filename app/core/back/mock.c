#include "inc/mock.h"
#include "inc/as.h"
#include "inc/axis_leds.h"
#include "inc/global_state.h"
#include <delay.h>


void MMR_BACK_MOCK_AxisLeds() {
  static MmrDelay delay = { .ms = 5000 };

  MMR_AXIS_LEDS_Run(gs.stateAs);

  if (!MMR_DELAY_WaitAsync(&delay)) {
    return;
  }

  gs.stateAs++;
  gs.stateAs %= MMR_AS_FINISHED + 1;
}
