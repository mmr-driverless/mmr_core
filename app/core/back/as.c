#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/as.h"
#include "inc/apps.h"
#include "inc/peripherals.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <EBS.h>
#include <stdint.h>
#include <stdbool.h>

static MmrEbsCheck ebs = EBS_IDLE;

static MmrAsState idle(MmrAsState state);
static MmrAsState off(MmrAsState state);
static MmrAsState ready(MmrAsState state);
static MmrAsState driving(MmrAsState state);
static MmrAsState emergency(MmrAsState state);
static MmrAsState finished(MmrAsState state);


MmrAsState MMR_AS_Run(MmrAsState state) {
  MMR_GS_UpdateFromCan(asp.can);

  ebs = ebsCheck(ebs);
  if (ebs == EBS_ERROR)
    ;

  if (ebs != EBS_OK)
    return state;

  switch (state) {
  case MMR_AS_IDLE: return idle(state);
  case MMR_AS_OFF: return off(state);
  case MMR_AS_READY: return ready(state);
  case MMR_AS_DRIVING: return driving(state);
  case MMR_AS_EMERGENCY: return emergency(state);
  case MMR_AS_FINISHED: return finished(state);
  }

  return state;
}
