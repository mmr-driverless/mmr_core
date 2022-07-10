#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous.h"
#include "inc/manual.h"
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

MmrAsPeripherals asp;

static MmrEbsCheck ebs = EBS_IDLE;


MmrAsState MMR_AS_Run(MmrAsState state) {
  MMR_GS_UpdateFromCan(asp.can);

  ebs = ebsCheck(ebs);
  if (ebs == EBS_ERROR)
    ;

  if (ebs != EBS_OK)
    return state;

  return state;
}

