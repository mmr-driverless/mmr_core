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


void MMR_AS_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,
  MmrPin *changeMode,
  uint32_t *apps,
  uint32_t *adc
) {
  asp = (MmrAsPeripherals) {
    .can = can,
    .gearN = gearN,
    .gearUp = gearUp,
    .gearDown = gearDown,
    .apps = apps,
    .adc = adc,
  };
}

MmrAsState MMR_AS_Run(MmrAsState mission) {
  MMR_GS_UpdateFromCan(__can);

  ebs = ebsCheck(ebs);
  if (ebs == EBS_ERROR)
    ;

  if (ebs == EBS_OK)
}

