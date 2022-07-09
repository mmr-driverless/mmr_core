#include "inc/global_state.h"
#include "inc/gear_change.h"
#include "inc/autonomous.h"
#include "inc/manual.h"
#include "inc/as.h"
#include "inc/apps.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <stdint.h>
#include <stdbool.h>



static MmrCan *__can;
static MmrPin *__gearN;
static MmrPin *__gearUp;
static MmrPin *__gearDown;
static MmrButton __changeModeButton;
static uint32_t *__apps;
static uint32_t *__adc;


static MmrAutonomousState as = MMR_AUTONOMOUS_WAITING;
static MmrManualState ms = MMR_MANUAL_WAITING;


void MMR_AS_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,
  MmrPin *changeMode,
  uint32_t *apps,
  uint32_t *adc
) {
  __can = can;
  __gearN = gearN;
  __gearUp = gearUp;
  __gearDown = gearDown;
  __changeModeButton = MMR_Button(changeMode);

  __apps = apps;
  __adc = adc;

  MMR_MANUAL_Init(can, apps, adc);
  MMR_GEAR_CHANGE_Init(gearUp, gearDown);
  MMR_AUTONOMOUS_Init(can, gearN, apps, adc);
}

MmrLaunchControlMode MMR_AS_Run(MmrLaunchControlMode mode) {
  MMR_GS_UpdateFromCan(__can);

  switch (mode) {
  case MMR_AS_MODE_MANUAL:
    MMR_GS_SetLap(1);
    ms = MMR_MANUAL_Run(ms);
    break;

  case MMR_AS_MODE_GEAR_CHANGE:
    MMR_GS_SetLap(2);
    MMR_GEAR_CHANGE_Run();
    ms = MMR_MANUAL_Run(ms);
    break;

  case MMR_AS_MODE_AUTONOMOUS:
    MMR_GS_SetLap(1);
    as = MMR_AUTONOMOUS_Run(as);
    break;
  }

  return mode;
}
