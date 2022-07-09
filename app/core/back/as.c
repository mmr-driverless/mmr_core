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


static MmrMission manual();
static MmrMission skidpad();
static MmrMission ebsTest();
static MmrMission autocross();
static MmrMission inspection();
static MmrMission trackdrive();
static MmrMission acceleration();


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

MmrMission MMR_AS_Run(MmrMission mission) {
  MMR_GS_UpdateFromCan(__can);

  switch (mission) {
  case MMR_MISSION_IDLE: return mission;
  case MMR_MISSION_MANUAL: return manual();
  case MMR_MISSION_SKIDPAD: return skidpad();
  case MMR_MISSION_EBS_TEST: return ebsTest();
  case MMR_MISSION_AUTOCROSS: return autocross();
  case MMR_MISSION_INSPECTION: return inspection();
  case MMR_MISSION_TRACKDRIVE: return trackdrive();
  case MMR_MISSION_ACCELERATION: return acceleration();
  default: return mission;
  }
}


static MmrMission manual() {
  ms = MMR_MANUAL_Run(ms);
  return MMR_MISSION_MANUAL;
}

static MmrMission skidpad() {
  
}

static MmrMission ebsTest() {

}

static MmrMission autocross() {

}

static MmrMission inspection() {

}

static MmrMission trackdrive() {

}

static MmrMission acceleration() {

}

