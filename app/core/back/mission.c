#include "inc/mission.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/global_state.h"
#include <EBS.h>

static MmrMission manual();
static MmrMission skidpad();
static MmrMission ebsTest();
static MmrMission autocross();
static MmrMission inspection();
static MmrMission trackdrive();
static MmrMission acceleration();


MmrMission MMR_MISSION_Run(MmrMission mission) {
  switch (mission) {
  case MMR_MISSION_IDLE: return mission;
  case MMR_MISSION_MANUAL: return manual();
  case MMR_MISSION_SKIDPAD: return skidpad(); // tutti check + acceleratore info + mission finished +
  case MMR_MISSION_EBS_TEST: return ebsTest(); // tutti check + acceleratore info + partenza + mission finished + attiva ebs
  case MMR_MISSION_AUTOCROSS: return autocross(); // tutti check + acceleratore info
  case MMR_MISSION_INSPECTION: return inspection(); // partenza + minimo + mission finished + va in as finished
  case MMR_MISSION_TRACKDRIVE: return trackdrive(); // tutti check + acceleratore info +  mission finished + attiva ebs
  case MMR_MISSION_ACCELERATION: return acceleration(); // tutti check + acceleratore info +
  default: return mission;
  }
}


static MmrMission manual() {
  gs.ms = MMR_MANUAL_LAUNCH_Run(gs.ms);
  return MMR_MISSION_MANUAL;
}

static MmrMission skidpad() {
  
  return MMR_MISSION_SKIDPAD;
}

static MmrMission ebsTest() {

  return MMR_MISSION_EBS_TEST;
}

static MmrMission autocross() {

  return MMR_MISSION_AUTOCROSS;
}

static MmrMission inspection() {

  return MMR_MISSION_INSPECTION;
}

static MmrMission trackdrive() {
  
  return MMR_MISSION_TRACKDRIVE;
}

static MmrMission acceleration() {

  return MMR_MISSION_ACCELERATION;
}
