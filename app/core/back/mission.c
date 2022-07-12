#include "inc/ebs.h"
#include "inc/apps.h"
#include "inc/peripherals.h"
#include "inc/mission.h"
#include "inc/autonomous_launch.h"
#include "inc/manual_launch.h"
#include "inc/global_state.h"

static MmrMission manual();
static MmrMission skidpad();
static MmrMission ebsTest();
static MmrMission autocross();
static MmrMission inspection();
static MmrMission trackdrive();
static MmrMission acceleration();
static MmrMission finished();


MmrMission MMR_MISSION_Run(MmrMission mission) {
  switch (mission) {
  case MMR_MISSION_IDLE: return mission;
  case MMR_MISSION_MANUAL: return manual();
  case MMR_MISSION_SKIDPAD: return skidpad();
  case MMR_MISSION_EBS_TEST: return ebsTest();
  case MMR_MISSION_AUTOCROSS: return autocross();
  case MMR_MISSION_INSPECTION: return inspection();
  case MMR_MISSION_TRACKDRIVE: return trackdrive();
  case MMR_MISSION_ACCELERATION: return acceleration();
  case MMR_MISSION_FINISHED: return finished();
  default: return mission;
  }
}


static MmrMission manual() {
  *(asp.appsOut) = *(asp.appsIn);
  gs.ms = MMR_MANUAL_LAUNCH_Run(gs.ms);
  return MMR_MISSION_MANUAL;
}

static MmrMission skidpad() {
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);
  if (gs.as == MMR_AUTONOMOUS_LAUNCH_DONE) {
    *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  }

  if (gs.missionFinished) {
      
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_SKIDPAD;
}

static MmrMission ebsTest() {
  *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);

  if (gs.missionFinished) {
    MMR_EBS_Disarm();
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_EBS_TEST;
}

static MmrMission autocross() {
  *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);

  if (gs.missionFinished) {
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_AUTOCROSS;
}

static MmrMission inspection() {
  *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);

  if (gs.missionFinished) {
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_INSPECTION;
}

static MmrMission trackdrive() {
  *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);

  if (gs.missionFinished) {
    MMR_EBS_Disarm();
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_TRACKDRIVE;
}

static MmrMission acceleration() {
  *(asp.appsOut) = MMR_APPS_ComputeSpeed(gs.infoThrottle);
  gs.as = MMR_AUTONOMOUS_LAUNCH_Run(gs.as);

  if (gs.missionFinished) {
    MMR_EBS_Disarm();
    return MMR_MISSION_FINISHED;
  }

  return MMR_MISSION_ACCELERATION;
}

static MmrMission finished() {
  return MMR_MISSION_FINISHED;
}
