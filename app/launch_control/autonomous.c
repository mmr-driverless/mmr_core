#include "inc/autonomous.h"
#include "inc/launch_control.h"
#include "delay.h"
#include "message_id.h"

#include <stdint.h>
#include <stdbool.h>

static MmrAutonomousState waiting(MmrAutonomousState state);
static MmrAutonomousState pullClutch(MmrAutonomousState state);
static MmrAutonomousState setLaunchControl(MmrAutonomousState state);
static MmrAutonomousState setGear(MmrAutonomousState state);
static MmrAutonomousState accelerate(MmrAutonomousState state);
static MmrAutonomousState releaseClutch(MmrAutonomousState state);
static MmrAutonomousState start(MmrAutonomousState state);
static MmrAutonomousState done(MmrAutonomousState state);


static MmrCan *__can;


void MMR_AUTONOMOUS_Init(MmrCan *can) {
  __can = can;
}

MmrAutonomousState MMR_AUTONOMOUS_Run(MmrAutonomousState state) {
  switch (state) {
  case MMR_AUTONOMOUS_WAITING: return waiting(state);
  case MMR_AUTONOMOUS_PULL_CLUTCH: return pullClutch(state);
  case MMR_AUTONOMOUS_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_AUTONOMOUS_CHANGE_GEAR: return changeGear(state);
  case MMR_AUTONOMOUS_ACCELERATE: return accelerate(state);
  case MMR_AUTONOMOUS_RELEASE_CLUCTH: return releaseClutch(state);
  case MMR_AUTONOMOUS_START: return start(state);
  case MMR_AUTONOMOUS_DONE: return done(state);
  default: return state;
  }
}


static MmrAutonomousState waiting(MmrAutonomousState state) {
  return MMR_AUTONOMOUS_PULL_CLUTCH;
}

static MmrAutonomousState pullClutch(MmrAutonomousState state) {
  static MmrDelay delay = MMR_Delay(5);
  static MmrCanPacket clutchPullPacket = {
    .header.messageId = MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL,
  };

  bool canPullClutch = MMR_LAUNCH_CONTROL_GetNmot() >= 1000;
  bool isClutchPulled = MMR_LAUNCH_CONTROL_GetClutchState() == MMR_CLUTCH_PULLED;

  if (canPullClutch && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(clutchPullPacket);
  }

  if (isClutchPulled) {
    return MMR_AUTONOMOUS_SET_LAUNCH_CONTROL;
  }

  return state;
}

static MmrAutonomousState setLaunchControl(MmrAutonomousState state) {

}

static MmrAutonomousState setGear(MmrAutonomousState state);
static MmrAutonomousState accelerate(MmrAutonomousState state);
static MmrAutonomousState releaseClutch(MmrAutonomousState state);
static MmrAutonomousState start(MmrAutonomousState state);
static MmrAutonomousState done(MmrAutonomousState state);

