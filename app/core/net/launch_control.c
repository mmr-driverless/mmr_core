#include "../back/inc/global_state.h"
#include "inc/net.h"

bool MMR_NET_LaunchControlSetAsync(MmrCan *can) {
  static MmrDelay messagesDelay = { .ms = 250 };
  static MmrDelay phasesDelay = { .ms = 50 };
  static bool sendingFirst = false;

  static MmrCanBuffer launchSet = { 0x8 };
  static MmrCanBuffer launchReset = { 0x0 };

  bool rpmOk = gs.rpm >= 1000;
  bool isLaunchSet = gs.launchControl == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = gs.gear == 1;

  if (sendingFirst && rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&messagesDelay)) {
    if (sendingFirst && MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchSet, 8);
    }
    else if (MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchReset, 8);
    }

    sendingFirst = !sendingFirst;
  }

  return isLaunchSet;
}

bool MMR_NET_LaunchControlUnsetAsync(MmrCan *can) {
  static MmrDelay messagesDelay = { .ms = 250 };
  static MmrDelay phasesDelay = { .ms = 50 };
  static bool sendingFirst = false;

  static MmrCanBuffer launchSet = { 0x8 };
  static MmrCanBuffer launchReset = { 0x0 };

  if (MMR_DELAY_WaitAsync(&messagesDelay)) {
    if (sendingFirst && MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchSet, 8);
    }
    else if (MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchReset, 8);
    }

    sendingFirst = !sendingFirst;
  }

  bool isClutchReleased = gs.clutch == MMR_CLUTCH_RELEASED;
  bool isLaunchUnset = gs.launchControl == MMR_LAUNCH_CONTROL_NOT_SET;

  if (sendingFirst && isClutchReleased && !isLaunchUnset && MMR_DELAY_WaitAsync(&messagesDelay)) {
    if (sendingFirst && MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchSet, 8);
    }
    else if (MMR_DELAY_WaitAsync(&phasesDelay)) {
      MmrCanMessage setLaunchMsg = {};
      MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
      MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
      MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, launchReset, 8);
    }

    sendingFirst = !sendingFirst;
  }

  return isLaunchUnset;
}
