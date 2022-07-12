#include "../back/inc/global_state.h"
#include "inc/net.h"

bool MMR_NET_LaunchControlSetAsync(MmrCan *can) {
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanMessage setLaunchMsg = {};
  MMR_CAN_MESSAGE_SetId(&setLaunchMsg, 0x628);
  MMR_CAN_MESSAGE_SetStandardId(&setLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&setLaunchMsg, buffer, 8);

  bool rpmOk = gs.rpm >= 1000;
  bool isLaunchSet = gs.launchControl == MMR_LAUNCH_CONTROL_SET;
  bool isInFirstGear = gs.gear == 1;

  if (rpmOk && isInFirstGear && !isLaunchSet && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(can, &setLaunchMsg);
  }

  return isLaunchSet;
}

bool MMR_NET_LaunchControlUnsetAsync(MmrCan *can) {
  static MmrDelay delay = { .ms = 250 };
  static MmrCanBuffer buffer = { 0x8 };

  MmrCanHeader header = MMR_CAN_HEADER_FromBits(0x628);
  MmrCanMessage unsetLaunchMsg = MMR_CAN_OutMessage(header);
  MMR_CAN_MESSAGE_SetStandardId(&unsetLaunchMsg, true);
  MMR_CAN_MESSAGE_SetPayload(&unsetLaunchMsg, buffer, 8);

  bool isClutchReleased = gs.clutch == MMR_CLUTCH_RELEASED;
  bool isLaunchUnset = gs.launchControl == MMR_LAUNCH_CONTROL_NOT_SET;

  if (isClutchReleased && !isLaunchUnset && MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(can, &unsetLaunchMsg);
  }

  return isLaunchUnset;
}
