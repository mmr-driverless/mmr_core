#include "../back/inc/global_state.h"
#include "inc/net.h"
#include <delay.h>


bool MMR_NET_PullClutchAsync(MmrCan *can) {
  static MmrDelay delay = { .ms = 5 };

  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL);
  MmrCanMessage clutchPullMsg = MMR_CAN_OutMessage(header);

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(can, &clutchPullMsg);
  }

  return gs.clutch == MMR_CLUTCH_PULLED;
}

bool MMR_NET_ReleaseClutchAsync(MmrCan *can) {
  static MmrDelay delay = { .ms = 5 };
  
  MmrCanHeader header = MMR_CAN_ScsHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE);
  MmrCanMessage clutchReleaseMsg = MMR_CAN_OutMessage(header);

  if (MMR_DELAY_WaitAsync(&delay)) {
    MMR_CAN_Send(can, &clutchReleaseMsg);
  }

  return gs.clutch == MMR_CLUTCH_RELEASED;
}  