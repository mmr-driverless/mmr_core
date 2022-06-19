#include "inc/manual.h"
#include "inc/launch_control.h"
#include "delay.h"
#include "message_id.h"

#include <stdint.h>
#include <stdbool.h>

static MmrManualState waiting(MmrManualState state);
static MmrManualState setLaunchControl(MmrManualState state);
static MmrManualState stopLaunchControl(MmrManualState state);
static MmrManualState done(MmrManualState state);

static MmrCan *__can;

void MMR_MANUAL_Init(MmrCan *can) {
  __can = can;
}

static bool waitMs(bool reset, int ms) {
  static int start = 0;
  if (reset) {
    start = uwTick;
  }

  return uwTick - start >= ms;
}

MmrAutonomousState MMR_Manual_Run(MmrManualState state) {
  switch (state) {
  case MMR_MANUAL_WAITING: return waiting(state);
  case MMR_MANUAL_SET_LAUNCH_CONTROL: return setLaunchControl(state);
  case MMR_MANUAL_STOP: return start(state);
  case MMR_MANUAL_DONE: return done(state);
  default: return state;
  }
}


static MmrManualState waiting(MmrManualState state) {
	if (isClutchPulled) {
		  sendLaunch = true;
	  }
  return MMR_MANUAL_SET_LAUNCH_CONTROL;
}

static MmrManualState setLaunchControl(MmrManualState state) {
	static MmrDelay delay = MMR_Delay(5);
		int isLaunchControlSetCnt = 0;
		static uint8_t readLaunch(CanRxBuffer buffer) {
		  int bytes7_8 = (buffer[7] << 8 | buffer[6]);
		  int isLaunchControlBitHigh = (bytes7_8 & 0x8) == 0x8;

		  return isLaunchControlBitHigh ? +1 : -1;
		}
		static MmrCanPacket launchControl = {
		    .header = MMR_CAN_HEADER_FromBits(0x628),
		  };

		if (nMot > 1000 && sendLaunch) {
		      if (isLaunchControlSetCnt >= 2 || isLaunchControlSetCnt <= -2) {
		        sendLaunch = false;
		      }
		      status = MMR_CAN_SendNoTamper(&hcan, launchControl);
		    }
		return MMR_AUTONOMOUS_CHANGE_GEAR;
	}
  return MMR_MANUAL_STOP;
}

static MmrManualState stopLaunchControl(MmrManualState state) {

	if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK) {

		        clutchMsgStart = uwTick;
		        launchControlStop = uwTick;
		        clutchMsgCnt = 0;
		        launchControlCnt = 0;
		        isLaunchControlSetCnt = 0;
		      }
		if (clutchMsgCnt < 5 && uwTick - clutchMsgStart > 5) {
			        clutchMsgStart = uwTick;
			        clutchMsgCnt++;
			      }

		if (clutchMsgCnt >= 5) {
			const uint16_t DAC_0 = 500;
		}
  return MMR_MANUAL_DONE;
}

static MmrManualState done(MmrManualState state) {
  // FERMA TUTTO CHE SEMBRA UNA SAGRA
}
