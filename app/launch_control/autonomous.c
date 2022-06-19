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

static bool waitMs(bool reset, int ms) {
  static int start = 0;
  if (reset) {
    start = uwTick;
  }

  return uwTick - start >= ms;
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
	  sendLaunch = true;
  }

  return MMR_AUTONOMOUS_SET_LAUNCH_CONTROL;
}

static MmrAutonomousState setLaunchControl(MmrAutonomousState state) {
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

static MmrAutonomousState setGear(MmrAutonomousState state){
	static MmrDelay delay = MMR_Delay(5);
	if (gear) {
	        changeGear(true);
	        HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
	       //GEAR_CHANGED;
	             waitMs(true, 1000);
	      }
	      if (!gear || !changeGear(false)) {
	        waitMs(true, 2000);
	        // GEAR_NOT_CHANGED;
	        if (waitMs(false, 2000)) {
	                HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
	                // GEAR_NOT_SET;
	              }
	        break;
	      }
	  	return  MMR_AUTONOMOUS_ACCELERATE;
}

static MmrAutonomousState accelerate(MmrAutonomousState state){
	static MmrDelay delay = MMR_Delay(5);
	const uint16_t DAC_30 = 865;
	if (waitMs(false, 1000)) {
		dacValue = DAC_30;
		{
	return MMR_AUTONOMOUS_RELEASE_CLUCTH;
}
static MmrAutonomousState releaseClutch(MmrAutonomousState state);{
	  static MmrDelay delay = MMR_Delay(5);
	 if (uwTick - clutchMsgStart > 1) {
	        status = MMR_CAN_Send(&hcan, clutchRelease);
	        clutchMsgStart = uwTick;

	        if (clutchMsgCnt++ >= 5 && nMot >= 6000) {
	        	return MMR_AUTONOMOUS_START;
	        }
	      }
}
static MmrAutonomousState start(MmrAutonomousState state){
	  static MmrDelay delay = MMR_Delay(5);
	if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK) {

	        clutchMsgStart = uwTick;
	        launchControlStop = uwTick;
	        clutchMsgCnt = 0;
	        launchControlCnt = 0;
	        isLaunchControlSetCnt = 0;
	      }
	if (clutchMsgCnt < 5 && uwTick - clutchMsgStart > 5) {
		        status = MMR_CAN_Send(&hcan, clutchSetManual);
		        clutchMsgStart = uwTick;
		        clutchMsgCnt++;
		      }

	if (clutchMsgCnt >= 5) {
		const uint16_t DAC_0 = 500;
	}
	return MMR_AUTONOMOUS_DONE;
}
static MmrAutonomousState done(MmrAutonomousState state){
	  static MmrDelay delay = MMR_Delay(5);
		// VAMONOS CARLOS
}


