#include "inc/global_state.h"
#include <buffer.h>
#include <EBS.h>
#include <timing.h>
#include <delay.h>




struct MmrGlobalState {
  float infoSpeed;
  int16_t steeringAngle;
  uint8_t lap;
  uint16_t ath;//<-- primo valore farfalla
  uint16_t ath2;//<-- secondo valore farfalla
  uint16_t gear;
  uint16_t rpm;
  uint16_t speed;
  uint8_t uThrottle;
  uint8_t uThrottleB;
  uint16_t ebs1;
  uint16_t ebs2;
  uint16_t brakePf;
  uint16_t brakePr;

  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


static uint8_t __GoSignal;
static uint8_t __MissionFinished;
static uint8_t __MissionReady;
static uint8_t __AsbCheck;
static MmrMission __CurrentMissionSelected = MMR_MISSION_IDLE;
static uint8_t __ResEmergencyButton = 0;

/**/

static uint8_t stateAS_IDLE = 0;
static uint8_t stateAS_OFF = 0;
static uint8_t stateAS_DRIVING = 0;
static uint8_t stateAS_READY = 0;
static uint8_t stateAS_EMERGENCY = 0;
static uint8_t stateAS_FINISHED = 0;



/**/


/*EXTERNAL VARIABLES TO SEND PERIODICALLY*/
extern MmrAsState as_state;
extern EbsStates Ebsstate;
extern ebsflag EBSflag;
extern uint32_t dac;
extern asStatus R2D;
extern asStatus CHECK_ASB_STATE;
extern uint8_t TS_EBS;
/**/



void MMR_GS_Init() {
  __state = (struct MmrGlobalState){
    .lap = 0,
    .clutch = MMR_CLUTCH_UNKNOWN,
    .launchControl = MMR_LAUNCH_CONTROL_UNKNOWN,
  };
}


void MMR_GS_UpdateFromCan(MmrCan *can) {
  static MmrCanBuffer buffer = {};
  static MmrCanMessage msg = {
    .payload = buffer,
  };

  if (MMR_CAN_ReceiveAsync(can, &msg) == MMR_TASK_COMPLETED) {
    MmrCanHeader header = MMR_CAN_MESSAGE_GetHeader(&msg);
    switch (header.messageId) {

    case MMR_CAN_MESSAGE_ID_ECU_PEDAL_THROTTLE:
      __state.uThrottle = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
      __state.uThrottleB = MMR_BUFFER_ReadUint16(buffer, 6, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN1:
      __state.rpm = MMR_BUFFER_ReadUint16(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
      __state.speed = MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN)/100;
      __state.gear = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
      __state.ath = 100*MMR_BUFFER_ReadUint16(buffer, 6, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2:
      __state.launchControl = MMR_BUFFER_ReadByte(buffer, 6) == 0x1
        ? MMR_LAUNCH_CONTROL_SET
        : MMR_LAUNCH_CONTROL_NOT_SET;

      break;

    case MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL_OK:
      __state.clutch = MMR_CLUTCH_PULLED;
      break;

    case MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK:
      __state.clutch = MMR_CLUTCH_RELEASED;
      break;

    case MMR_CAN_MESSAGE_ID_D_SPEED_TARGET:
      __state.infoSpeed = *(float*)(buffer);
      break;

    case MMR_CAN_MESSAGE_ID_D_LAP_COUNTER:
      __state.lap = *(uint8_t *)(buffer);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_BRAKE_PRESSURES:
    	__state.brakePf = (0.005f/100)*MMR_BUFFER_ReadUint16(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
    	__state.brakePr = (0.005f/100)*MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_EBS_PRESSURES:
    	__state.ebs1 = MMR_BUFFER_ReadUint32(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
    	__state.ebs2 = MMR_BUFFER_ReadUint32(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      break;
    case MMR_CAN_MESSAGES_ID_GO_SIGNAL:
    	__GoSignal = (uint8_t )(buffer);
    	break;
    case MMR_CAN_MESSAGES_ID_MISSION_READY:
    	__MissionReady = (uint8_t )(buffer);
        break;
    case MMR_CAN_MESSAGES_ID_MISSION_FINISHED:
    	__MissionFinished = (uint8_t )(buffer);

    	break;
    case MMR_CAN_MESSAGE_ID_ASB_CHECK:
    	__AsbCheck = (uint8_t )(buffer);
    	break;
    case MMR_CAN_MESSAGE_ID_M_MISSION_SELECTED:
    	__CurrentMissionSelected = (MmrMission )buffer;
    	break;

    case MMR_CAN_MESSAGES_ID_RES_EMERGENCY:
    	__ResEmergencyButton = (uint8_t)(buffer);
    	break;

    }

  }
}


void MMR_GS_SendByCan(MmrCan* can)
{

static MmrDelay Txdelay = { .ms = 250 };

static  MmrCanMessage R2DmexCAN, EBSflagmexCAN, TS_EBSmexCAN, APPSmexCAN,
		              AS_OFFmexCAN, AS_DRIVINGmexCAN, AS_FINISHEDmexCAN,
		              AS_EMERGENCYmexCAN, AS_READYmexCAN = {};
//
//static MmrCanBuffer R2Dbuffer, EBSflagbuffer, TSEBSbuffer, APPSbuffer,
//                    AS_OFFbuffer, AS_READYbuffer, AS_DRIVINGbuffer,
//					AS_EMERGENCYbuffer, AS_FINISHEDbuffer = {};


if( MMR_DELAY_WaitAsync(&Txdelay))
{
MMR_CAN_MESSAGE_SetId(&R2DmexCAN, MMR_CAN_MESSAGE_ID_AS_R2D);
MMR_CAN_MESSAGE_SetStandardId(&R2DmexCAN, true);
MMR_CAN_MESSAGE_SetPayload(&R2DmexCAN, &R2D, sizeof(R2D));
MMR_CAN_Send(can, &R2DmexCAN);

MMR_CAN_MESSAGE_SetId(&EBSflagmexCAN, MMR_CAN_MESSAGES_ID_EBS_FLAG);
MMR_CAN_MESSAGE_SetStandardId(&EBSflagmexCAN, true);
MMR_CAN_MESSAGE_SetPayload(&EBSflagmexCAN, &EBSflag, sizeof(EBSflag));
MMR_CAN_Send(can, &EBSflagmexCAN);

MMR_CAN_MESSAGE_SetId(&TS_EBSmexCAN, MMR_CAN_MESSAGE_ID_TS_EBS);
MMR_CAN_MESSAGE_SetStandardId(&TS_EBSmexCAN, true);
MMR_CAN_MESSAGE_SetPayload(&TS_EBSmexCAN, &TS_EBS, sizeof(TS_EBS));
MMR_CAN_Send(can, &TS_EBSmexCAN);

MMR_CAN_MESSAGE_SetId(&APPSmexCAN, MMR_CAN_MESSAGE_ID_S_APPS);
MMR_CAN_MESSAGE_SetStandardId(&R2DmexCAN, true);
MMR_CAN_MESSAGE_SetPayload(&APPSmexCAN, &dac, sizeof(dac));
MMR_CAN_Send(can, &APPSmexCAN);

MMR_CAN_MESSAGE_SetId(&AS_OFFmexCAN,   MMR_CAN_MESSAGE_ID_AS_OFF);
MMR_CAN_MESSAGE_SetStandardId(&AS_OFFmexCAN, true);
MMR_AS_asConversion(as_state);
MMR_CAN_MESSAGE_SetPayload(&AS_OFFmexCAN, &stateAS_OFF, sizeof(stateAS_OFF));
MMR_CAN_Send(can, &AS_OFFmexCAN);

MMR_CAN_MESSAGE_SetId(&AS_DRIVINGmexCAN, MMR_CAN_MESSAGE_ID_AS_DRIVING);
MMR_CAN_MESSAGE_SetStandardId(&AS_DRIVINGmexCAN, true);
MMR_AS_asConversion(as_state);
MMR_CAN_MESSAGE_SetPayload(&AS_DRIVINGmexCAN, &stateAS_DRIVING, sizeof(stateAS_DRIVING));
MMR_CAN_Send(can, &AS_DRIVINGmexCAN);

MMR_CAN_MESSAGE_SetId(&AS_READYmexCAN,   MMR_CAN_MESSAGE_ID_AS_READY);
MMR_CAN_MESSAGE_SetStandardId(&AS_READYmexCAN, true);
MMR_AS_asConversion(as_state);
MMR_CAN_MESSAGE_SetPayload(&AS_READYmexCAN, &stateAS_READY, sizeof(stateAS_READY));
MMR_CAN_Send(can, &AS_READYmexCAN);

MMR_CAN_MESSAGE_SetId(&AS_EMERGENCYmexCAN,   MMR_CAN_MESSAGE_ID_AS_EMERGENCY);
MMR_CAN_MESSAGE_SetStandardId(&AS_EMERGENCYmexCAN, true);
MMR_AS_asConversion(as_state);
MMR_CAN_MESSAGE_SetPayload(&AS_EMERGENCYmexCAN, &stateAS_EMERGENCY, sizeof(stateAS_EMERGENCY));
MMR_CAN_Send(can, &AS_EMERGENCYmexCAN);

MMR_CAN_MESSAGE_SetId(&AS_FINISHEDmexCAN,   MMR_CAN_MESSAGE_ID_AS_FINISHED);
MMR_CAN_MESSAGE_SetStandardId(&AS_FINISHEDmexCAN, true);
MMR_AS_asConversion(as_state);
MMR_CAN_MESSAGE_SetPayload(&AS_FINISHEDmexCAN, &AS_FINISHEDmexCAN, sizeof(AS_FINISHEDmexCAN));
MMR_CAN_Send(can, &AS_FINISHEDmexCAN);

}

}
MmrClutchState MMR_GS_GetClutchState() {
  return __state.clutch;
}

MmrLaunchControlState MMR_GS_GetLaunchControlState() {
  return __state.launchControl;
}

uint16_t MMR_GS_GetGear() {
  return __state.gear;
}

uint16_t MMR_GS_GetRpm() {
  return __state.rpm;
}

uint16_t MMR_GS_GetSpeed() {
  return __state.speed;
}

int16_t MMR_GS_GetSteeringAngle() {
  return __state.steeringAngle;
}

void MMR_GS_SetLap(uint8_t lap) {
  __state.lap = lap;
}

uint8_t MMR_GS_GetLap() {
  return __state.lap;
}

uint16_t MMR_GS_GetAth() {
  return __state.ath;
}

uint8_t MMR_GS_GetUThrottle() {
  return __state.uThrottle;
}

uint8_t MMR_GS_GetUThrottleB() {
  return __state.uThrottleB;
}

uint16_t MMR_GS_GetEbs1() {
  return __state.ebs1;
}

uint16_t MMR_GS_GetEbs2() {
  return __state.ebs2;
}

uint16_t MMR_GS_GetBreakPf() {
return __state.brakePf;
}

uint16_t MMR_GS_GetBreakPr() {
	return __state.brakePr;
}

uint32_t MMR_GS_GetInfoSpeed() {
  return __state.infoSpeed;
}

uint8_t MMR_AS_MissionReady(){
return __MissionReady;
}


uint8_t MMR_AS_MissionFinished(){
  return __MissionFinished;
}


uint8_t MMR_AS_Go_Signal(){
    return __GoSignal;
}

uint8_t MMR_AS_ASB_Check(){
	return __AsbCheck;
}

MmrMission MMR_AS_GetMission(){
	return __CurrentMissionSelected;
}

void MMR_AS_asConversion(MmrAsState state){
switch(state)
{
case AS_IDLE:

	 stateAS_IDLE = 1;
	 stateAS_OFF = 0;
	 stateAS_DRIVING = 0;
	 stateAS_READY = 0;
	 stateAS_EMERGENCY = 0;
	 stateAS_FINISHED = 0;
	 break;

case AS_OFF:
	 stateAS_IDLE = 0;
	 stateAS_OFF = 1;
	 stateAS_DRIVING = 0;
	 stateAS_READY = 0;
	 stateAS_EMERGENCY = 0;
	 stateAS_FINISHED = 0;
	 break;
case  AS_READY:
	 stateAS_IDLE = 0;
	 stateAS_OFF = 0;
	 stateAS_DRIVING = 0;
	 stateAS_READY = 1;
	 stateAS_EMERGENCY = 0;
	 stateAS_FINISHED = 0;
	 break;
case AS_DRIVING:
	 stateAS_IDLE = 0;
	 stateAS_OFF = 0;
	 stateAS_DRIVING = 1;
	 stateAS_READY = 0;
	 stateAS_EMERGENCY = 0;
	 stateAS_FINISHED = 0;
	 break;
case AS_EMERGENCY:
	 stateAS_IDLE = 0;
	 stateAS_OFF = 0;
	 stateAS_DRIVING = 0;
	 stateAS_READY = 0;
	 stateAS_EMERGENCY = 1;
	 stateAS_FINISHED = 0;
	 break;
case AS_FINISHED:
	 stateAS_IDLE = 0;
	 stateAS_OFF = 0;
	 stateAS_DRIVING = 0;
	 stateAS_READY = 0;
	 stateAS_EMERGENCY = 0;
	 stateAS_FINISHED = 1;
	 break;


}
}


uint8_t MMR_Get_AS_GetResEB()
{  return __ResEmergencyButton;
}
