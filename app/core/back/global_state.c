#include "inc/global_state.h"
#include <buffer.h>
#include <timing.h>
#include <delay.h>
#include <stdint.h>
#include <EBS.h>


MmrGlobalState gs = {};


void MMR_GS_Init() {
  gs = (struct MmrGlobalState){
    .can = {
      .lap = 0,
      .clutch = MMR_CLUTCH_UNKNOWN,
      .launchControl = MMR_LAUNCH_CONTROL_UNKNOWN,
      .currentMissionSelected = MMR_MISSION_IDLE,
      .resEmergencyButton = MMR_BUTTON_RELEASED,
    },
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
      gs.uThrottle = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
      gs.uThrottleB = MMR_BUFFER_ReadUint16(buffer, 6, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN1:
      gs.rpm = MMR_BUFFER_ReadUint16(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
      gs.speed = MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN) / 100;
      gs.gear = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
      gs.ath = 100 * MMR_BUFFER_ReadUint16(buffer, 6, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2:
      gs.launchControl = MMR_BUFFER_ReadByte(buffer, 6) == 0x1
        ? MMR_LAUNCH_CONTROL_SET
        : MMR_LAUNCH_CONTROL_NOT_SET;

      break;

    case MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL_OK:
      gs.clutch = MMR_CLUTCH_PULLED;
      break;

    case MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK:
      gs.clutch = MMR_CLUTCH_RELEASED;
      break;

    case MMR_CAN_MESSAGE_ID_D_SPEED_TARGET:
      gs.infoSpeed = *(float*)(buffer);
      break;

    case MMR_CAN_MESSAGE_ID_D_LAP_COUNTER:
      gs.lap = MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_BRAKE_PRESSURES:
      gs.brakePf = (0.005f / 100) * MMR_BUFFER_ReadUint16(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
      gs.brakePr = (0.005f / 100) * MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGE_ID_ECU_EBS_PRESSURES:
      gs.ebs1 = MMR_BUFFER_ReadUint32(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
      gs.ebs2 = MMR_BUFFER_ReadUint32(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      break;

    case MMR_CAN_MESSAGES_ID_GO_SIGNAL:
      gs.goSignal = MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGES_ID_MISSION_READY:
      gs.missionReady = MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGES_ID_MISSION_FINISHED:
      gs.missionFinished = MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGE_ID_ASB_CHECK:
      gs.asbCheck = MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGE_ID_M_MISSION_SELECTED:
      gs.currentMissionSelected = (MmrMission)MMR_BUFFER_ReadByte(buffer, 0);
      break;

    case MMR_CAN_MESSAGES_ID_RES_EMERGENCY:
      gs.resEmergencyButton = MMR_BUFFER_ReadByte(buffer, 0) == 0
        ? MMR_BUTTON_RELEASED
        : MMR_BUTTON_PRESSED;

      break;
    }
  }
}


void MMR_GS_SendByCan(MmrCan* can) {
  static MmrDelay messageBatchDelay = { .ms = 250 };
  static MmrDelay betweenMessagesDelay = { .ms = 1 };
  static bool isSending = false;


  if (!isSending && MMR_DELAY_WaitAsync(&messageBatchDelay)) {
    isSending = true;
  }

  if (isSending) {
//    MMR_CAN_MESSAGE_SetId(&TS_EBSmexCAN, MMR_CAN_MESSAGE_ID_TS_EBS);
//    MMR_CAN_MESSAGE_SetStandardId(&TS_EBSmexCAN, true);
//    MMR_CAN_MESSAGE_SetPayload(&TS_EBSmexCAN, &TS_EBS, sizeof(TS_EBS));
//    MMR_CAN_Send(can, &TS_EBSmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&APPSmexCAN, MMR_CAN_MESSAGE_ID_S_APPS);
//    MMR_CAN_MESSAGE_SetStandardId(&R2DmexCAN, true);
//    MMR_CAN_MESSAGE_SetPayload(&APPSmexCAN, &dac, sizeof(dac));
//    MMR_CAN_Send(can, &APPSmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&AS_OFFmexCAN,   MMR_CAN_MESSAGE_ID_AS_OFF);
//    MMR_CAN_MESSAGE_SetStandardId(&AS_OFFmexCAN, true);
//    MMR_AS_asConversion(as_state);
//    MMR_CAN_MESSAGE_SetPayload(&AS_OFFmexCAN, &stateAS_OFF, sizeof(stateAS_OFF));
//    MMR_CAN_Send(can, &AS_OFFmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&AS_DRIVINGmexCAN, MMR_CAN_MESSAGE_ID_AS_DRIVING);
//    MMR_CAN_MESSAGE_SetStandardId(&AS_DRIVINGmexCAN, true);
//    MMR_AS_asConversion(as_state);
//    MMR_CAN_MESSAGE_SetPayload(&AS_DRIVINGmexCAN, &stateAS_DRIVING, sizeof(stateAS_DRIVING));
//    MMR_CAN_Send(can, &AS_DRIVINGmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&AS_READYmexCAN,   MMR_CAN_MESSAGE_ID_AS_READY);
//    MMR_CAN_MESSAGE_SetStandardId(&AS_READYmexCAN, true);
//    MMR_AS_asConversion(as_state);
//    MMR_CAN_MESSAGE_SetPayload(&AS_READYmexCAN, &stateAS_READY, sizeof(stateAS_READY));
//    MMR_CAN_Send(can, &AS_READYmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&AS_EMERGENCYmexCAN,   MMR_CAN_MESSAGE_ID_AS_EMERGENCY);
//    MMR_CAN_MESSAGE_SetStandardId(&AS_EMERGENCYmexCAN, true);
//    MMR_AS_asConversion(as_state);
//    MMR_CAN_MESSAGE_SetPayload(&AS_EMERGENCYmexCAN, &stateAS_EMERGENCY, sizeof(stateAS_EMERGENCY));
//    MMR_CAN_Send(can, &AS_EMERGENCYmexCAN);
//
//    MMR_CAN_MESSAGE_SetId(&AS_FINISHEDmexCAN,   MMR_CAN_MESSAGE_ID_AS_FINISHED);
//    MMR_CAN_MESSAGE_SetStandardId(&AS_FINISHEDmexCAN, true);
//    MMR_AS_asConversion(as_state);
//    MMR_CAN_MESSAGE_SetPayload(&AS_FINISHEDmexCAN, &AS_FINISHEDmexCAN, sizeof(AS_FINISHEDmexCAN));
//    MMR_CAN_Send(can, &AS_FINISHEDmexCAN);
  }
}
