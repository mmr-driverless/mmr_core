#include "inc/global_state.h"
#include <buffer.h>


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
  uint16_t brkp1;
  uint16_t brkp2;

  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


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
      __state.speed = MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      __state.gear = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
      __state.ath = MMR_BUFFER_ReadUint16(buffer, 6, MMR_ENCODING_LITTLE_ENDIAN);
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
      //da aggiungere lettura messaggi can sensori di pressione
      break;

    case MMR_CAN_MESSAGE_ID_ECU_EBS_PRESSURES:
      //da aggiungere letura messaggi can ebs
      break;
    }
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

uint16_t MMR_GS_GetBreakP1() {

}

uint16_t MMR_GS_GetBreakP2() {

}

uint32_t MMR_GS_GetInfoSpeed() {
  return __state.infoSpeed;
}
