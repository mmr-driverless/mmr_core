#include "inc/gear_change.h"
#include "inc/autonomous.h"
#include "inc/manual.h"
#include "inc/as.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <stdint.h>
#include <stdbool.h>


#define APPS_offset 650
#define APPS_slope 716

static bool handlePreStart(MmrLaunchControlMode *mode);


struct MmrLaunchControl {
  float infoSpeed;
  int16_t steeringAngle;
  uint8_t lap;
  uint16_t ath; //<--farfalla
  uint16_t ath2;
  uint16_t gear;
  uint16_t rpm;
  uint16_t speed;
  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


static MmrCan *__can;
static MmrPin *__gearN;
static MmrPin *__gearUp;
static MmrPin *__gearDown;
static MmrButton __changeModeButton;
static uint32_t *__apps;
static uint32_t *__adc;
static MmrMission __mission;

static MmrAutonomousState as = MMR_AUTONOMOUS_WAITING;
static MmrManualState ms = MMR_MANUAL_WAITING;


void MMR_AS_Init(
  MmrCan *can,
  MmrPin *gearUp,
  MmrPin *gearDown,
  MmrPin *gearN,
  MmrPin *changeMode,
  uint32_t *apps,
  uint32_t *adc
) {
  __can = can;
  __gearN = gearN;
  __gearUp = gearUp;
  __gearDown = gearDown;
  __changeModeButton = MMR_Button(changeMode);
  __state = (struct MmrLaunchControl){
    .lap = 1,
    .clutch = MMR_CLUTCH_UNKNOWN,
    .launchControl = MMR_LAUNCH_CONTROL_UNKNOWN,
  };

  __apps = apps;
  __adc = adc;

  MMR_MANUAL_Init(can, apps, adc);
  MMR_GEAR_CHANGE_Init(gearUp, gearDown);
  MMR_AUTONOMOUS_Init(can, gearN, apps, adc);
}

MmrLaunchControlMode MMR_AS_Run(MmrLaunchControlMode mode) {
  static MmrCanBuffer buffer = {};
  static MmrCanMessage msg = {
    .payload = buffer,
  };

  if (MMR_CAN_ReceiveAsync(__can, &msg) == MMR_TASK_COMPLETED) {
    MmrCanHeader header = MMR_CAN_MESSAGE_GetHeader(&msg);

    if (header.messageId < 192 || header.messageId > 199) {
    	;
    }

    switch (header.messageId) {
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

    case MMR_CAN_MESSAGE_ID_D_ACCELERATOR_PERCENTAGE:
      __state.infoSpeed = *(float*)(buffer);
      break;

    case MMR_CAN_MESSAGE_ID_D_LAP_COUNTER:
     	__state.lap = *(uint8_t*)(buffer);
     	break;

    case MMR_CAN_MESSAGE_ID_MISSION_SELECTED:
    	__mission = *(MmrMission*)(buffer);
      break;
    }
  }

  bool canStart = handlePreStart(&mode);
  if (!canStart) {
    as = MMR_AUTONOMOUS_WAITING;
    ms = MMR_MANUAL_WAITING;
    return mode;
  }

  switch (mode) {
  case MMR_AS_MODE_MANUAL:
    MMR_AS_SetLap(1);
    ms = MMR_MANUAL_Run(ms);
    break;

  case MMR_AS_MODE_GEAR_CHANGE:
    MMR_AS_SetLap(2);
    MMR_GEAR_CHANGE_Run();
    ms = MMR_MANUAL_Run(ms);
    break;

  case MMR_AS_MODE_AUTONOMOUS:
    MMR_AS_SetLap(1);
    as = MMR_AUTONOMOUS_Run(as);
    break;
  }

  return mode;
}


static bool handlePreStart(MmrLaunchControlMode *mode) {
  static bool waitForStart = true;
  static MmrDelay waitForStartDelay = { .ms = 10000 };

  if (MMR_BUTTON_Read(&__changeModeButton) == MMR_BUTTON_JUST_RELEASED) {
    waitForStart = true;
    MMR_DELAY_Reset(&waitForStartDelay);
    *mode += 1;
    *mode %= 3;
    return false;
  }


  bool isInAutonomous = *mode == MMR_AS_MODE_AUTONOMOUS;
  bool isClutchPulled = MMR_AS_GetClutchState() == MMR_CLUTCH_PULLED;

  if (isInAutonomous && isClutchPulled && waitForStart) {
    waitForStart = true;
    MMR_DELAY_Reset(&waitForStartDelay);
    *mode = MMR_AS_MODE_MANUAL;
  }

  bool canStart = waitForStart && MMR_DELAY_WaitAsync(&waitForStartDelay);
  if (canStart) {
    waitForStart = false;
  }

  return !waitForStart;
}


MmrClutchState MMR_AS_GetClutchState() {
  return __state.clutch;
}

MmrLaunchControlState MMR_AS_GetLaunchControlState() {
  return __state.launchControl;
}

uint16_t MMR_AS_GetGear() {
  return __state.gear;
}

uint16_t MMR_AS_GetRpm() {
  return __state.rpm;
}

uint16_t MMR_AS_GetSpeed() {
  return __state.speed;
}

int16_t MMR_AS_GetSteeringAngle() {
  return __state.steeringAngle;
}

void MMR_AS_SetLap(uint8_t lap) {
  __state.lap = lap;
}

uint8_t MMR_AS_GetLap() {
  return __state.lap;
}

uint16_t MMR_AS_GetAth() {
  return __state.ath;
}

uint32_t MMR_AS_GetInfoSpeed() {
  return APPS_slope * (__state.infoSpeed) + APPS_offset;
}


MmrMission MMR_AS_GetMission(){
	return __mission;
}
