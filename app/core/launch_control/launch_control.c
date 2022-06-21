#include "inc/launch_control.h"
#include "inc/autonomous.h"
#include "inc/manual.h"
#include <button.h>
#include <buffer.h>
#include <delay.h>
#include <can.h>
#include <stdint.h>
#include <stdbool.h>

static bool handlePreStart(MmrLaunchControlMode *mode);


struct MmrLaunchControl {
  uint16_t gear;
  uint16_t rpm;
  uint16_t speed;
  MmrClutchState clutch;
  MmrLaunchControlState launchControl;
} __state = {};


static MmrCan *__can;
static MmrPin *__gearDown;
static MmrButton __changeModeButton;


void MMR_LAUNCH_CONTROL_Init(MmrCan *can, MmrPin *gearDown, MmrPin *changeMode, uint32_t *apps) {
  __can = can;
  __gearDown = gearDown;
  __changeModeButton = MMR_Button(changeMode);
  __state = (struct MmrLaunchControl){
    .clutch = MMR_CLUTCH_UNKNOWN,
    .launchControl = MMR_LAUNCH_CONTROL_UNKNOWN,
  };

  MMR_AUTONOMOUS_Init(can, gearDown, apps);
}

MmrLaunchControlMode MMR_LAUNCH_CONTROL_Run(MmrLaunchControlMode mode) {
  static MmrAutonomousState as = MMR_AUTONOMOUS_WAITING;
  static MmrManualState ms = MMR_MANUAL_WAITING;
  static MmrCanBuffer buffer = {};
  static MmrCanMessage msg = {
    .payload = buffer,
  };

  if (MMR_CAN_ReceiveAsync(__can, &msg) == MMR_TASK_COMPLETED) {
    MmrCanHeader header = MMR_CAN_MESSAGE_GetHeader(&msg);
    switch (header.messageId) {
    case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN1:
      __state.rpm = MMR_BUFFER_ReadUint16(buffer, 0, MMR_ENCODING_LITTLE_ENDIAN);
      __state.speed = MMR_BUFFER_ReadUint16(buffer, 2, MMR_ENCODING_LITTLE_ENDIAN);
      __state.gear = MMR_BUFFER_ReadUint16(buffer, 4, MMR_ENCODING_LITTLE_ENDIAN);
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
    }
  }

  bool canStart = handlePreStart(&mode);
  if (!canStart) {
    return mode;
  }

  switch (mode) {
  case MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS: as = MMR_AUTONOMOUS_Run(as); break;
  case MMR_LAUNCH_CONTROL_MODE_MANUAL: ms = MMR_MANUAL_Run(ms); break;
  }

  return mode;
}


static bool handlePreStart(MmrLaunchControlMode *mode) {
  static bool waitForStart = true;
  static MmrDelay waitForStartDelay = { .ms = 10000 };

  if (MMR_BUTTON_Read(&__changeModeButton) == MMR_BUTTON_JUST_RELEASED) {
    waitForStart = true;
    MMR_DELAY_Reset(&waitForStartDelay);

    *mode = *mode == MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS
      ? MMR_LAUNCH_CONTROL_MODE_MANUAL
      : MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS;

    return false;
  }


  bool isInAutonomous = mode == MMR_LAUNCH_CONTROL_MODE_AUTONOMOUS;
  bool isClutchPulled = MMR_LAUNCH_CONTROL_GetClutchState() == MMR_CLUTCH_PULLED;

  if (isInAutonomous && isClutchPulled && waitForStart) {
    waitForStart = true;
    MMR_DELAY_Reset(&waitForStartDelay);
    *mode = MMR_LAUNCH_CONTROL_MODE_MANUAL;
  }

  bool canStart = waitForStart && MMR_DELAY_WaitAsync(&waitForStartDelay);
  if (canStart) {
    waitForStart = false;
    return true;
  }

  return false;
}


MmrClutchState MMR_LAUNCH_CONTROL_GetClutchState() {
  return __state.clutch;
}

MmrLaunchControlState MMR_LAUNCH_CONTROL_GetLaunchControlState() {
  return __state.launchControl;
}

uint16_t MMR_LAUNCH_CONTROL_GetGear() {
  return __state.gear;
}

uint16_t MMR_LAUNCH_CONTROL_GetRpm() {
  return __state.rpm;
}

uint8_t MMR_LAUNCH_CONTROL_GetSpeed() {
  return __state.speed;
}
