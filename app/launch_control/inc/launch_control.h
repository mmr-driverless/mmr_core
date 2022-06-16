#ifndef APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
#define APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_

typedef enum ClutchState {
  START,
  PULL,
  PULLED,
  GEAR_NOT_SET,
  GEAR_CHANGING,
  GEAR_CHANGED,
  GEAR_NOT_CHANGED,
  LAUNCH_CONTROL_SET,
  APPS_30,
  RELEASE,
  RELEASED,
  DONE,
} ClutchState;

typedef enum GearN {
  GEAR_N_OFF = GPIO_PIN_RESET,
  GEAR_N_ON = GPIO_PIN_SET,
} GearN;

static ClutchState state = START;
static HalStatus status = HAL_OK;


static bool changeGearStop = false;
static bool sendLaunch = false;
static bool sendLaunchM = false;
static bool isManual = true;
static int debounce = 0;
static bool firstrun=0;


static uint8_t changeGear(bool reset) {
  static int start = 0;
  if (reset)
    start = uwTick;

  if (changeGearStop || gear) {
    HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
    return true;
  }

  bool isWithin50ms = uwTick - start < 350;
  uint8_t out = isWithin50ms
    ? GEAR_N_ON
    : GEAR_N_OFF;

  HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, out);
  return out == GEAR_N_ON;
}



static bool waitMs(bool reset, int ms) {
  static int start = 0;
  if (reset) {
    start = uwTick;
  }

  return uwTick - start >= ms;
}

static uint16_t readGear(CanRxBuffer buffer) {
  return buffer[5] << 8 | buffer[4];
}

static uint16_t readRPM(CanRxBuffer buffer) {
  return buffer[1] << 8 | buffer[0];
}


typedef enum ButtonState {
  BTN_PRESSED,
  BTN_JUST_PRESSED,
  BTN_RELEASED,
  BTN_JUST_RELEASED,
} ButtonState;


static ButtonState readButton() {
  int samples = 0;
  //static tick = 0;
  static ButtonState state = BTN_RELEASED;
  static int preValue = 0;

  if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0 && debounce == 0) {
    if (preValue == 0) {
      preValue = 1;
      samples = 1;
    } else {
      preValue = 0;
      samples = 0;
    }

    debounce++;
  }

//  samples <<= 1;
//  samples = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

//  if (samples == 0xFF) {
//    state = state != BTN_RELEASED
//      ? BTN_JUST_RELEASED
//      : BTN_RELEASED;
//  }
//  else if (samples == 0x00) {
//    state = state != BTN_PRESSED
//      ? BTN_JUST_PRESSED
//      : BTN_PRESSED;
//  }

  if (samples == 1) {
    state = BTN_JUST_PRESSED;
  } else {
    state = BTN_JUST_RELEASED;
  }

  return state;
}


void MMR_LaunchControlStart() {
  if (readButton() == BTN_JUST_PRESSED) {
    isManual = !isManual;
    if (isManual) {
      for (int i = 0; i < 10; i++) {
        status = MMR_CAN_Send(&hcan, clutchSetManual);
        HAL_Delay(5);
      }
    }
    else {
      dacValue = DAC_0;
    }

    stateChangeStart = uwTick;
    waitForStateChange = true;
    state = START;
  }

  if (waitForStateChange && uwTick - stateChangeStart < 10000) {
    continue;
  }


  if (nMot > 1000 && sendLaunch && uwTick - launchControlStart > 25) {
    status = MMR_CAN_SendNoTamper(&hcan, launchControl);
    launchControlStart = uwTick;
  }

  switch (state) {
  case START:
    if (isManual) {
      state = RELEASE;
    } else {
      state = PULL;
      debounce = 0;
    }


  case PULL:
    if (nMot > 1000){
    if (uwTick - clutchMsgStart > 1) {
      status = MMR_CAN_Send(&hcan, clutchPull);
      clutchMsgStart = uwTick;
    }

    if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL_OK) {
    sendLaunch = true;
    launchControlStart = uwTick;
    state = PULLED;
    }
    }
    break;

  case PULLED:
    if (msg.header.messageId == MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2) {
      if (buffer[6]) {
        clutchMsgStart = uwTick;
        clutchMsgCnt = 0;
        state = GEAR_NOT_SET;
      }
    }
    break;

  case GEAR_NOT_SET:
    if (!gear)
      changeGear(true);

    state = GEAR_CHANGING;
    break;

  case GEAR_CHANGING:
    if (gear) {
      changeGearStop = true;
      changeGear(true);
      HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
      state = GEAR_CHANGED;
      break;
    }

    if (!gear || !changeGear(false)) {
      waitMs(true, 2000);
      state = GEAR_NOT_CHANGED;
      break;
    }

    break;

  case GEAR_NOT_CHANGED:
    if (waitMs(false, 2000)) {
      HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
      state = GEAR_NOT_SET;
    }

    break;

  case GEAR_CHANGED:
    HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
    waitMs(true, 1000);
    state = LAUNCH_CONTROL_SET;
    break;

  case LAUNCH_CONTROL_SET:
    dacValue = DAC_30;
    if (waitMs(false, 1000)) {
      state = APPS_30;
    }
    break;

  case APPS_30:
    if (uwTick - clutchMsgStart > 1) {
      status = MMR_CAN_Send(&hcan, clutchRelease);
      clutchMsgStart = uwTick;

      if (clutchMsgCnt++ >= 5 && nMot >= 6000) {
        state = RELEASE;
      }
    }

    break;

  case RELEASE:
    if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK) {
      firstrun=1;
      clutchMsgStart = uwTick;
      launchControlStop = uwTick;
      clutchMsgCnt = 0;
      launchControlCnt = 0;
      _launchControlData[0] = 0x0;
      state = RELEASED;
    }
    break;

  case RELEASED:
    if (!isManual)
      dacValue = DAC_0;

    if (clutchMsgCnt < 5 && uwTick - clutchMsgStart > 5) {
      status = MMR_CAN_Send(&hcan, clutchSetManual);
      clutchMsgStart = uwTick;
      clutchMsgCnt++;
    }

    if (launchControlCnt < 1 && uwTick - launchControlStop > 100) {
      sendLaunch = false;
      launchControlStop = uwTick;
      launchControlCnt++;
    }

    if (clutchMsgCnt >= 5 && launchControlCnt >= 1) {
      state = DONE;
    }
    break;

  case DONE:
    break;
  }
}


#endif // !APP_LAUNCH_CONTROL_INC_LAUNCH_CONTROL_H_
