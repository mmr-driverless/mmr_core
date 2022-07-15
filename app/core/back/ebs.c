#include "inc/ebs.h"
#include "inc/peripherals.h"
#include "inc/global_state.h"

const uint8_t EBS_CHECK_MIN_PRESSURE = 20;
const uint8_t BRAKE_MIN_PRESSURE = 2;


static bool checkEbsPressureOk();
static bool checkBrakesPressureOk();


void MMR_EBS_Arm() {
  MMR_PIN_Write(asp.ebsAsCloseSdc, MMR_PIN_HIGH);
  MMR_PIN_Write(asp.ebs1, MMR_PIN_LOW);
  MMR_PIN_Write(asp.ebs2, MMR_PIN_LOW);
}

void MMR_EBS_Disarm() {
  MMR_PIN_Write(asp.ebs1, MMR_PIN_HIGH);
  MMR_PIN_Write(asp.ebs2, MMR_PIN_HIGH);
}

void MMR_EBS_Brake() {
  MMR_PIN_Write(asp.ebs1, MMR_PIN_LOW);
  MMR_PIN_Write(asp.ebs2, MMR_PIN_LOW);
}

void MMR_EBS_SetDrivingMode(MmrEbsDrivingMode mode) {
  MmrPinState out = mode == MMR_EBS_CHECK_DRIVING_MODE_MANUAL
    ? MMR_PIN_HIGH
    : MMR_PIN_LOW;

  MMR_PIN_Write(asp.ebsAsDrivingMode, out);
}

bool MMR_EBS_SdcIsReady() {
  return MMR_PIN_Read(asp.ebsAsSdcIsReady) == MMR_PIN_HIGH;
}

static MmrEbsCheckState idle(MmrEbsCheckState state);
static MmrEbsCheckState startWatchdog(MmrEbsCheckState state);
static MmrEbsCheckState sdcWaitHigh(MmrEbsCheckState state);
static MmrEbsCheckState stopWatchdog(MmrEbsCheckState state);
static MmrEbsCheckState sdcWaitLow(MmrEbsCheckState state);
static MmrEbsCheckState retoggleWatchdog(MmrEbsCheckState state);
static MmrEbsCheckState ebsPressureOk(MmrEbsCheckState state);
static MmrEbsCheckState brakePressureOk(MmrEbsCheckState state);
static MmrEbsCheckState activateTs(MmrEbsCheckState state);
static MmrEbsCheckState waitTsActivation(MmrEbsCheckState state);
static MmrEbsCheckState waitTsActivated(MmrEbsCheckState state);
static MmrEbsCheckState disableActuator1(MmrEbsCheckState state);
static MmrEbsCheckState checkAct1BrakePressure(MmrEbsCheckState state);
static MmrEbsCheckState enableActuator1(MmrEbsCheckState state);
static MmrEbsCheckState disableActuator2(MmrEbsCheckState state);
static MmrEbsCheckState checkAct2BrakePressure(MmrEbsCheckState state);
static MmrEbsCheckState enableActuator2(MmrEbsCheckState state);

static MmrEbsCheckState armEbs(MmrEbsCheckState state);
static MmrEbsCheckState ready(MmrEbsCheckState state);
static MmrEbsCheckState error(MmrEbsCheckState state);


MmrEbsCheckState MMR_EBS_Check(MmrEbsCheckState state) {
  static MmrDelay blinkDelay = { .ms = 500 };
  if (state != EBS_CHECK_ERROR) {
    MMR_LED_BlinkAsync(asp.ebsErrorLed, &blinkDelay);
  }

  switch (state) {
  case EBS_CHECK_IDLE: return idle(state);

  case EBS_CHECK_START_WATCHDOG: return startWatchdog(state);
  case EBS_CHECK_SDC_WAIT_HIGH: return sdcWaitHigh(state);
  case EBS_CHECK_STOP_WATCHDOG: return stopWatchdog(state);
  case EBS_CHECK_SDC_WAIT_LOW: return sdcWaitLow(state);
  case EBS_CHECK_RETOGGLE_WATCHDOG: return retoggleWatchdog(state);

  case EBS_CHECK_EBS_PRESSURE_OK: return ebsPressureOk(state);
  case EBS_CHECK_BRAKE_PRESSURE_OK: return brakePressureOk(state);

  case EBS_CHECK_ACTIVATE_TS: return activateTs(state);
  case EBS_CHECK_WAIT_TS_ACTIVATION: return waitTsActivation(state);
  case EBS_WAIT_TS_ACTIVATED: return waitTsActivated(state);

  case EBS_CHECK_DISABLE_ACTUATOR_1: return disableActuator1(state);
  case EBS_CHECK_CHECK_ACT_1_BRAKE_PRESSURE: return checkAct1BrakePressure(state);
  case EBS_CHECK_ENABLE_ACTUATOR_1: return enableActuator1(state);

  case EBS_CHECK_DISABLE_ACTUATOR_2: return disableActuator2(state);
  case EBS_CHECK_CHECK_ACT_2_BRAKE_PRESSURE: return checkAct2BrakePressure(state);
  case EBS_CHECK_ENABLE_ACTUATOR_2: return enableActuator2(state);
  
  case EBS_CHECK_ARM: return armEbs(state);

  case EBS_CHECK_READY: return ready(state);
  case EBS_CHECK_ERROR: return error(state);
  default: return error(state);
  }
}


static MmrEbsCheckState idle(MmrEbsCheckState state) {
  return EBS_CHECK_START_WATCHDOG;
}


static MmrEbsCheckState startWatchdog(MmrEbsCheckState state) {
  asp.watchdogStart();
  return EBS_CHECK_SDC_WAIT_HIGH;
}

static MmrEbsCheckState sdcWaitHigh(MmrEbsCheckState state) {
  if (MMR_EBS_SdcIsReady()) {
    return EBS_CHECK_STOP_WATCHDOG;
  }

  return state;
}

static MmrEbsCheckState stopWatchdog(MmrEbsCheckState state) {
  asp.watchdogStop();
  return EBS_CHECK_SDC_WAIT_LOW;
}

static MmrEbsCheckState sdcWaitLow(MmrEbsCheckState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (!MMR_EBS_SdcIsReady()) {
    return EBS_CHECK_RETOGGLE_WATCHDOG;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_CHECK_ERROR;
  }

  return state;
}

static MmrEbsCheckState retoggleWatchdog(MmrEbsCheckState state) {
  asp.watchdogStart();
  return EBS_CHECK_EBS_PRESSURE_OK;
}

static MmrEbsCheckState ebsPressureOk(MmrEbsCheckState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkEbsPressureOk()) {
    return EBS_CHECK_BRAKE_PRESSURE_OK;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_CHECK_ERROR;
  }

  return state;
}

static MmrEbsCheckState brakePressureOk(MmrEbsCheckState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakesPressureOk()) {
    return EBS_CHECK_ACTIVATE_TS;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_CHECK_ERROR;
  }

  return state;
}

static MmrEbsCheckState activateTs(MmrEbsCheckState state) {
  MMR_EBS_Arm();
  return EBS_CHECK_WAIT_TS_ACTIVATION;
}

static MmrEbsCheckState waitTsActivation(MmrEbsCheckState state) {
  static MmrDelay tsSimulationDelay = { .ms = 3000 };  // TODO: faked
  if (MMR_DELAY_WaitAsync(&tsSimulationDelay)) {
	  return EBS_WAIT_TS_ACTIVATED;
  }

  /*
  if (gs.gear == 0 && gs.rpm >= 1000) {
    return EBS_WAIT_TS_ACTIVATED;
  }
  */
  return state;
}


static MmrEbsCheckState waitTsActivated(MmrEbsCheckState state) {
  if (MMR_EBS_SdcIsReady()) {
    return EBS_CHECK_DISABLE_ACTUATOR_1;
  }

  return state;
}


static MmrEbsCheckState disableActuator1(MmrEbsCheckState state) {
  static MmrDelay delay = { .ms = 2000 };

  MMR_PIN_Write(asp.ebs1, MMR_PIN_HIGH);

  if (MMR_DELAY_WaitAsync(&delay)) {
    return EBS_CHECK_CHECK_ACT_1_BRAKE_PRESSURE;
  }

  return state;
}

static MmrEbsCheckState checkAct1BrakePressure(MmrEbsCheckState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakesPressureOk()) {
    return EBS_CHECK_ENABLE_ACTUATOR_1;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_CHECK_ERROR;
  }

  return state;
}

static MmrEbsCheckState enableActuator1(MmrEbsCheckState state) {
  static MmrDelay delay = { .ms = 2000 };

  MMR_PIN_Write(asp.ebs1, MMR_PIN_LOW);

  if (MMR_DELAY_WaitAsync(&delay)) {
    return EBS_CHECK_DISABLE_ACTUATOR_2;
  }

  return state;
}


static MmrEbsCheckState disableActuator2(MmrEbsCheckState state) {
  static MmrDelay delay = { .ms = 2000 };

  MMR_PIN_Write(asp.ebs2, MMR_PIN_HIGH);

  if (MMR_DELAY_WaitAsync(&delay)) {
    return EBS_CHECK_CHECK_ACT_2_BRAKE_PRESSURE;
  }

  return state;
}

static MmrEbsCheckState checkAct2BrakePressure(MmrEbsCheckState state) {
  static MmrDelay timeout = { .ms = 1000 };

  if (checkBrakesPressureOk()) {
    return EBS_CHECK_ENABLE_ACTUATOR_2;
  }

  if (MMR_DELAY_WaitAsync(&timeout)) {
    return EBS_CHECK_ERROR;
  }

  return state;
}

static MmrEbsCheckState enableActuator2(MmrEbsCheckState state) {
  MMR_PIN_Write(asp.ebs2, MMR_PIN_LOW);
  return EBS_CHECK_READY;
}


static MmrEbsCheckState armEbs(MmrEbsCheckState state) {
  static MmrDelay delay = { .ms = 2000 };

  if (MMR_DELAY_WaitAsync(&delay)) {
    return EBS_CHECK_READY;
  }

  return state;
}


static MmrEbsCheckState ready(MmrEbsCheckState state) {
  MMR_PIN_Write(asp.ebs1, MMR_PIN_HIGH);
  MMR_PIN_Write(asp.ebs2, MMR_PIN_HIGH);
  return EBS_CHECK_READY;
}

static MmrEbsCheckState error(MmrEbsCheckState state) {
  MMR_LED_Set(asp.ebsErrorLed, MMR_LED_ON);
  return EBS_CHECK_ERROR;
}


static bool checkEbsPressureOk() {
  return
    gs.ebs1Pressure >= EBS_CHECK_MIN_PRESSURE &&
    gs.ebs2Pressure >= EBS_CHECK_MIN_PRESSURE;
}

static bool checkBrakesPressureOk() {
  return
    gs.brakePressureFront >= BRAKE_MIN_PRESSURE &&
    gs.brakePressureRear >= BRAKE_MIN_PRESSURE;
}
