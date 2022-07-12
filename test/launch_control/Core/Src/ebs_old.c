#include "ebs_old.h"
#include <mission.h>
#include <global_state.h>
#include "delay.h"
#include "timing.h"
#include "as.h"


static MmrEbsCheck ebsIdle();
static MmrEbsCheck ebsSdcIsReady();
static MmrEbsCheck ebsSdcIsNotReady();
static MmrEbsCheck ebsPressureCheck();
static MmrEbsCheck ebsTsCheck();
static MmrEbsCheck ebs1Control();
static MmrEbsCheck ebs2Control();
static MmrEbsCheck ebsError();
static MmrEbsCheck ebsCheckNotEnded();
static MmrEbsCheck ebsFinalCheck();


extern TIM_HandleTypeDef htim16;
static MmrPin *__ebs1;
static MmrPin *__ebs2;
static MmrPin* __asclSDC;
static MmrPin *__EBSLedPin;
static MmrEbsState EBSflag;
extern uint8_t TS_EBS;

void WATCHDOG_Activation() {
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

void WATCHDOG_Disable() {
  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
}


MmrEbsState MMR_AS_GetEbsState() {
  return EBSflag;
}

bool EBS_sensor_check() {
  return
    gs.ebs1Pressure >= EBS_min_Pressure &&
    gs.ebs2Pressure >= EBS_min_Pressure;
}

bool BRAKE_pressure_check() {
  return
    gs.brakePressureFront >= BRAKE_pressure &&
    gs.brakePressureRear >= BRAKE_pressure;
}

bool TS_Activation() {
  return gs.rpm >= min_RPM && gs.gear == NEUTRAL;
}


void EBS_Init(MmrPin *Epin1, MmrPin *Epin2, MmrPin *asClSDC, MmrPin *ebsledPin) {
  __ebs1 = Epin1;
  __ebs2 = Epin2;
  __asclSDC = asClSDC;
  __EBSLedPin = ebsledPin;
}


void EBS_Management(MmrPin *EBS_pin, bool state) {
  // TODO
  if (EBS_pin->pin== EBS_CONTROL1_Pin) {
    if (state == CLOSE) {
      MMR_PIN_Write(EBS_pin, MMR_PIN_HIGH);
    }
    else if (state == OPEN) {
      MMR_PIN_Write(EBS_pin, MMR_PIN_LOW);
    }
  }
  else if(EBS_pin->pin == EBS_CONTROL2_Pin) {
    if (state == CLOSE) {
      MMR_PIN_Write(EBS_pin, MMR_PIN_HIGH);
    }
    else if (state == OPEN) {
      MMR_PIN_Write(EBS_pin, MMR_PIN_LOW);
    }
  }
}

void LSW_EBSLed (MmrPin *led, bool state) {
  if (state == CLOSE) {
    MMR_PIN_Write(led, MMR_PIN_HIGH);
  }
  else if(state == OPEN) {
    MMR_PIN_Write(led, MMR_PIN_LOW);
  }
}

void AS_Close_SDC(MmrPin* asClSDC) {
  HAL_GPIO_WritePin(asClSDC->port, asClSDC->pin, GPIO_PIN_SET);
}



MmrEbsCheck ebsCheck(MmrEbsCheck state) {
  switch (state) {
  case EBS_IDLE: return ebsIdle();
  case EBS_SDC_IS_READY: return ebsSdcIsReady();
  case EBS_SDC_IS_NOT_READY: return ebsSdcIsNotReady();
  case EBS_PRESSURE_CHECK: return ebsPressureCheck();
  case EBS_TS_CHECK: return ebsTsCheck();
  case EBS1_CONTROL: return ebs1Control();
  case EBS2_CONTROL: return ebs2Control();
  case EBS_ERROR: return ebsError();
  case EBS_CHECK_NOT_ENDED: return ebsCheckNotEnded();
  case EBS_FINAL_CHECK: return ebsFinalCheck();
  }
}


static MmrEbsCheck ebsIdle() {
  static MmrDelay delay = { .ms = 200 };

  if (SDC_is_Ready() == GPIO_PIN_SET) {
    WATCHDOG_Disable();
    if (MMR_DELAY_WaitAsync(&delay)) {
      return EBS_SDC_IS_READY;
    }
  }
  
  return EBS_CHECK_NOT_ENDED;
}

static MmrEbsCheck ebsSdcIsReady() {
  if (SDC_is_Ready() == GPIO_PIN_RESET) {
    WATCHDOG_Activation();
    return EBS_PRESSURE_CHECK;
  }

  return EBS_SDC_IS_NOT_READY;
}

static MmrEbsCheck ebsSdcIsNotReady() {
  if (SDC_is_Ready() == GPIO_PIN_RESET) {
    WATCHDOG_Activation();
    return EBS_PRESSURE_CHECK;
  }

  EBSflag = EBS_STATE_UNAVAILABLE;
  return EBS_ERROR;
}

static MmrEbsCheck ebsPressureCheck() {
  static MmrDelay delay = { .ms = 200 };

  // TODO
  while (MMR_DELAY_WaitAsync(&delay)) {

  }
  if (EBS_sensor_check()) {
    EBS_Management(__ebs1, OPEN);  // TODO: can't enable EBS while the car is running
    EBS_Management(__ebs2, OPEN);
    HAL_Delay(250);

    if(BRAKE_pressure_check()) {
      EBSflag = EBS_STATE_ARMED;
      TS_EBS = 1;
      AS_Close_SDC(__asclSDC);
      return EBS_TS_CHECK;
    }
  }

  EBSflag = EBS_STATE_UNAVAILABLE;
  return EBS_ERROR;
}

static MmrEbsCheck ebsTsCheck() {
  if (TS_Activation()) {
    return EBS1_CONTROL;
  }

  return EBS_TS_CHECK;
}

static MmrEbsCheck ebs1Control() {
  EBS_Management(__ebs1, CLOSE);
  HAL_Delay(20); // ma modificare

  if (BRAKE_pressure_check()) {
    EBS_Management(__ebs1, OPEN);
    return EBS2_CONTROL;
  }
  
  EBSflag = EBS_STATE_UNAVAILABLE;
  return EBS_ERROR;
}

static MmrEbsCheck ebs2Control() {
  static MmrDelay delay = { .ms = 200 };

  EBS_Management(__ebs2, CLOSE);
  EBSflag = EBS_STATE_ACTIVATED;
  
  while( MMR_DELAY_WaitAsync(&delay) != 0)
    ;

  if (BRAKE_pressure_check()) {
    EBS_Management(__ebs2, OPEN);
    return EBS_FINAL_CHECK;
  }

  EBSflag = EBS_STATE_UNAVAILABLE;
  return EBS_ERROR;
}

static MmrEbsCheck ebsError() {
  LSW_EBSLed(__EBSLedPin,OPEN);
  return EBS_ERROR;
}

static MmrEbsCheck ebsCheckNotEnded() {
  static MmrDelay delay = { .ms = 200 };

  while( MMR_DELAY_WaitAsync(&delay) != 0)
    ;

  if(SDC_is_Ready() == GPIO_PIN_SET) {
    return EBS_IDLE;
  }

  return EBS_ERROR;
}

static MmrEbsCheck ebsFinalCheck() {
  EBS_Management(__ebs1, OPEN);
  EBS_Management(__ebs2, OPEN);
  return EBS_OK;
}


MmrEbsState EBS_Activation(
  MmrMission currentMission,
  bool Missionflag /* se la missione è finita */,
  bool ResEMergencyflag /* emergenza */
)
{
  if (Missionflag || ResEMergencyflag) {
    if (currentMission != MMR_MISSION_INSPECTION || currentMission != MMR_MISSION_MANUAL) {
      EBS_Management(__ebs1, OPEN);
      EBS_Management(__ebs2, OPEN);
      return EBS_STATE_ACTIVATED;
    }
  }

  return EBS_STATE_DISACTIVATED;
}



