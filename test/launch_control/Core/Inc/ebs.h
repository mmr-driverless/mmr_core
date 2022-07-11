#ifndef INC_EBS_H_
#define INC_EBS_H_


#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include <mission.h>
#include <can0.h>
#include "stm_pin.h"
#include "as.h"


#define EBS_min_Pressure 10 // indica la pressione minima che la linea EBS deve raggiungere quando attiviamo tale sistema
#define BRAKE_pressure 10 // indica un valore di pressione che deve essere raggiunto in fase di test dell ebs check()
#define min_RPM 1000
#define NEUTRAL 0
#define OPEN 1
#define CLOSE 0
#define SDC_is_Ready() HAL_GPIO_ReadPin(SDC_IS_READY_GPIO_Port, SDC_IS_READY_Pin)


typedef enum MmrEbsCheck {
	EBS_IDLE,
	EBS_SDC_IS_READY,
	EBS_SDC_IS_NOT_READY,
	EBS_CHECK_NOT_ENDED,
	EBS_PRESSURE_CHECK,
	EBS_TS_CHECK,
	EBS1_CONTROL,
	EBS2_CONTROL,
	EBS_ERROR,
	EBS_FINAL_CHECK,
	EBS_OK,
} MmrEbsCheck;

typedef enum MmrEbsState {
	EBS_STATE_UNAVAILABLE,
	EBS_STATE_ARMED,
	EBS_STATE_ACTIVATED,
	EBS_STATE_DISACTIVATED,
} MmrEbsState;


//uint8_t  EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value, uint16_t RPM, uint8_t gear); // funzione per check ebs
void WATCHDOG_Activation(); // funzione per  attivazione il  watchdog
void WATCHDOG_Disable(); // funzione per disabilitare il watchdog
void EBS_Management(MmrPin* EBS_pin, bool state); // funzione per controllare i due attuatatori dell EBS
void EBS_Init(MmrPin* pin1, MmrPin* pin2, MmrPin* asClSDC, MmrPin* ebsledPin);
void LSW_EBSLed (MmrPin* led,bool state);
MmrEbsCheck ebsCheck(MmrEbsCheck state);

bool EBS_sensor_check();
void AS_Close_SDC(MmrPin* asClSDC);
bool BRAKE_pressure_check();
MmrEbsState MMR_AS_GetEbsStates();
MmrEbsState EBS_Activation(MmrMission currentMission, bool Missionflag, bool ResEMergencyflag);
uint8_t MMR_Get_AS_GetResEB();


bool TS_Activation();



#endif /* INC_EBS_H_ */
