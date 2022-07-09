/*
 * EBS.h
 *
 *  Created on: 7 lug 2022
 *      Author: Maxmi
 */

#ifndef INC_EBS_H_
#define INC_EBS_H_


#include "stdbool.h"
#include "stm32f3xx_hal.h"
#include "main.h"
#include <can0.h>
#include "stm_pin.h"


#define EBS_min_Pressure 10 // indica la pressione minima che la linea EBS deve raggiungere quando attiviamo tale sistema
#define BRAKE_pressure 10 // indica un valore di pressione che deve essere raggiunto in fase di test dell ebs check()
#define min_RPM 1000
#define NEUTRAL 0
#define OPEN 1
#define CLOSE 0
#define SDC_is_Ready() HAL_GPIO_ReadPin(SDC_IS_READY_GPIO_Port, SDC_IS_READY_Pin)



typedef enum EbsStates
{
	EBS_IDLE,
	EBS_SDC_IS_READY,
	EBS_SDC_IS_NOT_READY,
	EBS_CHECK_NOT_ENDED,
	EBS_PRESSURE_CHECK,
	EBS_TS_CHECK,
	EBS1_CONTROL,
	EBS2_CONTROL,
	EBS_ERROR,
	EBS_OK,


}EbsStates;

typedef enum ebsflag
{
	EBS_STATE_UNAVAILABLE,
	EBS_STATE_ARMED,
	EBS_STATE_ACTIVATED,

}ebsflag;



//uint8_t  EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value, uint16_t RPM, uint8_t gear); // funzione per check ebs
void WATCHDOG_Activation(); // funzione per  attivazione il  watchdog
void WATCHDOG_Disable(); // funzione per disabilitare il watchdog
void EBS_Management(MmrPin* EBS_pin, bool state); // funzione per controllare i due attuatatori dell EBS
void EBS_Init(MmrPin* pin1, MmrPin* pin2, MmrPin* asClSDC, MmrPin* ebsledPin);
void LSW_EBSLed (MmrPin* led,bool state);
EbsStates ebsCheck(EbsStates state);

bool EBS_sensor_check(uint8_t EBS1_Value, uint8_t EBS2_value);
void AS_Close_SDC(MmrPin* asClSDC);
bool BRAKE_pressure_check(uint8_t Brake1_value, uint8_t Brake2_value);
ebsflag MMR_AS_GetEbsStates();


bool TS_Activation(uint16_t RPM, uint8_t gear);



#endif /* INC_EBS_H_ */