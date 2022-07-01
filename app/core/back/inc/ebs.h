/*
 * ebs.h
 *
 *  Created on: 29 giu 2022
 *      Author: Maxmi
 */

#ifndef CORE_BACK_INC_EBS_H_
#define CORE_BACK_INC_EBS_H_

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
#define SDC_is_Ready() HAL_GPIO_ReadPin(SDC_READY_GPIO_Port, SDC_READY_Pin)
#define AS_CLOSE_SDC() HAL_GPIO_WritePin( AS_SDC_CLOSE_GPIO_Port, AS_SDC_CLOSE_Pin, GPIO_PIN_SET);
#define EBS_NOT_ENDED 2
#define EBS_ENDED 1
#define EBS_ERROR 0


uint8_t  EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value, uint16_t RPM, uint8_t gear); // funzione per check ebs
void WATCHDOG_Activation(); // funzione per  attivazione il  watchdog
void WATCHDOG_Disable(); // funzione per disabilitare il watchdog
void EBS_Management(uint16_t EBS_act, bool state); // funzione per controllare i due attuatatori dell EBS
bool EBS_sensor_check(uint8_t EBS1_Value, uint8_t EBS2_value);
bool BRAKE_pressure_check(uint8_t Brake1_value, uint8_t Brake2_value);
bool TS_Activation(uint16_t RPM, uint8_t gear);

#endif /* CORE_BACK_INC_EBS_H_ */
