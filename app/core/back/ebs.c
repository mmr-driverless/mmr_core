/*
 * ebs.c
 *
 *  Created on: 29 giu 2022
 *      Author: Maxmi
 */


#include "ebs.h"


extern TIM_HandleTypeDef htim16;

void WATCHDOG_Activation() { HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); }
void WATCHDOG_Disable() { HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1); }
bool EBS_sensor_check(uint8_t EBS1_value, uint8_t EBS2_value)
{
	if(EBS1_value >= EBS_min_Pressure && EBS2_value >= EBS_min_Pressure) return 1;
	  else return 0;
}



bool BRAKE_pressure_check(uint8_t Brake1_value, uint8_t Brake2_value)
{
	if(Brake1_value >= BRAKE_pressure && Brake2_value >= BRAKE_pressure) return 1;
      else return 0;
}

bool TS_Activation(uint16_t RPM, uint8_t gear)
{
	if(RPM >= min_RPM && gear == NEUTRAL) return 1;
	else return 0;
}

void EBS_Management(uint16_t EBS_act, bool state)
{
	if(EBS_act == EBS_CONTROL1_Pin)
	{
		if(state == CLOSE) HAL_GPIO_WritePin(EBS_CONTROL1_GPIO_Port,EBS_CONTROL1_Pin, GPIO_PIN_RESET);
		else
			if(state == OPEN) HAL_GPIO_WritePin(EBS_CONTROL1_GPIO_Port,EBS_CONTROL1_Pin, GPIO_PIN_SET);

	}
	else
		if(EBS_act == EBS_CONTROL2_Pin)
		{
			if(state == CLOSE) HAL_GPIO_WritePin(EBS_CONTROL2_GPIO_Port, EBS_CONTROL2_Pin, GPIO_PIN_RESET);
					else
						if(state == OPEN) HAL_GPIO_WritePin(EBS_CONTROL2_GPIO_Port,EBS_CONTROL2_Pin,GPIO_PIN_SET);

		}

}


uint8_t EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value,uint16_t RPM, uint8_t gear )
{
/*---------------------------------------------  PUNTO 2-3 */

	if(SDC_is_Ready() == true)
		{
		WATCHDOG_Disable(); // disabilito il watchdog
		HAL_Delay(10); // da rimpiazzare con delay + accurato
		}
	else
		return EBS_NOT_ENDED;
/*----------------------------------------------- PUNTO 4-5*/
	if(SDC_is_Ready() == false)
	{
		// invio mex alla scheda del display con ok ;
		WATCHDOG_Activation();
	}
	else
		{
		//invio mex can alla scheda display con errore
		return EBS_ERROR;
		}

/*-------------------------------------------------PUNTO 6-7-8-9*/
if(BRAKE_pressure_check(Brake1_value,Brake2_value) && EBS_sensor_check(EBS1_Value, EBS2_value))
	{
	AS_CLOSE_SDC();
	//invio mex can true al display ts_Ebs()
	}
	else return EBS_ERROR;
/*---------------------------------------------------- PUNTO 10-11-12-13 */

  if(TS_Activation(RPM, gear)) EBS_Management(EBS_CONTROL1_Pin, OPEN);
  else return EBS_NOT_ENDED;

  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
	  {
	  EBS_Management(EBS_CONTROL1_Pin, CLOSE);
	  HAL_Delay(100);
	  EBS_Management(EBS_CONTROL2_Pin, OPEN);
	  }
  //ACCENDERE LED EBS FAIL
  else return EBS_ERROR;

  /*--------------------------------------------------------- PUNTO 14-15-16*/

  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
	   {
	    EBS_Management(EBS_CONTROL2_Pin, CLOSE);
	    return EBS_ENDED;
	   }
  //ACCENDERE LED EBS FAIL
  else return EBS_ERROR;


}
