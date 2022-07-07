/*
 * EBS.c
 *
 *  Created on: 7 lug 2022
 *      Author: Maxmi
 */


#include "EBS.h"
#include "delay.h"
#include "timing.h"


extern TIM_HandleTypeDef htim16;
static MmrPin *__ebs1;
static MmrPin *__ebs2;
static MmrPin* __asclSDC;




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


void EBS_Init(MmrPin* Epin1, MmrPin* Epin2, MmrPin* asClSDC)
{
	__ebs1 = Epin1;
	__ebs2 = Epin2;
	__asclSDC = asClSDC;
}
void EBS_Management(MmrPin* EBS_pin, bool state)
{
	if(EBS_pin->pin== EBS_CONTROL1_Pin)
	{
		if(state == CLOSE) HAL_GPIO_WritePin(EBS_pin->port,EBS_pin->pin, GPIO_PIN_SET);
		else
			if(state == OPEN) HAL_GPIO_WritePin(EBS_pin->port,EBS_pin->pin, GPIO_PIN_RESET);

	}
	else
		if(EBS_pin->pin == EBS_CONTROL2_Pin)
		{
			if(state == CLOSE) HAL_GPIO_WritePin(EBS_pin->port,EBS_pin->pin, GPIO_PIN_SET);
					else
						if(state == OPEN) HAL_GPIO_WritePin(EBS_pin->port,EBS_pin->pin,GPIO_PIN_RESET);

		}

}

void AS_Close_SDC(MmrPin* asClSDC)
{
	HAL_GPIO_WritePin(asClSDC->port, asClSDC->pin, GPIO_PIN_SET);
}

uint8_t EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value,uint16_t RPM, uint8_t gear)
{
/*---------------------------------------------  PUNTO 2-3 */

	if(SDC_is_Ready() == true) WATCHDOG_Disable(); // disabilito il watchdog
	else return EBS_NOT_ENDED;
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
	AS_Close_SDC(__asclSDC);
	//invio mex can true al display ts_Ebs()
	}
	else return EBS_ERROR;
/*---------------------------------------------------- PUNTO 10-11-12-13 */

  if(TS_Activation(RPM, gear)) EBS_Management(__ebs1, OPEN);
  else return EBS_NOT_ENDED;

  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
	  {
	  EBS_Management(__ebs1, CLOSE);
	  EBS_Management(__ebs2, OPEN);
	  }
  //ACCENDERE LED EBS FAIL
  else return EBS_ERROR;

  /*--------------------------------------------------------- PUNTO 14-15-16*/

  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
	   {
	    EBS_Management(__ebs2, CLOSE);
	    return EBS_ENDED;
	   }
  //ACCENDERE LED EBS FAIL
  else return EBS_ERROR;


}


