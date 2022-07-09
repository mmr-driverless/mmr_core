/*
 * EBS.c
 *
 *  Created on: 7 lug 2022
 *      Author: Maxmi
 */


#include "EBS.h"
#include "delay.h"
#include "timing.h"
#include "as.h"


extern TIM_HandleTypeDef htim16;
static MmrPin *__ebs1;
static MmrPin *__ebs2;
static MmrPin* __asclSDC;
static MmrPin *__EBSLedPin;
static ebsflag EBSflag;


void WATCHDOG_Activation() { HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); }
void WATCHDOG_Disable() { HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);}

ebsflag MMR_AS_GetEbsStates()
{
return EBSflag;
}

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


void EBS_Init(MmrPin* Epin1, MmrPin* Epin2, MmrPin* asClSDC, MmrPin* ebsledPin)
{
	__ebs1 = Epin1;
	__ebs2 = Epin2;
	__asclSDC = asClSDC;
	__EBSLedPin = ebsledPin;
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

void LSW_EBSLed (MmrPin* led,bool state)
{
	if(state == CLOSE)HAL_GPIO_WritePin(led->port,led->pin, GPIO_PIN_SET);
	else
		if(state == OPEN)HAL_GPIO_WritePin(led->port,led->pin, GPIO_PIN_RESET);

}

void AS_Close_SDC(MmrPin* asClSDC)
{
	HAL_GPIO_WritePin(asClSDC->port, asClSDC->pin, GPIO_PIN_SET);
}

//uint8_t EBS_check(uint8_t EBS1_Value, uint8_t EBS2_value, uint8_t Brake1_value, uint8_t Brake2_value,uint16_t RPM, uint8_t gear)
//{
///*---------------------------------------------  PUNTO 2-3 */
//
//	if(SDC_is_Ready() == true) WATCHDOG_Disable(); // disabilito il watchdog
//	else return EBS_NOT_ENDED;
///*----------------------------------------------- PUNTO 4-5*/
//	if(SDC_is_Ready() == false)
//	{
//		// invio mex alla scheda del display con ok ;
//		WATCHDOG_Activation();
//	}
//	else
//		{
//		//invio mex can alla scheda display con errore
//		return EBS_ERROR;
//		}
//
///*-------------------------------------------------PUNTO 6-7-8-9*/
//if(BRAKE_pressure_check(Brake1_value,Brake2_value) && EBS_sensor_check(EBS1_Value, EBS2_value))
//	{
//	AS_Close_SDC(__asclSDC);
//	//invio mex can true al display ts_Ebs()
//	}
//	else return EBS_ERROR;
///*---------------------------------------------------- PUNTO 10-11-12-13 */
//
//  if(TS_Activation(RPM, gear)) EBS_Management(__ebs1, OPEN);
//  else return EBS_NOT_ENDED;
//
//  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
//	  {
//	  EBS_Management(__ebs1, CLOSE);
//	  EBS_Management(__ebs2, OPEN);
//	  }
//  //ACCENDERE LED EBS FAIL
//  else return EBS_ERROR;
//
//  /*--------------------------------------------------------- PUNTO 14-15-16*/
//
//  if(BRAKE_pressure_check(Brake1_value,Brake2_value))
//	   {
//	    EBS_Management(__ebs2, CLOSE);
//	    return EBS_ENDED;
//	   }
//  //ACCENDERE LED EBS FAIL
//  else return EBS_ERROR;
//
//
//}

EbsStates ebsCheck(EbsStates state)
{
	switch(state)
	{

	case EBS_IDLE: if(SDC_is_Ready() == GPIO_PIN_SET)
	               {
		            WATCHDOG_Disable();
		            HAL_Delay(250); // ma modificare
		            return EBS_SDC_IS_READY;
		            break;
		            }
	                else
	                	{

	                	return EBS_CHECK_NOT_ENDED;
	                	break;

	                	}

	case EBS_SDC_IS_READY: if(SDC_is_Ready() == GPIO_PIN_RESET)
	                         {

		                      WATCHDOG_Activation();
		                      return EBS_PRESSURE_CHECK;break;
	                         }
	                         else
	                        	 {return EBS_SDC_IS_NOT_READY;
	                        	 break;}


   case EBS_SDC_IS_NOT_READY:  HAL_Delay(250); // ma modificare
	                        if(SDC_is_Ready() == GPIO_PIN_RESET)
	                        {
	                          WATCHDOG_Activation();
	                          return EBS_PRESSURE_CHECK;
	                          break;
	                         }
	                        else
	                        	{
	                        	EBSflag = EBS_STATE_UNAVAILABLE;
	                        	return EBS_ERROR;
	                        	break;
	                        	}

	case EBS_PRESSURE_CHECK: HAL_Delay(200); // ma modificare
	                         if(BRAKE_pressure_check(MMR_GS_GetBreakP1(),MMR_GS_GetBreakP2()) && EBS_sensor_check(MMR_GS_GetEbs1(),MMR_GS_GetEbs2()))
		                      {
	                        	 EBSflag = EBS_STATE_ARMED;
	                        	 AS_Close_SDC(__asclSDC);
		                       return EBS_TS_CHECK; break;
		                       }
		                      else {
		                        	EBSflag = EBS_STATE_UNAVAILABLE;
                                    return EBS_ERROR;
		                      break;}

	case EBS_TS_CHECK: if(TS_Activation(MMR_GS_GetRpm(), MMR_GS_GetGear()))
		{return EBS1_CONTROL;
		break;
		}

	case EBS1_CONTROL: EBS_Management(__ebs1, OPEN);

                       HAL_Delay(20); // ma modificare
		            if(BRAKE_pressure_check(MMR_GS_GetBreakP1(),MMR_GS_GetBreakP2()))
		            	{
		            	EBS_Management(__ebs1, CLOSE);
		          	    return EBS2_CONTROL;
		          	    break;
		            	}
		              else {
		            	  EBSflag = EBS_STATE_UNAVAILABLE;
		            	  return EBS_ERROR;
		              break;}

	case EBS2_CONTROL: 	  EBS_Management(__ebs2, OPEN);
	                      EBSflag = EBS_STATE_ACTIVATED;
	                      HAL_Delay(20); // ma modificare
	                   if(BRAKE_pressure_check(MMR_GS_GetBreakP1(),MMR_GS_GetBreakP2()))
	                 	   {
	               	      EBS_Management(__ebs2, CLOSE);

	                 	   return EBS_OK; break;
	                 	   }
	                   else  {
			            	  EBSflag = EBS_STATE_UNAVAILABLE;
			            	  return EBS_ERROR;
	                	      break;
	                   }

	case EBS_ERROR: LSW_EBSLed(__EBSLedPin,OPEN);

	case EBS_CHECK_NOT_ENDED: HAL_Delay(100);
	                          if(SDC_is_Ready() == GPIO_PIN_SET)
	                           {
	                             return EBS_IDLE;
	                             break;
	                           }
	                          else {
	                        	  return EBS_ERROR;
	                        	  break;
	                          }


	}
}

