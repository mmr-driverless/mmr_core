/*
 * pid.c
 *
 *  Created on: Feb 15, 2022
 *      Author: Maxmi
 */
#include "pid.h"
#include <stdbool.h>

void init_PID()
{
	Max_out_f = 1;
	Min_out_f = 0;

	Kp_f = 22.59f;
	Ki_f = 8.6f;
	Kd_f = 0.7f;

	tau = 1/14565;

	Sample_time = 1/9000;

	PIDOutput = 0;

	PIDOutput_PreSaturation = 0;

	last_error = 0;

	last_PIDOutput = 0;

	last_PIDOutput_PreSaturation = 0;

	OutputEnabled = 0;


};


float PID_Compute(float reference, float measured, uint8_t Derivative_Enable)
{
	/*if (OutputEnabled == 0) {
		NSLEEP_GPIO_Port->BSRR = NSLEEP_Pin; // Disattivo il driver --> implicitamente non seguire più l'obiettivo, fermati
		TIM3->CCR2 = 0; // Inoltre imposto il duty cycle allo 0% --> non è detto che sia a 0% --> per questo si usa NSLEEP per togliere corrente al motore
		return 0;
	}*/

	bool compare_signal = 0;
	float error = reference - measured;

	//Proportional term
	Proportional = (Kp_f) * error;

	//Integral term + Anti-windup clamping
	compare_signal = (last_PIDOutput_PreSaturation != last_PIDOutput) ? 1 : 0;

	if (compare_signal) // Significa che è andato in saturazione
		Integral = 0;
	else
		Integral = Integral + (Ki_f) * 0.5 * (Sample_time) * (error + last_error);

	if(Derivative_Enable == 1)
	{
		Derivative = (2*Kd_f/(2*tau + Sample_time))*(error-last_error) + ((2*tau - Sample_time)/(2*tau + Sample_time))*Derivative;
	}
	else if (Derivative_Enable == 0)
		Derivative = 0;

	// Control Signal
	PIDOutput_PreSaturation = Proportional + Integral + Derivative;
	PIDOutput = PIDOutput_PreSaturation;

	//Saturation
	if (PIDOutput >= Max_out_f)
		PIDOutput = Max_out_f;
	else if (PIDOutput <= Min_out_f)
		PIDOutput = Min_out_f;
	//update terms

	last_error = error;
	last_PIDOutput_PreSaturation = PIDOutput_PreSaturation;
	last_PIDOutput = PIDOutput;

	//TIM4->CCR2 = (PIDOutput)*799;
	return PIDOutput;
};


