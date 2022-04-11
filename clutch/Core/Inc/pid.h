/*
 * pid.h
 *
 *  Created on: Feb 15, 2022
 *      Author: Maxmi
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

extern void init_PID();
extern float PID_Compute(float reference, float measured, uint8_t Derivative_Enable);

float Max_out_f;
float Min_out_f;

float Kp_f;
float Ki_f;
float Kd_f;

float Sample_time;
float tau;

float PIDOutput;

float PIDOutput_PreSaturation;

float last_error;

float last_PIDOutput;

float last_PIDOutput_PreSaturation;

float OutputEnabled;

float Proportional;
float Integral;
float Derivative;



#endif /* INC_PID_H_ */
