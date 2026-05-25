#include "encoder.h"
#include "moto.h"
/*
 * encoder.c
 *
 *  Created on: 2025年8月4日
 *      Author: 27962
 */
SUDO sudo_PID;
void encoder_PID_init(float P,float I ,float D)
{
	sudo_PID.Kp = P;
	sudo_PID.Ki = I;
	sudo_PID.Kd = D;
}
float integral=0;
void encoder_PID_output(float current_sudo)
{
	sudo_PID.error = sudo_PID.sudo_setpoint - current_sudo;

	float proportional = sudo_PID.Kp * sudo_PID.error;
	sudo_PID.integral += sudo_PID.error;
	if(sudo_PID.integral <= 30000)
	{
		sudo_PID.integral += sudo_PID.error;
	}
	else
	{
		sudo_PID.integral = 10000;
	}
	if(integral <= 30000)
	{
		integral = sudo_PID.Ki * sudo_PID.integral;
	}
	else
	{
		sudo_PID.integral = 0;
	}


	sudo_PID.sudo_out = proportional + integral;
}
void encoder_contorl_sudo(float setsudo , float current_sudo)
{
	sudo_PID.sudo_setpoint = setsudo;
	encoder_PID_output(current_sudo);
	dlw_set_motor_duty(sudo_PID.sudo_out);
}
