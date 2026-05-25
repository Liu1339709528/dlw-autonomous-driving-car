/*
 * encoder.h
 *
 *  Created on: 2025年8月4日
 *      Author: 27962
 */

#ifndef USER_CODE_ENCODER_H_
#define USER_CODE_ENCODER_H_

#include "zf_common_typedef.h"
#include <math.h>
typedef struct
{
    float Kp;             // 横滚角 (弧度)
    float Ki;            // 俯仰角 (弧度)
    float Kd;              // 偏航角 (弧度)

    float error;			//误差值
    float last_error;		//上一次误差值

    float sudo_setpoint; //目标速度值
    float integral; 	//积分项累加值
    uint32_t sudo_out;		//输出值
} SUDO;

void encoder_PID_init(float P,float I ,float D);
void encoder_PID_output(float current_sudo);
void encoder_contorl_sudo(float setsudo , float current_sudo);
#endif /* USER_CODE_ENCODER_H_ */
