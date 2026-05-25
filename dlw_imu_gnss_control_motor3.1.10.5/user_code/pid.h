/*
 * pid.h
 *
 *  Created on: 2025年6月18日
 *      Author:
 */

#ifndef USER_CODE_PID_H_
#define USER_CODE_PID_H_

#include "zf_common_headfile.h"
#include "zf_device_imu660rb.h"
#include "quaternion.h"
#include "servo.h"
#include <math.h>

// PID控制器参数结构体
typedef struct
{
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float setpoint;     // 目标值（期望航向角）
    float integral;     // 积分项累积值
    float prev_error;   // 上一次误差值
    float output;       // 当前输出值
    float out_min;      // 输出下限
    float out_max;      // 输出上限
} pid_controller;

// 函数原型
void pid_init(pid_controller *pid);
void pid_set_limits(pid_controller *pid, float min, float max);
float pid_compute(pid_controller *pid, float input);
void pid_reset(pid_controller *pid);

#endif /* USER_CODE_PID_H_ */
