/*
 * pid.c
 *
 *  Created on: 2025年6月18日
 *      Author: 刘珈琪
 */


#include "pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID初始化
// 参数说明     *pid                PID结构体
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_init(pid_controller *pid)
{
    pid->Kp = 3.0f;             // 设置PID控制器的比例系数为2.0（单精度浮点数）
    pid->Ki = 0.01f;            // 设置PID控制器的积分系数为0.01（单精度浮点数）
    pid->Kd = 3.0f;             // 设置PID控制器的微分系数为0.5（单精度浮点数），用于预测误差变化趋势减少超调
    pid->setpoint = 0.0f;       // 设置PID控制器的目标设定值为0.0（单精度浮点数）
    pid_reset(pid);             // 调用复位函数对PID控制器内部状态进行清零，为新控制过程做准备
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID设置输出限制
// 参数说明     *pid                PID结构体
// 参数说明     min                 设置输出最小值
// 参数说明     max                 设置输出最大值
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_set_limits(pid_controller *pid, float min, float max)
{
    pid->out_min = min;
    pid->out_max = max;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算PID输出
// 参数说明     *pid                PID控制器
// 参数说明     input               设置输出
// 返回参数     float
// 使用示例     pid_output = pid_compute(&heading_pid, current_angles.yaw);
// 备注信息     此示例可以输出通过PID调节后的IMU航向角数据
//-------------------------------------------------------------------------------------------------------------------
float pid_compute(pid_controller *pid, float input)
{
    // 计算原始误差
    float raw_error = pid->setpoint - input;

    // 处理角度环绕问题
    float error = raw_error;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    // 比例项
    float proportional = pid->Kp * error;

    // 积分项（抗饱和处理）
    pid->integral += error;
    if (pid->integral * pid->Ki > pid->out_max)
    {
        pid->integral = pid->out_max / pid->Ki;
    }
    else if (pid->integral * pid->Ki < pid->out_min)
    {
        pid->integral = pid->out_min / pid->Ki;
    }
    float integral = pid->Ki * pid->integral;

    // 微分项（使用处理后的误差）
    float derivative = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    // 计算总输出
    pid->output = proportional + integral + derivative;

    // 限制输出范围
    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    return pid->output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置PID控制器
// 参数说明     *pid                PID控制器
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void pid_reset(pid_controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}
