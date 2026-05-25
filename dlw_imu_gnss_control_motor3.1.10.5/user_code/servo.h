/*
 * servo.h
 *
 *  Created on: 2025年6月3日
 *      Author:
 */

#ifndef USER_CODE_SERVO_H_
#define USER_CODE_SERVO_H_

// 需要包含的系统头文件
#include "zf_libraries_headfile.h"

// 宏定义
#define SERVO_MIN_ANGLE     (67.5f)             // 最小角度 (67.5度)
#define SERVO_MED_ANGLE     (90.0f)             // 中值角度 (90度)
#define SERVO_MAX_ANGLE     (106.2f)            // 最大角度 (106.2度)

#define SERVO_MIN_PULSE     (620)               // 70.2度对应的脉冲宽度 (620)
#define SERVO_MED_PULSE     (750)               // 90度对应的脉冲宽度 (750)
#define SERVO_MAX_PULSE     (840)               // 106.2度对应的脉冲宽度 (840)

#define SERVO_FREQ          50                  // PWM频率 (50HZ = 0.02s = 20ms)

// 枚举值
typedef enum
{
    SERVO_OK = 0,                         // 操作成功
    SERVO_ERROR_UNKNOW,                   // 未知错误
    SERVO_ERROR_ANGLE_OUT_OF_RANGE,       // 角度超出范围
    SERVO_ERROR_PULSE_OUT_OF_RANGE,       // 脉冲宽度超出范围
    SERVO_ERROR_PWM_NOT_INIT,             // PWM模块未初始化
    SERVO_ERROR_PWM_CHANNEL_NOT_INIT,     // PWM通道未初始化
    SERVO_ERROR_PWM_SET_FAILED            // PWM设置失败
} servo_operation_state_enum;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     init_angle          要设置的初始化占空比
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_init(PWM_TIM8_CH1_I0, 90);
// 备注信息     PWM_TIM8_CH1_I0中I0引脚对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_init(zf_pwm_positive_channel_enum pin, uint16 init_angle);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机设置占空比
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     servo_duty          要设置的占空比
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_set_angle(PWM_TIM8_CH1_I0, 750);
// 备注信息     PWM_TIM8_CH1_I0中I0口对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_set_duty(zf_pwm_positive_channel_enum pin, uint32 servo_duty);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机设置转动角度
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     angle               要设置的转动的角度
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_set_angle(PWM_TIM8_CH1_I0, 90);
// 备注信息     PWM_TIM8_CH1_I0中I0口对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_set_angle(zf_pwm_positive_channel_enum pin, float servo_angle);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机遥控器控制
// 参数说明     channel_data           遥控器通道数据
// 返回参数     void
// 使用示例     dlw_moto_control(channel_ptr[1]);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_servo_control(uint16 channel_data);

///-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机PID控制
// 参数说明     pid_output        PID输出值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_servo_stabilize(float pid_output);

#endif /* USER_CODE_SERVO_H_ */
