/*
 * servo.c
 *
 *  Created on: 2025年6月3日
 *      Author: 刘珈琪
 */

#include "servo.h"
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     init_servo_duty     要设置的初始化占空比
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_init(PWM_TIM8_CH1_I0, 90);
// 备注信息     PWM_TIM8_CH1_I0中I0引脚对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_init(zf_pwm_positive_channel_enum pin, uint16 init_servo_duty)
{
    servo_operation_state_enum return_state = SERVO_ERROR_UNKNOW;
    do
    {
        if(init_servo_duty < SERVO_MIN_PULSE || init_servo_duty > SERVO_MAX_PULSE)
        {
            return_state = SERVO_ERROR_PULSE_OUT_OF_RANGE;
            break;
        }

        zf_pwm_module_init(PWM_TIM8, PWM_ALIGNMENT_EDGE, 50);
        zf_pwm_channel_init(pin, SERVO_MED_PULSE);

        dlw_servo_set_duty(pin, 800);
        zf_delay_ms(500);

        dlw_servo_set_duty(pin, 650);
        zf_delay_ms(500);

        dlw_servo_set_duty(pin, 750);
        zf_delay_ms(500);

        dlw_servo_set_duty(pin, init_servo_duty);
        zf_delay_ms(1000);

        return_state = SERVO_OK;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机设置占空比
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     servo_duty          要设置的占空比度数
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_set_angle(PWM_TIM8_CH1_I0, 90);
// 备注信息     PWM_TIM8_CH1_I0中I0口对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_set_duty(zf_pwm_positive_channel_enum pin, uint32 servo_duty)
{
    servo_operation_state_enum return_state = SERVO_ERROR_UNKNOW;
    do
    {
      if(servo_duty < SERVO_MIN_PULSE || servo_duty > SERVO_MAX_PULSE)
      {
          return_state = SERVO_ERROR_PULSE_OUT_OF_RANGE;
          break;
      }
      zf_pwm_set_duty(pin, servo_duty);

      return_state = SERVO_OK;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机设置转动角度
// 参数说明     pin                 舵机PWM引脚 (详见 zf_driver_pwm.h)
// 参数说明     angle               要设置的转动的角度
// 返回参数     uint8               操作状态 ZF_NO_ERROR / TIM_OPERATION_DONE - 完成 其余值为异常
// 使用示例     dlw_servo_set_angle(PWM_TIM8_CH1_I0, 90);
// 备注信息     PWM_TIM8_CH1_I0中I0口对应舵机的信号接口
//-------------------------------------------------------------------------------------------------------------------
uint8 dlw_servo_set_angle(zf_pwm_positive_channel_enum pin, float servo_angle)
{
    servo_operation_state_enum return_state = SERVO_ERROR_UNKNOW;
    uint32 servo_duty = 0;
    do
    {
      if(servo_angle < SERVO_MIN_ANGLE || servo_angle > SERVO_MAX_ANGLE)
      {
          return_state = SERVO_ERROR_ANGLE_OUT_OF_RANGE;
          zf_gpio_set_level(G14, GPIO_LOW);
          break;
      }
      servo_duty = (servo_angle * 1000 / 180) +250;
      if(servo_duty < SERVO_MIN_PULSE || servo_duty > SERVO_MAX_PULSE)
      {
          return_state = SERVO_ERROR_PULSE_OUT_OF_RANGE;
          zf_gpio_set_level(G14, GPIO_LOW);
          break;
      }
      zf_pwm_set_duty(pin, servo_duty);

      return_state = SERVO_OK;
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机遥控器控制
// 参数说明     channel_data           遥控器通道数据
// 返回参数     void
// 使用示例     dlw_moto_control(channel_ptr[1]);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_servo_control(uint16 channel_data)
{
    static uint32 srevo_duty = 0;
    // 线性映射通道数据到PWM占空比
    srevo_duty = ((channel_data - 200) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / (1800 - 200)) + SERVO_MIN_PULSE;

    if(!(uart_receiver.state))
    {
        srevo_duty = SERVO_MED_PULSE;
    }
    // 确保占空比在有效范围内
    if (srevo_duty < SERVO_MIN_PULSE)
    {
        srevo_duty = SERVO_MIN_PULSE;
    }
    else if (srevo_duty > SERVO_MAX_PULSE)
    {
        srevo_duty = SERVO_MAX_PULSE;
    }
    // 设置舵机PWM占空比
    dlw_servo_set_duty(PWM_TIM8_CH1_I0, srevo_duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机PID控制
// 参数说明     pid_output        PID输出值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_servo_stabilize(float pid_output)
{
    // 计算负向区间比例 (70.2° ~ 90°)
    const float NEGATIVE_RANGE = SERVO_MED_ANGLE - SERVO_MIN_ANGLE; // 19.8°
    // 计算正向区间比例 (90° ~ 106.2°)
    const float POSITIVE_RANGE = SERVO_MAX_ANGLE - SERVO_MED_ANGLE; // 16.2°

    float angle;

    // 处理负向输出
    if (pid_output < 0)
    {
       angle = SERVO_MED_ANGLE + pid_output * (NEGATIVE_RANGE / 100.0f);
    }
    // 处理正向输出
    else
    {
       angle = SERVO_MED_ANGLE + pid_output * (POSITIVE_RANGE / 100.0f);
    }

    // 角度限幅
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;

    // 设置舵机角度
    dlw_servo_set_angle(PWM_TIM8_CH1_I0, angle);
    ips200_show_string(0, 16*17, "pid_output_angle:");
    ips200_show_float(50, 16*18, (angle), 3, 6);
}
