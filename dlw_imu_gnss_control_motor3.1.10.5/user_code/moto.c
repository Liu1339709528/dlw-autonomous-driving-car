/*
 * moto.c
 *
 *  Created on: 2025年6月6日
 *      Author:
 */

#include "moto.h"

#define MAX_DUTY            (3000 )                                               // 最大 MAX_DUTY% 占空比
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机PWM初始化
// 返回参数     void
// 使用示例
// 备注信息     PE4对应PWM输出引脚，PE2对应DIR正反转引脚
//-------------------------------------------------------------------------------------------------------------------
void dlw_moto_pwm_init(void)
{
    zf_hrtim_module_init(HRTIM_1, 17000);
    zf_hrtim_channel_init(HRTIM1_CHA1_E4 , 0);
    zf_hrtim_channel_init(HRTIM1_CHD1_D13, 0);
    zf_gpio_init(E2 , GPO_PUSH_PULL, GPIO_HIGH);
    dlw_set_motor_duty(0);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机设置占空比
// 参数说明     duty                要设置的占空比
// 返回参数     void
// 使用示例
// 备注信息     PE4对应PWM输出引脚，PE2对应DIR正反转引脚
//-------------------------------------------------------------------------------------------------------------------
void dlw_set_motor_duty(uint32 duty)
{
    if (duty > MAX_DUTY)
    {
        duty = MAX_DUTY;
    }
    zf_hrtim_set_duty(HRTIM1_CHA1_E4, duty);
    zf_hrtim_set_duty(HRTIM1_CHD1_D13, duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制
// 参数说明     channel_data           遥控器传入数据
// 返回参数     void
// 使用示例
// 备注信息     PE4对应PWM输出引脚，PE2对应DIR正反转引脚
//-------------------------------------------------------------------------------------------------------------------
void dlw_moto_control(uint16 channel_data)
{
    static uint32 moto_duty = 0;

    // 正转逻辑：channel_data在1000-1800时，占空比0到MAX_DUTY线性增加
    if (channel_data > 1000)
    {
        zf_gpio_set_level(E2, GPIO_HIGH);       // DIR引脚高电平（正转）
        moto_duty = (channel_data - 1000) * MAX_DUTY / 800;  // 线性映射：(1800-1000)=800对应MAX_DUTY
        if (moto_duty > MAX_DUTY)               // 限制最大占空比（防止超范围）
            moto_duty = MAX_DUTY;
    }
    // 反转逻辑：channel_data在200-1000时，占空比0到MAX_DUTY线性增加
    else if (channel_data < 1000)
    {
        zf_gpio_set_level(E2, GPIO_LOW);        // DIR引脚低电平（反转）
        moto_duty = (1000 - channel_data) * MAX_DUTY / 800;  // 线性映射：(1000-200)=800对应MAX_DUTY
        if (moto_duty > MAX_DUTY)               // 限制最大占空比（防止超范围）
            moto_duty = MAX_DUTY;
    }
    // 中间点：channel_data=1000时，占空比为0（电机停止）
    else
    {
        moto_duty = 0;
    }
    if(!(uart_receiver.state))
    {
        moto_duty = 0;
    }

    dlw_set_motor_duty(moto_duty);              // 设置电机占空比
}
