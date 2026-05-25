/*
 * remote_control.c
 *
 *  Created on: 2025年6月4日
 *      Author: 刘珈琪
 */

#include "remote_control.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     遥控器初始化
// 返回参数     void
// 使用示例
// 备注信息     遥控器的接收引脚是串口2的PA6
//-------------------------------------------------------------------------------------------------------------------
void dlw_remote_control_init(void)
{
		if(uart_receiver_init())
		{
			printf("remote control init error.\r\n");
			zf_gpio_set_level(H0, GPIO_LOW);                     // 串口接收机初始化失败
		}
		else
		{
		    printf("remote control init successful.\r\n");
            zf_gpio_set_level(H0, GPIO_HIGH);
		}

}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     遥控器得到数据
// 返回参数     void
// 使用示例
// 备注信息     遥控器得到数据（数据处理在回调函数中完成，这里仅将串口标志位置0）
//            遥控器得到的处理后的数据存在uart_receiver_struct结构体里
//-------------------------------------------------------------------------------------------------------------------
void dlw_remote_control_get_data(void)
{
	if(uart_receiver.finsh_flag)
	{
		uart_receiver.finsh_flag = 0;

		if(1 == uart_receiver.state)                             // 遥控器失控状态判断
		{
			zf_gpio_set_level(H0, GPIO_HIGH);
		}
		else
		{
			zf_gpio_set_level(H0, GPIO_LOW);
		}
		uart_receiver.finsh_flag = 0;                            // 帧完成标志复位
	}
}

