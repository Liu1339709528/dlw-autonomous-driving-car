/*
 * led.h
 *
 *  Created on: 2025年6月4日
 *      Author: 刘珈琪
 */

#ifndef USER_CODE_LED_H_
#define USER_CODE_LED_H_

// 需要包含的系统头文件
#include "zf_common_headfile.h"
#include "zf_driver_headfile.h"

// 函数原形
void dlw_led_init(void);                                // LED灯初始化
void rgb_flicker_handler(uint32 event, void *ptr);      // LED灯周期中断触发回调函数

#endif /* USER_CODE_LED_H_ */
