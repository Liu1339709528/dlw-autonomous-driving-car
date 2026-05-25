/*
 * key.h
 *
 *  Created on: 2025年6月25日
 *      Author: 刘珈琪
 */

#ifndef USER_CODE_KEY_H_
#define USER_CODE_KEY_H_

#include "zf_device_key.h"
#include "zf_driver_gpio.h"

//按键宏定义
#define KEY_C7                       (C7)
#define KEY_C8                       (C8)
#define KEY_C9                       (C9)
#define KEY_C10                      (C10)

#define KEY_C7_VALUE        zf_gpio_get_level(KEY_C7)     /*  读取KEY_C7引脚  */
#define KEY_C8_VALUE        zf_gpio_get_level(KEY_C8)     /*  读取KEY_C8引脚  */
#define KEY_C9_VALUE        zf_gpio_get_level(KEY_C9)     /*  读取KEY_C9引脚  */
#define KEY_C10_VALUE       zf_gpio_get_level(KEY_C10)    /*  读取KEY_C10引脚 */

#define KEY_C7_PRES                   4              /*  KEYC7按下  */
#define KEY_C8_PRES                   3              /*  KEYC8按下  */
#define KEY_C9_PRES                   2              /*  KEYC9按下  */
#define KEY_C10_PRES                  1              /*  KEYC10按下 */

void dlw_key_init(void);               /* 按键初始化函数 */
uint8_t dlw_key_scan(uint8_t mode);    /* 按键扫描函数 */

#endif /* USER_CODE_KEY_H_ */
