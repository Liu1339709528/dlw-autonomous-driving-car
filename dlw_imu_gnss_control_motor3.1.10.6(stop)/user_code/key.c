/*
 * key.c
 *
 *  Created on: 2025年6月25日
 *      Author: 刘珈琪
 */

#include "key.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键初始化
// 参数说明     void            无返回值
// 返回参数     void            无返回值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_key_init(void)
{
    zf_gpio_init(KEY_C7,  GPI_PULL_UP, GPIO_HIGH);
    zf_gpio_init(KEY_C8,  GPI_PULL_UP, GPIO_HIGH);
    zf_gpio_init(KEY_C9,  GPI_PULL_UP, GPIO_HIGH);
    zf_gpio_init(KEY_C10, GPI_PULL_UP, GPIO_HIGH);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描
// 参数说明     mode            按键模式（1为可以连按，0为不可以连按）
// 返回参数     uint8_t         哪个按键被按下
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8_t dlw_key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;                          /* 按键按松开标志 */
    uint8_t keyval = 0;
    if (mode) key_up = 1;                               /* 支持连按 */

    if (key_up && (KEY_C7_VALUE == 0 || KEY_C8_VALUE == 0 || KEY_C9_VALUE == 0 || KEY_C10_VALUE == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
        zf_delay_ms(10);           /* 去抖动 */
        key_up = 0;

        if (KEY_C7_VALUE == 0)  keyval = KEY_C7_PRES;

        if (KEY_C8_VALUE == 0)  keyval = KEY_C8_PRES;

        if (KEY_C9_VALUE == 0)  keyval = KEY_C9_PRES;

        if (KEY_C10_VALUE == 0) keyval = KEY_C10_PRES;
    }
    else if (KEY_C7_VALUE == 1 && KEY_C8_VALUE == 1 && KEY_C9_VALUE == 1 && KEY_C10_VALUE == 1)         /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;                  /* 返回键值 */
}
