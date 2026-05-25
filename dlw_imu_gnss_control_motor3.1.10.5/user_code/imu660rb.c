/*
 * imu660rb.c
 *
 *  Created on: 2025年6月18日
 *      Author:
 */

#include "imu660rb.h"

int16 gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;        // 陀螺仪零偏值变量
extern uint8_t calibration_done;                                // 校准完成标志

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     imu660rb初始化
// 参数说明     void                     当前欧拉角
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_imu660rb_init(void)
{
    while(1)
    {
        if(imu660rb_init())
        {
            printf("imu660rb init error.\r\n");                      // IMU660RB 初始化失败
            zf_gpio_set_level(G14, GPIO_LOW);                        // 初始化失败红灯点亮
        }
        else
        {
            zf_gpio_set_level(G14, GPIO_HIGH);                       // 初始化成功红灯熄灭
            break;
        }
    }
    zf_delay_ms(1000);                                               // 延时一段时间，等待数据准确
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     imu校准
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void dlw_imu660rb_calibration(void)
{
    uint16 samples = 100;                               // 采集100个样本
    int32 sum_x = 0, sum_y = 0, sum_z = 0;

    for(uint16 i = 0; i < samples; i++)
    {
        imu660rb_measurement_data_struct gyro_raw;
        imu660rb_get_gyro(&gyro_raw);                   // 直接读取原始陀螺仪数据

        sum_x += gyro_raw.x;
        sum_y += gyro_raw.y;
        sum_z += gyro_raw.z;

        zf_delay_ms(10);                                // 10ms采样间隔
    }
    gyro_bias_x = sum_x / samples;
    gyro_bias_y = sum_y / samples;
    gyro_bias_z = sum_z / samples;
    calibration_done = 1;                           // 校准完成标志置位
}
