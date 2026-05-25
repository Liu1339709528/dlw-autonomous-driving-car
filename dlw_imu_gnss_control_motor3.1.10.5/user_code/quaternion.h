/*
 * quaternion.h
 *
 *  Created on: 2025年6月16日
 *      Author:
 */

#ifndef USER_CODE_QUATERNION_H_
#define USER_CODE_QUATERNION_H_

#include "zf_common_typedef.h"
#include "zf_device_imu660rb.h"
#include <math.h>

#define GYRO_LPF_FACTOR                      (0.2f)              // 低通滤波系数
#define MADGWICK_PARAMETERS                  (0.1f)              // madgwick滤波器增益参数

// 四元数数据结构体
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quaternion;

// 欧拉角数据结构体
typedef struct
{
    float roll;             // 横滚角 (弧度)
    float pitch;            // 俯仰角 (弧度)
    float yaw;              // 偏航角 (弧度)
    uint8_t calibrated;     // 校准状态标志 (0=未校准, 1=已校准)
} eulerangles;

// madgwick滤波器参数结构体
typedef struct
{
    quaternion quat;         // 当前四元数
    eulerangles euler;       // 当前欧拉角
    float beta;              // 滤波器增益
    float sample_period;     // 采样周期(秒)
} quaternionFilter;

void quaternion_init(void);                                  // 初始化四元数算法
void quaternion_update(float gx, float gy, float gz);        // 更新四元数状态（使用陀螺仪数据）
void quaternion_get_euler(eulerangles *angles);              // 获取当前欧拉角

#endif /* USER_CODE_QUATERNION_H_ */
