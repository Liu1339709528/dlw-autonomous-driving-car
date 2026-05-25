/*
 * quaternion.c
 *
 *  Created on: 2025年6月16日
 *      Author: 86155
 */

#include "quaternion.h"

quaternionFilter qt_filter = {0};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数初始化
// 参数说明     void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void quaternion_init(void)
{
    // 初始化为单位四元数
    qt_filter.quat.q0 = 1.0f;
    qt_filter.quat.q1 = 0.0f;
    qt_filter.quat.q2 = 0.0f;
    qt_filter.quat.q3 = 0.0f;

    // madgwick滤波器参数 (根据实际调整)
    qt_filter.beta = MADGWICK_PARAMETERS;

    // 采样周期 (与PIT中断周期一致)
    qt_filter.sample_period = 0.005f; // 5ms

    // 初始化校准状态
    qt_filter.euler.calibrated = 0;

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数更新函数
// 参数说明     ax                      加速度计x轴
// 参数说明     ay                      加速度计y轴
// 参数说明     az                      加速度计z轴
// 参数说明     gx                      陀螺仪x轴
// 参数说明     gy                      陀螺仪y轴
// 参数说明     gz                      陀螺仪z轴
// 返回参数     void
// 使用示例
// 备注信息     此函数在定时器中断中实时更新四元数
//-------------------------------------------------------------------------------------------------------------------
void quaternion_update(float ax, float ay, float az, float gx, float gy, float gz)
{
    static float last_gx = 0, last_gy = 0, last_gz = 0;
    // 获取陀螺仪数据 (转换为弧度/秒)
    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // 低通滤波：gyro = factor * new + (1-factor) * old
    gx = GYRO_LPF_FACTOR * gx + (1 - GYRO_LPF_FACTOR) * last_gx;
    gy = GYRO_LPF_FACTOR * gy + (1 - GYRO_LPF_FACTOR) * last_gy;
    gz = GYRO_LPF_FACTOR * gz + (1 - GYRO_LPF_FACTOR) * last_gz;

    last_gx = gx;
    last_gy = gy;
    last_gz = gz;

    float q0 = qt_filter.quat.q0;
    float q1 = qt_filter.quat.q1;
    float q2 = qt_filter.quat.q2;
    float q3 = qt_filter.quat.q3;

    // 陀螺仪积分
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * qt_filter.sample_period * 0.5f;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * qt_filter.sample_period * 0.5f;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * qt_filter.sample_period * 0.5f;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * qt_filter.sample_period * 0.5f;

    // 归一化
    float recip_norm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    qt_filter.quat.q0 = q0 * recip_norm;
    qt_filter.quat.q1 = q1 * recip_norm;
    qt_filter.quat.q2 = q2 * recip_norm;
    qt_filter.quat.q3 = q3 * recip_norm;

    // 计算欧拉角
    quaternion_get_euler(&qt_filter.euler);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从四元数中计算欧拉角
// 参数说明     *angles;                     当前欧拉角
// 返回参数     void
// 使用示例
// 备注信息     得到计算后的欧拉角
//-------------------------------------------------------------------------------------------------------------------
void quaternion_get_euler(eulerangles *angles)
{
    float q0 = qt_filter.quat.q0;
    float q1 = qt_filter.quat.q1;
    float q2 = qt_filter.quat.q2;
    float q3 = qt_filter.quat.q3;

    // Roll (X轴旋转)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    angles->roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f)
    {
        angles->pitch = copysign(M_PI / 2.0f, sinp); // 如果超出范围，请使用90度
    }
    else
    {
        angles->pitch = asin(sinp);
    }

    // Yaw (Z轴旋转)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    angles->yaw = atan2(siny_cosp, cosy_cosp);

    // 转换为角度
    angles->roll  *= 180.0f / M_PI;
    angles->pitch *= 180.0f / M_PI;
    angles->yaw   *= 180.0f / M_PI;
    angles->calibrated = qt_filter.euler.calibrated; // 传递校准状态
}
