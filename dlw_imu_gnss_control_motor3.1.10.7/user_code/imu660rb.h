/*
 * imu660rb.h
 *
 *  Created on: 2025年6月18日
 *      Author: 刘珈琪
 */

#ifndef USER_CODE_IMU660RB_H_
#define USER_CODE_IMU660RB_H_

#include "zf_libraries_headfile.h"

void dlw_imu660rb_init(void);                              // IMU初始化
void imu_pit_handler (uint32 event, void *ptr);            // IMU定时器中断回调函数
void dlw_imu660rb_calibration(void);                       // IMU校准函数


#endif /* USER_CODE_IMU660RB_H_ */
