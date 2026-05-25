/*
 * GPS.h
 *
 *  Created on: 2025年5月28日
 *      Author: 王森，刘珈琪
 */

#ifndef USER_CODE_GPS_H_
#define USER_CODE_GPS_H_

#include "zf_libraries_headfile.h"

#include "led.h"
#include "moto.h"
#include "servo.h"
#include "key.h"
#include "remote_control.h"
#include "quaternion.h"
#include "imu660rb.h"
#include "pid.h"

void dlw_gps_init(void);
void GPS_GL_GET(void);
uint8 GL_CRC(void);
void RTK_pit_handler(uint32 event, void *ptr);
void encoder_pit_handler (uint32 event, void *ptr);
void GPS_Flash_use(void);
extern int GL_IMU_Flag;
extern double Target_point[2][150];
extern float virtual_north ;                            //虚拟北
extern double initial_imu_heading;                      //获取初始航向角

#endif /* USER_CODE_GPS_H_ */
