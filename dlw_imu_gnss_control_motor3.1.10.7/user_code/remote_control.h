/*
 * remote_control.h
 *
 *  Created on: 2025年6月4日
 *      Author: 刘珈琪
 */

#ifndef USER_CODE_REMOTE_CONTROL_H_
#define USER_CODE_REMOTE_CONTROL_H_

// 需要包含的系统头文件
#include "zf_libraries_headfile.h"

// 函数原形
void dlw_remote_control_init(void);                 // 遥控器初始化
void dlw_remote_control_get_data(void);             // 遥控器得到数据（数据处理在回调函数中完成，这里仅将串口标志位置0）

#endif /* USER_CODE_REMOTE_CONTROL_H_ */
