/*********************************************************************************************************************
* Stellar-SR5E1E3 Opensource Library 即（Stellar-SR5E1E3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 Stellar-SR5E1E3 开源库的一部分
*
* Stellar-SR5E1E3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          StellarStudio 7.0.0
* 适用平台          Stellar-SR5E1E3
* 店铺链接          https://www.seekfree.cn/
********************************************************************************************************************/

#include "zf_libraries_headfile.h"

#include "led.h"
#include "moto.h"
#include "servo.h"
#include "key.h"
#include "remote_control.h"
#include "quaternion.h"
#include "imu660rb.h"
#include "pid.h"
#include "GPS.h"

// ****************************** 测试说明 ***************************
// PE4------>电机驱动PWM引脚
// PE2------>电机驱动DIR引脚
// PD13----->调试用示波器引脚
// PI0------>舵机信号引脚
// PG15----->绿色灯引脚
// PG14----->红色灯引脚
// PH0------>蓝色灯引脚
// PA6------>遥控器接收数据引脚
// ******************************************************************

#define MAX_DUTY                       (30)                     // 最大 MAX_DUTY% 占空比

static const uint16 *channel_ptr  =   uart_receiver.channel;    // 指向遥控器存储数据区域的指针

extern int16 gyro_bias_x;                                       // 陀螺仪零偏值变量
extern int16 gyro_bias_y;                                       // 陀螺仪零偏值变量
extern int16 gyro_bias_z;                                       // 陀螺仪零偏值变量

imu660rb_measurement_data_struct imu_data[2]          = {0};    // IMU原始数据
imu660rb_physical_data_struct    imu_physical_data[2] = {0};    // IMU物理量数据（物理量为弧度每秒）
pid_controller imu_pid;                                         // 航向角pid控制器
eulerangles    current_angles;                                  // 当前欧拉角
uint8 key = 0;                                                  // 键值
uint8 calibration_done;                                         // 校准标志

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主函数
// 参数说明     void            无参数
// 返回参数     void            无返回值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
int main (void)
{
    zf_system_clock_init(SYSTEM_CLOCK_300M);                         // 初始化芯片时钟 工作频率为 300MHz
    debug_init();                                                    // 初始化默认 Debug UART

    float pid_output = 0;                                            // PID输出
    uint8 gnns_sum = 1;
    double current_yaw;                                              // 当前航向角
    double current_distance;                                         // 当前坐标到目标点的GPS距离
    double current_yaw_180;                                          // 当前航向角(-180-180)

    dlw_led_init();                                                  // LED初始化
    dlw_key_init();                                                  // 按键初始化
    dlw_moto_pwm_init();                                             // 电机初始化
    dlw_servo_init(PWM_TIM8_CH1_I0, SERVO_MED_PULSE);                // 舵机初始化
    dlw_remote_control_init();                                       // 遥控器初始化
    dlw_imu660rb_init();											 // IMU初始化
    dlw_imu660rb_calibration();                                      // IMU校准
    quaternion_init();                                               // 四元数初始化
    pid_init(&imu_pid);                                              // pid初始化
    pid_set_limits(&imu_pid, -100.0f, 100.0f);                       // 输出限制在±100之间
    ips200_init(IPS200_TYPE_SPI);                                    // 屏幕初始化
    dlw_gps_init();                                                  // RTK初始化

    zf_pit_ms_init(PIT_TIM7, 500, rgb_flicker_handler, NULL);        // LED灯专属定时器中断初始化
    zf_pit_ms_init(PIT_TIM6, 5, imu_pit_handler, imu_data);			 // imu定时器中断初始化
    zf_pit_ms_init(PIT_TIM1, 10, RTK_pit_handler, NULL);			 // RTK定时器中断初始化

    zf_pit_set_interrupt_priority(PIT_TIM7, INTERRUPT_PRIORITY_LOW); // 中断优先级最低
    zf_pit_set_interrupt_priority(PIT_TIM6, INTERRUPT_PRIORITY_HIGH);// 中断优先级最高
    zf_pit_set_interrupt_priority(PIT_TIM1, INTERRUPT_PRIORITY_1);   // 中断优先级中间

    zf_delay_ms(1000);

    for( ; ; )
    {
        zf_delay_ms(1);
        key = dlw_key_scan(0);
        if(GL_IMU_Flag == 1)
        {
           zf_gpio_set_level(E2, GPIO_HIGH);       // DIR引脚高电平（正转）
           quaternion_get_euler(&current_angles);//计算欧拉角

           ips200_show_string(0, 16*0, "car run:");
           ips200_show_string(0, 16*1, "next lat/lot:");
           ips200_show_string(0, 16*3, "now lat/lot:");
           ips200_show_string(0, 16*5, "go angle:");
           ips200_show_string(0, 16*7, "virtual_north:");
           ips200_show_string(0, 16*9, "current_yaw:");
           ips200_show_string(0, 16*11, "imu_yaw:");

           ips200_show_float(50, 16*2, Target_point[0][gnns_sum], 3, 6);    //显示完整纬度坐标
           ips200_show_float(150, 16*2, Target_point[1][gnns_sum], 3, 6);   //显示完整经度坐标
           ips200_show_float(50, 16*4, gnss_info.latitude, 3, 6);           //显示当前纬度坐标
           ips200_show_float(150, 16*4, gnss_info.longitude, 3, 6);         //显示当前经度坐标
           ips200_show_float(50, 16*6, (current_yaw_180), 3, 6);			//显示航向角
           ips200_show_float(50, 16*8, (virtual_north), 3, 6);              //显示修正角
           ips200_show_float(50, 16*10, (current_yaw), 3, 6);               //显示当前方向角
           ips200_show_float(50, 16*12, (current_angles.yaw), 3, 6);        //显示当前IMU测量角度（相对于发车点是0度）
           //角度变换
           current_yaw=gnss_get_two_points_azimuth(gnss_info.latitude,gnss_info.longitude,Target_point[0][gnns_sum],Target_point[1][gnns_sum]);//获得方位角
           current_yaw_180=current_yaw-virtual_north;
           while (current_yaw_180 >= 180.0) current_yaw_180 -= 360.0;
           while (current_yaw_180 <-180.0) current_yaw_180 += 360.0;
           imu_pid.setpoint=-(current_yaw_180);
           //距离检测
           current_distance=gnss_get_two_points_distance(gnss_info.latitude,gnss_info.longitude,Target_point[0][gnns_sum],Target_point[1][gnns_sum]);//获得两点之间的距离
           pid_output = pid_compute(&imu_pid, current_angles.yaw);
           dlw_servo_stabilize(pid_output);
           if(gnns_sum >= 5&&gnns_sum <= 8)
           {
        	   dlw_set_motor_duty(2400);
           }
           else
           {
               dlw_set_motor_duty(2600);
           }
           if(current_distance < 1)
           {
               if(gnns_sum==0)
               {
                   GL_IMU_Flag=0;//stop
                   dlw_set_motor_duty(0);
               }
               gnns_sum++;

               if(Target_point[0][gnns_sum] == 0)
               {
                   gnns_sum=0;
               }
           }

        }
        else if(GL_IMU_Flag == 0)
        {
              dlw_remote_control_get_data();
              dlw_servo_control(channel_ptr[0]);
              dlw_moto_control(channel_ptr[1]);
              ips200_show_string(70, 0, "remote control");
              GL_CRC();
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     imu周期中断触发函数
// 参数说明     event           事件
// 参数说明     *ptr            用户传参句柄
// 返回参数     void            无返回值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void imu_pit_handler (uint32 event, void *ptr)
{
    (void)event;
    imu660rb_measurement_data_struct *data_buffer       = (imu660rb_measurement_data_struct *)ptr;
//    imu660rb_physical_data_struct *physical_data_buffer = (imu660rb_physical_data_struct *)imu_physical_data;
    imu660rb_get_acc (data_buffer + 0);
    imu660rb_get_gyro(data_buffer + 1);

    if(calibration_done)
    {
        data_buffer[1].x -= gyro_bias_x;
        data_buffer[1].y -= gyro_bias_y;
        data_buffer[1].z -= gyro_bias_z;
    }

    data_buffer[1].x = data_buffer[1].x / 10 * 10;
    data_buffer[1].y = data_buffer[1].y / 100 * 100;
    data_buffer[1].z = data_buffer[1].z / 10 * 10;      // 粗糙滤波

    imu660rb_physical_data_struct acc_phy, gyro_phy;

    // 获取物理量数据（已去除零偏）
    imu660rb_get_physical_acc((imu660rb_measurement_data_struct *)ptr, IMU_ACC_RANGE_SGN_8G, &acc_phy);
    imu660rb_get_physical_gyro((imu660rb_measurement_data_struct *)ptr + 1, IMU_GYRO_RANGE_SGN_2000DPS, &gyro_phy);

    // 传入四元数算法
    quaternion_update(
        gyro_phy.x, gyro_phy.y, gyro_phy.z
    );
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     RTK周期中断触发函数
// 参数说明     event           事件
// 参数说明     *ptr            用户传参句柄
// 返回参数     void            无返回值
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void RTK_pit_handler(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    gnss_data_parse();
}
