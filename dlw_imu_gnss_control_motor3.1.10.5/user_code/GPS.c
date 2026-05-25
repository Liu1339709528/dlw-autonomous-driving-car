/*
 * GPS.c
 *
 *  Created on: 2025年5月28日
 *      Author:
 */

#include "GPS.h"

#define FLASH_SECTION_INDEX       (0)                                           // 存储数据用的扇区 倒数第一个扇区
#define FLASH_PAGE_INDEX          (0)                                           // 存储数据用的页码 倒数第一个页码

int      	RP_MAX=30;//点数
int  		GL_NUM=0;

double    Target_point[2][150]={0};//用于储存采集的目标点信息，用于后续的位置计算(第一层是纬度，第二层是经度)
float    Work_target_array[2][150];//用于将从flash中读取的目标点数组的数据复制过来，当这个数组被赋值时，后续的一切和Flash再无关系
double   lat; //纬度
double   lot; //经度

uint32   I_lat;//double类型纬度数据整数部分
float    F_lat;//double类型纬度数据小数部分(强制转换为float)(这里用float类型定义是因为当整数部分位0时，float类型可以存到后7位)

uint32   I_lot;//double类型经度数据整数部分
float    F_lot;//double类型经度数据小数部分(强制转换为float)

int Number = 0;
int GL_IMU_Flag = 0;                                        //发车标志位
float virtual_north ;                                       //虚拟北
double initial_imu_heading;                                 //获取初始航向角
extern eulerangles current_angles;                          // 当前欧拉角
extern uint8 key;

void GPS_GL_GET(void)
{
	 static int NUM=0;                                      //采集点的次数
	 {
		lat=gnss_info.latitude;
		lot=gnss_info.longitude;

		 I_lat=(uint32)(lat);//经过强转对lat取整得到整数部分
		 F_lat=lat-I_lat; //用double类型数据-整数部分得到小数部分并强制转换为float类型

		 I_lot=(uint32)(lot);//经过强转对lat取整得到整数部分
		 F_lot=lot-I_lot; //用double类型数据-整数部分得到小数部分并强制转换为float类型
		ips200_clear();
		ips200_show_string(0,16*0,"R:");
		{
			flash_union_buffer[Number].uint32_type=I_lat;  //将整数维度数据强制转换后储存在FLASH 操作的数据缓冲区
			ips200_show_string(50, 16*0, "lat:");
			ips200_show_uint (50, 16*1, flash_union_buffer[Number].uint32_type, 5);  //显示整数部分
			ips200_show_uint(150, 16*1, Number, 3);//缓冲区数组位数

			Number++;

			flash_union_buffer[Number].float_type=F_lat; //将浮点数维度数据强制转换后储存在FLASH 操作的数据缓冲区
			ips200_show_float(50, 16*2, flash_union_buffer[Number].float_type, 3, 6);//显示小数部分
			ips200_show_uint(150, 16*2, Number, 3);//缓冲区数组位数

			ips200_show_float(50, 16*3, gnss_info.latitude, 3, 6);//显示完整坐标

		}
		 Number++;//数组下标+1,切换储存经度

		{
		    ips200_show_string(50, 16*0, "lot:");
		    flash_union_buffer[Number].uint32_type=I_lot;  //将整数经度数据强制转换后储存在FLASH 操作的数据缓冲区
		    ips200_show_uint (50, 16*5, flash_union_buffer[Number].uint32_type, 5);  //显示整数部分
		    ips200_show_uint(150, 16*5, Number, 3);//缓冲区数组位数

		    Number++;

		    flash_union_buffer[Number].float_type=F_lot; //将浮点数经度数据强制转换后储存在FLASH 操作的数据缓冲区
		    ips200_show_float(50, 16*6, flash_union_buffer[Number].float_type, 3, 6);//显示小数部分
		    ips200_show_uint(150, 16*6, Number, 3);//缓冲区数组位数

		    ips200_show_float(50, 16*7, gnss_info.longitude, 3, 6);//显示完整坐标
		}
		 Number++;//数组下标+1,切换储存纬度
         //如果我没有手动清除GPS_FLASH，则下面这段语句会自动清除GPS_FLASH，正常流程是:每次采集点位之前先清空GPS_FLASH
		 if(zf_flash_check_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))              // 判断是否有数据
		 {
		         zf_flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);             // 擦除这一页
		 }
		 zf_flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);     // 向指定 Flash 扇区的页码写入缓冲区数据
	 }
	 NUM++;
	 GL_NUM=NUM;
	     if(NUM>RP_MAX)                                                               //如果采集点次数大于规定次数的最大值，则让他等于1
	     {
	          NUM=1;
	     }

	     ips200_show_uint(210, 0, NUM, 3);                                              //显示已采集的点数
}
void GPS_Flash_use(void)//将GPS_FLASH的数据重新读回缓冲区并赋值给数组
{

	zf_flash_buffer_clear();//清空缓冲区

	 if(zf_flash_check_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX)==FLASH_STATE_PAGE_OCCUPIED)                      //判断Flash是否有数据
    {
		 zf_flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);       //将数据从FLASH指定扇区页码放入到缓冲区

        int TG=0;//用以切换二维数组下标

        for(int data=0;data<200;data+=4)
          {

            Target_point[0][TG]=(double)flash_union_buffer[data].uint32_type+(double)flash_union_buffer[data+1].float_type; //纬

            Target_point[1][TG]=(double)flash_union_buffer[data+2].uint32_type+(double)flash_union_buffer[data+3].float_type; //经

            TG++;//下标+1

          }


    }

}

uint8 GL_CRC()//按照规则按下就会发车
{
	static uint8 state = 0; // 状态：0-初始，1-采集模式，2-使用Flash模式
	uint16 current_ch3 = uart_receiver.channel[2];
	static uint16 last_ch3_state = 0; // 记录上次通道3的值
	// 状态0：初始状态
	if (state == 0)
	{
	    if (key == KEY_C10_PRES) // 如果按下key1，进入采集模式
	    {
	        state = 1;
	        ips200_clear();
	        ips200_show_string(0, 0, "Sampling mode");
	    }
	    if (key == KEY_C9_PRES) // 如果按下key2，进入Flash模式
	    {
	        ips200_clear();
            ips200_show_string(0, 0, "Flash");
	        state = 2;
	    }
	    if (key == KEY_C7_PRES) // 如果按下key4，进入PID调节角度模式
        {
            ips200_clear();
            ips200_show_string(0, 0, "PID control");
            state = 3;
        }
	}
	// 状态1：采集模式（由key1进入）
	else if (state == 1)
	{
	    current_ch3 = uart_receiver.channel[2];
	    // 在采集模式下，如果按下遥控器的SWA，则采集一个点
	    // 检测通道3状态变化（192->1792或1792->192）
        if ((last_ch3_state < 1000 && current_ch3 > 1000) ||
            (last_ch3_state > 1000 && current_ch3 < 1000))
        {
            // 加入50ms延时防抖
            zf_delay_ms(50);
            // 延时后重新读取通道值，确认状态变化稳定
            current_ch3 = uart_receiver.channel[2];
            if ((last_ch3_state < 1000 && current_ch3 > 1000) ||
                (last_ch3_state > 1000 && current_ch3 < 1000))
            {
                GPS_GL_GET();  // 状态变化时采集GPS数据
            }
            last_ch3_state = current_ch3; // 更新状态记录
        }
	    // 如果按下key3，则退出采集模式
	    if (key == KEY_C8_PRES)
	    {
	        state = 0; // 回到初始状态
	        ips200_clear();
	        ips200_show_string(0, 0, "exit Sampling mode");
	    }
	}
	// 状态2：使用Flash模式（由key2进入）
	else if (state == 2)
	{
	    // 在Flash模式下，如果按下key3，则读取Flash并打印
	    if (key == KEY_C8_PRES)
	    {
	        GPS_Flash_use();
	        int TG = 0;
	        for (int data = 0; data <= 20; data++)
	        {
	            printf("\r\nzushu-%d,latitude-%f,longitude-%f", data + 1, Target_point[0][TG], Target_point[1][TG]);
	            zf_delay_ms(50);
	            TG++; // 下标+1
	        }
	    }
	    // 如果按下key4，则退出Flash模式
	    if (key == KEY_C7_PRES)
	    {
	        state = 0; // 回到初始状态
	        ips200_clear();
	        zf_delay_ms(5000);
	        virtual_north=gnss_get_two_points_azimuth(gnss_info.latitude,gnss_info.longitude,Target_point[0][1],Target_point[1][1]);//获得方位角
	        initial_imu_heading=current_angles.yaw;
	        virtual_north=virtual_north-initial_imu_heading;
	        // 标准化偏移量
	        while (virtual_north >= 180.0) virtual_north -= 360.0;
	        while (virtual_north <-180.0) virtual_north += 360.0;
	        GL_IMU_Flag=1; // 设置标志
	    }
	}
	return state;

}

void dlw_gps_init(void)
{
    if(gnss_init(GNSS_TYPE_GN43RFA))
    {
        printf("GPS init error! \r\n");
    }
}
