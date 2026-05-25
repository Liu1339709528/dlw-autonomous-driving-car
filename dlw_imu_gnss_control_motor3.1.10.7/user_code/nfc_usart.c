#include "nfc_usart.h"
/*
 * nfc_usart.c
 *
 *  Created on: 2025年8月8日
 *      Author: 27962
 */
uint32 get_data_len = 20;
uint8_t   get_data[20];
uint8_t   fifo_buffer[20];
uint8_t left = 0;
uint8_t right = 0;
zf_fifo_obj_struct uart_fifo;
extern int GL_IMU_Flag;
extern eulerangles    current_angles;                                  // 当前欧拉角
void dlw_nfc_usart_init(void)
{
    zf_fifo_init(&uart_fifo, FIFO_DATA_8BIT, fifo_buffer, sizeof(fifo_buffer));
//    zf_uart_init(UART_2, 115200, UART2_TX_D8, UART2_RX_D7);
//    zf_uart_set_interrupt_callback(UART_2, NFC_uart_handler, &uart_fifo);
//    zf_uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
}
void dlw_nfc_usartdata_process(void)
{
//	zf_fifo_read_buffer(&uart_fifo, get_data, &get_data_len, FIFO_READ_WITH_CLEAN);
	get_data_len =  debug_read_ring_buffer(get_data, 20);
	if (get_data_len)
	{
		debug_uart_write_buffer(get_data, &get_data_len);
		get_data_len = 0;
	}

	if(strstr((const char*)get_data,"left") != NULL)
	{
		left = 1;
		right = 0;
		GPS_Flash_use();
        ips200_clear();
		zf_delay_ms(5000);
		virtual_north=gnss_get_two_points_azimuth(gnss_info.latitude,gnss_info.longitude,Target_point[0][1],Target_point[1][1]);//获得方位角
		initial_imu_heading=current_angles.yaw;
		virtual_north=virtual_north-initial_imu_heading;
		// 标准化偏移量
		while (virtual_north >= 180.0) virtual_north -= 360.0;
		while (virtual_north <-180.0) virtual_north += 360.0;
		GL_IMU_Flag=1; // 如果需要设置标志，可以在这里设置
	}
	if(strstr((const char*)get_data,"right") != NULL)
	{
		left = 0;
		right = 1;
		GPS_Flash_use();
        ips200_clear();
		zf_delay_ms(5000);
		virtual_north=gnss_get_two_points_azimuth(gnss_info.latitude,gnss_info.longitude,Target_point[0][1],Target_point[1][1]);//获得方位角
		initial_imu_heading=current_angles.yaw;
		virtual_north=virtual_north-initial_imu_heading;
		// 标准化偏移量
		while (virtual_north >= 180.0) virtual_north -= 360.0;
		while (virtual_north <-180.0) virtual_north += 360.0;
		GL_IMU_Flag=1; // 如果需要设置标志，可以在这里设置
	}
}
