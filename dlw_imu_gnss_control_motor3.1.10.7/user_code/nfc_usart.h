/*
 * nfc_usart.h
 *
 *  Created on: 2025年8月8日
 *      Author: 27962
 */

#ifndef USER_CODE_NFC_USART_H_
#define USER_CODE_NFC_USART_H_

#include "zf_libraries_headfile.h"
#include "stdio.h"
#include "GPS.h"
void dlw_nfc_usart_init(void);
void NFC_uart_handler (uint32 event, void *ptr);
void dlw_nfc_usartdata_process(void);
#endif /* USER_CODE_NFC_USART_H_ */
