/**
 ******************************************************************************
 * @file    usb.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "usb.h"

extern __IO uint8_t Receive_Buffer[64]; // 接收缓冲区 由库定义
extern __IO  uint32_t Receive_length ; // 接收数据长度 由库定义
__IO uint32_t packet_sent = 1; // 发送完成标志位
__IO uint32_t packet_receive = 1; // 接收完成标志位


void Usb_Init(void)
{
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
}
