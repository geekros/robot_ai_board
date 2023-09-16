/**
 ******************************************************************************
 * @file    usb.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "usb.h"

extern __IO uint8_t Receive_Buffer[64]; // ���ջ����� �ɿⶨ��
extern __IO  uint32_t Receive_length ; // �������ݳ��� �ɿⶨ��
__IO uint32_t packet_sent = 1; // ������ɱ�־λ
__IO uint32_t packet_receive = 1; // ������ɱ�־λ


void Usb_Init(void)
{
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
}
