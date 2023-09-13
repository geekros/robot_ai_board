/**
 ******************************************************************************
 * @file    usb.c
 * @author  GEEKROS,  site:www.geekros.com
 ******************************************************************************
 */

#include "usb.h"

extern __IO uint8_t Receive_Buffer[1024];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[1024];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

void Usb_Start(void)
{
    Set_System();
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
}
