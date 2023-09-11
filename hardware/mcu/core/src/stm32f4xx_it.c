/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  GEEKROS,  site:www.geekros.com
  ******************************************************************************
*/

#include "stm32f4xx_it.h"

extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE * pdev);

extern void xPortSysTickHandler(void);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler(USB_OTG_CORE_HANDLE * pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler(USB_OTG_CORE_HANDLE * pdev);
#endif

/**
  ******************************************************************************
  * This function handles NMI exception.
  ******************************************************************************
*/
void NMI_Handler(void)
{
	
}

/**
  ******************************************************************************
  * This function handles Hard Fault exception.
  ******************************************************************************
*/
void HardFault_Handler(void)
{
  while (1)
  {
		
  }
}

/**
  ******************************************************************************
  * This function handles Memory Manage exception.
  ******************************************************************************
*/
void MemManage_Handler(void)
{
  while (1)
  {
		
  }
}

/**
  ******************************************************************************
  * This function handles Bus Fault exception.
  ******************************************************************************
*/
void BusFault_Handler(void)
{
  while (1)
  {
		
  }
}

/**
  ******************************************************************************
  * This function handles Usage Fault exception.
  ******************************************************************************
*/
void UsageFault_Handler(void)
{
  while (1)
  {
		
  }
}

/**
  ******************************************************************************
  * This function handles SVCall exception.
  ******************************************************************************
*/
//void SVC_Handler(void)
//{
//	
//}

/**
  ******************************************************************************
  * This function handles Debug Monitor exception.
  ******************************************************************************
*/
void DebugMon_Handler(void)
{
	
}

/**
  ******************************************************************************
  * This function handles PendSVC exception.
  ******************************************************************************
*/
//void PendSV_Handler(void)
//{
//
//}

/**
  ******************************************************************************
  * This function handles SysTick Handler.
  ******************************************************************************
*/
volatile uint32_t ms_ticks = 0;
void SysTick_Handler(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}else{
		ms_ticks++;
	}
}

uint32_t Get_SysTick(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)
	{
		return xTaskGetTickCount();
	}
	else
	{
		return ms_ticks;
	}
}

#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
/**
  * @brief  This function handles EP1_IN Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler(&USB_OTG_dev);
}

/**
  * @brief  This function handles EP1_OUT Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler(&USB_OTG_dev);
}
#endif
