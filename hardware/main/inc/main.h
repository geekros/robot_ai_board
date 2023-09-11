/**
  ******************************************************************************
  * @file    main.h
  * @author  GEEKROS,  site:www.geekros.com
  ******************************************************************************
*/

#ifndef MAIN
#define MAIN

#include "app.h"
#include "module.h"
#include "utils.h"

#define MAIN_TASK_PRIO 1
#define MAIN_TASK_SIZE 1024
static TaskHandle_t Main_Task_Handler;

#define SYSTEM_TASK_PRIO 2
#define SYSTEM_TASK_SIZE 256
static TaskHandle_t System_Task_Handler;

#define SERIAL_TASK_PRIO 3
#define SERIAL_TASK_SIZE 1024
static TaskHandle_t Serial_Task_Handler;

#define USER_TASK_PRIO 4
#define USER_TASK_SIZE 1024
static TaskHandle_t User_Task_Handler;

uint32_t Serial_Task_Buffer_Len = 1024;
uint8_t Serial_Task_Buffer[1024];

void System_Task(void);

void Serial_Task(void);

void User_Task(void);

void Task_Manage(void);

void Main_Task(void *pvParameters);

#endif
