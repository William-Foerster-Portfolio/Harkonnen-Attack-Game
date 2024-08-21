/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#ifndef APP_H
#define APP_H

#include "cmu.h"
#include "gpio.h"
#include "capsense.h"
#include "os.h"
#include "stdio.h"
#include "stdlib.h"
#include "glib.h"
#include "dmd.h"
#include "fifo.h"
#include "physics.h"



#define BUTTON_TASK_STACK_SIZE         512
#define PHYSICS_TASK_STACK_SIZE        512
#define PLATFORM_TASK_STACK_SIZE       512
#define LCD_TASK_STACK_SIZE            512
#define LED0_TASK_STACK_SIZE           512
#define IDLE_TASK_STACK_SIZE           256
#define LED1_TASK_STACK_SIZE           512


#define BUTTON_TASK_PRIO               20
#define PHYSICS_TASK_PRIO              20
#define PLATFORM_TASK_PRIO             18
#define LCD_TASK_PRIO                  20
#define LED0_TASK_PRIO                 21
#define IDLE_TASK_PRIO                 22
#define LED1_TASK_PRIO                 21


//
//static uint32_t CAP_DIRECTION;
//static bool BUTTON0;
//static bool BUTTON1;
//
//

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/




void app_init(void);
void resource_create(void);
void ButtonInput_Task_Init(void);
void Physics_Task_Init(void);
void Platform_Task_Init(void);
void LCDDisplay_Task_Init(void);
void LED0_Task_Init(void);
void LED1_Task_Init(void);
void MyCallback(OS_TMR * p_tmr, void  *p_arg);
void Idle_Task_Init(void);
void MyCallback_physics(OS_TMR * p_tmr, void * p_arg);
void MyCallback_platform(OS_TMR * p_tmr, void * p_arg);
void MyCallback_lcd(OS_TMR * p_tmr, void * p_arg);
void MyCallback_Shield_Active(OS_TMR * p_tmr, void * p_arg);
void MyCallback_Shield_Recharge(OS_TMR * p_tmr, void * p_arg);


#endif  // APP_H
