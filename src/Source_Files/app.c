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
#include "app.h"
#include "os.h"
#include "em_emu.h"
#include "glib.h"
#include "dmd.h"
#include "fifo.h"
#include "sl_board_control.h"
#include "sl_board_control_config.h"
#include <stdio.h>
#include <stdlib.h>


/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

static OS_TCB idle_tcb;
static CPU_STK idle_stack[IDLE_TASK_STACK_SIZE];
static OS_TCB button_tcb;
static CPU_STK button_stack[BUTTON_TASK_STACK_SIZE];
static OS_TCB physics_tcb;
static CPU_STK physics_stack[PHYSICS_TASK_STACK_SIZE];
static OS_TCB platform_tcb;
static CPU_STK platform_stack[PLATFORM_TASK_STACK_SIZE];
static OS_TCB lcd_tcb;
static CPU_STK lcd_stack[LCD_TASK_STACK_SIZE];
static OS_TCB led0_tcb;
static CPU_STK led0_stack[LED0_TASK_STACK_SIZE];
static OS_TCB led1_tcb;
static CPU_STK stack[LED1_TASK_STACK_SIZE];

// tasks

static void ButtonInput_task(void *arg);
static void Physics_task(void *arg);
static void Platform_task(void *arg);
static void LCDDisplay_task(void *arg);
static void idle_task(void *arg);
static void read_capsense(void);
static void led0_task(void *arg);
static void led1_task(void *arg);

 // semaphore

static OS_SEM      sem_button;

 // mutex

static OS_MUTEX button_mutex;
static OS_MUTEX platform_mutex;
static OS_MUTEX hm_mutex;

// buttons

bool BUTTON0;
bool BUTTON1;
int BUTTON_STATE; // 0: none pressed/ both pressed, 1: b0 pressed, 2: b1 pressed

// slider

int CAP_DIRECTION;


static GLIB_Context_t glibContext;


/*******************************************************************************
 ***************************  GAME SETTINGS   ********************************
 ******************************************************************************/

typedef struct{
  bool                enable;
  bool                limited;
  int                 max_bounce_speed;

}WALL_BOUNCE_TypeDef;
const WALL_BOUNCE_TypeDef wb_settings =
{                       //  Bounce from canyon walls
  .enable = true,       //  Enabled [T/F]
  .limited = false,     //  Limited [T/F]
  .max_bounce_speed = 0 //  MaxPlatformBounceSpeed [cm/s] (“don’t care” for Limited=FALSE)
};


typedef struct{
  int          num;
  int          display_diameter;
  int          initial_conditions;
  int          v0[2];
  int          max_xv;
  int          x0_position;
  int          user[8];

}HM_SETTINGS_TypeDef;
const HM_SETTINGS_TypeDef hm_settings =
    {                             //  HoltzmanMasses
      .num = 3,                   //  Num [-]
      .display_diameter = 10000,  //  DisplayDiameter [cm]
      .initial_conditions = 0,    //  InitialConditions [enum: Fixed=0.  127+ are available for user-defined modes]
      .mass = 100,                //  HM mass [g]
      .v0 = {                     //  (signed) InitialVelocity
               4000,              //  xvel [cm/s]
               0                  //  yvel [cm/s]
             },
      .max_xv = 10000,            //  max x velocity [cm/s]
      .x0_position = 0,           //  (signed) InitialHorizontalPosition [cm]
      .user = {0,0,0,0,0,0,0,0}   //  UserDefinedModeInput[0..7] (all “don’t care” for InitialConditions=0)
    };

typedef struct{
  int                           max_force; // platform: both or none: 0, left = 1, hard left = 2, right = 3, hard left = 4
  int                           mass;
  int                           length;
  WALL_BOUNCE_TypeDef           wall_bounce;
  bool                          auto_control;
}PLATFORM_SETTINGS_TypeDef;

static const PLATFORM_SETTINGS_TypeDef plat_settings =
    {                               // Platform
      .max_force = 20000000,        //  MaxForce [N]
      .mass = 10,                   //  Mass [kg]
      .length = 15000,              //  Length [cm]
      .wall_bounce = wb_settings,
      .auto_control = false
    };


typedef struct{
  double           KE_increase;
  int         time_active;
  int         recharge_time;

}BOOST_TypeDef;
const BOOST_TypeDef boost_settings =
    {                           //  Boost
      .KE_increase = 40,        //  KineticEnergyIncrease[%]
      .time_active = 500,       //  ArmingWindowBeforeImpact[ms]
      .recharge_time = 1000     //  RechargeTimeAfterDisarm[ms]
    };

typedef struct{
  int             min_velocity;
  double          passive_KE_reduction;
  BOOST_TypeDef             boost;

}SHIELD_TypeDef;
const SHIELD_TypeDef shield_settings =
    {                               //  HoltzmanShield
      .min_velocity = 1000,         //  MinimumEffectivePerpendicularSpeed [cm/s]
      .passive_KE_reduction = 70,   //  ExclusivelyPassiveBounceKineticEnergyReduction[%]
      .boost = boost_settings
    };


typedef struct{
  int               num_activations;
  bool              auto_control;
}LASER_TypeDef;
const LASER_TypeDef laser_settings =
    {                           //  Laser
      .num_activations = 1,     //  NumActivations [-]
      .auto_control = false     //  Automatic Control [T/F]
    };

typedef struct{
  int                                 version;
  int                                 tau_physics;
  int                                 tau_LCD;
  int                                 gravity;
  int                                 canyon_size;
  float                               canyon_pixel_conversion;
  int                                 capsense_sample_rate;
  HM_SETTINGS_TypeDef                 hm;
  PLATFORM_SETTINGS_TypeDef           platform;
  SHIELD_TypeDef                      shield;
  LASER_TypeDef                       laser;
}GAME_SETTINGS_TypeDef;
static const GAME_SETTINGS_TypeDef ConfigData =
  {
    .version = 4,                //  Data Structure Version
    .tau_physics = 50,           //  TauPhysics [ms]
    .tau_LCD = 100,              //  TauLCD [ms]
    .gravity = 9800,             //  Gravity [cm/s^2]
    .canyon_size = 100000,       //  CanyonSize [cm]
    .canyon_pixel_conversion = 128/ConfigData.canyon_size,  // Pixels to game demensions [pixel/cm]
    .capsense_sample_rate = 300, //  Capsense sample rate [ms]
    .hm = hm_settings,
    .platform = plat_settings,
    .shield = shield_settings,
    .laser = laser_settings
  };

/*******************************************************************************
 ***************************  GAME STRUCTURES   ********************************
 ******************************************************************************/

typedef struct{
  bool                   shield_active;
  bool                   shield_available;
  int                    laser_num; // number of button1 presses
  bool                   laser_active;
}Button_TypeDef;

volatile Button_TypeDef button = 
  {
    .shield_available = true, 
    .laser_num = ConfigData.laser.num_activations, 
    .shield_active = false, 
    .laser_active = false
  };

typedef struct{
  int             direction_input; // platform: both or none: 0, left = 1, hard left = 2, right = 3, hard left = 4
  int             left_position;
  int             right_position;
  int             height;
  int             velocity;
  int             mass;
  int             force;
  int             length;
  int             health;
}platform_TypeDef;

static platform_TypeDef platform = 
{
  .direction_input = 0, 
  .left_position = 0, 
  .height = 2, 
  .velocity = 0, 
  .mass = ConfigData.platform.mass, 
  .force = 0, 
  .length = ConfigData.platform.length, 
  .health = 6
  };


typedef struct{
  int         velocity[2];
  int         position[2];
  int         num;
  int         mass;
  int         min_velocity;
  int         max_velocity;
}HM_TypeDef;

static HM_TypeDef harkonnan = 
{
  .velocity = {ConfigData.hm.v0[0],ConfigData.hm.v0[1]}, 
  .position = {2,1563}, 
  .num = ConfigData.hm.num, 
  .mass = ConfigData.hm.mass, 
  .min_velocity = ConfigData.shield.min_velocity, 
  .max_velocity = -34846
  };

//timers
static OS_TMR               timer_physics;
static OS_TMR               timer_lcd;
static OS_TMR               timer_shield_active;
static OS_TMR               timer_shield_recharge;
static void                 *timer_callback_arg;


typedef enum{
  RUN,
  START,
  LASER,
  LEVEL1,
  LEVEL2,
  LEVEL3,
  GAME_OVER,
  GAME_WIN

}STAGES_TypeDef;
int current_state = RUN;
RTOS_ERR     err;
/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  int int_flag = GPIO->IF & GPIO->IEN;                          // Interupt flag handling
  GPIO->IFC = int_flag;
  BUTTON0 = GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN); // Get button inputs
  BUTTON1 = GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN);
  if(BUTTON1)
    {
      BUTTON_STATE = 1;
    }
  else
    {
      BUTTON_STATE = 0;
    }
  write_fifo(BUTTON_STATE);                                     // Add button state to FIFO
  OSSemPost(&sem_button, OS_OPT_POST_ALL, &err);
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  int int_flag = GPIO->IF & GPIO->IEN;                          // Interupt flag handling
  GPIO->IFC = int_flag;
  BUTTON0 = GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN); // Get button inputs
  BUTTON1 = GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN);
  if(BUTTON0)
    {
      BUTTON_STATE = 2;
    }
  else
    {
      BUTTON_STATE = 0;
    }
  write_fifo(BUTTON_STATE);                                     // Add button state to FIFO
  OSSemPost(&sem_button, OS_OPT_POST_ALL, &err);
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{

  resource_create();
  gpio_open();
  ButtonInput_Task_Init();
  Physics_Task_Init();
  Platform_Task_Init();
  LCDDisplay_Task_Init();
  LED0_Task_Init();
  LED1_Task_Init();
  Idle_Task_Init();

}

/***************************************************************************//**
 * Initialize ButtonInput Task
 ******************************************************************************/
void ButtonInput_Task_Init(void){



  OSTaskCreate(&button_tcb,                       // Pointer to the task's TCB.
               "button task",                     // Name to help debugging.
               ButtonInput_task,                  // Pointer to the task's code.
                DEF_NULL,                         // Pointer to task's argument.
                BUTTON_TASK_PRIO,                 // Task's priority.
               &button_stack[0],                  // Pointer to base of stack.
               (BUTTON_TASK_STACK_SIZE / 10u),    // Stack limit, from base.
               BUTTON_TASK_STACK_SIZE,            // Stack size, in CPU_STK.
                0u,                               // Messages in task queue.
                0u,                               // Round-Robin time quanta.
                DEF_NULL,                         // External TCB data.
                (OS_OPT_TASK_STK_CLR),            // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * Button task.
 ******************************************************************************/

static void ButtonInput_task(void *arg){ //semaphore pend that will be posted by the Button ISRs

  PP_UNUSED_PARAM(arg);
  int button_event;


  while (1)
  {
      OSSemPend(&sem_button, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      button_event = read_fifo();                                             // Read button state from FIFO
      OSMutexPend(&button_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      if((button_event == 1) && (button.shield_available))                    // Activate shield if availible
        {
          OSTmrStart(&timer_shield_active, &err);
          button.shield_available = false;
          button.shield_active = true;
        }
      else if((button_event == 2) && (button.laser_num != 0))                 // Activate lazer if availible
        {
          button.laser_num -= 1;
          button.laser_active = true;
        }
      OSMutexPost (&button_mutex, OS_OPT_POST_NONE, &err);
  }
}

/***************************************************************************//**
 * Initialize Physics Task
 ******************************************************************************/
void Physics_Task_Init(void){



  OSTaskCreate(&physics_tcb,                       // Pointer to the task's TCB.
               "physics task",                     // Name to help debugging.
               Physics_task,                       // Pointer to the task's code.
                DEF_NULL,                          // Pointer to task's argument.
                PHYSICS_TASK_PRIO,                 // Task's priority.
               &physics_stack[0],                  // Pointer to base of stack.
               (PHYSICS_TASK_STACK_SIZE / 10u),    // Stack limit, from base.
               PHYSICS_TASK_STACK_SIZE,            // Stack size, in CPU_STK.
                0u,                                // Messages in task queue.
                0u,                                // Round-Robin time quanta.
                DEF_NULL,                          // External TCB data.
                (OS_OPT_TASK_STK_CLR),             // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * Physics task.
 ******************************************************************************/

static void Physics_task(void *arg){ 

  PP_UNUSED_PARAM(arg);
  OS_TICK time;
  int left_plat = 0;
  int right_plat = 0;
  int plat_velocity = 0;
  float time_increment = .05;
  bool s_active = false;
  bool s_recharge = false;
  bool laser_active = false;

  OSTmrStart(&timer_physics, &err);


  while (1)
  {
    OSTimeDly(ConfigData.tau_physics/10, OS_OPT_TIME_DLY, &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
    OSMutexPend(&button_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
    s_active = button.shield_active;
    laser_active = button.laser_active;
    OSMutexPost (&button_mutex, OS_OPT_POST_NONE, &err);
    OSMutexPend(&platform_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
    platform.velocity = calc_velocity(platform.velocity, platform.force, platform.mass, time_increment);
    if(platform.velocity >= 70000) // Maximum velocity handling 
      {
        platform.velocity = 70000;
      }
    if(platform.velocity <= -70000)
      {
        platform.velocity = -70000;
      }
    plat_velocity = platform.velocity;
    platform.left_position = calc_position(platform.left_position, platform.velocity, time_increment);
    if(platform.left_position <= 0) // Left edge of screen handling
      {
        platform.left_position = 0;
      }
    platform.right_position = platform.left_position + platform.length;
    if(platform.right_position >= 100000) // Right edge of screen handling
      {
        platform.right_position = 100000;
        platform.left_position = platform.right_position - platform.length;
      }

    left_plat = platform.left_position;
    right_plat = platform.right_position;
    OSMutexPost (&platform_mutex, OS_OPT_POST_NONE, &err);

    OSMutexPend(&hm_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
    if((harkonnan.velocity[1] > 20000) && (harkonnan.position[1] >= (114.5/ConfigData.canyon_pixel_conversion)) &&      // Check velocity of HM and vertical position
        (harkonnan.position[0] >= left_plat) && (harkonnan.position[0] <= (right_plat)))    // Check HM horizontal position
      {
        if(s_active) // Handle shield active bounce 
          {
            OSTmrStart(&timer_shield_recharge, &err);
            harkonnan.velocity[0] = harkonnan.velocity[0] + (plat_velocity/2);
            harkonnan.velocity[1] = bounce(1.4, harkonnan.mass, harkonnan.velocity[1]);
            harkonnan.max_velocity = harkonnan.velocity[1];
            harkonnan.position[0] = calc_position(harkonnan.position[0], harkonnan.velocity[0], time_increment);
            harkonnan.position[1] = calc_position(harkonnan.position[1], harkonnan.velocity[1], time_increment);
          }
        else // Handle shield inactive bounce
          {
            harkonnan.velocity[0] = harkonnan.velocity[0] + (plat_velocity/2);
            harkonnan.velocity[1] = bounce(.7, harkonnan.mass, harkonnan.velocity[1]);
            harkonnan.max_velocity = harkonnan.velocity[1];
            harkonnan.position[0] = calc_position(harkonnan.position[0], harkonnan.velocity[0], time_increment);
            harkonnan.position[1] = calc_position(harkonnan.position[1], harkonnan.velocity[1], time_increment);
          }
      }
    if((harkonnan.position[0] <= (1.5*ConfigData.canyon_pixel_conversion))
        || (harkonnan.position[0] >= (126.5/ConfigData.canyon_pixel_conversion)))  // Edge of screen bounce handling
      {
        harkonnan.velocity[0] = (-1 * harkonnan.velocity[0]);
      }
    if ((harkonnan.position[1] <= 0) || (harkonnan.position[1]
         >= (118/ConfigData.canyon_pixel_conversion)) || laser_active)             // Check if HM has been bounced out of the top of the screen, hit the ground, or hit with laser
      {
        harkonnan.num --;
        if(harkonnan.position[1] >= (118/ConfigData.canyon_pixel_conversion))      // HM hit ground = Game Over
          {
            current_state = GAME_OVER;
          }
        else if(harkonnan.num == 0)                                                // Last HM has reach escape velocity or was destroyed by laer
          {
            current_state = GAME_WIN;
          }
        else                                                                       // Reset the positin and velocity for next HM
          {
            harkonnan.position[0] = 50000;
            harkonnan.position[1] = 1563;
            harkonnan.velocity[0] = 0;
            harkonnan.velocity[1] = 0;
          }
      }
    else // Normal movement update
      {
        harkonnan.velocity[0] = harkonnan.velocity[0]; 
        harkonnan.velocity[1] = gravity(harkonnan.velocity[1], ConfigData.gravity, time_increment);
        harkonnan.position[0] = calc_position(harkonnan.position[0], harkonnan.velocity[0], time_increment);
        harkonnan.position[1] = calc_position(harkonnan.position[1], harkonnan.velocity[1], time_increment);
      }
      OSMutexPost (&hm_mutex, OS_OPT_POST_NONE, &err);

  }
}

/***************************************************************************//**
 * Initialize Platform Task
 ******************************************************************************/
void Platform_Task_Init(void){

  RTOS_ERR     err;

  OSTaskCreate(&platform_tcb,                      // Pointer to the task's TCB.
               "platform task",                    // Name to help debugging.
               Platform_task,                      // Pointer to the task's code.
                DEF_NULL,                          // Pointer to task's argument.
                PLATFORM_TASK_PRIO,                // Task's priority.
               &platform_stack[0],                 // Pointer to base of stack.
               (PLATFORM_TASK_STACK_SIZE / 10u),   // Stack limit, from base.
               PLATFORM_TASK_STACK_SIZE,           // Stack size, in CPU_STK.
                0u,                                // Messages in task queue.
                0u,                                // Round-Robin time quanta.
                DEF_NULL,                          // External TCB data.
                (OS_OPT_TASK_STK_CLR),             // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * Platform task.
 ******************************************************************************/

static void Platform_task(void *arg){ // Awakens periodically to sample the capsense and change for the force applied to the platform

  PP_UNUSED_PARAM(arg);
  CAPSENSE_Init();

  while (1)
  {
    OSTimeDly(ConfigData.capsense_sample_rate/10, OS_OPT_TIME_DLY, &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
    OSMutexPend(&platform_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
    read_capsense(); // Read capsense slider positions
    if(((platform.left_position <= 0) || (platform.right_position >= 100000)) && (ConfigData.platform.wall_bounce.enable)) // Wall bounce handling
      {
        platform.velocity = (-1 * platform.velocity);
      }
    if(platform.direction_input == 1)       // Update force applied to platform based on slider input (left = negative, right = positive)
      {
        platform.force -= 2000000;
      }
    else if (platform.direction_input == 2)
      {
        platform.force -= 1000000;
      }
    else if(platform.direction_input == 3)
      {
        platform.force += 1000000;
      }
    else if(platform.direction_input == 4)
      {
        platform.force += 2000000;
      }
    else
      {
        platform.force = 0;
      }
    OSMutexPost (&platform_mutex, OS_OPT_POST_NONE, &err);
  }
}

/***************************************************************************//**
 * Initialize LCDDisplay Task
 ******************************************************************************/
void LCDDisplay_Task_Init(void){



  OSTaskCreate(&lcd_tcb,                             // Pointer to the task's TCB.
               "lcdDisplay task",                    // Name to help debugging.
               LCDDisplay_task,                      // Pointer to the task's code.
                DEF_NULL,                            // Pointer to task's argument.
                LCD_TASK_PRIO,                       // Task's priority.
               &lcd_stack[0],                        // Pointer to base of stack.
               (LCD_TASK_STACK_SIZE / 10u),          // Stack limit, from base.
               LCD_TASK_STACK_SIZE,                  // Stack size, in CPU_STK.
                0u,                                  // Messages in task queue.
                0u,                                  // Round-Robin time quanta.
                DEF_NULL,                            // External TCB data.
                (OS_OPT_TASK_STK_CLR),               // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * LCD Display task.
 ******************************************************************************/
static void LCDDisplay_task(void *arg){ // Periodically updating LCD screen

  PP_UNUSED_PARAM(arg);
  /* Local Variables */
  int hm_y = 0;
  int hm_x = 0;
  int left_plat = 0;
  int right_plat = 0;
  bool shield_active = false;
  bool laser_active = false;
  int hm_explosion = 5;
  int status;

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = Black;
  glibContext.foregroundColor = White;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  OSTmrStart(&timer_lcd, &err);

  RTOS_ERR err;
  while (1)
  {
      OSTimeDly(ConfigData.tau_lcd/10, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      GLIB_clear(&glibContext);
      OSMutexPend(&button_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      shield_active = button.shield_active;
      laser_active = button.laser_active;
      if(button.laser_active)
        {
          button.laser_active = false;
        }
      OSMutexPost(&button_mutex, OS_OPT_POST_NONE, &err);


      OSMutexPend(&platform_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      left_plat = platform.left_position * ConfigData.canyon_pixel_conversion;                      // Get platform position and scale to pixels
      right_plat = platform.right_position * ConfigData.canyon_pixel_conversion;                    // Get platform position and scale to pixels
      const GLIB_Rectangle_t plat_display = {left_plat, 116, right_plat, (116 + platform.height)};  // Create platform 
      OSMutexPost (&platform_mutex, OS_OPT_POST_NONE, &err);
      OSMutexPend(&hm_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      hm_x = harkonnan.position[0] * ConfigData.canyon_pixel_conversion;                            // Get HM position and scale to pixels
      hm_y = harkonnan.position[1] * ConfigData.canyon_pixel_conversion;                            // Get HM position and scale to pixels
      GLIB_drawCircleFilled(&glibContext, hm_x, hm_y, 3);                                           // Draw HM
      OSMutexPost (&hm_mutex, OS_OPT_POST_NONE, &err);
      GLIB_drawRectFilled(&glibContext, &plat_display);                                             // Draw platform
      if(shield_active)                                                                             // Check shield active
        {
          const GLIB_Rectangle_t shield_display = {left_plat - 3, 113, right_plat + 3, (121)};      // Create shield with platform position
          GLIB_drawRect(&glibContext, &shield_display);                                             // Draw shield
        }
      if(laser_active)                                                                              // Check for laer active
        {
          GLIB_drawLine(&glibContext, 0, 128, hm_x, hm_y);                                          // Draw laser
          OSTmrStop(&timer_lcd, OS_OPT_TMR_NONE, MyCallback_blank, &err);                           // Stop task timers to ensure explosion effect is not interrupted
          OSTmrStop(&timer_physics, OS_OPT_TMR_NONE, MyCallback_blank, &err);
          while(hm_explosion < 100)                                                                 // Draw consentric rings for explosion effect
          {
            GLIB_drawCircle(&glibContext, hm_x, hm_y, hm_explosion);
            hm_explosion += 3;
            DMD_updateDisplay();
            OSTimeDly(50, OS_OPT_TIME_DLY, &err);
          }
          hm_explosion = 5;
          OSTmrStart(&timer_lcd, &err);                                                             // Start task timers again
          OSTmrStart(&timer_physics, &err);
        }
      if(current_state == GAME_OVER)
        {
          GLIB_clear(&glibContext);
          GLIB_drawStringOnLine(&glibContext, "GAME OVER", 6, GLIB_ALIGN_CENTER, 5, 5, true);
          DMD_updateDisplay();
          while(1);
        }
      if(current_state == GAME_WIN)
        {
          GLIB_clear(&glibContext);
          GLIB_drawStringOnLine(&glibContext, "WINNER", 6, GLIB_ALIGN_CENTER, 5, 5, true);
          DMD_updateDisplay();
          while(1);
        }

      DMD_updateDisplay();
  }
}

/***************************************************************************//**
 * Initialize LED0
 ******************************************************************************/
void LED0_Task_Init(void){



  OSTaskCreate(&led0_tcb,                      // Pointer to the task's TCB.
               "led0 task",                    // Name to help debugging.
               led0_task,                      // Pointer to the task's code.
                DEF_NULL,                      // Pointer to task's argument.
                LED0_TASK_PRIO,                // Task's priority.
               &led0_stack[0],                 // Pointer to base of stack.
               (LED0_TASK_STACK_SIZE / 10u),   // Stack limit, from base.
               LED0_TASK_STACK_SIZE,           // Stack size, in CPU_STK.
                0u,                            // Messages in task queue.
                0u,                            // Round-Robin time quanta.
                DEF_NULL,                      // External TCB data.
                (OS_OPT_TASK_STK_CLR),         // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * LED0 task.
 ******************************************************************************/
static void led0_task(void *arg){ // Blink LED0 based on the ratio of hm velocity to max velocity 

  PP_UNUSED_PARAM(arg);
  int hm_vy = 0;
  int max_velocity = 43000;
  float ratio = 0;


  while (1)
  {
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      OSMutexPend(&hm_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
      hm_vy = -1 * harkonnan.max_velocity;

      OSMutexPost (&hm_mutex, OS_OPT_POST_NONE, &err);
      ratio = hm_vy;
      ratio /= max_velocity;
      ratio *= 250;
      if(current_state != GAME_OVER || current_state != GAME_WIN )
        {
          GPIO_PinOutToggle(LED0_port, LED0_pin);
        }
      OSTimeDly(ratio, OS_OPT_TIME_DLY, &err);

  }
}

/***************************************************************************//**
 * Initialize LED1
 ******************************************************************************/
void LED1_Task_Init(void)
{
  RTOS_ERR err;

  // Create Blink Task
  OSTaskCreate(&led1_tcb,
               "led1 task",
               led1_task,
               DEF_NULL,
               LED1_TASK_PRIO,
               &stack[0],
               (LED1_TASK_STACK_SIZE / 10u),
               LED1_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * LED1 task.
 ******************************************************************************/
static void led1_task(void *arg) // Blink LED1 based on the difference in the x velocity and position of hm and platform and the hm distance to ground
{                                // Also blink LED1 once per second when GAME OVER
    PP_UNUSED_PARAM(arg);
    int plat_position = 0;
    int plat_v = 0;
    int hm_x = 0;
    int hm_y = 0;
    int hm_vx = 0;
    int hm_vy = 0;
    int gravity = ConfigData.gravity;
    float blink_rate = 0;


    RTOS_ERR err;
    while (1)
    {
        OSMutexPend(&platform_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        plat_position = (platform.left_position + platform.right_position)/2;
        plat_v = platform.velocity;
        OSMutexPost (&platform_mutex, OS_OPT_POST_NONE, &err);
        OSMutexPend(&hm_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        hm_x = harkonnan.position[0];
        hm_y = harkonnan.position[1];
        hm_vx = harkonnan.velocity[0];
        hm_vy = harkonnan.velocity[1];
        OSMutexPost (&hm_mutex, OS_OPT_POST_NONE, &err);
        if(current_state!= GAME_OVER)
          {
            blink_rate = ball_meet_plat(plat_position, plat_v, hm_x, hm_y, 118/ConfigData.canyon_pixel_conversion);
            blink_rate = 0.4*(abs(hm_vx - plat_v)/17000) + .3*((hm_y - 100000)/100000) // Normalized weighted sum of hm and platform x velocity difference, hm y position,
                         + .4*(abs(hm_x - plat_position)/100000);                      // hm and platform x position difference
            blink_rate = 1 - blink_rate;
            OSTimeDly(100*blink_rate, OS_OPT_TIME_DLY, &err);
          }
        else
          {
            OSTimeDly(100, OS_OPT_TIME_DLY, &err);
          }
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        GPIO_PinOutToggle(LED1_port, LED1_pin); 
    }
}

/***************************************************************************//**
 * Initialize Idel
 ******************************************************************************/

void Idle_Task_Init(void){

  RTOS_ERR     err;

  OSTaskCreate(&idle_tcb,                      // Pointer to the task's TCB.
               "idle task",                    // Name to help debugging.
               idle_task,                      // Pointer to the task's code.
                DEF_NULL,                      // Pointer to task's argument.
                IDLE_TASK_PRIO,                // Task's priority.
               &idle_stack[0],                 // Pointer to base of stack.
               (IDLE_TASK_STACK_SIZE / 10u),   // Stack limit, from base.
               IDLE_TASK_STACK_SIZE,           // Stack size, in CPU_STK.
                0u,                            // Messages in task queue.
                0u,                            // Round-Robin time quanta.
                DEF_NULL,                      // External TCB data.
                (OS_OPT_TASK_STK_CLR),         // Task options.
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));


}

/***************************************************************************//**
 * Idel task.
 ******************************************************************************/

static void idle_task(void *arg){

  PP_UNUSED_PARAM(arg);

  while (1)
  {
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      EMU_EnterEM1();
      OSTimeDly(20, OS_OPT_TIME_DLY, &err);
  }

}

void resource_create(void)
{
  RTOS_ERR     err;
  OSTmrCreate(&timer_shield_active, "Timer_Shield_Active", 50,  0/*ConfigData.tau_LCD*/,        // Track time shield is active. When timer runs out, deactivate
              OS_OPT_TMR_ONE_SHOT,  MyCallback_Shield_Active, timer_callback_arg, &err);
  OSTmrCreate(&timer_shield_recharge, "Timer_Shield_Recharge", 100,  0/*ConfigData.tau_LCD*/,   // Track time shield recharging. When timer runs out, shield available 
              OS_OPT_TMR_ONE_SHOT,  MyCallback_Shield_Recharge, timer_callback_arg, &err);
  OSSemCreate (&sem_lcd, "lcd_semaphore", 0, &err);
  OSMutexCreate (&button_mutex, "button_mutex", &err);
  OSMutexCreate (&platform_mutex, "platform_mutex", &err);
  OSMutexCreate (&hm_mutex, "hm_mutex", &err);
}

/***************************************************************************//**
 * Reading Capsense function
 ******************************************************************************/

void read_capsense(void)
{
  CAPSENSE_Sense();  // Read capacitive touch sensor
  bool channel0 = CAPSENSE_getPressed(0); 
  bool channel1 = CAPSENSE_getPressed(1);
  bool channel2 = CAPSENSE_getPressed(2);
  bool channel3 = CAPSENSE_getPressed(3); 
  OSMutexPend(&platform_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
  if((channel0 || channel1) && (channel2 || channel3))          // Register no input if both sides of the slider are pressed
    {
      platform.direction_input = 0;
    }
  else if((channel0 || channel1) && !(channel2 || channel3))    // If left side is pressed and not right side
    {
      if(channel0 && !channel1)                                 // If far left channel is pressed and not middle left channel
        {
          platform.direction_input = 1;
        }
      else if(!channel0 && channel1)                            // If middle left channel is pressed and not far left channel
        {
          platform.direction_input = 2;
        }
      else                                                      // If both are pressed, register as middle left 
        {
          platform.direction_input = 2;
        }
    }
  else if((channel2 || channel3) && !(channel0 || channel1))    // If right side is pressed and not left side
    {
      if(channel2 && !channel3)                                 // If middle right channel is pressed and not far right channel
      {
        platform.direction_input = 3;
      }
      else if (!channel2 && channel3)                           // If far right channel is pressed and not middle right channel
      {
        platform.direction_input = 4;
      }
      else                                                      // If both are pressed, register as middle right
      {
        platform.direction_input = 3;
      }
    }
  else                                                          // Default = no input
    {
      platform.direction_input = 0;
    }
  OSMutexPost (&platform_mutex, OS_OPT_POST_NONE, &err);
}

/***************************************************************************//**
 * Timer Callback Functions
 ******************************************************************************/

void MyCallback_Shield_Active(OS_TMR * p_tmr, void * p_arg)                  // Call back function for shield active timer
{
  OSMutexPend(&button_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
  if(button.shield_active)
    {
      OSTmrStart(&timer_shield_recharge, &err);                              // Start recharge timer 
      button.shield_active = false;                                          // Deactivate shield
    }

  OSMutexPost (&button_mutex, OS_OPT_POST_NONE, &err);
}

void MyCallback_Shield_Recharge(OS_TMR * p_tmr, void * p_arg)                // Call back function for shield recharge timer
{
  OSMutexPend(&button_mutex, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
  button.shield_available = true;                                            // Make shield availible again
  OSMutexPost (&button_mutex, OS_OPT_POST_NONE, &err);
}

void MyCallback_blank(OS_TMR * p_tmr, void * p_arg)                          // Blank call back function. Called when stopping timers
{
  
}


