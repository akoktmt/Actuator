/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Motor_Driver.h"
#include <string.h>
#include "EncoderVelocity.h"
#include "PID.h"
#include "CAN_Handle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t rcdata[8];
extern uint8_t Can_RecFlag;
  double VelSetpoint;
  Encoder Enco;
  double speed;
  double PIDOut,PWM;
  PID_TypeDef TPID;
  VNH3SP30_t driver;
/* USER CODE END Variables */
/* Definitions for mControl_PIDTas */
osThreadId_t mControl_PIDTasHandle;
const osThreadAttr_t mControl_PIDTas_attributes = {
  .name = "mControl_PIDTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mCAN_RecTask */
osThreadId_t mCAN_RecTaskHandle;
const osThreadAttr_t mCAN_RecTask_attributes = {
  .name = "mCAN_RecTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myCAN_TransmitT */
osThreadId_t myCAN_TransmitTHandle;
const osThreadAttr_t myCAN_TransmitT_attributes = {
  .name = "myCAN_TransmitT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Control_PIDTask(void *argument);
void CAN_RecTask(void *argument);
void CAN_TransmitTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	VelSetpoint = 2;
	VNH3SP30_Init(&driver);
	Encoder_Init(&Enco);
	PID(&TPID, &speed, &PIDOut, &VelSetpoint, 4, 0, 3, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID, 50);
	PID_SetOutputLimits(&TPID, -1, 1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mControl_PIDTas */
  mControl_PIDTasHandle = osThreadNew(Control_PIDTask, NULL, &mControl_PIDTas_attributes);

  /* creation of mCAN_RecTask */
  mCAN_RecTaskHandle = osThreadNew(CAN_RecTask, NULL, &mCAN_RecTask_attributes);

  /* creation of myCAN_TransmitT */
  myCAN_TransmitTHandle = osThreadNew(CAN_TransmitTask, NULL, &myCAN_TransmitT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Control_PIDTask */
/**
  * @brief  Function implementing the mControl_PIDTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Control_PIDTask */
void Control_PIDTask(void *argument)
{
  /* USER CODE BEGIN Control_PIDTask */
	uint32_t Time=50;
  /* Infinite loop */
  for(;;)
  {
	  PWM += PIDOut;
	  VNH3SP30_SetSpeed(&driver,PWM);
	  Encoder_CaculateSpeed(&Enco,Time);
	  speed=Get_Speed(&Enco);
	  PID_Compute(&TPID);
	  osDelay(Time);
  }
  /* USER CODE END Control_PIDTask */
}

/* USER CODE BEGIN Header_CAN_RecTask */
/**
* @brief Function implementing the mCAN_RecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_RecTask */
void CAN_RecTask(void *argument)
{
  /* USER CODE BEGIN CAN_RecTask */
  /* Infinite loop */
  for(;;)
  {
	  if(Can_RecFlag==1){
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }
    osDelay(200);
  }
  /* USER CODE END CAN_RecTask */
}

/* USER CODE BEGIN Header_CAN_TransmitTask */
/**
* @brief Function implementing the myCAN_TransmitT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_TransmitTask */
void CAN_TransmitTask(void *argument)
{
  /* USER CODE BEGIN CAN_TransmitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_TransmitTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
