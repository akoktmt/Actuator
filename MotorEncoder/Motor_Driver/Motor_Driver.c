/*
 * Motor_Driver.c
 *
 *  Created on: Nov 6, 2023
 *      Author: win 10
 */
#include "tim.h"
#include "Motor_Driver.h"
#include "stm32f1xx_hal.h"
#include <math.h>
void VNH3SP30_Init(VNH3SP30_t* driver)
{
	driver->forward=0;
	driver->speed=0;
}

void VNH3SP30_SetSpeed(VNH3SP30_t* driver, int16_t speed)
{
	  if (speed > 400) {
	    speed = 400;
	  }
int pwm_duty = abs(speed) * 255 / 400;
  if (speed > 0 || (speed == 0 && driver->forward)) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);
	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    driver->forward = 1;
  } else if (speed < 0) {
	     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	   	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	   	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	   	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_duty);
    driver->forward = 0;
  }
}

void VNH3SP30_Brake(VNH3SP30_t* driver, int16_t brakePower)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
  driver->speed = 0;
}

uint8_t VNH3SP30_GetStatus(VNH3SP30_t* driver)
{
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
}

